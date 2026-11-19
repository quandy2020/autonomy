"""Local tensor/freeze contracts only; fixtures are not real DeFM weights or experts."""
from pathlib import Path
import sys
import tempfile
import unittest
import torch
from torch import nn

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'training'))
from defm_sand_encoder import FrozenDefmSandEncoder, FrozenOnnxDefmSandEncoder, validate_real_expert, fixed_adaptive_average


class ControlledBackbone(nn.Module):
    """Small instrumented dependency to exercise the freeze and token protocol."""
    def __init__(self):
        super().__init__()
        self.weight = nn.Parameter(torch.arange(384, dtype=torch.float32) / 384)
        self.norm = nn.BatchNorm1d(384)
        self.invalid_shape = False

    def forward_features(self, depth):
        count = 255 if self.invalid_shape else 256
        features = self.norm(self.weight[None, :].expand(depth.shape[0], -1))
        features = features + depth.flatten(1).mean(dim=1)[:, None]
        return {'x_norm_patchtokens': features[:, None, :].expand(-1, count, -1)}


def controlled_preprocess(depth, **kwargs):
    """Capture original metric values; production still requires official preprocessing."""
    if depth.max() != 2:
        raise ValueError('test expected unnormalized metric depth')
    return depth


class DefmSandContractTest(unittest.TestCase):
    def test_fixed_pool_export_preserves_overlapping_adaptive_bins(self):
        value = torch.randn(2, 384, 16, 16)
        expected = torch.nn.functional.adaptive_avg_pool2d(value, (8, 12))
        actual = fixed_adaptive_average(value, (8, 12))
        torch.testing.assert_close(actual, expected, rtol=1e-6, atol=1e-6)
        impulse = torch.zeros(1, 1, 16, 16); impulse[0, 0, 7, 7] = 1
        torch.testing.assert_close(fixed_adaptive_average(impulse, (8, 12)),
                                  torch.nn.functional.adaptive_avg_pool2d(impulse, (8, 12)), rtol=0, atol=0)

    def test_exact_graph_provider_uses_original_pixels_and_its_real_token_cache(self):
        import numpy as np
        from types import SimpleNamespace

        class ControlledSession:
            calls = 0
            def get_inputs(self):
                return [SimpleNamespace(name='depth_metric', shape=[1, 1, 8, 8], type='tensor(float)')]
            def get_outputs(self):
                return [SimpleNamespace(name='patch_tokens', shape=[1, 256, 384])]
            def run(self, names, inputs):
                self.calls += 1
                self.latest = inputs['depth_metric'].copy()
                return [np.full((1, 256, 384), self.latest.mean(), dtype=np.float32)]

        with tempfile.TemporaryDirectory() as directory:
            session = ControlledSession(); backbone = ControlledBackbone()
            backbone.forward_features = lambda value: (_ for _ in ()).throw(RuntimeError('different feature provider'))
            module = FrozenOnnxDefmSandEncoder(backbone, controlled_preprocess, session,
                                              cache_directory=directory, history=1, feature_dim=32).eval()
            depth = torch.full((1, 1, 1, 8, 8), 2.)
            first, _ = module(depth)
            repeated, _ = module(depth)
            self.assertEqual(session.calls, 1)
            self.assertTrue(torch.equal(first, repeated))
            self.assertTrue(np.array_equal(session.latest, depth.numpy().reshape(1, 1, 8, 8)))
            cache = next(Path(directory).glob('*.npy'))
            np.save(cache, np.zeros((1, 8, 384), dtype=np.float32))
            with self.assertRaises(ValueError):
                module(depth)

    def test_optimizer_cannot_change_frozen_backbone_or_statistics(self):
        backbone = ControlledBackbone()
        module = FrozenDefmSandEncoder(backbone, controlled_preprocess, feature_dim=32)
        before = {key: value.clone() for key, value in backbone.state_dict().items()}
        module.train()
        optimizer = torch.optim.AdamW((p for p in module.parameters() if p.requires_grad), lr=.01)
        output, motion = module(torch.full((2, 4, 1, 8, 8), 2.))
        self.assertEqual(tuple(output.shape), (2, 384, 32))
        self.assertIsNone(motion)
        output.square().mean().backward()
        self.assertIsNotNone(module.projection[0].weight.grad)
        optimizer.step()
        self.assertFalse(backbone.training)
        self.assertTrue(all(p.grad is None and not p.requires_grad for p in backbone.parameters()))
        for key, value in backbone.state_dict().items():
            self.assertTrue(torch.equal(before[key], value), key)

    def test_protocol_failures_do_not_become_plausible_tokens(self):
        backbone = ControlledBackbone()
        module = FrozenDefmSandEncoder(backbone, controlled_preprocess)
        with self.assertRaises(ValueError):
            module(torch.full((1, 3, 1, 8, 8), 2.))
        backbone.invalid_shape = True
        with self.assertRaises(ValueError):
            module(torch.full((1, 4, 1, 8, 8), 2.))

    def test_shared_tokens_need_no_second_backbone_execution(self):
        backbone = ControlledBackbone()
        module = FrozenDefmSandEncoder(backbone, controlled_preprocess, feature_dim=32)
        with torch.no_grad():
            tokens = backbone.forward_features(torch.full((4, 1, 8, 8), 2.))['x_norm_patchtokens']
        module.eval()
        direct, _ = module(torch.full((1, 4, 1, 8, 8), 2.))
        backbone.forward_features = lambda depth: (_ for _ in ()).throw(RuntimeError('duplicate backbone call'))
        shared, _ = module.project_tokens(tokens.reshape(1, 4, 256, 384))
        self.assertTrue(torch.equal(direct, shared))
        supplied = tokens.reshape(1, 4, 256, 384).clone().requires_grad_(True)
        module.project_tokens(supplied)[0].square().mean().backward()
        self.assertIsNone(supplied.grad)
        backbone.requires_grad_(True)
        with self.assertRaises(RuntimeError):
            module(torch.full((1, 4, 1, 8, 8), 2.))

    def test_dummy_and_missing_normalization_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            sample = {'bspline_success': True, 'run_id': 'controlled_fixture', 'original_run_id': directory,
                      'depth_sequence': torch.ones(4, 1, 480, 640),
                      'control_points_norm': torch.zeros(8, 3), 'end_relative_pose_norm': torch.zeros(3)}
            validate_real_expert(sample)
            sample['run_id'] = 'dummy'
            with self.assertRaises(RuntimeError):
                validate_real_expert(sample)

    def test_short_upstream_run_id_keeps_actual_source_path(self):
        with tempfile.TemporaryDirectory() as directory:
            sample = {'bspline_success': True, 'run_id': 'dataset_avoid_run_0001',
                      'original_run_id': 'run_0001', 'source_run_path': directory,
                      'depth_sequence': torch.ones(4, 1, 480, 640),
                      'control_points_norm': torch.zeros(8, 3), 'end_relative_pose_norm': torch.zeros(3)}
            validate_real_expert(sample)
            self.assertEqual(sample['original_run_id'], 'run_0001')
            sample['run_id'] = 'controlled_fixture'
            del sample['control_points_norm']
            with self.assertRaises(RuntimeError):
                validate_real_expert(sample)

    def test_out_of_range_or_unmeasured_history_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            sample = {'bspline_success': True, 'run_id': 'controlled_fixture', 'original_run_id': directory,
                      'depth_sequence': torch.ones(4, 1, 480, 640),
                      'control_points_norm': torch.full((8, 3), 2.), 'end_relative_pose_norm': torch.zeros(3)}
            with self.assertRaises(RuntimeError):
                validate_real_expert(sample)
            sample['control_points_norm'].zero_()
            sample['depth_sequence'][1].zero_()
            with self.assertRaises(RuntimeError):
                validate_real_expert(sample)


if __name__ == '__main__':
    unittest.main()
