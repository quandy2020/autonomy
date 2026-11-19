#!/usr/bin/env python3
"""Export trained SanD denoising weights with shared frozen DeFM token inputs.

The graph contains the learned projection, condition Transformer and original
UNet. It contains no second backbone and no local spline implementation. Expert
sample provenance and checkpoint hashes are checked before held-out tensor parity;
ONNX parity is not a collision, motion or model-quality acceptance criterion.
"""

import argparse
import importlib
import json
from pathlib import Path
import sys
import tempfile

import torch
from torch import nn

from defm_sand_encoder import FrozenDefmSandEncoder, validate_real_expert, fixed_adaptive_average
from train_defm_sand import digest, source, verify_expert_dataset


class SharedTokenEncoder(FrozenDefmSandEncoder):
    """Reuse the exact trained projection without constructing or executing a backbone."""
    def __init__(self):
        # The checkpoint deliberately excludes backbone parameters. Identity is
        # only an empty registration holder and forward never invokes it.
        super().__init__(nn.Identity(), lambda value, **kwargs: value)

    def forward(self, tokens):
        return self.project_tokens(tokens)

    def pool_spatial(self, spatial):
        return fixed_adaptive_average(spatial, self.condition_grid)


class DenoisingStep(nn.Module):
    """One native-layout v-prediction step; sampling and public spline math remain outside."""
    def __init__(self, model):
        super().__init__()
        self.condition_encoder = model.condition_encoder
        self.unet = model.unet

    def forward(self, depth_tokens, local_goal, sample, timestep):
        condition = self.condition_encoder(depth_tokens, local_goal)
        controls = sample.reshape(1, 8, 3).transpose(1, 2)
        prediction = self.unet(sample=controls, timestep=timestep,
                               encoder_hidden_states=condition).sample
        return prediction.transpose(1, 2).reshape(1, 24)


def main():
    """Require real trained tensors and compare native ONNX with the exact head on held-out experts."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sand-source', type=Path, required=True)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--checkpoint-sha256', required=True)
    parser.add_argument('--defm-onnx', type=Path, required=True)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--dataset-manifest', type=Path, required=True)
    parser.add_argument('--stats', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--allow-pilot-checkpoint', action='store_true')
    parser.add_argument('--validation-samples', type=int, default=4)
    parser.add_argument('--emit-native-fixture', action='store_true',
                        help='Save actual expert metric images and Torch step outputs for the native C++ consumer test')
    args = parser.parse_args()
    if args.output.exists() or not 1 <= args.validation_samples <= 64:
        parser.error('output must be a new directory and validation sample count must be bounded')
    if digest(args.checkpoint) != args.checkpoint_sha256:
        parser.error('actual trained checkpoint fingerprint differs')
    state = torch.load(args.checkpoint, map_location='cpu', weights_only=True)
    manifest = state.get('training_manifest', {})
    tensors = state.get('head_state_dict')
    if (manifest.get('schema') != 'autonomy.defm.sand.training.v1' or not manifest.get('epochs') or
            manifest.get('explicit_previous_plan_conditioning') is not False or
            manifest.get('history') != 4 or manifest.get('control_points') != 8 or
            not manifest.get('defm_frozen') or not isinstance(tensors, dict) or not tensors):
        parser.error('checkpoint is not the supported actual trained token-condition architecture')
    if manifest.get('pilot_only') and not args.allow_pilot_checkpoint:
        parser.error('pilot checkpoint export must be explicit and remains ineligible for deployment')
    if digest(args.defm_onnx) != manifest.get('frozen_defm_graph_sha256') or digest(args.stats) != manifest.get('stats_sha256'):
        parser.error('frozen graph or expert normalization differs from actual training')
    if digest(args.dataset_manifest) != manifest.get('dataset_manifest_sha256'):
        parser.error('actual expert set differs from checkpoint provenance')
    verify_expert_dataset(args.dataset, args.dataset_manifest, args.allow_pilot_checkpoint)
    upstream = source(args.sand_source, manifest['sand_source_revision'])
    sys.path.insert(0, str(upstream))
    import numpy as np
    import onnx
    import onnxruntime as ort
    conditions = importlib.import_module('sand_planner.nn.condition_encoders')
    diffusion = importlib.import_module('sand_planner.nn.ddpm')
    data = importlib.import_module('sand_planner.training.dataloader_bspline')
    normalization = importlib.import_module('sand_planner.utils.normalize')
    for module in (conditions, diffusion, data, normalization):
        if not Path(module.__file__).resolve().is_relative_to(upstream):
            raise RuntimeError('imported inference implementation is outside its pinned source')
    torch.set_num_threads(4)
    torch.manual_seed(manifest['arguments']['seed']); np.random.seed(manifest['arguments']['seed'])
    condition = conditions.ConcatConditionEncoder(feature_dim=256, seq_len=4,
        num_transformer_layers=4, num_heads=4, use_initial_turn=False, use_tea=False, use_type_embed=False)
    condition.depth_encoder = SharedTokenEncoder()
    normalizer = normalization.TrajectoryNormalizer(str(args.stats), method='percentile', margin=0., clamp=True)
    model = diffusion.BSplineDDPM(condition, fix_first_cp_zero=True, normalizer=normalizer,
        trajectory_interpolation='bspline', prediction_mode='control_points', num_control_points=8)
    model.load_state_dict(tensors, strict=True)
    graph = DenoisingStep(model).eval().requires_grad_(False)
    # The fused eval Transformer operator has no legacy ONNX symbolic. Use
    # Torch's unfused implementation of the same trained layers, and compare
    # it with the fused path on the real expert input before publication.
    original_fastpath = torch.backends.mha.get_fastpath_enabled()
    dataset = data.SandPlannerBSplineDataset(dataset_root=str(args.dataset.resolve()), sequence_length=4,
        frame_skip=3, min_gap=2, max_gap=42, image_size=(480, 640),
        normalize_depth=False, depth_scale=manifest['arguments']['depth_scale'], downscale_depth_half=False,
        data_augmentation=False, normalize_targets=True, stats_json_path=str(args.stats),
        norm_method='percentile', norm_margin=0., norm_clamp=True, trajectory_interpolation='bspline',
        prediction_mode='control_points', num_control_points=8, split_mode='val', train_ratio=.9,
        random_seed=manifest['arguments']['seed'], samples_per_epoch=args.validation_samples)
    if set(dataset.run_info) != set(manifest['validation_run_ids']):
        raise RuntimeError('held-out expert run identities differ from actual training')
    options = ort.SessionOptions(); options.intra_op_num_threads = 4
    features = ort.InferenceSession(str(args.defm_onnx), sess_options=options, providers=['CPUExecutionProvider'])

    def example():
        expert = dataset[0]
        record = dataset.run_info.get(expert.get('run_id'))
        if not record or 'run_path' not in record:
            raise RuntimeError('held-out input is a fallback rather than an actual expert sample')
        expert['source_run_path'] = record['run_path']; validate_real_expert(expert)
        frames = expert['depth_sequence'].numpy()
        tokens = np.concatenate([features.run(['patch_tokens'], {'depth_metric': frame[None]})[0] for frame in frames])
        token_tensor = torch.from_numpy(tokens.reshape(1, 4, 256, 384))
        goal = expert['end_relative_pose_norm'][None]
        sample = torch.randn(1, 24)
        # The existing native diffusion runner supplies a float32 timestep.
        # Integer-valued float steps preserve this model's time embedding while
        # avoiding an undeclared int64 input at the native TensorMap boundary.
        timestep = torch.randint(0, model.num_train_timesteps, (1,), dtype=torch.int64).to(torch.float32)
        return (token_tensor, goal, sample, timestep), expert['run_id'], expert

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.defm-sand-export-', dir=args.output.parent) as temporary:
        stage = Path(temporary); onnx_path = stage/'defm_sand_denoising_step.onnx'
        arguments, _, _ = example()
        with torch.inference_mode():
            fused_reference = graph(*arguments).numpy()
        torch.backends.mha.set_fastpath_enabled(False)
        with torch.inference_mode():
            unfused_reference = graph(*arguments).numpy()
        np.testing.assert_allclose(unfused_reference, fused_reference, rtol=5e-4, atol=5e-4)
        fastpath_error = float(np.max(np.abs(unfused_reference-fused_reference)))
        # The original upstream 1D UNet is evaluated/exported unchanged. No
        # approximate operator or alternate network is inserted to make it pass.
        with torch.inference_mode():
            torch.onnx.export(graph, arguments, str(onnx_path), dynamo=False, opset_version=17,
                input_names=['observation_depth_tokens', 'observation_goal', 'sample', 'timestep'],
                output_names=['action'], do_constant_folding=True)
        onnx.checker.check_model(str(onnx_path))
        native = ort.InferenceSession(str(onnx_path), sess_options=options, providers=['CPUExecutionProvider'])
        differences = []
        native_cases = []
        for case_index in range(args.validation_samples):
            arguments, run, expert = example()
            with torch.inference_mode():
                expected = graph(*arguments).numpy()
            feeds = dict(zip(['observation_depth_tokens', 'observation_goal', 'sample', 'timestep'],
                             [value.numpy() for value in arguments]))
            actual = native.run(['action'], feeds)[0]
            np.testing.assert_allclose(actual, expected, rtol=5e-4, atol=5e-4)
            differences.append({'held_out_run': run, 'max_abs_error': float(np.max(np.abs(actual-expected)))})
            if args.emit_native_fixture:
                from automsgs.msgs.sensor_msgs.image_pb2 import Image
                image_names = []
                for frame_index, depth in enumerate(expert['depth_sequence'].numpy()):
                    image = Image(width=640, height=480, step=640*4, encoding='32FC1', is_bigendian=False)
                    image.header.frame_id = 'test_depth_optical'
                    image.header.stamp.sec = 1000
                    image.header.stamp.nanosec = frame_index*100000000
                    image.data = np.asarray(depth[0], dtype='<f4').tobytes()
                    name = f'expert-{case_index}-depth-{frame_index}.pb'
                    (stage/name).write_bytes(image.SerializeToString()); image_names.append(name)
                native_cases.append({'expert_run': run, 'expert_start_index': int(expert['start_idx']),
                    'expert_end_index': int(expert['end_idx']), 'images': image_names,
                    'goal': arguments[1].numpy().reshape(-1).tolist(),
                    'sample': arguments[2].numpy().reshape(-1).tolist(),
                    'timestep': arguments[3].numpy().reshape(-1).tolist(),
                    'expected_v_prediction': expected.reshape(-1).tolist()})
        verify_expert_dataset(args.dataset, args.dataset_manifest, args.allow_pilot_checkpoint)
        result = {'schema': 'autonomy.defm.sand.denoising_step.v1',
            'checkpoint_sha256': args.checkpoint_sha256,
            'frozen_defm_graph_sha256': manifest['frozen_defm_graph_sha256'],
            'sand_source_revision': manifest['sand_source_revision'],
            'stats_sha256': manifest['stats_sha256'],
            'input_shapes': {'observation_depth_tokens': [1, 4, 256, 384], 'observation_goal': [1, 3],
                             'sample': [1, 24], 'timestep': [1]},
            'input_types': {value.name: value.type for value in native.get_inputs()},
            'prediction_type': 'v_prediction', 'contains_depth_backbone': False,
            'transformer_fastpath_reference_max_abs_error': fastpath_error,
            'explicit_previous_plan_conditioning': False, 'onnx_tensor_parity_verified': True,
            'parity_samples': differences, 'pilot_only': manifest.get('pilot_only') is True,
            'native_runtime_consumption_verified': False, 'closed_loop_verified': False,
            'deployment_ready': False, 'onnx_sha256': digest(onnx_path),
            'torch_version': str(torch.__version__), 'onnxruntime_version': ort.__version__}
        if args.emit_native_fixture:
            fixture = {'schema': 'autonomy.defm.sand.native_fixture.v1',
                'head_model_sha256': result['onnx_sha256'], 'defm_model_sha256': manifest['frozen_defm_graph_sha256'],
                'source_checkpoint_sha256': args.checkpoint_sha256, 'actual_held_out_expert_images': True,
                'controlled_offline_frame_timestamps': True, 'controlled_test_camera_metadata': True,
                'actual_localization_or_motion_verified': False,
                'camera': {'width': 640, 'height': 480, 'fx': 500., 'fy': 500., 'cx': 320., 'cy': 240.},
                'frame_id': 'test_depth_optical', 'calibration_revision': 7,
                'now_sec': 1000.35, 'cases': native_cases, 'pilot_only': manifest.get('pilot_only') is True}
            fixture_path = stage/'native-fixture.json'
            fixture_path.write_text(json.dumps(fixture, indent=2, allow_nan=False)+'\n')
            fixture_manifest = {'schema': 'autonomy.defm.sand.native_fixture_artifacts.v1',
                'head_model_sha256': result['onnx_sha256'],
                'files_sha256': {p.name: digest(p) for p in stage.iterdir()
                                 if p.name == 'native-fixture.json' or p.suffix == '.pb'},
                'pilot_only': manifest.get('pilot_only') is True, 'motion_verified': False}
            (stage/'native-fixture-artifacts.json').write_text(json.dumps(fixture_manifest, indent=2)+'\n')
            result['native_fixture_artifact_manifest_sha256'] = digest(stage/'native-fixture-artifacts.json')
        (stage/'manifest.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        stage.rename(args.output)
    torch.backends.mha.set_fastpath_enabled(original_fastpath)
    print(json.dumps({'exported': True, 'output': str(args.output), 'deployment_ready': False}), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
