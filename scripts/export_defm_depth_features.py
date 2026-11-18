#!/usr/bin/env python3
"""Export a frozen, strictly loaded official DeFM with its metric preprocessing.

The source checkout and checkpoint are explicit inputs. No random model, network
download, or alternative normalization is substituted. A static camera raster
and target patch grid are exported together, and actual ONNX Runtime results are
compared with the official PyTorch implementation before a bundle is published.
"""

import argparse
import hashlib
import importlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def sha256(path):
    """Digest the exact bytes of one source, weight or generated artifact."""
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def main():
    """Validate provenance, export the real frozen graph, and verify parity."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--source-revision', required=True)
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--weights-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--variant', default='defm_vit_s14')
    parser.add_argument('--camera-height', type=int, default=480)
    parser.add_argument('--camera-width', type=int, default=640)
    parser.add_argument('--target-height', type=int, default=224)
    parser.add_argument('--target-width', type=int, default=224)
    args = parser.parse_args()
    if args.variant != 'defm_vit_s14':
        parser.error('this patch-token contract currently supports official ViT-S/14 only')
    if args.output.exists():
        parser.error('output already exists; use a new bundle directory')
    if not (1 <= args.camera_height <= 1080 and 1 <= args.camera_width <= 1920):
        parser.error('camera raster exceeds the declared work bound')
    if any(x < 14 or x > 448 or x % 14 for x in (args.target_height, args.target_width)):
        parser.error('target dimensions must be multiples of 14 in [14,448]')
    if len(args.source_revision) != 40 or any(c not in '0123456789abcdef' for c in args.source_revision):
        parser.error('provide the full official source commit')
    if sha256(args.weights) != args.weights_sha256:
        parser.error('checkpoint digest mismatch')
    source = args.source.resolve(strict=True)
    # A Git checkout must match the requested commit and have no tracked edits.
    # Source archives are also supported, with their exact Python bytes retained.
    if (source / '.git').exists():
        commit = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
        dirty = subprocess.check_output(['git', '-C', str(source), 'status', '--porcelain', '--untracked-files=no'], text=True)
        if commit != args.source_revision or dirty:
            parser.error('source checkout revision or tracked bytes do not match')
    sys.path.insert(0, str(source))
    import numpy as np
    import torch
    import onnx
    import onnxruntime as ort

    factory = importlib.import_module('defm.model_factory')
    utilities = importlib.import_module('defm.utils.utils')
    if not Path(factory.__file__).resolve().is_relative_to(source):
        parser.error('imported DeFM is outside the requested source checkout')
    model = factory.create_defm_model(args.variant, pretrained=False)
    state = torch.load(args.weights, map_location='cpu', weights_only=True)
    if 'model' in state:
        state = state['model']
    model.load_state_dict(state, strict=True)
    model.eval().requires_grad_(False)
    if any(p.requires_grad for p in model.parameters()):
        raise RuntimeError('encoder weights are not frozen')

    class OfficialDepthFeatures(torch.nn.Module):
        """Apply the exact official tensor pipeline and return real patch tokens."""
        def __init__(self, backbone):
            super().__init__()
            self.backbone = backbone

        def forward(self, depth_metric):
            prepared = utilities.preprocess_depth_batch(
                depth_metric, target_size=(args.target_height, args.target_width),
                patch_size=14, device=depth_metric.device)
            features = self.backbone.forward_features(prepared)
            return prepared, features['x_norm_patchtokens'], features['x_norm_clstoken']

    graph = OfficialDepthFeatures(model).eval()
    rng = np.random.default_rng(20261118)
    depth = rng.uniform(.15, 8, (1, 1, args.camera_height, args.camera_width)).astype(np.float32)
    # Exercise actual invalid samples and the officially clipped metric scales.
    depth[0, 0, 0, :min(args.camera_width, 6)] = np.resize(
        np.array([0, np.nan, np.inf, -1, 9, 150], np.float32), min(args.camera_width, 6))
    tensor = torch.from_numpy(depth)
    with torch.inference_mode():
        official = tuple(x.cpu().numpy() for x in graph(tensor))
        image_prepared = utilities.preprocess_depth_image(
            depth[0, 0], target_size=(args.target_height, args.target_width), patch_size=14).numpy()
    # The two official entrypoints compute log1p in different float32
    # implementations (Torch and NumPy). At a full camera raster their tiny
    # differences survive antialiased resizing, including near zero. Check this
    # separately; the exported graph is still compared with its exact Torch
    # reference below, with no alternate preprocessing or resize implementation.
    image_batch_error = float(np.max(np.abs(official[0] - image_prepared)))
    np.testing.assert_allclose(official[0], image_prepared, rtol=3e-5, atol=3e-5)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.defm-export-', dir=args.output.parent) as directory:
        stage = Path(directory)
        model_path = stage / 'depth_features.onnx'
        # Opset 18 Resize supports the official antialiased resize. Do not replace
        # it with a nearest/ordinary bilinear approximation to make export pass.
        torch.onnx.export(graph, (tensor,), str(model_path), dynamo=True,
                          opset_version=18, external_data=False,
                          input_names=['depth_metric'],
                          output_names=['normalized_depth', 'patch_tokens', 'global_feature'])
        onnx.checker.check_model(str(model_path))
        session = ort.InferenceSession(str(model_path), providers=['CPUExecutionProvider'])
        actual = session.run(None, {'depth_metric': depth})
        errors = {}
        for name, expected, value in zip(('normalized_depth', 'patch_tokens', 'global_feature'), official, actual):
            np.testing.assert_allclose(value, expected, rtol=3e-4, atol=3e-4)
            errors[name] = float(np.max(np.abs(value - expected)))
        # Also check finite constant/all-invalid inputs: their geometric validity
        # is rejected separately by the native encoder, not fabricated here.
        for fill in (0., 2.):
            constant = torch.full_like(tensor, fill)
            with torch.inference_mode():
                expected = tuple(x.cpu().numpy() for x in graph(constant))
            values = session.run(None, {'depth_metric': constant.numpy()})
            for value, reference in zip(values, expected):
                np.testing.assert_allclose(value, reference, rtol=3e-4, atol=3e-4)
        fixture = {
            'camera_height': args.camera_height, 'camera_width': args.camera_width,
            'target_height': args.target_height, 'target_width': args.target_width,
            'patch_count': int(official[1].shape[1]), 'feature_dimensions': int(official[1].shape[2]),
            'depth_metric': np.nan_to_num(depth, nan=0, posinf=0, neginf=0).ravel().tolist(),
            'normalized_depth': official[0].ravel().tolist(),
            'patch_tokens': official[1].ravel().tolist(),
            'global_feature': official[2].ravel().tolist(),
        }
        (stage / 'parity.json').write_text(json.dumps(fixture, allow_nan=False) + '\n')
        manifest = {
            'schema': 'autonomy.defm.depth_features.v1', 'variant': args.variant,
            'source_revision': args.source_revision, 'weights_sha256': args.weights_sha256,
            'frozen': True, 'strict_checkpoint_load': True,
            'official_preprocessor': 'defm.utils.utils.preprocess_depth_batch',
            'source_sha256': {str(p.relative_to(source)): sha256(p) for p in sorted((source / 'defm').rglob('*.py'))},
            'input_shape': list(depth.shape), 'output_shapes': [list(x.shape) for x in official],
            'onnx_opset': 18, 'max_abs_errors': errors,
            'official_image_batch_max_abs_error': image_batch_error,
            'official_image_batch_tolerance': {'rtol': 3e-5, 'atol': 3e-5},
            'torch_version': torch.__version__, 'ort_version': ort.__version__,
            'artifacts': {n: sha256(stage / n) for n in ('depth_features.onnx', 'parity.json')},
            'scope': 'Real frozen encoder and official preprocessing parity; not a trained SanD adapter or robot trial.'
        }
        (stage / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        os.rename(stage, args.output)
    print(json.dumps({'bundle': str(args.output), 'passed': True, 'max_abs_errors': errors}))


if __name__ == '__main__':
    main()
