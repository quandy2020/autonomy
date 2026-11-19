#!/usr/bin/env python3
"""Train a real frozen-DeFM SanD head using pinned official sources and experts.

There is no simulator or online navigation here. Real expert labels and the
upstream diffusion implementation are retained. Dummy dataset fallback and
missing normalization are rejected. This script cannot declare deployment ready:
an ONNX export, native parity and closed-loop validation remain separate gates.
"""

import argparse
import hashlib
import importlib
import json
from pathlib import Path
import random
import subprocess
import sys

from defm_sand_encoder import FrozenOnnxDefmSandEncoder, validate_real_expert


def digest(path):
    """Hash actual source/weight bytes without trusting a filename."""
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(chunk)
    return value.hexdigest()


def source(path, revision):
    """Require the exact reviewed Git source before executing upstream modules."""
    path = path.resolve(strict=True)
    actual = subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip()
    dirty = subprocess.check_output(['git', '-C', str(path), 'status', '--porcelain', '--untracked-files=no'], text=True)
    if actual != revision or dirty:
        raise ValueError('upstream source revision or tracked bytes differ: ' + str(path))
    return path


def verify_expert_dataset(root, manifest_path, allow_pilot=False):
    """Check actual extracted expert files and reject a partial set for production training."""
    root = root.resolve(strict=True)
    manifest = json.loads(manifest_path.read_text())
    files = manifest.get('files')
    if (manifest.get('schema') != 'autonomy.sand.experts.v1' or not isinstance(files, dict) or not files or
            manifest.get('dummy_or_synthetic_records') is not False or
            Path(manifest.get('dataset_root', '')).resolve() != root):
        raise ValueError('actual expert provenance manifest does not match the selected dataset')
    pilot = manifest.get('pilot_only') is True
    if (pilot and not allow_pilot) or (not pilot and manifest.get('full_archive_sha256_verified') is not True):
        raise ValueError('production expert training requires the verified complete archive; pilot use must be explicit')
    actual = {str(p.relative_to(root)) for p in root.rglob('*') if p.is_file()}
    if actual != set(files):
        raise ValueError('actual expert dataset differs from its exact extracted-file manifest')
    for name, evidence in files.items():
        relative = Path(name)
        path = root / relative
        if (relative.is_absolute() or '..' in relative.parts or not path.resolve(strict=True).is_relative_to(root) or
                path.stat().st_size != evidence.get('bytes') or digest(path) != evidence.get('sha256')):
            raise ValueError('actual expert bytes or path differ from recorded archive member: ' + name)
    return manifest


def main():
    """Train only actual experts; write a head checkpoint without frozen backbone duplication."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sand-source', type=Path, required=True)
    parser.add_argument('--sand-revision', required=True)
    parser.add_argument('--defm-source', type=Path, required=True)
    parser.add_argument('--defm-revision', required=True)
    parser.add_argument('--defm-weights', type=Path, required=True)
    parser.add_argument('--defm-weights-sha256', required=True)
    parser.add_argument('--defm-onnx', type=Path, required=True)
    parser.add_argument('--defm-export-manifest', type=Path, required=True)
    parser.add_argument('--token-cache', type=Path, help='Separate learned-head cache; contents are namespaced by the frozen graph digest')
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--dataset-manifest', type=Path, required=True)
    parser.add_argument('--allow-pilot-dataset', action='store_true',
                        help='Exercise training with explicit complete expert runs; this cannot produce a deployment-ready result')
    parser.add_argument('--stats', type=Path, required=True)
    parser.add_argument('--stats-sha256', required=True)
    parser.add_argument('--depth-scale', type=float, required=True,
                        help='Actual source units per metre; never infer units from image dtype')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--device', default='cuda')
    parser.add_argument('--epochs', type=int, default=100)
    parser.add_argument('--samples-per-epoch', type=int, default=19200)
    parser.add_argument('--batch-size', type=int, default=8)
    parser.add_argument('--learning-rate', type=float, default=2e-4)
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()
    if (args.output.exists() or not args.dataset.is_dir() or not 1 <= args.epochs <= 10000 or
            not 1 <= args.batch_size <= 256 or not 1 <= args.samples_per_epoch <= 1000000 or
            not 0 < args.learning_rate <= .1 or not 0 < args.depth_scale <= 1000000):
        parser.error('real dataset, new output directory and bounded training policy are required')
    for revision in (args.sand_revision, args.defm_revision):
        if len(revision) != 40 or any(c not in '0123456789abcdef' for c in revision):
            parser.error('full pinned official source revisions are required')
    if digest(args.defm_weights) != args.defm_weights_sha256 or digest(args.stats) != args.stats_sha256:
        parser.error('actual pretrained backbone or expert-normalization digest differs')
    expert_manifest = verify_expert_dataset(args.dataset, args.dataset_manifest, args.allow_pilot_dataset)
    export = json.loads(args.defm_export_manifest.read_text())
    graph_digest = digest(args.defm_onnx)
    if (export.get('schema') != 'autonomy.defm.depth_features.v1' or not export.get('frozen') or
            not export.get('strict_checkpoint_load') or export.get('weights_sha256') != args.defm_weights_sha256 or
            export.get('source_revision') != args.defm_revision or export.get('artifacts', {}).get('depth_features.onnx') != graph_digest):
        parser.error('frozen native DeFM graph provenance does not match the actual checkpoint and source')
    sand = source(args.sand_source, args.sand_revision)
    defm = source(args.defm_source, args.defm_revision)
    sys.path[:0] = [str(sand), str(defm)]
    import numpy as np
    import torch
    import onnxruntime as ort
    from torch.utils.data import DataLoader
    factory = importlib.import_module('defm.model_factory')
    utilities = importlib.import_module('defm.utils.utils')
    conditions = importlib.import_module('sand_planner.nn.condition_encoders')
    diffusion = importlib.import_module('sand_planner.nn.ddpm')
    data = importlib.import_module('sand_planner.training.dataloader_bspline')
    normalization = importlib.import_module('sand_planner.utils.normalize')
    for module, root in ((factory, defm), (utilities, defm), (conditions, sand), (diffusion, sand), (data, sand), (normalization, sand)):
        if not Path(module.__file__).resolve().is_relative_to(root):
            raise RuntimeError('imported implementation is outside its pinned source: ' + module.__name__)
    random.seed(args.seed); np.random.seed(args.seed); torch.manual_seed(args.seed)
    backbone = factory.create_defm_model('defm_vit_s14', pretrained=False)
    weights = torch.load(args.defm_weights, map_location='cpu', weights_only=True)
    backbone.load_state_dict(weights.get('model', weights), strict=True)
    session_options = ort.SessionOptions(); session_options.intra_op_num_threads = 4
    frozen_session = ort.InferenceSession(str(args.defm_onnx), sess_options=session_options, providers=['CPUExecutionProvider'])
    cache = args.token_cache / graph_digest if args.token_cache is not None else None
    encoder = FrozenOnnxDefmSandEncoder(backbone, utilities.preprocess_depth_batch, frozen_session, cache_directory=cache)
    condition = conditions.ConcatConditionEncoder(feature_dim=256, seq_len=4,
        num_transformer_layers=4, num_heads=4, use_initial_turn=False, use_tea=False, use_type_embed=False)
    condition.depth_encoder = encoder
    normalizer = normalization.TrajectoryNormalizer(str(args.stats), method='percentile', margin=0., clamp=True)
    model = diffusion.BSplineDDPM(condition, fix_first_cp_zero=True, normalizer=normalizer,
        trajectory_interpolation='bspline', prediction_mode='control_points', num_control_points=8).to(args.device)

    class RealExperts(data.SandPlannerBSplineDataset):
        """Keep official fitting/labels while rejecting its all-zero fallback."""
        def _get_dummy_sample(self):
            raise RuntimeError('actual expert loading/fitting failed; dummy training is forbidden')

        def __getitem__(self, index):
            sample = super().__getitem__(index)
            actual = self.run_info.get(sample.get('run_id'))
            if not actual or 'run_path' not in actual:
                raise RuntimeError('expert run is absent from the actual selected dataset split')
            run = Path(actual['run_path']).resolve(strict=True)
            if not run.is_relative_to(args.dataset.resolve()):
                raise RuntimeError('expert run escapes the selected actual dataset')
            sample['source_run_path'] = str(run)
            validate_real_expert(sample)
            return sample

    def dataset(split):
        return RealExperts(dataset_root=str(args.dataset.resolve()), sequence_length=4,
            frame_skip=3, min_gap=2, max_gap=42, image_size=(480, 640),
            normalize_depth=False, depth_scale=args.depth_scale, downscale_depth_half=False, data_augmentation=False,
            normalize_targets=True, stats_json_path=str(args.stats.resolve()),
            norm_method='percentile', norm_margin=0., norm_clamp=True,
            trajectory_interpolation='bspline', prediction_mode='control_points', num_control_points=8,
            samples_per_epoch=args.samples_per_epoch, split_mode=split, train_ratio=.9, random_seed=args.seed)
    # Use the native Torch collator to retain provenance/normalization keys; the
    # upstream convenience collator would hide a missing normalized target.
    training = DataLoader(dataset('train'), batch_size=args.batch_size, num_workers=0)
    validation = DataLoader(dataset('val'), batch_size=args.batch_size, num_workers=0)
    train_runs, val_runs = set(training.dataset.run_info), set(validation.dataset.run_info)
    if not train_runs or not val_runs or train_runs & val_runs:
        raise RuntimeError('actual expert train/validation run identities overlap or a split is empty')
    optimizer = torch.optim.AdamW((p for p in model.parameters() if p.requires_grad), lr=args.learning_rate)
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = {'schema': 'autonomy.defm.sand.training.v1', 'defm_frozen': True,
        'sand_source_revision': args.sand_revision, 'defm_source_revision': args.defm_revision,
        'defm_weights_sha256': args.defm_weights_sha256, 'stats_sha256': args.stats_sha256,
        'frozen_defm_graph_sha256': graph_digest, 'training_feature_provider': 'verified_native_frozen_onnx',
        'onnxruntime_version': ort.__version__,
        'dataset_root': str(args.dataset.resolve()), 'dataset_bytes_immutability_verified': True,
        'dataset_manifest_sha256': digest(args.dataset_manifest),
        'pilot_only': expert_manifest.get('pilot_only') is True,
        'full_expert_archive_verified': expert_manifest.get('full_archive_sha256_verified') is True,
        'training_run_ids': sorted(train_runs), 'validation_run_ids': sorted(val_runs),
        'history': 4, 'control_points': 8, 'metric_depth': True, 'dummy_samples_allowed': False,
        'torch_version': str(torch.__version__), 'numpy_version': str(np.__version__),
        'motion_conditioning': 'actual_four_frame_depth_history',
        'explicit_previous_plan_conditioning': False,
        'closed_loop_verified': False, 'native_export_verified': False, 'deployment_ready': False,
        'arguments': {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
        'epochs': []}
    def step(batch):
        return model(batch['control_points_norm'].to(args.device), batch['depth_sequence'].to(args.device),
            batch['end_relative_pose_norm'].to(args.device))['loss']
    try:
        for epoch in range(args.epochs):
            model.train(); train_loss = 0.; train_count = 0
            for batch in training:
                optimizer.zero_grad(set_to_none=True); loss = step(batch)
                if not torch.isfinite(loss): raise RuntimeError('actual diffusion loss is nonfinite')
                loss.backward(); torch.nn.utils.clip_grad_norm_([p for p in model.parameters() if p.requires_grad], 1., error_if_nonfinite=True)
                if any(p.grad is not None or p.requires_grad for p in encoder.backbone.parameters()):
                    raise RuntimeError('shared DeFM freeze was violated')
                optimizer.step(); train_loss += loss.item(); train_count += 1
            model.eval(); validation_loss = 0.; validation_count = 0
            with torch.no_grad():
                for batch in validation:
                    loss = step(batch)
                    if not torch.isfinite(loss): raise RuntimeError('held-out diffusion loss is nonfinite')
                    validation_loss += loss.item(); validation_count += 1
            if not train_count or not validation_count: raise RuntimeError('actual training or validation split is empty')
            record = {'epoch': epoch + 1, 'train_loss': train_loss/train_count,
                      'validation_loss': validation_loss/validation_count}
            manifest['epochs'].append(record); print(json.dumps(record), flush=True)
            head = {k: v.detach().cpu() for k, v in model.state_dict().items()
                    if not k.startswith('condition_encoder.depth_encoder.backbone.')}
            verify_expert_dataset(args.dataset, args.dataset_manifest, args.allow_pilot_dataset)
            # TorchVersion is a string subclass requiring a pickle allowlist.
            # Persist only ordinary JSON primitives so weights_only=True is
            # sufficient for the exported inference checkpoint.
            portable_manifest = json.loads(json.dumps(manifest, allow_nan=False))
            torch.save({'head_state_dict': head, 'training_manifest': portable_manifest}, args.output/'defm_sand_head.pt')
    except BaseException as error:
        manifest['training_error'] = type(error).__name__ + ': ' + str(error)
        raise
    finally:
        (args.output/'training_manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False) + '\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
