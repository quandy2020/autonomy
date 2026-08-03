"""Load simplified Docker platform YAML."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError as exc:
    raise ImportError('PyYAML required: pip3 install pyyaml') from exc


CONFIG_DIR = Path(__file__).resolve().parent.parent / 'config'
PLATFORMS_FILE = CONFIG_DIR / 'platforms.yaml'

# Image name → build metadata (not duplicated in YAML)
IMAGE_REGISTRY: dict[str, dict[str, Any]] = {
    'autonomy.platform.x86_64': {
        'build_script': 'build_docker_x86_64.py',
        'dockerfile': 'dockerfile/autonomy.x86_64.dockerfile',
        'arch': 'x86_64',
    },
    'autonomy.platform.x86_64.nvidia': {
        'build_script': 'build_docker_x86_64.py',
        'dockerfile': 'dockerfile/autonomy.x86_64.nvidia.dockerfile',
        'arch': 'x86_64',
        'build_nvidia': True,
    },
    'autonomy.platform.aarch64': {
        'build_script': 'build_docker_aarch64.py',
        'dockerfile': 'dockerfile/autonomy.aarch64.dockerfile',
        'arch': 'aarch64',
        'docker_platform': 'linux/arm64',
    },
}

DEFAULT_CONTAINER_NAME = 'SpaceHero'
DEFAULT_PORTS = ['8765:8765']
DEFAULT_ENV = {
    'QT_X11_NO_MITSHM': '1',
    'AUTONOMY_DEV_DIR': '/workspace/autonomy',
}
ISAAC_ENV = {
    'ACCEPT_EULA': 'Y',
    'OMNI_KIT_ACCEPT_EULA': 'YES',
}

# CLI (-p, -n) → platform id in platforms.yaml
CLI_PLATFORM_MAP = {
    ('x86_64', 'no'): 'x86_64',
    ('x86_64', 'yes'): 'nvidia',
    ('x86_64', 'auto'): 'x86_64',
    ('aarch64', 'auto'): 'aarch64',
    ('aarch64', 'no'): 'aarch64',
    ('arm64', 'auto'): 'aarch64',
}


def _load_catalog() -> dict[str, Any]:
    if not PLATFORMS_FILE.is_file():
        raise FileNotFoundError(f'Missing {PLATFORMS_FILE}')
    with PLATFORMS_FILE.open(encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError('platforms.yaml root must be a mapping')
    return data


def _merge_dict(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    out = dict(base)
    for key, value in override.items():
        if key == 'use':
            continue
        if key == 'env' and isinstance(value, dict) and isinstance(out.get('env'), dict):
            out['env'] = {**out['env'], **value}
        elif key == 'ports' and isinstance(value, list) and isinstance(out.get('ports'), list):
            out['ports'] = list(out['ports']) + [p for p in value if p not in out['ports']]
        else:
            out[key] = value
    return out


def resolve_profile_overlay(catalog: dict[str, Any], profile: str) -> dict[str, Any]:
    profiles = catalog.get('profiles') or {}
    if profile == 'default' or profile not in profiles:
        if profile != 'default':
            known = ', '.join(sorted(profiles)) or '(none)'
            raise KeyError(f'Unknown profile "{profile}". Known: {known}')
        return {}

    overlay = dict(profiles[profile])
    use = overlay.pop('use', None)
    if use:
        base = resolve_profile_overlay(catalog, str(use))
        overlay = _merge_dict(base, overlay)
    return overlay


def resolve_platform_id(platform_arch: str, use_nvidia: str) -> str:
    arch = platform_arch
    if arch in ('amd64', 'x64'):
        arch = 'x86_64'
    if arch == 'arm64':
        arch = 'aarch64'
    key = (arch, use_nvidia)
    if key in CLI_PLATFORM_MAP:
        return CLI_PLATFORM_MAP[key]
    if arch == 'aarch64':
        return 'aarch64'
    return 'nvidia' if use_nvidia == 'yes' else 'x86_64'


def platform_id_for_image(image_name: str) -> str:
    if 'nvidia' in image_name:
        return 'nvidia'
    if 'aarch64' in image_name:
        return 'aarch64'
    return 'x86_64'


@dataclass(frozen=True)
class PlatformRunConfig:
    """Resolved platform + profile settings."""

    platform_id: str
    profile: str
    image_name: str
    arch: str
    gpu: bool
    isaac: bool
    keep_entrypoint: bool
    container_name: str
    ports: list[str]
    environment: dict[str, str]
    docker_platform: str | None
    build_script: str
    dockerfile: str
    build_nvidia: bool

    @classmethod
    def load(cls, platform_id: str, profile: str = 'default') -> PlatformRunConfig:
        catalog = _load_catalog()
        platforms = catalog.get('platforms') or {}
        if platform_id not in platforms:
            known = ', '.join(sorted(platforms))
            raise KeyError(f'Unknown platform "{platform_id}". Known: {known}')

        merged = dict(platforms[platform_id])
        merged = _merge_dict(merged, resolve_profile_overlay(catalog, profile))

        image_name = str(merged.get('image', ''))
        meta = IMAGE_REGISTRY.get(image_name, {})
        if not image_name:
            raise ValueError(f'Platform "{platform_id}" missing image')

        env = dict(DEFAULT_ENV)
        if merged.get('isaac') or merged.get('gpu'):
            env.update(ISAAC_ENV)
        env.update({str(k): str(v) for k, v in (merged.get('env') or {}).items()})

        ports = list(DEFAULT_PORTS)
        for item in merged.get('ports') or []:
            spec = str(item)
            if spec not in ports:
                ports.append(spec)

        return cls(
            platform_id=platform_id,
            profile=profile,
            image_name=image_name,
            arch=str(merged.get('arch') or meta.get('arch', 'x86_64')),
            gpu=bool(merged.get('gpu', False)),
            isaac=bool(merged.get('isaac', False)),
            keep_entrypoint=bool(merged.get('keep_entrypoint', False)),
            container_name=str(merged.get('name', DEFAULT_CONTAINER_NAME)),
            ports=ports,
            environment=env,
            docker_platform=meta.get('docker_platform'),
            build_script=str(meta.get('build_script', '')),
            dockerfile=str(meta.get('dockerfile', '')),
            build_nvidia=bool(meta.get('build_nvidia', False)),
        )


def list_platform_configs() -> list[dict[str, str]]:
    catalog = _load_catalog()
    platforms = catalog.get('platforms') or {}
    profiles = catalog.get('profiles') or {}
    entries = []
    for pid, spec in platforms.items():
        image = spec.get('image', '') if isinstance(spec, dict) else ''
        entries.append({
            'id': pid,
            'image': str(image),
            'profiles': ', '.join(sorted(profiles)) or 'default',
        })
    return entries


def format_config_catalog() -> str:
    lines = ['Platforms (config/platforms.yaml):', '']
    for entry in list_platform_configs():
        lines.append(f"  {entry['id']:10}  {entry['image']}")
        lines.append(f"             profiles: {entry['profiles']}")
    lines.extend([
        '',
        'Examples:',
        '  python3 run_autonomy.py --platform nvidia --profile webrtc',
        '  python3 run_autonomy.py -p x86_64 -n yes --profile navrl',
    ])
    return '\n'.join(lines)


# Backward-compatible aliases
CONFIG_PATH = PLATFORMS_FILE


def config_path_for_image(image_name: str, *, config_dir: Path | None = None) -> Path:
    return PLATFORMS_FILE


def load_yaml_config(path: Path, *, profile: str | None = None) -> dict[str, Any]:
    """Legacy helper — prefer PlatformRunConfig.load(platform_id, profile)."""
    pid = platform_id_for_image(path.stem if path.name != 'platforms.yaml' else 'nvidia')
    cfg = PlatformRunConfig.load(pid, profile or 'default')
    return {
        'image': {'name': cfg.image_name, 'build_nvidia': cfg.build_nvidia},
        'platform': {'arch': cfg.arch, 'docker_platform': cfg.docker_platform},
        'container': {'name': cfg.container_name},
        'ports': cfg.ports,
        'environment': cfg.environment,
        'isaac': {'accept_eula': cfg.isaac, 'override_entrypoint': not cfg.keep_entrypoint},
        'gpu': {'prefer': 'yes' if cfg.gpu else 'no'},
    }
