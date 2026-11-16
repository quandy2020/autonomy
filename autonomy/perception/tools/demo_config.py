"""Read one perception demo profile with command-line overrides."""

import json
from pathlib import Path


def parse_demo_arguments(parser, profile, *, shared=None, required=()):
    parser.add_argument("--conf", type=Path,
                        help="JSON configuration")
    partial, _ = parser.parse_known_args()
    if partial.conf:
        try:
            with partial.conf.open(encoding="utf-8") as config:
                document = json.load(config)
            if isinstance(document, dict) and profile in document:
                values = document[profile]
                defaults = document.get(shared, {}) if shared else {}
                if not isinstance(defaults, dict) or not isinstance(values, dict):
                    raise ValueError("demo sections must be JSON objects")
                values = {**defaults, **values}
            else:
                values = document
            if not isinstance(values, dict):
                raise ValueError("expected a JSON object")
            actions = {action.dest: action for action in parser._actions
                       if action.dest not in ("help", "conf")}
            for name, value in values.items():
                action = actions[name]
                if value is None:
                    continue
                if action.type is Path:
                    if not isinstance(value, str) or not value:
                        raise ValueError(f"{name} must be a nonempty path")
                    values[name] = partial.conf.parent / value
                elif action.type is float:
                    if type(value) not in (float, int):
                        raise ValueError(f"{name} must be numeric")
                elif not isinstance(value, str):
                    raise ValueError(f"{name} must be a string")
            parser.set_defaults(**{key: value for key, value in values.items()
                                   if value is not None})
        except (OSError, ValueError, KeyError) as error:
            parser.error(f"{partial.conf}: {error}")
    args = parser.parse_args()
    for name in required:
        if not getattr(args, name):
            parser.error(f"Missing {name}.")
    return args
