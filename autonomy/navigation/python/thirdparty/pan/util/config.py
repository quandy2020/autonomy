"""Dataclass configs double as command-line interfaces: one flag per field declared with `arg`."""
from __future__ import annotations

import argparse
import dataclasses
from typing import Optional, Sequence


def arg(default, help: str, *, flag: Optional[str] = None, choices: Optional[Sequence] = None):
    """A config field with a command-line flag; `flag` overrides the derived `--field-name`."""
    return dataclasses.field(default=default, metadata=dict(help=help, flag=flag, choices=choices))


def _flags(cls, prefix: str):
    """(field, flag, argparse dest) for the fields of `cls` declared with `arg`."""
    for f in dataclasses.fields(cls):
        if "help" in f.metadata:
            flag = f.metadata["flag"] or "--" + (f"{prefix}-" if prefix else "") + f.name.replace("_", "-")
            yield f, flag, flag[2:].replace("-", "_")


def add_config_args(parser: argparse.ArgumentParser, cls, prefix: str = "", defaults=None) -> None:
    """Register the flags of `cls` (bools as --x / --no-x), defaulting to the values of the
    instance `defaults`, else to the class defaults."""
    group = parser.add_argument_group(cls.__name__)
    for f, flag, _ in _flags(cls, prefix):
        kw = dict(default=f.default if defaults is None else getattr(defaults, f.name),
                  help=f.metadata["help"], choices=f.metadata["choices"])
        if isinstance(f.default, bool):
            kw["action"] = argparse.BooleanOptionalAction      # appends the default itself
        else:
            kw["type"] = type(f.default)
            kw["help"] += " (default: %(default)s)"
        group.add_argument(flag, **kw)


def config_from_args(cls, args: argparse.Namespace, prefix: str = "", **values):
    """`cls` from the flags `add_config_args` registered; `values` set the remaining fields."""
    return cls(**{**{f.name: getattr(args, dest) for f, _, dest in _flags(cls, prefix)}, **values})
