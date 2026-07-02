"""Sphinx / MyST heading slug helpers for Autonomy docs.

Generates stable anchors like ``410-故障排查`` from headings such as
``### 4.10 故障排查``, matching cross-reference links in the docs.
"""

from __future__ import annotations

import re


def section_heading_slug(title: str) -> str:
    """Convert a heading title to a documentation anchor slug."""
    title = title.strip()
    match = re.match(r"^(\d+(?:\.\d+)*)\s+(.+)$", title)
    if match:
        number = match.group(1).replace(".", "")
        rest = match.group(2).strip().lower()
        rest = re.sub(r"[^\w\s\u4e00-\u9fff-]", "", rest, flags=re.UNICODE)
        rest = re.sub(r"\s+", "-", rest)
        rest = re.sub(r"-+", "-", rest).strip("-")
        return f"{number}-{rest}" if rest else number

    slug = title.lower()
    slug = re.sub(r"[^\w\s\u4e00-\u9fff-]", "", slug, flags=re.UNICODE)
    slug = re.sub(r"\s+", "-", slug)
    return re.sub(r"-+", "-", slug).strip("-")
