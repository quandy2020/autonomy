"""Sphinx / MyST heading slug helpers for Autonomy docs.

Generates stable anchors like ``410-故障排查`` from headings such as
``### 4.10 故障排查``, matching cross-reference links in the docs.
"""

from __future__ import annotations

import re

from docutils import nodes
from sphinx.transforms.post_transforms import SphinxPostTransform


def section_heading_slug(title: str) -> str:
    """Convert a heading title to a documentation anchor slug."""
    title = title.strip()
    # H1 形如 ``1. 架构 Architecture``：归一化为 ``1 架构 Architecture``
    title = re.sub(r"^(\d+(?:\.\d+)*)\.\s+", r"\1 ", title)
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


def sync_myst_heading_ids(app, doctree, docname=None) -> None:
    """将 MyST ``slug`` 写入 section ``ids``，供 HTML 锚点与 sidebar TOC 使用。"""
    for node in doctree.traverse(nodes.section):
        slug = node.get("slug")
        if slug:
            node["ids"] = [slug]


class MystSlugHtmlIdTransform(SphinxPostTransform):
    """Post-read：再次同步 ``ids``，确保 writer 与 local TOC 使用语义化锚点。"""

    default_priority = 400

    def run(self) -> None:
        sync_myst_heading_ids(self.app, self.document)


def register_heading_id_transform(app) -> None:
    app.connect("doctree-resolved", sync_myst_heading_ids)
    app.add_post_transform(MystSlugHtmlIdTransform)


def setup(app):
    register_heading_id_transform(app)
    return {"version": "1", "parallel_read_safe": True, "parallel_write_safe": True}
