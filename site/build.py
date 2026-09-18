"""Assemble the docs site from the repository's own markdown.

The README, results files and docs/ are the single source of truth; this script copies them
into site/docs/ and rewrites relative links so they resolve on the site: files that become
pages point at those pages, images under docs/assets are copied alongside, and everything else
(scripts, raw data directories, LICENSE) links to the file on GitHub.

Run from the repository root:  python site/build.py && mkdocs build -f site/mkdocs.yml
"""

from __future__ import annotations

import os
import re
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "site" / "docs"
REPO_URL = "https://github.com/TanayK07/ros2_pulse"

# repo file -> site page
PAGES = {
    "README.md": "index.md",
    "tools/pulse-top/README.md": "pulse-top.md",
    "bench/RESULTS.md": "benchmarks.md",
    "bench/README.md": "bench-harness.md",
    "test/orin/RESULTS.md": "orin.md",
    "test/orin/ORIN_RUNBOOK.md": "orin-runbook.md",
    "docs/ALTERNATIVES.md": "alternatives.md",
    "docs/DESIGN.md": "design.md",
    "docs/KNOWN_ISSUES.md": "known-issues.md",
    "docs/ROADMAP.md": "roadmap.md",
    "video/README.md": "video.md",
    "CHANGELOG.md": "changelog.md",
    "CONTRIBUTING.md": "contributing.md",
    "SECURITY.md": "security.md",
    # blog posts keep their date-prefixed name; the page lives in a subdirectory, so links and
    # images in it are rewritten relative to that directory (see rewrite())
    "docs/blog/2026-09-18-watching-a-ros2-topic-changes-it.md":
        "blog/2026-09-18-watching-a-ros2-topic-changes-it.md",
}
ASSETS = "docs/assets"

# plain [text](target), image ![alt](target) and badge-style [![alt](img)](target) links
LINK = re.compile(r"((?:!?\[[^\]]*\]|(?<=\))\])\()([^)\s]+)(\))")


def github_url(path: str) -> str:
    kind = "tree" if (ROOT / path).is_dir() else "blob"
    return f"{REPO_URL}/{kind}/main/{path}"


def rewrite(src: str, page: str, text: str) -> str:
    src_dir = Path(src).parent
    page_dir = Path(page).parent

    def site_rel(site_path: str) -> str:
        # site-relative target -> relative to the page's own directory (pages under blog/ need ../)
        return Path(os.path.relpath(site_path, page_dir)).as_posix()

    def repl(m: re.Match) -> str:
        head, target, tail = m.groups()
        if re.match(r"^[a-z]+:", target) or target.startswith("#"):
            return m.group(0)
        path, _, anchor = target.partition("#")
        anchor = f"#{anchor}" if anchor else ""
        # repo-relative path with ./ and ../ collapsed (pathlib keeps .. segments, normpath folds them)
        repo_path = os.path.normpath(os.path.join(src_dir.as_posix(), path)) if path else src
        if repo_path in PAGES:
            return f"{head}{site_rel(PAGES[repo_path])}{anchor}{tail}"
        if repo_path.startswith(ASSETS + "/"):
            return f"{head}{site_rel('assets/' + repo_path[len(ASSETS) + 1:])}{tail}"
        if not (ROOT / repo_path).exists():
            return m.group(0)
        return f"{head}{github_url(repo_path)}{anchor}{tail}"

    return LINK.sub(repl, text)


def main() -> None:
    if OUT.exists():
        shutil.rmtree(OUT)
    OUT.mkdir(parents=True)
    shutil.copytree(ROOT / ASSETS, OUT / "assets")
    for src, page in PAGES.items():
        text = (ROOT / src).read_text(encoding="utf-8")
        (OUT / page).parent.mkdir(parents=True, exist_ok=True)
        (OUT / page).write_text(rewrite(src, page, text), encoding="utf-8")
    print(f"site/docs: {len(PAGES)} pages, assets from {ASSETS}")


if __name__ == "__main__":
    main()
