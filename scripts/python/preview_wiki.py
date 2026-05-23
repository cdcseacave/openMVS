#!/usr/bin/env python3
# Render docs/wiki/*.md pages locally with image URLs rewritten to point at a
# local openMVS_sample checkout, so wiki edits and freshly captured sample
# artifacts can be inspected before either repo is pushed.
#
# Usage:
#   python scripts/python/preview_wiki.py
#   python scripts/python/preview_wiki.py --sample-dir /path/to/openMVS_sample
#   python scripts/python/preview_wiki.py --no-open
#
# Defaults:
#   --wiki-dir    docs/wiki  (relative to the openMVS repo root inferred from this script)
#   --sample-dir  sibling openMVS_sample folder if present; else $OPENMVS_SAMPLE_DIR
#   --out-dir     <tempdir>/openmvs-wiki-preview  (wiped each run)
#
# The rewritten copies live under --out-dir; sample artifacts are copied next
# to the markdown so VS Code's Markdown preview renders the figures without
# needing network access or file:// privileges.

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


IMAGE_GLOBS = ('*.jpg', '*.jpeg', '*.png', '*.gif', '*.svg', '*.webp')

# Two GitHub URL shapes used in the wiki — both stripped to bare basename, so
# the rewritten markdown loads images relative to the file.
GITHUB_URL_PATTERNS = [
    re.compile(r'https?://github\.com/cdcseacave/openMVS_sample/(?:blob|raw)/[^/\s)]+/'),
    re.compile(r'https?://raw\.githubusercontent\.com/cdcseacave/openMVS_sample/[^/\s)]+/'),
    # logo and other docs/assets images hosted on the main repo
    re.compile(r'https?://raw\.githubusercontent\.com/cdcseacave/openMVS/[^/\s)]+/docs/assets/'),
    re.compile(r'https?://github\.com/cdcseacave/openMVS/(?:blob|raw)/[^/\s)]+/docs/assets/'),
]

IMG_REF_RE = re.compile(r'!\[[^\]]*\]\(([^)\s]+)')


def default_sample_dir(repo_root: Path) -> Path | None:
    env = os.environ.get('OPENMVS_SAMPLE_DIR')
    if env:
        return Path(env)
    sibling = repo_root.parent / 'openMVS_sample'
    if sibling.is_dir():
        return sibling
    return None


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    repo_root = script_dir.parent.parent
    default_wiki = repo_root / 'docs' / 'wiki'
    default_assets = repo_root / 'docs' / 'assets'
    default_out = Path(tempfile.gettempdir()) / 'openmvs-wiki-preview'

    p = argparse.ArgumentParser(description='Render docs/wiki locally with sample images inlined.')
    p.add_argument('--wiki-dir', type=Path, default=default_wiki,
                   help=f'Wiki folder to render (default: {default_wiki})')
    p.add_argument('--assets-dir', type=Path, default=default_assets,
                   help=f'Main-repo docs/assets folder providing logo etc. (default: {default_assets})')
    p.add_argument('--sample-dir', type=Path, default=default_sample_dir(repo_root),
                   help='openMVS_sample checkout providing the image assets '
                        '(default: $OPENMVS_SAMPLE_DIR or sibling openMVS_sample folder)')
    p.add_argument('--out-dir', type=Path, default=default_out,
                   help=f'Preview output folder, wiped each run (default: {default_out})')
    p.add_argument('--no-open', action='store_true',
                   help="Do not launch VS Code after rendering")
    return p.parse_args()


def collect_assets(sample_dir: Path) -> list[Path]:
    seen: set[Path] = set()
    assets: list[Path] = []
    for pattern in IMAGE_GLOBS:
        for f in sample_dir.rglob(pattern):
            if f.is_file() and f not in seen:
                seen.add(f)
                assets.append(f)
    return assets


def rewrite_markdown(text: str) -> str:
    for pat in GITHUB_URL_PATTERNS:
        text = pat.sub('', text)
    return text


def main() -> int:
    args = parse_args()

    wiki_dir: Path = args.wiki_dir.resolve()
    if not wiki_dir.is_dir():
        print(f'error: wiki folder not found: {wiki_dir}', file=sys.stderr)
        return 1

    if args.sample_dir is None:
        print('error: no sample folder; pass --sample-dir or set OPENMVS_SAMPLE_DIR',
              file=sys.stderr)
        return 1
    sample_dir: Path = args.sample_dir.resolve()
    if not sample_dir.is_dir():
        print(f'error: sample folder not found: {sample_dir}', file=sys.stderr)
        return 1

    out_dir: Path = args.out_dir.resolve()
    if out_dir.exists():
        shutil.rmtree(out_dir, ignore_errors=True)
    out_dir.mkdir(parents=True, exist_ok=True)

    asset_sources = [sample_dir]
    if args.assets_dir and args.assets_dir.is_dir():
        asset_sources.append(args.assets_dir.resolve())
    assets: list[Path] = []
    seen_names: set[str] = set()
    for src in asset_sources:
        for a in collect_assets(src):
            if a.name in seen_names:
                continue
            seen_names.add(a.name)
            assets.append(a)
            shutil.copy2(a, out_dir / a.name)

    md_files = sorted(p for p in wiki_dir.iterdir() if p.suffix.lower() == '.md')
    for md in md_files:
        rewritten = rewrite_markdown(md.read_text(encoding='utf-8'))
        (out_dir / md.name).write_text(rewritten, encoding='utf-8')

    missing: list[str] = []
    for md in out_dir.glob('*.md'):
        text = md.read_text(encoding='utf-8')
        for ref in IMG_REF_RE.findall(text):
            if ref.startswith(('http://', 'https://')):
                continue
            if not (out_dir / ref).exists():
                missing.append(f'{md.name} -> {ref}')

    print(f'Wrote rewritten wiki to: {out_dir}')
    print(f'Sample assets copied:    {len(assets)}')
    if missing:
        print('Image references with no matching local file:', file=sys.stderr)
        for m in missing:
            print(f'  {m}', file=sys.stderr)
    else:
        print('All local image references resolve.')

    if not args.no_open:
        code = shutil.which('code') or shutil.which('code.cmd')
        usage_md = out_dir / 'Usage.md'
        if code:
            subprocess.run([code, str(out_dir), str(usage_md), '--goto', str(usage_md)],
                           check=False)
            print("Opened in VS Code. Use Ctrl+Shift+V to toggle the markdown preview.")
        else:
            print(f"VS Code ('code') not in PATH; open {out_dir} manually or pass --no-open.")

    return 0


if __name__ == '__main__':
    sys.exit(main())
