#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output_dir="${1:-$repo_root/dist/docs}"

mkdir -p "$output_dir"

python3 "$repo_root/scripts/prepare_markdown_for_pdf.py" \
  "$repo_root/README.md" \
  "$output_dir/README.en.pdf.md"
python3 "$repo_root/scripts/prepare_markdown_for_pdf.py" \
  "$repo_root/README.ja.md" \
  "$output_dir/README.ja.pdf.md"

common_args=(
  --from
  gfm+tex_math_dollars
  --pdf-engine=xelatex
  --toc
  -V
  geometry:margin=20mm
  -V
  colorlinks=true
  -V
  linkcolor:NavyBlue
  -V
  urlcolor:NavyBlue
  -V
  mainfont=Noto Serif
  -V
  sansfont=Noto Sans
  -V
  monofont=DejaVu Sans Mono
  -V
  CJKmainfont=Noto Serif CJK JP
)

pandoc \
  "$output_dir/README.en.pdf.md" \
  -o "$output_dir/es_sim-readme-en.pdf" \
  "${common_args[@]}" \
  -M title="ES README (English)" \
  -M lang=en-US

pandoc \
  "$output_dir/README.ja.pdf.md" \
  -o "$output_dir/es_sim-readme-ja.pdf" \
  "${common_args[@]}" \
  -M title="ES README (Japanese)" \
  -M lang=ja-JP
