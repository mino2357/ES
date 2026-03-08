#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output_dir="${1:-$repo_root/dist/docs}"

mkdir -p "$output_dir"

for required_cmd in python3 pandoc; do
  if ! command -v "$required_cmd" >/dev/null 2>&1; then
    echo "error: required command '$required_cmd' is not available" >&2
    exit 127
  fi
done

if ! command -v xelatex >/dev/null 2>&1; then
  echo "error: required TeX engine 'xelatex' is not available" >&2
  exit 127
fi

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
  "mainfont=Noto Serif"
  -V
  "sansfont=Noto Sans"
  -V
  "monofont=DejaVu Sans Mono"
  -V
  "CJKmainfont=Noto Serif CJK JP"
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

for output_pdf in "$output_dir/es_sim-readme-en.pdf" "$output_dir/es_sim-readme-ja.pdf"; do
  if [[ ! -s "$output_pdf" ]]; then
    echo "error: expected PDF output '$output_pdf' was not created or is empty" >&2
    exit 1
  fi
done
