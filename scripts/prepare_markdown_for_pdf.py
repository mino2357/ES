#!/usr/bin/env python3
"""Prepare GitHub-flavored README markdown for Pandoc PDF conversion.

GitHub renders fenced ```math blocks, while Pandoc's GFM reader does not treat
those fences as math. We convert fenced math blocks into raw TeX display-math
blocks (\\[...\\]) so Pandoc keeps equations intact and does not misread lines
such as `=` inside equations as Markdown setext headings.
"""

from __future__ import annotations

import pathlib
import sys


def convert_math_fences(text: str) -> str:
    lines = text.splitlines()
    output: list[str] = []
    in_math_block = False

    for line in lines:
        stripped = line.strip()
        if not in_math_block and stripped == "```math":
            output.append(r"\[")
            in_math_block = True
            continue
        if in_math_block and stripped == "```":
            output.append(r"\]")
            in_math_block = False
            continue
        output.append(line)

    if in_math_block:
        raise ValueError("unterminated ```math block")

    return "\n".join(output) + "\n"


def main() -> int:
    if len(sys.argv) != 3:
        print(
            "usage: prepare_markdown_for_pdf.py <input.md> <output.md>",
            file=sys.stderr,
        )
        return 2

    input_path = pathlib.Path(sys.argv[1])
    output_path = pathlib.Path(sys.argv[2])

    converted = convert_math_fences(input_path.read_text(encoding="utf-8"))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(converted, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
