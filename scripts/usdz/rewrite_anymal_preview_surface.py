#!/usr/bin/env python3
from __future__ import annotations

import re
import sys
from pathlib import Path


MATERIAL_START_RE = re.compile(r'^(\s*)def Material "([^"]+)"')
DIFFUSE_COLOR_RE = re.compile(r'color3f inputs:diffuse_color_constant = (\([^)]+\))')
DIFFUSE_TEXTURE_RE = re.compile(r'asset inputs:diffuse_texture = @([^@]+)@')
EMISSIVE_COLOR_RE = re.compile(r'color3f inputs:emissive_color = (\([^)]+\))')


def rewrite_material_block(block_lines: list[str]) -> list[str]:
    match = MATERIAL_START_RE.match(block_lines[0])
    if match is None:
        return block_lines

    indent, material_name = match.groups()
    diffuse_color = "(1, 1, 1)"
    emissive_color = "(0, 0, 0)"
    diffuse_texture: str | None = None

    for line in block_lines:
        if color_match := DIFFUSE_COLOR_RE.search(line):
            diffuse_color = color_match.group(1)
        if texture_match := DIFFUSE_TEXTURE_RE.search(line):
            diffuse_texture = texture_match.group(1)
        if emissive_match := EMISSIVE_COLOR_RE.search(line):
            emissive_color = emissive_match.group(1)

    output: list[str] = [
        f'{indent}def Material "{material_name}"',
        f"{indent}{{",
        f"{indent}    token outputs:surface.connect = <PreviewSurface.outputs:surface>",
        "",
        f'{indent}    def Shader "PreviewSurface"',
        f"{indent}    {{",
        f'{indent}        uniform token info:id = "UsdPreviewSurface"',
    ]

    if diffuse_texture is None:
        output.append(f"{indent}        color3f inputs:diffuseColor = {diffuse_color}")
    else:
        output.append(f"{indent}        color3f inputs:diffuseColor.connect = <diffuseTexture.outputs:rgb>")

    output.extend(
        [
            f"{indent}        color3f inputs:emissiveColor = {emissive_color}",
            f"{indent}        token outputs:surface",
            f"{indent}    }}",
        ]
    )

    if diffuse_texture is not None:
        output.extend(
            [
                "",
                f'{indent}    def Shader "diffuseTexture"',
                f"{indent}    {{",
                f'{indent}        uniform token info:id = "UsdUVTexture"',
                f"{indent}        asset inputs:file = @{diffuse_texture}@",
                f"{indent}        float2 inputs:st.connect = <stReader.outputs:result>",
                f'{indent}        token inputs:sourceColorSpace = "sRGB"',
                f"{indent}        float3 outputs:rgb",
                f"{indent}    }}",
                "",
                f'{indent}    def Shader "stReader"',
                f"{indent}    {{",
                f'{indent}        uniform token info:id = "UsdPrimvarReader_float2"',
                f'{indent}        token inputs:varname = "st"',
                f"{indent}        float2 outputs:result",
                f"{indent}    }}",
            ]
        )

    output.append(f"{indent}}}")
    return [line + "\n" for line in output]


def rewrite_preview_surfaces(text: str) -> str:
    lines = text.splitlines(keepends=True)
    rewritten: list[str] = []
    index = 0

    while index < len(lines):
        start_match = MATERIAL_START_RE.match(lines[index])
        if start_match is None:
            rewritten.append(lines[index])
            index += 1
            continue

        block_lines = [lines[index]]
        brace_depth = lines[index].count("{") - lines[index].count("}")
        index += 1

        while index < len(lines):
            line = lines[index]
            block_lines.append(line)
            brace_depth += line.count("{") - line.count("}")
            index += 1
            if brace_depth == 0:
                break

        rewritten.extend(rewrite_material_block(block_lines))

    return "".join(rewritten)


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: rewrite_anymal_preview_surface.py <input-usda> <output-usda>", file=sys.stderr)
        return 1

    input_path = Path(sys.argv[1])
    output_path = Path(sys.argv[2])

    rewritten_text = rewrite_preview_surfaces(input_path.read_text())
    output_path.write_text(rewritten_text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
