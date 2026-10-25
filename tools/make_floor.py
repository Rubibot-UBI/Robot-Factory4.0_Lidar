#!/usr/bin/env python3
"""Gera a textura do chão (PNG) a partir de mapa_raf4.svg.

O SVG tem 20078 x 14174 unidades = 1700 x 1200 mm a 300 dpi (11.811 un/mm).
Os grupos das warehouses e das máquinas são removidos da textura porque no
Gazebo entram como modelos 3D (meshes ratf_warehouse / ratf_machine); assim
não há linhas duplicadas por baixo das paredes.

Uso:
    pip install cairosvg
    python3 tools/make_floor.py                 # -> src/worlds/textures/floor.png
    python3 tools/make_floor.py --keep-objects  # mantém warehouses/máquinas desenhadas
    python3 tools/make_floor.py --flip-y        # se a textura aparecer espelhada no Gazebo
"""
import argparse
import io
from pathlib import Path

import cairosvg
from lxml import etree
from PIL import Image, ImageOps

ROOT = Path(__file__).resolve().parent.parent
OBJECT_GROUPS = {"Warehouse-Entrada", "Maquina-A", "Maquina-B", "Warehouse-Saída"}

FLOOR_W_MM, FLOOR_H_MM = 1700, 1200


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--svg", default=ROOT / "mapa_raf4.svg")
    ap.add_argument("--out", default=ROOT / "src/worlds/textures/floor.png")
    ap.add_argument("--px-per-mm", type=float, default=2.4,
                    help="resolução (2.4 -> 4080x2880, cabe nos 4096 px de textura)")
    ap.add_argument("--keep-objects", action="store_true")
    ap.add_argument("--flip-y", action="store_true", help="espelha na vertical")
    args = ap.parse_args()

    tree = etree.parse(str(args.svg))
    if not args.keep_objects:
        for g in tree.xpath("//*[@id]"):
            if g.get("id") in OBJECT_GROUPS:
                g.getparent().remove(g)

    w, h = round(FLOOR_W_MM * args.px_per_mm), round(FLOOR_H_MM * args.px_per_mm)
    png = cairosvg.svg2png(bytestring=etree.tostring(tree), output_width=w,
                           output_height=h, background_color="white")
    img = Image.open(io.BytesIO(png)).convert("RGB")
    if args.flip_y:
        img = ImageOps.flip(img)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    img.save(out, optimize=True)
    print(f"{out}  {img.size[0]}x{img.size[1]} px")


if __name__ == "__main__":
    main()
