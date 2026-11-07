#!/usr/bin/env python3
"""
Gerador de objetos aleatorios FORA da pista, para testar o AMCL/LIDAR.

Le o mundo base (factory.world: pista + warehouses + maquinas + 6 pilares das regras) e acrescenta
pilares extra, pessoas, cadeiras, mesas, quadrados (caixas), cones e baldes de lixo, todos fora do
chao da competicao (1.7 x 1.2 m). Cada geracao e aleatoria; com a mesma seed o mapa e identico.

So usa a biblioteca padrao (corre dentro do sim.launch.py). Uso isolado:

    python3 world_generator.py                       # seed aleatoria (e impressa)
    python3 world_generator.py --seed 42             # reproduzivel
    python3 world_generator.py --seed 42 --counts pilar=3,cone=0,pessoa=2
    python3 world_generator.py --seed 42 --preview mapa.png     # desenho 2D (precisa de matplotlib)
    python3 world_generator.py --check-meshes ../description/meshes   # confere o CATALOG com os .stl (precisa de trimesh)

Referencial = o das regras: origem no centro da pista, x para a direita, y para cima, metros.
"""
import argparse
import json
import math
import os
import random
import re
import sys
import tempfile

# --- pista (regras, seccao 3): chao 1.7 x 1.2 m, centro na origem ---------------------------------
ARENA_HALF_X = 0.85
ARENA_HALF_Y = 0.60

MESH_URI = "model://my_bot/description/meshes/{}"

# --- catalogo de modelos ---------------------------------------------------------------------------
# bmin/bmax: bounding box MEDIDA na mesh, nas unidades originais do .stl (o --check-meshes confere).
# As meshes vem todas com z para cima, mas cada uma tem a sua unidade e a sua origem:
#   - cubo e mesa tem a origem fora do centro; o cone tem a base em z=-6; pessoa/balde/mesa estao em
#     unidades arbitrarias (~5 u de altura) e a cadeira/cone em cm.
# Por isso cada modelo e posto com: escala uniforme = height / (altura da mesh), centro da bbox em xy
# na origem do modelo, e o ponto mais baixo em z=0 (assenta no chao).
# height: altura real alvo em metros (a escala sai daqui; mudar aqui muda o tamanho).
# front_yaw: rotacao (rad) que poe a "frente" da mesh a apontar para +x. Pessoa e cadeira tem a frente
#   em -Y no .stl (medido: pes/nariz da pessoa e assento da cadeira saem para -Y) -> +90 graus.
# count: intervalo (min, max) de quantos objetos deste tipo entram em cada geracao.
# clearance/band (opcionais): folga minima ao chao da pista e largura da faixa onde o tipo pode aparecer;
#   sem estes campos usa os valores globais (CLEARANCE/BAND, ou --clearance/--band). Medidos no bordo real da pegada.
CATALOG = {
    "pilar": dict(      # igual aos 6 pilares das regras: Ø100 mm x 210 mm (stl em mm)
        mesh="pilar.stl", bmin=(-50.0, -50.0, 0.0), bmax=(50.0, 50.0, 210.0),
        height=0.21, front_yaw=0.0, color=(0.90, 0.90, 0.90), count=(2, 5)),
    "pessoa": dict(     # em pe, bracos ligeiramente afastados (largura ~1.07 m)
        mesh="person.stl", bmin=(-1.7584, -0.5763, 0.016), bmax=(1.7584, 0.4181, 5.5894),
        height=1.70, front_yaw=math.pi / 2, color=(0.85, 0.55, 0.20), count=(4, 8),
        clearance=0.05, band=0.50),   # encostadas a pista: retangulo da pegada a 5-55 cm do chao da pista
    "cadeira": dict(    # cadeira de plastico com bracos
        mesh="lawnchair.stl", bmin=(-23.773, -29.8122, 0.0113), bmax=(23.9253, 19.1381, 72.6463),
        height=0.80, front_yaw=math.pi / 2, color=(0.20, 0.45, 0.85), count=(1, 4)),
    "mesa": dict(       # tampo fino + 4 pernas, quadrada (~0.83 m de lado)
        mesh="table.stl", bmin=(-13.0, -18.0, 0.0), bmax=(8.0, 3.0, 19.0),
        height=0.75, front_yaw=0.0, color=(0.55, 0.38, 0.22), count=(1, 2)),
    "quadrado": dict(   # caixa/cubo
        mesh="cube.stl", bmin=(-31.0, 4.0, 0.0), bmax=(-11.0, 24.0, 20.0),
        height=0.30, front_yaw=0.0, color=(0.70, 0.70, 0.30), count=(1, 4)),
    "cone": dict(       # cone de sinalizacao com base quadrada
        mesh="traffic-cone.stl", bmin=(-27.5, -27.5, -6.0), bmax=(27.5, 27.5, 65.0),
        height=0.50, front_yaw=0.0, color=(0.95, 0.40, 0.05), count=(2, 5)),
    "balde": dict(      # balde/caixote do lixo cilindrico (Ø ~0.47 m)
        mesh="trash-can.stl", bmin=(-2.1671, -2.1671, -0.001), bmax=(2.1671, 2.1671, 5.0395),
        height=0.55, front_yaw=0.0, color=(0.25, 0.30, 0.25), count=(1, 3)),
}

# defaults de colocacao (metros)
CLEARANCE = 0.20   # folga minima entre a borda do objeto e o chao da pista
BAND = 2.00        # largura da faixa em volta da pista onde os objetos podem aparecer (LIDAR alcanca 8 m)
GAP = 0.15         # folga minima entre objetos
MAX_TRIES = 500


def _scale(spec):
    return spec["height"] / (spec["bmax"][2] - spec["bmin"][2])


def _size(spec):
    """(largura, profundidade) da pegada em metros, ja escalada (antes de rodar)."""
    s = _scale(spec)
    return ((spec["bmax"][0] - spec["bmin"][0]) * s, (spec["bmax"][1] - spec["bmin"][1]) * s)


def _radius(spec):
    """Raio do circulo que contem a pegada, qualquer que seja a rotacao."""
    w, d = _size(spec)
    return 0.5 * math.hypot(w, d)


def _rot(angle, a, b):
    c, s = math.cos(angle), math.sin(angle)
    return a * c - b * s, a * s + b * c


def visual_pose(spec):
    """Pose da mesh dentro do modelo: centro da bbox (xy) na origem, pe em z=0, frente para +x."""
    s = _scale(spec)
    cx = 0.5 * (spec["bmin"][0] + spec["bmax"][0]) * s
    cy = 0.5 * (spec["bmin"][1] + spec["bmax"][1]) * s
    f = spec["front_yaw"]
    tx, ty = _rot(f, -cx, -cy)
    tz = -spec["bmin"][2] * s
    return tx, ty, tz, f


def parse_counts(text):
    """'pilar=3,cone=0' -> {'pilar': 3, 'cone': 0}"""
    out = {}
    for item in filter(None, (t.strip() for t in (text or "").split(","))):
        key, _, val = item.partition("=")
        key = key.strip()
        if key not in CATALOG or not val.strip().lstrip("-").isdigit() or int(val) < 0:
            raise ValueError("contagem invalida '%s' (tipos: %s)" % (item, ", ".join(CATALOG)))
        out[key] = int(val)
    return out


def resolve_seed(seed):
    """None / '' / 'random' -> seed nova aleatoria; senao inteiro."""
    if seed is None or str(seed).strip().lower() in ("", "random"):
        return random.SystemRandom().randrange(1, 2 ** 31)
    return int(seed)


def reserved_circles(world_text):
    """Pilares ja existentes no mundo base (nome 'pilar*'): (x, y, raio). Le as poses do proprio ficheiro."""
    out = []
    for m in re.finditer(r'<model\s+name="pilar[^"]*"', world_text):
        pm = re.search(r"<pose>\s*([^<]+?)\s*</pose>", world_text[m.end():])
        if pm:
            v = [float(t) for t in pm.group(1).split()]
            out.append((v[0], v[1], 0.05))
    return out


def footprint(spec, x, y, yaw):
    """Pegada (retangulo da bbox) no mundo: 4 cantos, rodada por yaw + front_yaw, centrada em (x, y)."""
    w, d = _size(spec)
    a = yaw + spec["front_yaw"]
    return [(x + _rot(a, px, py)[0], y + _rot(a, px, py)[1])
            for px, py in ((-w / 2, -d / 2), (w / 2, -d / 2), (w / 2, d / 2), (-w / 2, d / 2))]


ARENA_POLY = [(-ARENA_HALF_X, -ARENA_HALF_Y), (ARENA_HALF_X, -ARENA_HALF_Y),
              (ARENA_HALF_X, ARENA_HALF_Y), (-ARENA_HALF_X, ARENA_HALF_Y)]


def _edges(P):
    return [(P[i], P[(i + 1) % len(P)]) for i in range(len(P))]


def _pt_seg(p, a, b):
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    dx, dy = bx - ax, by - ay
    t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


def _overlap(P, Q):
    """Poligonos convexos sobrepostos? (eixo separador)"""
    for poly in (P, Q):
        for a, b in _edges(poly):
            nx, ny = b[1] - a[1], a[0] - b[0]
            pp = [nx * v[0] + ny * v[1] for v in P]
            qq = [nx * v[0] + ny * v[1] for v in Q]
            if max(pp) < min(qq) or max(qq) < min(pp):
                return False
    return True


def poly_dist(P, Q):
    """Distancia minima entre dois poligonos convexos (0 se se sobrepoem)."""
    if _overlap(P, Q):
        return 0.0
    return min([_pt_seg(v, a, b) for v in P for a, b in _edges(Q)] +
               [_pt_seg(v, a, b) for v in Q for a, b in _edges(P)])


def _dist_circle(P, c):
    """Distancia do poligono ao disco c=(x, y, r) (0 se tocam)."""
    inside = all((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) >= 0 for a, b in _edges(P))
    return 0.0 if inside else max(0.0, min(_pt_seg(c, a, b) for a, b in _edges(P)) - c[2])


def place_objects(seed, counts=None, reserved=(), clearance=CLEARANCE, band=BAND, gap=GAP):
    """Escolhe tipos, posicoes e rotacoes. Determinista para a mesma seed. Devolve lista de dicts."""
    rng = random.Random(seed)
    # quantos de cada tipo (sorteado sempre, para a seed dar sempre o mesmo; depois aplica overrides)
    wanted = {k: rng.randint(*spec["count"]) for k, spec in CATALOG.items()}
    wanted.update(counts or {})

    # primeiro os tipos com faixa mais estreita (pessoas, encostadas a pista), depois os maiores
    order = sorted(CATALOG, key=lambda k: (CATALOG[k].get("band", band), -_radius(CATALOG[k])))
    polys = []   # pegadas ja colocadas
    objects = []
    for kind in order:
        spec = CATALOG[kind]
        r = _radius(spec)
        clr, bnd = spec.get("clearance", clearance), spec.get("band", band)
        reach = ARENA_HALF_X + clr + bnd + r   # meia-largura do retangulo de sorteio (circulo envolvente)
        reach_y = ARENA_HALF_Y + clr + bnd + r
        for n in range(1, wanted[kind] + 1):
            for _ in range(MAX_TRIES):
                x = round(rng.uniform(-reach, reach), 4)
                y = round(rng.uniform(-reach_y, reach_y), 4)
                yaw = round(rng.uniform(-math.pi, math.pi), 4)   # direcao para onde o objeto "olha"
                fp = footprint(spec, x, y, yaw)
                d = poly_dist(fp, ARENA_POLY)
                if d < clr or d > clr + bnd:   # fora da pista, dentro da faixa
                    continue
                if any(_dist_circle(fp, c) < gap for c in reserved):
                    continue
                if any(poly_dist(fp, q) < gap for q in polys):
                    continue
                polys.append(fp)
                objects.append(dict(name="extra_%s_%d" % (kind, n), type=kind, x=x, y=y, yaw=yaw))
                break
            else:
                print("[world_generator] aviso: nao coube %s #%d (%d tentativas)" % (kind, n, MAX_TRIES),
                      file=sys.stderr)
    return objects


def model_xml(obj):
    spec = CATALOG[obj["type"]]
    s = _scale(spec)
    tx, ty, tz, f = visual_pose(spec)
    pose = "%.5f %.5f %.5f 0 0 %.5f" % (tx, ty, tz, f)
    geom = ("<geometry><mesh><uri>%s</uri><scale>%.6f %.6f %.6f</scale></mesh></geometry>"
            % (MESH_URI.format(spec["mesh"]), s, s, s))
    r, g, b = spec["color"]
    return """    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} 0 0 0 {yaw:.4f}</pose>
      <link name="link">
        <collision name="collision">
          <pose>{pose}</pose>
          {geom}
        </collision>
        <visual name="visual">
          <pose>{pose}</pose>
          {geom}
          <material>
            <ambient>{r} {g} {b} 1</ambient>
            <diffuse>{r} {g} {b} 1</diffuse>
            <specular>0 0 0 1</specular>
          </material>
        </visual>
      </link>
    </model>
""".format(name=obj["name"], x=obj["x"], y=obj["y"], yaw=obj["yaw"], pose=pose, geom=geom, r=r, g=g, b=b)


def generate_world(seed=None, base=None, out=None, counts=None, clearance=CLEARANCE, band=BAND, gap=GAP):
    """Gera o .world (e um .json com a verdade-terreno). Devolve dict(seed, world, manifest, objects)."""
    seed = resolve_seed(seed)
    base = base or os.path.join(os.path.dirname(os.path.abspath(__file__)), "factory.world")
    with open(base, encoding="utf-8") as fh:
        text = fh.read()
    objects = place_objects(seed, counts, reserved_circles(text), clearance, band, gap)

    block = ["\n    <!-- ================= OBJETOS ALEATORIOS FORA DA PISTA (seed=%d) =================\n"
             "         Gerado por worlds/world_generator.py. Nao editar: mudar a seed muda o mapa. -->\n" % seed]
    block += [model_xml(o) for o in objects]
    i = text.rindex("</world>")
    world_text = text[:i] + "".join(block) + text[i:]

    out = out or os.path.join(tempfile.gettempdir(), "my_bot_world_%d.world" % seed)
    with open(out, "w", encoding="utf-8") as fh:
        fh.write(world_text)

    manifest = os.path.splitext(out)[0] + ".json"
    with open(manifest, "w", encoding="utf-8") as fh:
        json.dump(dict(seed=seed, arena=dict(half_x=ARENA_HALF_X, half_y=ARENA_HALF_Y),
                       clearance=clearance, band=band,
                       objects=[dict(o, height=CATALOG[o["type"]]["height"],
                                     radius=round(_radius(CATALOG[o["type"]]), 4)) for o in objects]),
                  fh, indent=2)
    return dict(seed=seed, world=out, manifest=manifest, objects=objects)


def preview(objects, reserved, path, seed):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon, Rectangle

    fig, ax = plt.subplots(figsize=(9, 8))
    ax.add_patch(Rectangle((-ARENA_HALF_X, -ARENA_HALF_Y), 2 * ARENA_HALF_X, 2 * ARENA_HALF_Y,
                           fc="#f4f4f4", ec="k", lw=1.5, label="pista"))
    for x, y, r in reserved:
        ax.add_patch(Circle((x, y), r, fc="0.5", ec="k"))
    seen = set()
    for o in objects:
        spec = CATALOG[o["type"]]
        w, d = _size(spec)
        a = o["yaw"] + spec["front_yaw"]
        pts = [(o["x"] + _rot(a, px, py)[0], o["y"] + _rot(a, px, py)[1])
               for px, py in ((-w / 2, -d / 2), (w / 2, -d / 2), (w / 2, d / 2), (-w / 2, d / 2))]
        ax.add_patch(Polygon(pts, fc=spec["color"], ec="k", alpha=0.85,
                             label=None if o["type"] in seen else o["type"]))
        seen.add(o["type"])
        ax.add_patch(Circle((o["x"], o["y"]), _radius(spec), fill=False, ec=spec["color"], ls=":", lw=0.8))
        ax.annotate("", xy=(o["x"] + 0.18 * math.cos(o["yaw"]), o["y"] + 0.18 * math.sin(o["yaw"])),
                    xytext=(o["x"], o["y"]), arrowprops=dict(arrowstyle="->", lw=1))
    lim_x, lim_y = ARENA_HALF_X + CLEARANCE + BAND + 0.7, ARENA_HALF_Y + CLEARANCE + BAND + 0.7
    ax.set_xlim(-lim_x, lim_x)
    ax.set_ylim(-lim_y, lim_y)
    ax.set_aspect("equal")
    ax.grid(True, lw=0.3)
    ax.set_title("seed=%d  (%d objetos; seta = para onde olha; cinzento = pilares das regras)" % (seed, len(objects)))
    ax.legend(loc="upper right", fontsize=8)
    fig.savefig(path, dpi=90, bbox_inches="tight")


def check_meshes(mesh_dir):
    """Confere o CATALOG com os .stl (precisa de trimesh). Devolve lista de problemas."""
    import trimesh
    bad = []
    for kind, spec in CATALOG.items():
        b = trimesh.load(os.path.join(mesh_dir, spec["mesh"])).bounds
        for got, want, what in ((b[0], spec["bmin"], "bmin"), (b[1], spec["bmax"], "bmax")):
            for g, w in zip(got, want):
                if abs(g - w) > 1e-3 * max(1.0, abs(w)):
                    bad.append("%s %s: mesh=%s catalogo=%s" % (kind, what, [round(float(v), 4) for v in got], list(want)))
                    break
    return bad


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", default=None, help="inteiro; omitido = aleatoria (impressa no fim)")
    ap.add_argument("--base", default=None, help="mundo base (por omissao factory.world ao lado deste script)")
    ap.add_argument("--out", default=None, help="ficheiro .world de saida (por omissao na pasta temporaria)")
    ap.add_argument("--counts", default="", help="forcar quantidades, ex.: pilar=3,cone=0 (tipos: %s)" % ", ".join(CATALOG))
    ap.add_argument("--clearance", type=float, default=CLEARANCE, help="folga objeto-pista (m) para os tipos sem valor proprio (a pessoa tem o seu)")
    ap.add_argument("--band", type=float, default=BAND, help="largura da faixa de colocacao (m) para os tipos sem valor proprio")
    ap.add_argument("--preview", default=None, help="gravar desenho 2D (png)")
    ap.add_argument("--check-meshes", metavar="DIR", default=None, help="so conferir o CATALOG com os .stl de DIR")
    a = ap.parse_args()

    if a.check_meshes:
        bad = check_meshes(a.check_meshes)
        print("\n".join(bad) if bad else "CATALOG coincide com as meshes.")
        sys.exit(1 if bad else 0)

    try:
        counts = parse_counts(a.counts)
        res = generate_world(a.seed, a.base, a.out, counts, a.clearance, a.band)
    except ValueError as e:
        ap.error(str(e))
    print("seed=%d  objetos=%d  world=%s  manifest=%s" % (res["seed"], len(res["objects"]), res["world"], res["manifest"]))
    if a.preview:
        with open(a.base or os.path.join(os.path.dirname(os.path.abspath(__file__)), "factory.world"), encoding="utf-8") as fh:
            preview(res["objects"], reserved_circles(fh.read()), a.preview, res["seed"])
        print("preview=%s" % a.preview)


if __name__ == "__main__":
    main()
