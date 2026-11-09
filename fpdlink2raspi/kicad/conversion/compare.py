"""Compare KiCad-exported Gerbers against the original Altium Gerbers."""
import sys, os, glob
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, here); sys.path.insert(0, os.path.join(here, '..', 'pylib'))
import gerber, model
from shapely import affinity
from shapely.ops import unary_union
SRC, KG, OUTP = sys.argv[1], sys.argv[2], sys.argv[3]
pairs = [('Copper_Signal_Top', 'L1-Top.gtl'), ('Copper_Signal_1', 'L2-Sig.g1'), ('Copper_Signal_2', 'L3-Sig.g2'),
         ('Copper_Signal_Bot', 'L4-Bottom.gbl'), ('Soldermask_Top', 'F_Mask.gts'), ('Soldermask_Bot', 'B_Mask.gbs'),
         ('Paste_Top', 'F_Paste.gtp'), ('Paste_Bot', 'B_Paste.gbp')]
def img(path, shift):
    g = gerber.parse(path)
    im = model.render(g['objs'], g['apertures'])
    return affinity.translate(im, *shift) if shift else im
results = []
from shapely.geometry import Point
holes = []
for a in ('PTH_Drill', 'NPTH_Drill'):
    g = gerber.parse(f'{SRC}/GerberX2/fpdlink2raspi_{a}.gbr')
    holes += [Point(o['x'], o['y']).buffer(g['apertures'][o['ap']]['d'] / 2, 32) for o in g['objs'] if o['kind'] == 'flash']
holes = unary_union(holes)
for a, k in pairs:
    A = img(f'{SRC}/GerberX2/fpdlink2raspi_{a}.gbr', None)
    B = img(glob.glob(f'{KG}/*-{k}')[0], (-100, 156))
    if a.startswith('Copper'):   # copper inside drilled holes does not exist on the real board
        A, B = A.difference(holes), B.difference(holes)
    d = A.symmetric_difference(B)
    sig = d.buffer(-0.005).buffer(0.005)   # ignore < 10 um slivers (arc tessellation, rounding)
    results.append((a, A.area, B.area, d.area, sig.area))
    print(f'{a:20s} altium={A.area:9.3f} kicad={B.area:9.3f} mm^2  xor={d.area:7.4f}  xor(>10um)={sig.area:7.4f}')
    if sig.area > 1e-4:
        for p in model.polys(sig)[:8]:
            c = p.centroid; print(f'      diff {p.area:.4f} mm^2 at ({c.x:.3f},{c.y:.3f})')
    if OUTP:
        import plot
        plot.svg([(A, '#2060ff', 0.6), (B, '#ff6020', 0.6), (sig.buffer(0.05), '#ffff00', 1)], (-0.5, -0.5, 39.5, 56.5), f'{OUTP}/cmp_{a}', 15)
# drills
def drills(path, shift):
    g = gerber.parse(path)
    return sorted((round(o['x'] + shift[0], 3), round(o['y'] + shift[1], 3), round(g['apertures'][o['ap']]['d'], 3)) for o in g['objs'] if o['kind'] == 'flash')
for a, k in (('PTH_Drill', 'PTH-drl.gbr'), ('NPTH_Drill', 'NPTH-drl.gbr')):
    A = drills(f'{SRC}/GerberX2/fpdlink2raspi_{a}.gbr', (0, 0))
    B = drills(glob.glob(f'{KG}/*-{k}')[0], (-100, 156))
    ok = len(A) == len(B) and all(abs(p[0]-q[0]) < 0.002 and abs(p[1]-q[1]) < 0.002 and p[2] == q[2] for p, q in zip(A, B))
    print(f'{a}: altium {len(A)} kicad {len(B)} all positions/sizes match within 2um: {ok}')
