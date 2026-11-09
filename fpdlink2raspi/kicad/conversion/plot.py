import sys, os
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, here); sys.path.insert(0, os.path.join(here, '..', 'pylib'))
import fitz
def path_d(g):
    out = []
    for p in ([g] if g.geom_type == 'Polygon' else getattr(g, 'geoms', [])):
        if p.geom_type != 'Polygon' or p.is_empty: continue
        for ring in [p.exterior] + list(p.interiors):
            cs = list(ring.coords)
            out.append('M' + ' L'.join(f'{x:.3f},{-y:.3f}' for x, y in cs) + ' Z')
    return ' '.join(out)
def svg(layers, bbox, fn, scale=30):
    x0, y0, x1, y1 = bbox
    w, h = (x1 - x0), (y1 - y0)
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w*scale}" height="{h*scale}" viewBox="{x0} {-y1} {w} {h}">',
             f'<rect x="{x0}" y="{-y1}" width="{w}" height="{h}" fill="black"/>']
    for g, color, op in layers:
        parts.append(f'<path d="{path_d(g)}" fill="{color}" fill-opacity="{op}" fill-rule="evenodd"/>')
    parts.append('</svg>')
    open(fn + '.svg', 'w').write('\n'.join(parts))
    doc = fitz.open(fn + '.svg')
    pix = doc[0].get_pixmap()
    pix.save(fn + '.png')
