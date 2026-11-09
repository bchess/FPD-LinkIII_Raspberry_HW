"""Board model reconstructed from Altium Gerber X2 + pick&place + BOM + schematic-PDF netlist.

Everything here is pure Python + shapely (Gerber coordinates, mm, Y up).
build_kicad.py turns this model into a KiCad board.
"""
import collections
import glob
import math
import os
import re
import zipfile
import xml.etree.ElementTree as ET

from shapely import affinity
from shapely.geometry import LineString, MultiPolygon, Point, Polygon, box
from shapely.ops import polygonize, unary_union
from shapely.strtree import STRtree

import gerber

LAYER_FILES = {
    'F.Cu': 'Copper_Signal_Top', 'In1.Cu': 'Copper_Signal_1', 'In2.Cu': 'Copper_Signal_2', 'B.Cu': 'Copper_Signal_Bot',
}
CU = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
PAD_FUNCS = ('SMDPad,CuDef', 'ComponentPad', 'ConnectorPad', 'WasherPad')
EPS = 1e-3


# --------------------------------------------------------------------------- geometry helpers
def _eval(expr, var):
    e = expr.replace('x', '*').replace('X', '*')
    e = re.sub(r'\$(\d+)', lambda mo: repr(var.get(int(mo.group(1)), 0.0)), e)
    return float(eval(e, {'__builtins__': {}}))


def macro_geom(ap):
    var = {i + 1: v for i, v in enumerate(ap['params'])}
    out = Polygon()
    u = ap.get('units', 1.0)
    for prim in ap['macro']:
        if '=' in prim and prim.startswith('$'):
            k, v = prim.split('=', 1)
            var[int(k[1:])] = _eval(v, var)
            continue
        f = [_eval(t, var) for t in prim.split(',')]
        code, expo = int(f[0]), f[1]
        rot = 0
        if code == 1:
            g = Point(f[3] * u, f[4] * u).buffer(f[2] * u / 2, 32)
            rot = f[5] if len(f) > 5 else 0
        elif code == 4:
            n = int(f[2])
            pts = [(f[3 + 2 * i] * u, f[4 + 2 * i] * u) for i in range(n + 1)]
            g = Polygon(pts).buffer(0)
            rot = f[3 + 2 * (n + 1)]
        elif code == 5:
            n, cx, cy, d = int(f[2]), f[3] * u, f[4] * u, f[5] * u
            g = Polygon([(cx + d / 2 * math.cos(2 * math.pi * i / n), cy + d / 2 * math.sin(2 * math.pi * i / n))
                         for i in range(n)])
            rot = f[6]
        elif code == 20:
            w, x1, y1, x2, y2 = f[2] * u, f[3] * u, f[4] * u, f[5] * u, f[6] * u
            g = LineString([(x1, y1), (x2, y2)]).buffer(w / 2, cap_style=2) if (x1, y1) != (x2, y2) else Polygon()
            rot = f[7]
        elif code == 21:
            w, h, cx, cy = f[2] * u, f[3] * u, f[4] * u, f[5] * u
            g = box(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)
            rot = f[6]
        else:
            raise ValueError(f'macro primitive {code}')
        if rot:
            g = affinity.rotate(g, rot, origin=(0, 0))
        out = out.union(g) if expo else out.difference(g)
    return out


def ap_geom(ap, x, y):
    s = ap['shape']
    if s == 'M':
        if '_geom' not in ap:
            ap['_geom'] = macro_geom(ap)
        return affinity.translate(ap['_geom'], x, y)
    if s == 'C':
        return Point(x, y).buffer(ap['d'] / 2, 32)
    if s == 'R':
        return box(x - ap['w'] / 2, y - ap['h'] / 2, x + ap['w'] / 2, y + ap['h'] / 2)
    if s == 'O':
        w, h = ap['w'], ap['h']
        if w >= h:
            return LineString([(x - (w - h) / 2, y), (x + (w - h) / 2, y)]).buffer(h / 2, 16) if w > h else Point(x, y).buffer(w / 2, 32)
        return LineString([(x, y - (h - w) / 2), (x, y + (h - w) / 2)]).buffer(w / 2, 16)
    if s == 'RR':
        w, h, r = ap['w'], ap['h'], ap['r']
        g = box(-(w / 2 - r), -(h / 2 - r), w / 2 - r, h / 2 - r).buffer(r, 16) if r > 0 else box(-w / 2, -h / 2, w / 2, h / 2)
        g = affinity.rotate(g, ap['rot'], origin=(0, 0))
        return affinity.translate(g, x, y)
    raise ValueError(s)


def ap_size(ap):
    """(w, h) of the aperture in board orientation."""
    s = ap['shape']
    if s == 'C':
        return ap['d'], ap['d']
    if s == 'RR' and round(ap['rot']) % 180 == 90:
        return ap['h'], ap['w']
    return ap['w'], ap['h']


def arc_mid(o):
    cx, cy = o['cx'], o['cy']
    r = math.hypot(o['x1'] - cx, o['y1'] - cy)
    a1 = math.atan2(o['y1'] - cy, o['x1'] - cx)
    a2 = math.atan2(o['y2'] - cy, o['x2'] - cx)
    if o['cw']:
        if a2 >= a1:
            a2 -= 2 * math.pi
    else:
        if a2 <= a1:
            a2 += 2 * math.pi
    am = (a1 + a2) / 2
    return cx + r * math.cos(am), cy + r * math.sin(am)


def obj_geom(o, aps):
    if o['kind'] == 'flash':
        return ap_geom(aps[o['ap']], o['x'], o['y'])
    if o['kind'] == 'line':
        ap = aps[o['ap']]
        if ap['shape'] != 'C':
            raise ValueError('non-circular draw aperture')
        if abs(o['x1'] - o['x2']) < 1e-9 and abs(o['y1'] - o['y2']) < 1e-9:
            return Point(o['x1'], o['y1']).buffer(ap['d'] / 2, 16)
        return LineString([(o['x1'], o['y1']), (o['x2'], o['y2'])]).buffer(ap['d'] / 2, 16)
    if o['kind'] == 'arc':
        ap = aps[o['ap']]
        pts = [(o['x1'], o['y1'])] + gerber._arc_points(o['x1'], o['y1'], o['x2'], o['y2'], o['cx'], o['cy'], o['cw'], 0.02)
        return LineString(pts).buffer(ap['d'] / 2, 16)
    if o['kind'] == 'region':
        polys = [Polygon(c).buffer(0) for c in o['contours'] if len(c) >= 4]
        return unary_union(polys)
    raise ValueError(o['kind'])


def render(objs, aps, keep=lambda o: True):
    """Sequentially evaluate polarity. Dark objects are only added when keep(o) is true."""
    img = Polygon()
    run, run_pol = [], None
    def flush():
        nonlocal img
        if not run:
            return
        u = unary_union(run)
        img = img.union(u) if run_pol == 'D' else img.difference(u)
    for o in objs:
        if o['pol'] == 'D' and not keep(o):
            continue
        if o['pol'] != run_pol:
            flush()
            run, run_pol = [], o['pol']
        run.append(obj_geom(o, aps))
    flush()
    return img


def polys(g):
    if g.is_empty:
        return []
    if isinstance(g, Polygon):
        return [g]
    return [p for p in getattr(g, 'geoms', []) if isinstance(p, Polygon) and not p.is_empty]


class UF:
    def __init__(self):
        self.p = {}
    def find(self, a):
        self.p.setdefault(a, a)
        while self.p[a] != a:
            self.p[a] = self.p[self.p[a]]
            a = self.p[a]
        return a
    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.p[rb] = ra


# --------------------------------------------------------------------------- inputs
def read_pick_place(path):
    pp = collections.OrderedDict()
    for line in open(path, encoding='latin-1'):
        f = line.split()
        if len(f) == 5 and f[1] in ('TopLayer', 'BottomLayer'):
            pp[f[0]] = dict(ref=f[0], side='T' if f[1] == 'TopLayer' else 'B', x=float(f[2]), y=float(f[3]),
                            rot=float(f[4]) % 360)
    return pp


def read_bom(path):
    ns = {'m': 'http://schemas.openxmlformats.org/spreadsheetml/2006/main'}
    z = zipfile.ZipFile(path)
    ss = [''.join(t.text or '' for t in si.iter('{%s}t' % ns['m']))
          for si in ET.fromstring(z.read('xl/sharedStrings.xml')).findall('m:si', ns)]
    root = ET.fromstring(z.read('xl/worksheets/sheet1.xml'))
    rows = []
    for row in root.iter('{%s}row' % ns['m']):
        vals = {}
        for c in row.findall('m:c', ns):
            v = c.find('m:v', ns)
            if v is None:
                continue
            vals[re.sub(r'\d', '', c.get('r'))] = ss[int(v.text)] if c.get('t') == 's' else v.text
        rows.append(vals)
    hdr = rows[0]
    out = {}
    for r in rows[1:]:
        rec = {hdr[k]: v for k, v in r.items() if k in hdr}
        for ref in [s.strip() for s in rec.get('Designator', '').split(',') if s.strip()]:
            out[ref] = rec
    return out


def read_schematic(pdf_path):
    import fitz
    d = fitz.open(pdf_path)
    toc = d.get_toc(simple=True)
    # netlist: the last top-level-ish "Nets" block (flattened project netlist)
    idx = max(i for i, e in enumerate(toc) if e[1] == 'Nets' and e[0] == 2)
    nets = collections.OrderedDict()
    lvl0 = toc[idx][0]
    cur, cat = None, None
    for lvl, title, _ in toc[idx + 1:]:
        if lvl <= lvl0:
            break
        if lvl == lvl0 + 1:
            cur = title
            nets.setdefault(cur, [])
        elif lvl == lvl0 + 2:
            cat = title
        elif lvl == lvl0 + 3 and cat == 'Pins':
            nets[cur].append(title)
    pin_net = {}
    for n, pins in nets.items():
        for p in pins:
            ref, _, num = p.partition('-')
            pin_net[(ref, num)] = n
    # values printed next to designators
    values = {}
    for page in d:
        items = [t.strip() for t in page.get_text().split('\n')]
        for a, b in zip(items, items[1:]):
            if re.fullmatch(r'(C|R|L|D|Q|U|J|X|Y|TP)\d+', a) and b and not re.fullmatch(r'(C|R|L|D|Q|U|J|X|Y|TP)\d+', b):
                values.setdefault(a, b)
    return nets, pin_net, values


# --------------------------------------------------------------------------- model
class Board:
    def __init__(self, root):
        self.root = root
        gd = os.path.join(root, 'GerberX2')
        self.g = {}
        for f in glob.glob(gd + '/*.gbr'):
            key = os.path.basename(f)[len('fpdlink2raspi_'):-4]
            self.g[key] = gerber.parse(f)
        self.pp = read_pick_place(glob.glob(os.path.join(root, 'Pick Place', '*.txt'))[0])
        self.bom = read_bom(glob.glob(os.path.join(root, 'BOM', '*.xlsx'))[0])
        self.sch_nets, self.pin_net, self.sch_values = read_schematic(os.path.join(root, 'fpdlink2raspi_SCH_0v4.PDF'))
        self.log = []

    def warn(self, msg):
        self.log.append(msg)
        print('  !', msg)

    # ---------------------------------------------------------------- drills / holes
    def holes(self):
        out = []
        for key, plated in (('PTH_Drill', True), ('NPTH_Drill', False)):
            g = self.g[key]
            for o in g['objs']:
                ap = g['apertures'][o['ap']]
                out.append(dict(x=o['x'], y=o['y'], d=ap['d'], plated=plated, func=o['func']))
        return out

    # ---------------------------------------------------------------- pads
    def build_pads(self):
        """Pads from the Pad Master layers, merged with drills and copper flashes."""
        holes = self.holes()
        hole_at = lambda x, y: [h for h in holes if abs(h['x'] - x) < EPS and abs(h['y'] - y) < EPS]
        raw = []
        for side, key in (('T', 'Pads_Top'), ('B', 'Pads_Bot')):
            g = self.g[key]
            for o in g['objs']:
                if o['kind'] == 'flash':
                    raw.append(dict(side=side, x=o['x'], y=o['y'], ap=dict(g['apertures'][o['ap']]), func=o['func']))
        pads = []
        th = {}
        for r in raw:
            hs = [h for h in hole_at(r['x'], r['y']) if h['func'] != 'ViaDrill,NotFilled']
            if hs:
                k = (round(r['x'], 4), round(r['y'], 4))
                if k in th:
                    th[k]['ap_side'][r['side']] = r['ap']
                    continue
                p = dict(r, kind='PTH' if hs[0]['plated'] else 'NPTH', drill=hs[0]['d'], hole=hs[0],
                         ap_side={r['side']: r['ap']})
                th[k] = p
                pads.append(p)
            else:
                pads.append(dict(r, kind='SMD'))
        # holes with no pad at all
        for h in holes:
            if h['func'].startswith('ViaDrill'):
                continue
            if not any(p['kind'] != 'SMD' and p['hole'] is h for p in pads):
                pads.append(dict(side='T', x=h['x'], y=h['y'], ap={'shape': 'C', 'd': h['d']}, func='bare-hole',
                                 kind='PTH' if h['plated'] else 'NPTH', drill=h['d'], hole=h, ap_side={}))
        # extra copper conductor flashes glued to pads (e.g. WQFN pad tips) -> custom pads
        for side, cu in (('T', 'F.Cu'), ('B', 'B.Cu')):
            g = self.g[LAYER_FILES[cu]]
            for o in g['objs']:
                if o['kind'] == 'flash' and o['func'] == 'Conductor':
                    geo = obj_geom(o, g['apertures'])
                    hit = [p for p in pads if p['side'] == side and p['kind'] == 'SMD' and ap_geom(p['ap'], p['x'], p['y']).intersects(geo)]
                    if len(hit) != 1:
                        self.warn(f'conductor flash at {o["x"]:.3f},{o["y"]:.3f} touches {len(hit)} pads')
                        continue
                    hit[0].setdefault('extra', []).append(dict(ap=g['apertures'][o['ap']], x=o['x'], y=o['y']))
        # inner-layer annular rings for TH pads
        for cu in ('In1.Cu', 'In2.Cu'):
            g = self.g[LAYER_FILES[cu]]
            for o in g['objs']:
                if o['kind'] == 'flash' and o['func'] in ('ComponentPad', 'WasherPad'):
                    for p in pads:
                        if p['kind'] != 'SMD' and abs(p['x'] - o['x']) < EPS and abs(p['y'] - o['y']) < EPS:
                            p['ap_side'][cu] = g['apertures'][o['ap']]
        for i, p in enumerate(pads):
            p['id'] = i
            p['geom'] = ap_geom(p['ap'], p['x'], p['y'])
            if p.get('extra'):
                p['geom'] = unary_union([p['geom']] + [ap_geom(e['ap'], e['x'], e['y']) for e in p['extra']])
        self.pads = pads
        return pads

    def match_mask_paste(self):
        def flashes(key, funcs=None):
            g = self.g[key]
            return [dict(x=o['x'], y=o['y'], ap=g['apertures'][o['ap']], func=o['func'])
                    for o in g['objs'] if o['kind'] == 'flash']
        mask = {'T': flashes('Soldermask_Top'), 'B': flashes('Soldermask_Bot')}
        paste = {'T': flashes('Paste_Top'), 'B': flashes('Paste_Bot')}
        used_paste = set()
        for p in self.pads:
            for side in ('T', 'B'):
                if p['kind'] == 'SMD' and p['side'] != side:
                    continue
                ap = p['ap'] if p['kind'] == 'SMD' else p['ap_side'].get(side, p['ap'])
                at = lambda lst: [f for f in lst if abs(f['x'] - p['x']) < EPS and abs(f['y'] - p['y']) < EPS
                                  and f['func'] != 'Conductor']
                m = at(mask[side])
                p.setdefault('mask', {})[side] = round((ap_size(m[0]['ap'])[0] - ap_size(ap)[0]) / 2, 4) if m else None
                p.setdefault('mask_ap', {})[side] = m[0]['ap'] if m else None
                if p['kind'] == 'SMD':
                    pa = at(paste[side])
                    for f in pa:
                        used_paste.add(id(f))
                    p['paste'] = round((ap_size(pa[0]['ap'])[0] - ap_size(ap)[0]) / 2, 4) if pa else None
        # paste-only apertures (segmented exposed-pad stencils)
        self.paste_only = []
        for side in ('T', 'B'):
            for f in paste[side]:
                if id(f) in used_paste or f['func'] == 'Conductor':
                    continue
                self.paste_only.append(dict(f, side=side))
        self.via_mask_open = {s: [(f['x'], f['y']) for f in mask[s] if f['func'] == 'ViaPad'] for s in ('T', 'B')}

    def _attach_courtyards(self, comps):
        """Give each closed courtyard loop to the component(s) whose centre it encloses."""
        left = []
        for side, grp in self.loose_courtyard:
            side_comps = [c for c in comps if c['side'] == side]
            ls = [LineString([(o['x1'], o['y1']), (o['x2'], o['y2'])]) for o in grp]
            faces = list(polygonize(unary_union(ls)))
            outline = unary_union(faces) if faces else None
            inside = [c for c in side_comps if outline is not None and outline.contains(Point(c['x'], c['y']))]
            if not inside:
                left.append((side, grp))
                continue
            if len(inside) == 1:
                owners = {inside[0]['ref']: outline}
            else:  # several courtyards share corners/edges: split by faces
                owners = collections.defaultdict(list)
                for f in faces:
                    hit = [c for c in inside if f.contains(Point(c['x'], c['y']))]
                    if hit:
                        owners[hit[0]['ref']].append(f)
                owners = {r: unary_union(fs) for r, fs in owners.items()}
            for r, poly in owners.items():
                c = self.pp[r]
                if c['courtyard'] is not None and c['courtyard'].area <= poly.area:
                    continue
                c['courtyard'] = poly
                bd = poly.boundary
                c['courtyard_lines'] = [o for o in grp if bd.distance(Point(o['x1'], o['y1'])) < 1e-3
                                        and bd.distance(Point(o['x2'], o['y2'])) < 1e-3]
        self.loose_courtyard = left

    # ---------------------------------------------------------------- components
    def assign_components(self):
        pp = self.pp
        for c in pp.values():
            c['courtyard'], c['courtyard_lines'], c['pads'] = None, [], []
        self.loose_courtyard = []
        for key, side in (('Mechanical_15', 'T'), ('Mechanical_16', 'B')):
            g = self.g[key]
            lines = [dict(o, w=g['apertures'][o['ap']]['d']) for o in g['objs'] if o['kind'] == 'line']
            uf = UF()
            k = lambda x, y: (round(x, 3), round(y, 3))
            for i, o in enumerate(lines):
                uf.union(('l', i), k(o['x1'], o['y1']))
                uf.union(('l', i), k(o['x2'], o['y2']))
            groups = collections.defaultdict(list)
            for i, o in enumerate(lines):
                groups[uf.find(('l', i))].append(o)
            self.loose_courtyard += [(side, grp) for grp in groups.values()]
        self._attach_courtyards(list(pp.values()))
        unassigned = []
        # free pads: mounting holes (Altium free pads with NPTH 'OtherDrill')
        self.mounting = []
        for p in self.pads:
            if p['kind'] == 'NPTH' and p['hole']['func'].startswith('OtherDrill'):
                name = re.search(r'Pad Free-(\w+)', p['hole']['func']).group(1)
                self.mounting.append(dict(ref=name, pad=p))
                p['owner'] = name
            else:
                unassigned.append(p)
        # pass 1: two-terminal parts, pads symmetric about the part centre
        two_term = [c for c in pp.values() if re.match(r'(C|R|L|D)\d', c['ref'])]
        taken = set()
        for c in sorted(two_term, key=lambda c: c['ref']):
            best = None
            cand = [p for p in unassigned if p['kind'] == 'SMD' and p['side'] == c['side'] and p['id'] not in taken
                    and math.hypot(p['x'] - c['x'], p['y'] - c['y']) < 4]
            for i, a in enumerate(cand):
                for b in cand[i + 1:]:
                    if a['ap'] != b['ap']:
                        continue
                    mx, my = (a['x'] + b['x']) / 2, (a['y'] + b['y']) / 2
                    d = math.hypot(mx - c['x'], my - c['y'])
                    if d < 0.15 and (best is None or d < best[0]):
                        best = (d, a, b)
            if best is None:
                self.warn(f'{c["ref"]}: no symmetric pad pair found')
                continue
            for p in best[1:]:
                taken.add(p['id'])
                p['owner'] = c['ref']
                c['pads'].append(p)
        rest = [c for c in pp.values() if not c['pads']]
        orphans = []
        for p in unassigned:
            if p['id'] in taken:
                continue
            pt = Point(p['x'], p['y'])
            cands = [c for c in rest if p['kind'] != 'SMD' or c['side'] == p['side']]
            inside = [c for c in cands if c['courtyard'] is not None and c['courtyard'].buffer(0.05).contains(pt)]
            pool = inside or cands
            c = min(pool, key=lambda c: math.hypot(c['x'] - p['x'], c['y'] - p['y']))
            if not inside and math.hypot(c['x'] - p['x'], c['y'] - p['y']) > 3:
                orphans.append(p)
                continue
            p['owner'] = c['ref']
            c['pads'].append(p)
        # Orphan pads: footprints of parts that are "not fitted" in this variant and therefore missing
        # from pick&place. Pair them up; they are identified later from the netlist.
        self.orphans = []
        while orphans:
            a = orphans.pop(0)
            mates = [b for b in orphans if b['ap'] == a['ap'] and b['side'] == a['side']
                     and math.hypot(a['x'] - b['x'], a['y'] - b['y']) < 2.5]
            if not mates:
                self.warn(f'orphan pad at {a["x"]:.2f},{a["y"]:.2f} has no partner')
                continue
            b = min(mates, key=lambda b: math.hypot(a['x'] - b['x'], a['y'] - b['y']))
            orphans.remove(b)
            ref = f'?{len(self.orphans) + 1}'
            ang = math.degrees(math.atan2(b['y'] - a['y'], b['x'] - a['x'])) % 180
            c = dict(ref=ref, side=a['side'], x=(a['x'] + b['x']) / 2, y=(a['y'] + b['y']) / 2,
                     rot=round(ang / 90) * 90 % 360, courtyard=None, pads=[a, b], dnp=True)
            for p in (a, b):
                p['owner'] = ref
            self.orphans.append(c)
            pp[ref] = c
        self._attach_courtyards(self.orphans)
        for c in pp.values():
            if not c['pads']:
                self.warn(f'{c["ref"]} has no pads')

    # ---------------------------------------------------------------- copper / connectivity
    def build_copper(self):
        """Classify copper primitives, build per-layer item geometry, compute connectivity."""
        self.tracks, self.vias, self.pours, self.cu_graphics = [], [], [], []
        holes = self.holes()
        via_holes = [h for h in holes if h['func'].startswith('ViaDrill')]
        self.layer_image = {}
        for cu in CU:
            g = self.g[LAYER_FILES[cu]]
            aps = g['apertures']
            self.layer_image[cu] = render(g['objs'], aps)
            for o in g['objs']:
                if o['pol'] != 'D':
                    continue
                if o['kind'] in ('line', 'arc') and o['func'] == 'Conductor':
                    t = dict(o, layer=cu, w=aps[o['ap']]['d'])
                    t['geom'] = obj_geom(o, aps)
                    self.tracks.append(t)
                elif o['kind'] in ('line', 'arc') and o['func'] == 'NonConductor':
                    self.cu_graphics.append(dict(o, layer=cu, w=aps[o['ap']]['d']))
            # pour pieces: regions + every clear primitive, nothing else dark
            pour_img = render(g['objs'], aps, keep=lambda o: o['kind'] == 'region')
            for poly in polys(pour_img):
                if poly.area < 1e-4:
                    continue
                self.pours.append(dict(layer=cu, geom=poly))
        # vias
        for h in via_holes:
            sizes = {}
            for cu in CU:
                g = self.g[LAYER_FILES[cu]]
                for o in g['objs']:
                    if o['kind'] == 'flash' and o['func'] == 'ViaPad' and abs(o['x'] - h['x']) < EPS and abs(o['y'] - h['y']) < EPS:
                        sizes[cu] = g['apertures'][o['ap']]['d']
            if len(set(sizes.values())) != 1 or len(sizes) != 4:
                self.warn(f'via at {h["x"]:.3f},{h["y"]:.3f} has per-layer sizes {sizes}')
            d = max(sizes.values())
            tent = {s: not any(abs(x - h['x']) < EPS and abs(y - h['y']) < EPS for x, y in self.via_mask_open[s])
                    for s in ('T', 'B')}
            self.vias.append(dict(x=h['x'], y=h['y'], drill=h['d'], d=d, sizes=sizes, tent=tent,
                                  geom=Point(h['x'], h['y']).buffer(d / 2, 32)))

        # ---- connectivity
        items = []  # (key, layer, geom)
        for p in self.pads:
            if p['kind'] == 'SMD':
                items.append((('pad', p['id']), 'F.Cu' if p['side'] == 'T' else 'B.Cu', p['geom']))
            elif p['kind'] == 'PTH':
                for cu, key in (('F.Cu', 'T'), ('B.Cu', 'B'), ('In1.Cu', 'In1.Cu'), ('In2.Cu', 'In2.Cu')):
                    ap = p['ap_side'].get(key)
                    if ap is not None and ap_size(ap)[0] > p['drill'] + 1e-6:
                        items.append((('pad', p['id']), cu, ap_geom(ap, p['x'], p['y'])))
        for i, t in enumerate(self.tracks):
            items.append((('trk', i), t['layer'], t['geom']))
        for i, v in enumerate(self.vias):
            for cu in CU:
                items.append((('via', i), cu, Point(v['x'], v['y']).buffer(v['sizes'].get(cu, v['d']) / 2, 32)))
        for i, z in enumerate(self.pours):
            items.append((('pour', i), z['layer'], z['geom']))
        uf = UF()
        for it in items:
            uf.find(it[0])
        for cu in CU:
            li = [it for it in items if it[1] == cu]
            geoms = [it[2] for it in li]
            tree = STRtree(geoms)
            for i, it in enumerate(li):
                for j in tree.query(it[2].buffer(1e-4)):
                    j = int(j)
                    if j > i and geoms[j].distance(it[2]) < 1e-4:
                        uf.union(it[0], li[j][0])
        self.uf = uf
        self.cu_items = items

    # ---------------------------------------------------------------- pad numbering
    def numbering_candidates(self, c):
        pads = c['pads']
        ref = c['ref']
        n = len(pads)
        rot = math.radians(c['rot'])
        def local(p):
            dx, dy = p['x'] - c['x'], p['y'] - c['y']
            lx = dx * math.cos(-rot) - dy * math.sin(-rot)
            ly = dx * math.sin(-rot) + dy * math.cos(-rot)
            if c['side'] == 'B':
                lx = -lx
            return lx, ly
        loc = {p['id']: local(p) for p in pads}
        numbered = [p for p in pads if p['kind'] != 'NPTH']
        nonum = [p for p in pads if p['kind'] == 'NPTH']
        base = {p['id']: '' for p in nonum}
        area = lambda p: ap_size(p['ap'])[0] * ap_size(p['ap'])[1]
        def ring(sig, start_num=1):
            """CCW (top view) orderings starting at each pad, plus CW as fallback."""
            cx = sum(loc[p['id']][0] for p in sig) / len(sig)
            cy_ = sum(loc[p['id']][1] for p in sig) / len(sig)
            ordered = sorted(sig, key=lambda p: math.atan2(loc[p['id']][1] - cy_, loc[p['id']][0] - cx))
            out = []
            for direction in (1, -1):
                seq = ordered if direction == 1 else ordered[::-1]
                for s in range(len(seq)):
                    rot_seq = seq[s:] + seq[:s]
                    out.append({p['id']: str(start_num + k) for k, p in enumerate(rot_seq)})
            return out
        def row(sig, names):
            # sort along principal axis (largest spread)
            xs = [loc[p['id']][0] for p in sig]
            ys = [loc[p['id']][1] for p in sig]
            axis = 0 if (max(xs) - min(xs)) >= (max(ys) - min(ys)) else 1
            ordered = sorted(sig, key=lambda p: loc[p['id']][axis])
            return [{p['id']: names[k] for k, p in enumerate(seq)} for seq in (ordered, ordered[::-1])]
        def combine(*groups):
            res = [dict(base)]
            for g in groups:
                res = [{**a, **b} for a in res for b in g]
            return res
        if n == 1 or len(numbered) == 1 and ref.startswith('TP'):
            return combine([{numbered[0]['id']: '1'}])
        if re.match(r'(C|R|L|D|\?)\d', ref) and len(numbered) == 2:
            a, b = numbered
            return combine([{a['id']: '1', b['id']: '2'}, {a['id']: '2', b['id']: '1'}])
        if ref in ('X1', 'X2', 'J2'):
            key = lambda p: (p['ap']['shape'],) + tuple(round(v, 3) for v in ap_size(p['ap']))
            sizes = collections.Counter(key(p) for p in numbered)
            uniq = [p for p in numbered if sizes[key(p)] == 1]
            out = []
            for one in sorted(numbered, key=lambda p: p not in uniq):  # unique-shaped pad first
                others = [p for p in numbered if p is not one]
                out.append({one['id']: '1', **ring(others, 2)[0]})
            return combine(out)
        if ref == 'J1':
            sig = [p for p in numbered if abs(ap_size(p['ap'])[0] * ap_size(p['ap'])[1] - 1.2) < 0.01]
            shield = [p for p in numbered if p not in sig]
            return combine(row(sig, [str(i) for i in range(1, 16)]), row(shield, ['S1', 'S2']))
        if ref == 'J3':
            sig = [p for p in numbered if abs(area(p) - 1.35 * 0.4) < 0.01]
            shield = [p for p in numbered if p not in sig]
            return combine(row(sig, ['1', '2', '3', '4', '5']),
                           [{k: 'Z' + v for k, v in ring(shield, 1)[0].items()}])
        if ref in ('X3', 'X4'):
            return combine(row(numbered, ['1', '2', '3']))
        if ref.startswith('U') or ref.startswith('Q') or ref.startswith('Y'):
            big = max(numbered, key=area)
            if area(big) > 3 * sorted(area(p) for p in numbered)[len(numbered) // 2] and len(numbered) > 8:
                sig = [p for p in numbered if p is not big]
                return combine(ring(sig), [{big['id']: str(len(sig) + 1)}])
            return combine(ring(numbered))
        self.warn(f'{ref}: no numbering rule, using arbitrary order')
        return combine([{p['id']: str(k + 1) for k, p in enumerate(numbered)}])

    def solve_numbering(self):
        comps = [c for c in self.pp.values() if not c['ref'].startswith('?')]
        cands = {c['ref']: self.numbering_candidates(c) for c in comps}
        gnet = {p['id']: self.uf.find(('pad', p['id'])) for p in self.pads}
        decided = {r: cs[0] for r, cs in cands.items() if len(cs) == 1}
        def labels(exclude=None):
            lab = collections.defaultdict(collections.Counter)
            for r, num in decided.items():
                if r == exclude:
                    continue
                for pid, pn in num.items():
                    sn = self.pin_net.get((r, pn))
                    if sn:
                        lab[gnet[pid]][sn] += 1
            return lab
        pending = [r for r in cands if r not in decided]
        while pending:
            lab = labels()
            best_r, best_margin, best_num, best_score = None, -1, None, None
            for r in pending:
                scores = []
                for num in cands[r]:
                    s = 0
                    for pid, pn in num.items():
                        sn = self.pin_net.get((r, pn))
                        if sn and lab[gnet[pid]]:
                            top = lab[gnet[pid]].most_common(1)[0][0]
                            s += 1 if sn in lab[gnet[pid]] else 0
                    scores.append(s)
                order = sorted(range(len(scores)), key=lambda i: -scores[i])
                distinct = sorted(set(scores), reverse=True)
                margin = distinct[0] - distinct[1] if len(distinct) > 1 else 0
                if margin > best_margin:
                    best_r, best_margin, best_num, best_score = r, margin, cands[r][order[0]], scores[order[0]]
            if best_margin <= 0:
                for r in pending:
                    self.warn(f'{r}: pin numbering ambiguous, using default orientation')
                    decided[r] = cands[r][0]
                break
            if best_margin == 0:
                self.warn(f'{best_r}: pin numbering not constrained by the netlist, using default orientation')
            decided[best_r] = best_num
            pending.remove(best_r)
        # identify orphan footprints (not-fitted parts) from the netlist
        lab = labels()
        missing = sorted({r for (r, _) in self.pin_net if r not in self.pp},
                         key=lambda r: (re.sub(r'\d', '', r), int(re.sub(r'\D', '', r) or 0)))
        options = []
        for o in self.orphans:
            for num in self.numbering_candidates(o):
                for r in missing:
                    pins = {pn for (rr, pn) in self.pin_net if rr == r}
                    if pins != set(num.values()):
                        continue
                    s = sum(1 for pid, pn in num.items() if self.pin_net[(r, pn)] in lab[gnet[pid]])
                    options.append((s, o['ref'], r, num))
        options.sort(key=lambda t: -t[0])
        used_o, used_r = set(), set()
        for s, oref, r, num in options:
            if oref in used_o or r in used_r or s == 0:
                continue
            used_o.add(oref); used_r.add(r)
            c = self.pp.pop(oref)
            c['ref'] = r
            self.pp[r] = c
            for p in c['pads']:
                p['owner'] = r
            cands[r] = self.numbering_candidates(c)
            decided[r] = {pid: pn for pid, pn in num.items()}
            comps.append(c)
            print(f'  orphan {oref} at ({c["x"]:.2f},{c["y"]:.2f}) identified as not-fitted {r} (score {s}/2)')
        for o in list(self.orphans):
            if o['ref'].startswith('?'):
                self.warn(f'orphan {o["ref"]} at ({o["x"]:.2f},{o["y"]:.2f}) could not be identified')
                cands[o['ref']] = self.numbering_candidates(o)
                decided[o['ref']] = cands[o['ref']][0]
                comps.append(o)
        # final refinement pass with everything decided
        for _ in range(3):
            changed = False
            for r in cands:
                if len(cands[r]) == 1:
                    continue
                lab = labels(exclude=r)
                def score(num):
                    return sum(1 for pid, pn in num.items()
                               if self.pin_net.get((r, pn)) and self.pin_net.get((r, pn)) in lab[gnet[pid]])
                best = max(cands[r], key=score)
                if score(best) > score(decided[r]):
                    decided[r] = best
                    changed = True
            if not changed:
                break
        for c in comps:
            for p in c['pads']:
                p['number'] = decided[c['ref']][p['id']]
        self.numbering = decided

    # ---------------------------------------------------------------- nets
    def name_nets(self):
        groups = collections.defaultdict(list)
        for it in self.cu_items:
            groups[self.uf.find(it[0])].append(it[0])
        pad_by_id = {p['id']: p for p in self.pads}
        auto = lambda n: re.match(r'^(Net[A-Z]+\d+_|N\d{5})', n) is not None
        self.net_of = {}
        names_used = collections.Counter()
        report = []
        for root, keys in groups.items():
            pins = [pad_by_id[k[1]] for k in keys if k[0] == 'pad']
            sch = collections.Counter()
            for p in pins:
                sn = self.pin_net.get((p.get('owner'), p.get('number')))
                if sn:
                    sch[sn] += 1
            if not pins:
                name = ''
            elif sch:
                named = [n for n in sch if not auto(n)]
                pool = named or list(sch)
                name = max(pool, key=lambda n: (sch[n], -len(n)))
                if len(named) > 1:
                    report.append(f'copper net joins schematic nets {sorted(named)}')
            else:
                p = pins[0]
                name = f'Net-({p["owner"]}-Pad{p["number"]})'
            self.net_of[root] = name
            if name:
                names_used[name] += 1
        for n, k in names_used.items():
            if k > 1:
                report.append(f'net {n} is split into {k} separate copper islands')
        # pins whose copper net differs from schematic
        mism = []
        for p in self.pads:
            if not p.get('owner') or p.get('number') in (None, ''):
                continue
            sn = self.pin_net.get((p['owner'], p['number']))
            cn = self.net_of[self.uf.find(('pad', p['id']))] if ('pad', p['id']) in self.uf.p else ''
            if sn and cn and sn != cn:
                mism.append((f'{p["owner"]}-{p["number"]}', sn, cn))
        self.net_report = report
        self.net_mismatch = mism
        missing = [k for k in self.pin_net if k[0] in self.pp and not any(
            p.get('owner') == k[0] and p.get('number') == k[1] for p in self.pads)]
        self.missing_pins = missing

    def net_for(self, key):
        return self.net_of.get(self.uf.find(key), '') if key in self.uf.p else ''

    # ---------------------------------------------------------------- mechanical graphics
    def mech(self, key):
        g = self.g[key]
        out = []
        for o in g['objs']:
            if o['kind'] in ('line', 'arc'):
                out.append(dict(o, w=g['apertures'][o['ap']]['d']))
        return out

    def run(self):
        print('pads...');
        self.build_pads()
        self.match_mask_paste()
        print('components...')
        self.assign_components()
        print('copper + connectivity...')
        self.build_copper()
        print('pin numbering...')
        self.solve_numbering()
        print('nets...')
        self.name_nets()
        return self
