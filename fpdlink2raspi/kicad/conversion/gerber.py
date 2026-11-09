"""Minimal Gerber X2 parser sufficient for Altium Designer 20 output.

Produces a list of primitive objects in file order:
  {'kind': 'flash', 'x','y', 'ap', 'func', 'pol'}
  {'kind': 'line',  'x1','y1','x2','y2', 'ap', 'func', 'pol'}
  {'kind': 'arc',   'x1','y1','x2','y2','cx','cy','cw', 'ap', 'func', 'pol'}
  {'kind': 'region','pts':[(x,y),...] (closed, arcs linearised), 'raw':[segments], 'func', 'pol'}
Apertures: {'D10': {'shape':'C','d':..} | {'shape':'R','w','h'} | {'shape':'O','w','h'}
            | {'shape':'RR','w','h','r','rot'} , 'func': str}
Coordinates in mm.
"""
import math
import re


def _arc_points(x1, y1, x2, y2, cx, cy, cw, seg_len=0.05):
    r = math.hypot(x1 - cx, y1 - cy)
    a1 = math.atan2(y1 - cy, x1 - cx)
    a2 = math.atan2(y2 - cy, x2 - cx)
    if cw:
        if a2 >= a1 - 1e-12:
            a2 -= 2 * math.pi
    else:
        if a2 <= a1 + 1e-12:
            a2 += 2 * math.pi
    if abs(x1 - x2) < 1e-9 and abs(y1 - y2) < 1e-9:
        a2 = a1 + (-2 * math.pi if cw else 2 * math.pi)
    n = max(8, int(abs(a2 - a1) * r / seg_len))
    return [(cx + r * math.cos(a1 + (a2 - a1) * i / n), cy + r * math.sin(a1 + (a2 - a1) * i / n))
            for i in range(1, n + 1)]


def parse(path):
    text = open(path, encoding='latin-1').read()
    attrs = {}
    apertures = {}
    amparams = {}
    macros = {}
    objs = []
    fmt = (4, 5)
    units = 1.0
    cur_ap = None
    cur_func = None  # current TA.AperFunction
    pol = 'D'
    interp = 'G01'
    region = None
    x = y = 0.0
    i_ = j_ = 0.0

    # tokenise: extended commands %...% and normal word commands ending in *
    pos = 0
    tokens = []
    while pos < len(text):
        c = text[pos]
        if c in '\r\n ':
            pos += 1
            continue
        if c == '%':
            end = text.index('%', pos + 1)
            tokens.append(('X', text[pos + 1:end]))
            pos = end + 1
        else:
            end = text.index('*', pos)
            tokens.append(('W', text[pos:end]))
            pos = end + 1

    def coord(s):
        neg = s.startswith('-')
        s = s.lstrip('+-')
        v = int(s) / (10 ** fmt[1]) if s else 0
        return (-v if neg else v) * units

    for kind, tok in tokens:
        if kind == 'X':
            body = tok.rstrip('*')
            if body.startswith('FS'):
                m = re.match(r'FSLAX(\d)(\d)Y(\d)(\d)', body)
                fmt = (int(m.group(1)), int(m.group(2)))
            elif body.startswith('MO'):
                units = 1.0 if body == 'MOMM' else 25.4
            elif body.startswith('TF.'):
                k, _, v = body[3:].partition(',')
                attrs[k] = v
            elif body.startswith('TA.AperFunction'):
                cur_func = body.split(',', 1)[1]
            elif body.startswith('TD'):
                cur_func = None
            elif body.startswith('AM'):
                name, *prims = [p.strip() for p in body[2:].split('*')]
                macros[name] = [p.replace('\n', '').replace('\r', '') for p in prims if p and not p.startswith('0 ')]
            elif body.startswith('AD'):
                m = re.match(r'ADD(\d+)([A-Za-z_0-9]+?)(?:,(.*))?$', body)
                dn, shp, params = m.group(1), m.group(2), m.group(3)
                d = 'D' + dn
                if shp == 'C':
                    p = [float(v) for v in params.split('X')]
                    ap = {'shape': 'C', 'd': p[0] * units}
                elif shp in ('R', 'O'):
                    p = [float(v) for v in params.split('X')]
                    ap = {'shape': shp, 'w': p[0] * units, 'h': p[1] * units}
                elif shp == 'P':
                    p = [float(v) for v in params.split('X')]
                    ap = {'shape': 'P', 'd': p[0] * units, 'n': int(p[1]), 'rot': p[2] if len(p) > 2 else 0}
                elif dn in amparams:
                    ap = dict(amparams[dn])
                else:
                    vals = [float(v) for v in params.split('X')] if params else []
                    ap = {'shape': 'M', 'macro': macros[shp], 'params': vals, 'units': units}
                ap['func'] = cur_func
                apertures[d] = ap
            elif body.startswith('LP'):
                pol = body[2]
            continue

        w = tok
        if w.startswith('G04'):
            if w.startswith('G04:AMPARAMS|'):
                kv = dict(p.split('=', 1) for p in w[len('G04:AMPARAMS|'):].split('|') if '=' in p)
                mm = lambda s: float(s.replace('mm', ''))
                if kv.get('Shape') == 'RoundedRectangle':
                    amparams[kv['DCode']] = {'shape': 'RR', 'w': mm(kv['XSize']), 'h': mm(kv['YSize']),
                                             'r': mm(kv['CornerRadius']), 'rot': float(kv['Rotation'])}
                else:
                    raise ValueError(f'unknown AMPARAMS shape {kv}')
            continue
        if w in ('G01', 'G02', 'G03'):
            interp = w
            continue
        if w in ('G74', 'G75'):
            continue
        if w == 'G36':
            region = {'kind': 'region', 'func': 'Region', 'pol': pol,
                      'contours': [], 'raw': []}
            contour = None
            continue
        if w == 'G37':
            if region['contours']:
                objs.append(region)
            region = None
            continue
        if w == 'M02':
            break
        m = re.match(r'^(G0[123])?(?:X([+-]?\d+))?(?:Y([+-]?\d+))?(?:I([+-]?\d+))?(?:J([+-]?\d+))?(D0?[123])?$', w)
        if m and (m.group(2) or m.group(3) or m.group(6)):
            if m.group(1):
                interp = m.group(1)
            nx = coord(m.group(2)) if m.group(2) is not None else x
            ny = coord(m.group(3)) if m.group(3) is not None else y
            ii = coord(m.group(4)) if m.group(4) is not None else 0.0
            jj = coord(m.group(5)) if m.group(5) is not None else 0.0
            op = m.group(6) or 'D01'
            op = 'D0' + op[-1]
            if region is not None:
                if op == 'D02':
                    region['contours'].append([(nx, ny)])
                    region['raw'].append([('start', nx, ny)])
                elif op == 'D01':
                    if not region['contours']:
                        region['contours'].append([(x, y)])
                        region['raw'].append([('start', x, y)])
                    if interp == 'G01':
                        region['contours'][-1].append((nx, ny))
                        region['raw'][-1].append(('line', nx, ny))
                    else:
                        cw = interp == 'G02'
                        region['contours'][-1].extend(_arc_points(x, y, nx, ny, x + ii, y + jj, cw))
                        region['raw'][-1].append(('arc', nx, ny, x + ii, y + jj, cw))
            else:
                ap = apertures.get(cur_ap)
                func = ap['func'] if ap else None
                if op == 'D03':
                    objs.append({'kind': 'flash', 'x': nx, 'y': ny, 'ap': cur_ap, 'func': func, 'pol': pol})
                elif op == 'D01':
                    if interp == 'G01':
                        objs.append({'kind': 'line', 'x1': x, 'y1': y, 'x2': nx, 'y2': ny, 'ap': cur_ap,
                                     'func': func, 'pol': pol})
                    else:
                        objs.append({'kind': 'arc', 'x1': x, 'y1': y, 'x2': nx, 'y2': ny, 'cx': x + ii,
                                     'cy': y + jj, 'cw': interp == 'G02', 'ap': cur_ap, 'func': func, 'pol': pol})
            x, y = nx, ny
            continue
        m = re.match(r'^(?:G54)?D(\d+)$', w)
        if m and int(m.group(1)) >= 10:
            cur_ap = 'D' + m.group(1)
            continue
        raise ValueError(f'{path}: unhandled word {w!r}')
    return {'attrs': attrs, 'apertures': apertures, 'objs': objs}


def cur_func_region(apertures, cur_ap, cur_func):
    # Regions take the *current* aperture attribute dictionary in X2.
    return cur_func
