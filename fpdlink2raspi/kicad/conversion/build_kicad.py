"""Build a KiCad 10 project from the reconstructed board model.

usage: python build_kicad.py <fpdlink2raspi dir> <output dir>
"""
import collections
import json
import math
import os
import re
import shutil
import sys

here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, here)
sys.path.insert(0, os.path.join(here, '..', 'pylib'))

import pcbnew
from shapely.geometry import Point, Polygon

import model

SRC, OUT = sys.argv[1], sys.argv[2]
NAME = 'fpdlink2raspi'
OX, OY = 100.0, 156.0          # Altium (0,0) -> KiCad (100, 156); board spans y 100..156
mm = pcbnew.FromMM


def V(x, y):
    return pcbnew.VECTOR2I(mm(OX + x), mm(OY - y))


def ang(deg):
    return pcbnew.EDA_ANGLE(deg, pcbnew.DEGREES_T)


LAYER = {'F.Cu': pcbnew.F_Cu, 'In1.Cu': pcbnew.In1_Cu, 'In2.Cu': pcbnew.In2_Cu, 'B.Cu': pcbnew.B_Cu}
USER_LAYERS = [  # (gerber key, KiCad layer, name)
    ('Mechanical_7', pcbnew.User_1, 'M07.Component_Outlines'),
    ('Mechanical_11', pcbnew.User_2, 'M11.Assy_RefDes'),
    ('Mechanical_12', pcbnew.User_3, 'M12.Assy_Value'),
    ('Mechanical_19', pcbnew.User_4, 'M19.RefDes_Outline'),
    ('Mechanical_21', pcbnew.User_5, 'M21.Mechanical_21'),
    ('Mechanical_22', pcbnew.User_6, 'M22.Assy_Value-B'),
    ('Mechanical_23', pcbnew.User_7, 'M23.RefDes_Outline-B'),
    ('Keep-out', pcbnew.User_8, 'Altium.Keep-Out'),
]

# Footprint names (project library) keyed by the Altium BOM LibRef
LIBREF_FP = [
    (r'^C0402', 'C_0402_1005Metric'), (r'^C0603', 'C_0603_1608Metric'), (r'^C0805', 'C_0805_2012Metric'),
    (r'^C1206', 'C_1206_3216Metric'), (r'^C1812', 'C_1812_4532Metric'),
    (r'^R0402', 'R_0402_1005Metric'), (r'^R0603', 'R_0603_1608Metric'),
    (r'^L0402', 'L_0402_1005Metric'), (r'^L0603', 'L_0603_1608Metric'),
    (r'^L0805_SHCOIL', 'L_Murata_DFE201612E'), (r'^L1212_SHCOIL', 'L_Murata_LQH3NP_3.0x3.0mm'),
    (r'^L_WE_74404064120', 'L_Wurth_WE-LQS-6045'),
    (r'^D_WE_150060', 'LED_0603_1608Metric'), (r'^D_TVS', 'D_SMB'), (r'^D_DIODES_B340LA', 'D_SMA'),
    (r'^J_PI_CAMV2', 'Amphenol_SFW15R-1STE1LF_1x15_P1.00mm_Horizontal'),
    (r'^J_CUI_PJ-075', 'CUI_PJ-075CH-SMT_DC_Jack'), (r'^J_WE_629105136821', 'Wurth_629105136821_USB_Micro-B'),
    (r'^Q_DIODES_BSS138', 'SOT-23'), (r'^U_TI_TCA9534APW', 'TSSOP-16_4.4x5mm_P0.65mm'),
    (r'^U_TI_TCA9406DCUR', 'TI_US8_DCU_2.3x2.0mm_P0.5mm'),
    (r'^U_TI_DS90UB954', 'VQFN-48-1EP_7x7mm_P0.5mm_EP4.1x4.1mm'),
    (r'^U_TI_TPS55340RTE', 'WQFN-16-1EP_3x3mm_P0.5mm_EP1.7x1.7mm'), (r'^U_TI_LM3671', 'SOT-23-5'),
    (r'^X_MOLEX_0732511350', 'Molex_73251-1350_SMA_Vertical'),
    (r'^J_3x1_2.54mm_header_SMD', 'TE_1241150-3_1x03_P2.54mm_SMD'),
    (r'^Y_SG-210STF', 'Oscillator_Epson_SG-210STF_2.5x2.0mm'),
]


def fp_name(c, bom, sig_names):
    rec = bom.get(c['ref'])
    if rec:
        for pat, n in LIBREF_FP:
            if re.search(pat, rec['LibRef']):
                return n
        return rec['LibRef']
    if c['ref'].startswith('TP'):
        p = c['pads'][0]
        if p['kind'] == 'PTH':
            return f'TestPoint_THTPad_D{model.ap_size(p["ap"])[0]:g}mm_Drill{p["drill"]:g}mm'
        return f'TestPoint_Pad_D{model.ap_size(p["ap"])[0]:g}mm'
    n = sig_names.get(signature(c))
    return n or f'{re.sub(r"[0-9]", "", c["ref"])}_unknown'


def signature(c):
    pre = re.sub(r'\d', '', c['ref'])
    pads = sorted((p['ap']['shape'], round(p['ap'].get('w', p['ap'].get('d', 0)), 3), round(p['ap'].get('h', 0), 3))
                  for p in c['pads'])
    if len(c['pads']) == 2:
        a, b = c['pads']
        pads.append(round(math.hypot(a['x'] - b['x'], a['y'] - b['y']), 3))
    return pre, tuple(pads)


def poly_to_sps(poly, close=True):
    """shapely Polygon (gerber coords) -> SHAPE_POLY_SET in KiCad coords."""
    sps = pcbnew.SHAPE_POLY_SET()
    sps.NewOutline()
    for x, y in list(poly.exterior.coords)[:-1]:
        sps.Append(V(x, y))
    for ring in poly.interiors:
        h = sps.NewHole()
        for x, y in list(ring.coords)[:-1]:
            v = V(x, y)
            sps.Append(v.x, v.y, -1, h)
    return sps


def sps_to_shapely(sps):
    out = []
    for i in range(sps.OutlineCount()):
        ol = sps.Outline(i)
        ext = [((ol.CPoint(k).x / 1e6) - OX, OY - (ol.CPoint(k).y / 1e6)) for k in range(ol.PointCount())]
        holes = []
        for h in range(sps.HoleCount(i)):
            hl = sps.CHole(i, h)
            holes.append([((hl.CPoint(k).x / 1e6) - OX, OY - (hl.CPoint(k).y / 1e6)) for k in range(hl.PointCount())])
        if len(ext) >= 3:
            out.append(Polygon(ext, holes).buffer(0))
    return out


def main():
    b = model.Board(SRC).run()
    bom = b.bom
    os.makedirs(OUT, exist_ok=True)
    pcb_path = os.path.join(OUT, NAME + '.kicad_pcb')
    for ext in ('.kicad_pcb', '.kicad_pro', '.kicad_prl'):
        if os.path.exists(os.path.join(OUT, NAME + ext)):
            os.remove(os.path.join(OUT, NAME + ext))
    board = pcbnew.NewBoard(pcb_path)
    board.SetCopperLayerCount(4)
    ls = board.GetEnabledLayers()
    for _, lay, _ in USER_LAYERS:
        ls.AddLayer(lay)
    ls.AddLayer(pcbnew.F_Fab); ls.AddLayer(pcbnew.B_Fab)
    ls.AddLayer(pcbnew.F_CrtYd); ls.AddLayer(pcbnew.B_CrtYd)
    board.SetEnabledLayers(ls)
    board.SetVisibleLayers(ls)
    for lay, name in ((pcbnew.F_Cu, 'L1-Top'), (pcbnew.In1_Cu, 'L2-Sig'), (pcbnew.In2_Cu, 'L3-Sig'),
                      (pcbnew.B_Cu, 'L4-Bottom')):
        board.SetLayerName(lay, name)
    for _, lay, name in USER_LAYERS:
        board.SetLayerName(lay, name)

    # ------------------------------------------------------------------ design settings
    ds = board.GetDesignSettings()
    masks = collections.Counter()
    for p in b.pads:
        for s, m in p.get('mask', {}).items():
            if m is not None and p['kind'] == 'SMD':
                masks[round(m, 4)] += 1
    mask_common = masks.most_common(1)[0][0]
    # Board-level expansion is 0 so that the untented vias open at exactly their diameter like in Altium
    # (KiCad cannot store a per-via mask margin); every pad carries its Altium expansion explicitly.
    mask_default = 0.0
    ds.m_SolderMaskExpansion = mm(mask_default)
    ds.m_SolderMaskMinWidth = 0
    ds.m_SolderPasteMargin = 0
    ds.m_TrackMinWidth = mm(0.1)
    ds.m_MinClearance = mm(0.1)
    ds.m_ViasMinSize = mm(0.3)
    ds.m_ViasMinAnnularWidth = mm(0.1)
    ds.m_MinThroughDrill = mm(0.15)
    ds.m_HoleToHoleMin = mm(0.2)
    ds.m_HoleClearance = mm(0.2)
    ds.SetBoardThickness(mm(1.598))
    trk_w = collections.Counter(round(t['w'], 4) for t in b.tracks).most_common(1)[0][0]
    via_d = collections.Counter(round(v['d'], 4) for v in b.vias).most_common(1)[0][0]
    nc = ds.m_NetSettings.GetDefaultNetclass()
    nc.SetClearance(mm(0.125))
    nc.SetTrackWidth(mm(trk_w))
    nc.SetViaDiameter(mm(via_d))
    nc.SetViaDrill(mm(0.15))
    print(f'most common pad mask expansion {mask_common} mm, track {trk_w}, via {via_d}/0.15')

    # ------------------------------------------------------------------ nets
    netinfo = {}
    def net(name):
        if not name:
            return None
        if name not in netinfo:
            ni = pcbnew.NETINFO_ITEM(board, name)
            board.Add(ni)
            netinfo[name] = ni
        return netinfo[name]
    for n in sorted(set(v for v in b.net_of.values() if v)):
        net(n)

    # ------------------------------------------------------------------ board outline
    prof = b.g['Profile']
    xs = [v for o in prof['objs'] for v in (o['x1'], o['x2'])]
    ys = [v for o in prof['objs'] for v in (o['y1'], o['y2'])]
    x0, x1, y0, y1 = round(min(xs), 3), round(max(xs), 3), round(min(ys), 3), round(max(ys), 3)
    for (ax, ay), (bx, by) in (((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))):
        s = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(V(ax, ay)); s.SetEnd(V(bx, by)); s.SetLayer(pcbnew.Edge_Cuts); s.SetWidth(mm(0.05))
        board.Add(s)
    board.GetDesignSettings().SetAuxOrigin(V(0, 0))
    board.GetDesignSettings().SetGridOrigin(V(0, 0))
    print(f'outline {x1 - x0} x {y1 - y0} mm')

    # ------------------------------------------------------------------ footprints
    sig_names = {}
    for c in b.pp.values():
        if c['ref'] in bom:
            sig_names.setdefault(signature(c), fp_name(c, bom, {}))
    fps = {}
    comps = list(b.pp.values()) + [dict(ref=m['ref'], side='T', x=m['pad']['x'], y=m['pad']['y'], rot=0,
                                         pads=[m['pad']], courtyard=None, mounting=True) for m in b.mounting]
    not_fitted = {r for (r, _) in b.pin_net if r not in set(model.read_pick_place(
        [os.path.join(SRC, 'Pick Place', f) for f in os.listdir(os.path.join(SRC, 'Pick Place'))][0]))}

    def make_pad(fp, p, side, c_rot=0):
        pad = pcbnew.PAD(fp)
        pad.SetNumber(p.get('number', '') or '')
        ap = p['ap'] if p['kind'] == 'SMD' else (p['ap_side'].get(side) or p['ap'])
        shape, size, orient = None, None, 0
        if p['kind'] == 'NPTH':
            d = p['drill']
            ap = {'shape': 'C', 'd': d}
        if ap['shape'] == 'C':
            shape, size = pcbnew.PAD_SHAPE_CIRCLE, (ap['d'], ap['d'])
        elif ap['shape'] == 'R':
            shape, size = pcbnew.PAD_SHAPE_RECTANGLE, (ap['w'], ap['h'])
        elif ap['shape'] == 'O':
            shape, size = pcbnew.PAD_SHAPE_OVAL, (ap['w'], ap['h'])
        elif ap['shape'] == 'RR':
            shape, size, orient = pcbnew.PAD_SHAPE_ROUNDRECT, (ap['w'], ap['h']), ap['rot']
        pad.SetPosition(V(p['x'], p['y']))
        if p.get('extra'):
            # custom pad: union of the main flash and the glued conductor flashes
            pad.SetShape(pcbnew.PAD_SHAPE_CUSTOM)
            pad.SetAnchorPadShape(pcbnew.PADSTACK.ALL_LAYERS, pcbnew.PAD_SHAPE_CIRCLE)
            a = min(size)
            pad.SetSize(pcbnew.VECTOR2I(mm(a), mm(a)))
            pad.SetOrientation(ang(0))
            geo = p['geom'].simplify(0.0005)
            pts = pcbnew.VECTOR_VECTOR2I()
            for x, y in list(geo.exterior.coords)[:-1]:
                pts.append(pcbnew.VECTOR2I(mm(x - p['x']), mm(-(y - p['y']))))
            pad.AddPrimitivePoly(pcbnew.PADSTACK.ALL_LAYERS, pts, 0, True)
            kicad_w = model.ap_size(ap)[0]
        else:
            pad.SetShape(shape)
            pad.SetSize(pcbnew.VECTOR2I(mm(size[0]), mm(size[1])))
            rel = (orient - c_rot) % 180 if shape != pcbnew.PAD_SHAPE_CIRCLE else 0
            pad.SetOrientation(ang((c_rot + rel) % 360))
            if shape == pcbnew.PAD_SHAPE_ROUNDRECT:
                pad.SetRoundRectRadiusRatio(min(ap['r'] / min(ap['w'], ap['h']), 0.5))
            kicad_w = model.ap_size(ap)[0]
        cu = pcbnew.F_Cu if side == 'T' else pcbnew.B_Cu
        mask_l = pcbnew.F_Mask if side == 'T' else pcbnew.B_Mask
        paste_l = pcbnew.F_Paste if side == 'T' else pcbnew.B_Paste
        lset = pcbnew.LSET()
        if p['kind'] == 'SMD':
            pad.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
            lset.AddLayer(cu)
            m = p['mask'].get(side)
            if m is not None:
                lset.AddLayer(mask_l)
                if abs(m - mask_default) > 1e-4:
                    pad.SetLocalSolderMaskMargin(mm(m))
            if p.get('paste') is not None:
                lset.AddLayer(paste_l)
                if abs(p['paste']) > 1e-4:
                    pad.SetLocalSolderPasteMargin(mm(p['paste']))
        else:
            pad.SetAttribute(pcbnew.PAD_ATTRIB_PTH if p['kind'] == 'PTH' else pcbnew.PAD_ATTRIB_NPTH)
            pad.SetDrillSize(pcbnew.VECTOR2I(mm(p['drill']), mm(p['drill'])))
            lset = pcbnew.LSET(pcbnew.PAD.PTHMask() if p['kind'] == 'PTH' else pcbnew.PAD.UnplatedHoleMask())
            lset.RemoveLayer(pcbnew.F_Paste); lset.RemoveLayer(pcbnew.B_Paste)
            margins = set()
            for s_, ml in (('T', pcbnew.F_Mask), ('B', pcbnew.B_Mask)):
                mapr = p.get('mask_ap', {}).get(s_)
                if mapr is None:
                    lset.RemoveLayer(ml)
                else:
                    lset.AddLayer(ml)
                    margins.add(round((model.ap_size(mapr)[0] - kicad_w) / 2, 4))
            if margins:
                m = max(margins)
                if abs(m - mask_default) > 1e-4:
                    pad.SetLocalSolderMaskMargin(mm(m))
        pad.SetLayerSet(lset)
        return pad

    for c in comps:
        ref = c['ref']
        if c.get('dnp') and len(c['pads']) == 2:
            # not-fitted parts have no pick&place rotation: orient so that pad 1 is at local -X
            p1 = next(p for p in c['pads'] if p['number'] == '1')
            p2 = next(p for p in c['pads'] if p['number'] == '2')
            a = math.degrees(math.atan2(p2['y'] - p1['y'], p2['x'] - p1['x']))
            c['rot'] = (round(a / 90) * 90 + (180 if c['side'] == 'B' else 0)) % 360
        fp = pcbnew.FOOTPRINT(board)
        name = 'MountingHole_3.4mm_NPTH' if c.get('mounting') else fp_name(c, bom, sig_names)
        fp.SetFPID(pcbnew.LIB_ID(NAME, name))
        fp.SetReference(ref)
        rec = bom.get(ref)
        value = rec['Name'] if rec else b.sch_values.get(ref, '')
        if c.get('mounting'):
            value = 'MountingHole'
        if ref.startswith('TP') and not value:
            value = 'TestPoint'
        fp.SetValue(value or '~')
        fp.SetPosition(V(c['x'], c['y']))
        board.Add(fp)
        if c['side'] == 'B':
            fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
        fp.SetOrientationDegrees(c['rot'])
        fab = pcbnew.F_Fab if c['side'] == 'T' else pcbnew.B_Fab
        for fld, vis in ((fp.Reference(), True), (fp.Value(), False)):
            fld.SetLayer(fab)
            fld.SetTextSize(pcbnew.VECTOR2I(mm(0.4), mm(0.4)))
            fld.SetTextThickness(mm(0.06))
            fld.SetPosition(V(c['x'], c['y']))
            fld.SetVisible(vis)
            fld.SetKeepUpright(True)
            fld.SetMirrored(c['side'] == 'B')
        has_th = False
        for p in c['pads']:
            side = c['side'] if p['kind'] == 'SMD' else 'T'
            if p['kind'] == 'SMD':
                side = p['side']
            pad = make_pad(fp, p, side, c['rot'])
            fp.Add(pad)
            pnet = b.net_for(('pad', p['id']))
            if pnet:
                pad.SetNet(net(pnet))
            has_th |= p['kind'] == 'PTH'
        attrs = 0
        if c.get('mounting'):
            fp.SetBoardOnly(True)
            fp.SetExcludedFromBOM(True)
            fp.SetExcludedFromPosFiles(True)
        else:
            attrs = pcbnew.FP_THROUGH_HOLE if has_th else pcbnew.FP_SMD
            fp.SetAttributes(attrs)
        if ref in not_fitted:
            fp.SetDNP(True)
            fp.SetExcludedFromBOM(True)
            fp.SetExcludedFromPosFiles(True)
        elif not rec and not c.get('mounting'):
            fp.SetExcludedFromBOM(True)
        if rec:
            for k, fname in (('Manufacturer 1', 'Manufacturer'), ('Manufacturer Part Number 1', 'MPN'),
                             ('Supplier 1', 'Supplier'), ('Supplier Part Number 1', 'Supplier PN'),
                             ('LibRef', 'Altium LibRef')):
                if rec.get(k):
                    fp.SetField(fname, rec[k])
                    f = fp.GetField(fname)
                    f.SetLayer(fab); f.SetVisible(False); f.SetPosition(V(c['x'], c['y']))
                    f.SetTextSize(pcbnew.VECTOR2I(mm(0.4), mm(0.4)))
            if rec.get('Description'):
                fp.GetField(pcbnew.FIELD_T_DESCRIPTION).SetText(rec['Description'])
        fps[ref] = (fp, c)

    # paste-only stencil apertures -> unnumbered SMD pads on paste layer in the owning footprint
    for f in b.paste_only:
        owner = min((fc for fc in fps.values() if fc[1]['side'] == f['side']),
                    key=lambda fc: math.hypot(fc[1]['x'] - f['x'], fc[1]['y'] - f['y']))
        fp = owner[0]
        pad = pcbnew.PAD(fp)
        pad.SetNumber('')
        pad.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
        ap = f['ap']
        pad.SetShape(pcbnew.PAD_SHAPE_RECTANGLE if ap['shape'] == 'R' else pcbnew.PAD_SHAPE_CIRCLE)
        w, h = model.ap_size(ap)
        pad.SetSize(pcbnew.VECTOR2I(mm(w), mm(h)))
        pad.SetPosition(V(f['x'], f['y']))
        ls_ = pcbnew.LSET(); ls_.AddLayer(pcbnew.F_Paste if f['side'] == 'T' else pcbnew.B_Paste)
        pad.SetLayerSet(ls_)
        fp.Add(pad)

    # footprint graphics: courtyards + assembly drawings
    def owner_of(x, y, side, maxd=3.0, pts=None):
        pts = [Point(px, py) for px, py in (pts or [(x, y)])]
        inside = [fc for fc in fps.values() if fc[1]['side'] == side and fc[1].get('courtyard') is not None
                  and all(fc[1]['courtyard'].buffer(0.03).contains(p) for p in pts)]
        if inside:
            return min(inside, key=lambda fc: (math.hypot(fc[1]['x'] - x, fc[1]['y'] - y)))[0]
        near = [fc for fc in fps.values() if fc[1]['side'] == side]
        fc = min(near, key=lambda fc: math.hypot(fc[1]['x'] - x, fc[1]['y'] - y))
        return fc[0] if math.hypot(fc[1]['x'] - x, fc[1]['y'] - y) <= maxd else None

    def add_shape(parent, o, layer):
        if o['kind'] == 'arc':
            s = pcbnew.PCB_SHAPE(parent, pcbnew.SHAPE_T_ARC)
            mx, my = model.arc_mid(o)
            if abs(o['x1'] - o['x2']) < 1e-6 and abs(o['y1'] - o['y2']) < 1e-6:
                s = pcbnew.PCB_SHAPE(parent, pcbnew.SHAPE_T_CIRCLE)
                s.SetCenter(V(o['cx'], o['cy']))
                s.SetEnd(V(o['x1'], o['y1']))
            else:
                s.SetArcGeometry(V(o['x1'], o['y1']), V(mx, my), V(o['x2'], o['y2']))
        else:
            s = pcbnew.PCB_SHAPE(parent, pcbnew.SHAPE_T_SEGMENT)
            s.SetStart(V(o['x1'], o['y1'])); s.SetEnd(V(o['x2'], o['y2']))
        s.SetLayer(layer)
        s.SetWidth(mm(o['w']))
        parent.Add(s)
        return s

    stats = collections.Counter()
    for fp, c in fps.values():
        if c.get('courtyard') is not None:
            w = c['courtyard_lines'][0]['w'] if c.get('courtyard_lines') else 0.05
            for poly in model.polys(c['courtyard']):
                sh = pcbnew.PCB_SHAPE(fp, pcbnew.SHAPE_T_POLY)
                ring = Polygon(poly.exterior).simplify(1e-4)
                sh.SetPolyShape(poly_to_sps(ring))
                sh.SetFilled(False)
                sh.SetWidth(mm(w))
                sh.SetLayer(pcbnew.F_CrtYd if c['side'] == 'T' else pcbnew.B_CrtYd)
                fp.Add(sh)
                stats['courtyard polygon:fp'] += 1
    for side, grp in b.loose_courtyard:
        # open courtyard-layer strokes are Altium component-centre crosses: keep them on the Fab layer
        for o in grp:
            fp = owner_of((o['x1'] + o['x2']) / 2, (o['y1'] + o['y2']) / 2, side, maxd=0.6)
            if fp is not None:
                add_shape(fp, o, pcbnew.F_Fab if side == 'T' else pcbnew.B_Fab)
                stats['courtyard centre-cross -> fp Fab'] += 1
            else:
                add_shape(board, o, pcbnew.Dwgs_User)
                stats['courtyard loose -> Dwgs.User'] += 1
    for key, side, layer, board_layer in (('Mechanical_13', 'T', pcbnew.F_Fab, pcbnew.F_Fab),
                                         ('Mechanical_14', 'B', pcbnew.B_Fab, pcbnew.B_Fab)):
        for o in b.mech(key):
            if o['kind'] == 'arc':
                mx, my = model.arc_mid(o)
            else:
                mx, my = (o['x1'] + o['x2']) / 2, (o['y1'] + o['y2']) / 2
            fp = owner_of(mx, my, side, pts=[(o['x1'], o['y1']), (o['x2'], o['y2'])])
            if fp is not None:
                add_shape(fp, o, layer)
                stats[key + ':fp'] += 1
            else:
                add_shape(board, o, layer if 'Fab' in board.GetLayerName(layer) else board_layer)
                stats[key + ':board'] += 1
    print('footprint graphics', dict(stats))

    for key, lay, _ in USER_LAYERS:
        for o in b.mech(key):
            add_shape(board, o, lay)

    # ------------------------------------------------------------------ copper
    from shapely.ops import unary_union
    cover = {}
    for cu in model.CU:
        cover[cu] = unary_union([g for k, l, g in b.cu_items if l == cu and k[0] != 'trk'] +
                                [t['geom'] for t in b.tracks if t['layer'] == cu and
                                 not (abs(t['x1'] - t['x2']) < 1e-6 and abs(t['y1'] - t['y2']) < 1e-6)])
    for i, t in enumerate(b.tracks):
        n = net(b.net_for(('trk', i)))
        if t['kind'] == 'line' and abs(t['x1'] - t['x2']) < 1e-6 and abs(t['y1'] - t['y2']) < 1e-6:
            if t['geom'].difference(cover[t['layer']]).area < 1e-4:
                stats['zero-length track dropped (covered)'] += 1
                continue
            s = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_CIRCLE)
            s.SetCenter(V(t['x1'], t['y1'])); s.SetEnd(V(t['x1'] + t['w'] / 2, t['y1']))
            s.SetFilled(True); s.SetWidth(0); s.SetLayer(LAYER[t['layer']])
            if n:
                s.SetNet(n)
            board.Add(s)
            stats['zero-length track -> copper dot'] += 1
            continue
        if t['kind'] == 'arc':
            tr = pcbnew.PCB_ARC(board)
            mx, my = model.arc_mid(t)
            tr.SetStart(V(t['x1'], t['y1'])); tr.SetMid(V(mx, my)); tr.SetEnd(V(t['x2'], t['y2']))
        else:
            if abs(t['x1'] - t['x2']) < 1e-6 and abs(t['y1'] - t['y2']) < 1e-6:
                stats['zero-length track'] += 1
            tr = pcbnew.PCB_TRACK(board)
            tr.SetStart(V(t['x1'], t['y1'])); tr.SetEnd(V(t['x2'], t['y2']))
        tr.SetWidth(mm(t['w']))
        tr.SetLayer(LAYER[t['layer']])
        if n:
            tr.SetNet(n)
        board.Add(tr)
    for i, v in enumerate(b.vias):
        via = pcbnew.PCB_VIA(board)
        via.SetViaType(pcbnew.VIATYPE_THROUGH)
        via.SetPosition(V(v['x'], v['y']))
        via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        via.SetWidth(mm(v['d']))
        via.SetDrill(mm(v['drill']))
        via.SetFrontTentingMode(pcbnew.TENTING_MODE_TENTED if v['tent']['T'] else pcbnew.TENTING_MODE_NOT_TENTED)
        via.SetBackTentingMode(pcbnew.TENTING_MODE_TENTED if v['tent']['B'] else pcbnew.TENTING_MODE_NOT_TENTED)
        n = net(b.net_for(('via', i)))
        if n:
            via.SetNet(n)
        board.Add(via)
    # pours: netted & >= 1 mm^2 -> filled zones; small netted (teardrops/necks) and net-less (copper text
    # plates) -> filled copper polygons
    zones = []
    for i, z in enumerate(b.pours):
        nname = b.net_for(('pour', i))
        geo = z['geom'].simplify(0.0005, preserve_topology=True)
        if nname and geo.area >= 1.0:
            zones.append((geo.area, i, nname, geo, z['layer']))
        else:
            s = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_POLY)
            sps = poly_to_sps(geo)
            s.SetPolyShape(sps)
            s.SetFilled(True)
            s.SetWidth(0)
            s.SetLayer(LAYER[z['layer']])
            if nname:
                s.SetNet(net(nname))
            board.Add(s)
            stats['copper polygon (net)' if nname else 'copper polygon (no net)'] += 1
    zones.sort()
    for prio, (area, i, nname, geo, layer) in enumerate(reversed(zones)):
        pass
    for rank, (area, i, nname, geo, layer) in enumerate(zones):
        zone = pcbnew.ZONE(board)
        zone.SetLayer(LAYER[layer])
        zone.SetNet(net(nname))
        ol = zone.Outline()
        ol.NewOutline()
        ext = Polygon(geo.exterior).simplify(0.001)
        for x, y in list(ext.exterior.coords)[:-1]:
            ol.Append(V(x, y))
        zone.SetAssignedPriority(len(zones) - rank)  # smaller zones win
        zone.SetMinThickness(mm(0.1))
        zone.SetLocalClearance(mm(0.125))
        zone.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
        zone.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_NEVER)
        fill = poly_to_sps(geo)
        fill.Fracture()
        zone.SetFilledPolysList(LAYER[layer], fill)
        zone.SetIsFilled(True)
        zone.SetNeedRefill(False)
        board.Add(zone)
        stats['zone ' + layer] += 1
    for g in b.cu_graphics:
        s = add_shape(board, g, LAYER[g['layer']])
        stats['copper graphic line'] += 1
    print('copper', dict(stats))

    # ------------------------------------------------------------------ make instances of a footprint identical
    canon = unify_footprints(board)

    # ------------------------------------------------------------------ save
    pcbnew.SaveBoard(pcb_path, board)
    write_stackup(pcb_path)
    # project-local footprint library
    lib = os.path.join(OUT, NAME + '.pretty')
    if os.path.isdir(lib):
        shutil.rmtree(lib)
    io = pcbnew.PCB_IO_MGR.FindPlugin(pcbnew.PCB_IO_MGR.KICAD_SEXP)
    os.makedirs(lib)
    done = set()
    board2 = pcbnew.LoadBoard(pcb_path)
    for fp in board2.GetFootprints():
        nm = str(fp.GetFPID().GetLibItemName())
        if nm in done or fp.GetReference() != canon[nm]:
            continue
        done.add(nm)
        f2 = normalized(fp)
        f2.SetReference('REF**')
        if not f2.IsBoardOnly():
            f2.SetExcludedFromBOM(False)
            f2.SetExcludedFromPosFiles(False)
        for pad in f2.Pads():
            pad.SetNetCode(0)
        f2.SetDNP(False)
        io.FootprintSave(lib, f2)
    with open(os.path.join(OUT, 'fp-lib-table'), 'w') as f:
        f.write('(fp_lib_table\n  (version 7)\n  (lib (name "%s")(type "KiCad")(uri "${KIPRJMOD}/%s.pretty")(options "")'
                '(descr "Footprints reconstructed from the Altium Gerber X2 output"))\n)\n' % (NAME, NAME))
    print('saved', pcb_path, 'footprints in lib:', len(done))

    # reports for the README
    with open(os.path.join(OUT, '_build_report.json'), 'w') as f:
        json.dump(dict(warnings=b.log, net_report=b.net_report, net_mismatch=b.net_mismatch,
                       not_fitted=sorted(not_fitted), stats=stats), f, indent=1)


def normalized(fp):
    f2 = pcbnew.FOOTPRINT(fp)
    if f2.IsFlipped():
        f2.Flip(f2.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
    f2.SetOrientationDegrees(0)
    f2.SetPosition(pcbnew.VECTOR2I(0, 0))
    return f2


def _r(v):
    return (round(v.x / 1000), round(v.y / 1000))


def pad_sig(f2):
    return tuple(sorted((p.GetNumber(), _r(p.GetPosition()), round(p.GetOrientation().AsDegrees()) % 180,
                         _r(p.GetSize()), int(p.GetShape()), str(p.GetLocalSolderMaskMargin()),
                         str(p.GetLocalSolderPasteMargin()), p.GetLayerSet().FmtHex()) for p in f2.Pads()))


def gfx_sig(f2):
    out = []
    for g in f2.GraphicalItems():
        if g.GetClass() != 'PCB_SHAPE':
            continue
        if g.GetShape() == pcbnew.SHAPE_T_POLY:
            ol = g.GetPolyShape().Outline(0)
            pts = tuple(sorted(_r(ol.CPoint(i)) for i in range(ol.PointCount())))
        else:
            pts = tuple(sorted([_r(g.GetStart()), _r(g.GetEnd())]))
        out.append((g.GetLayer(), int(g.GetShape()), pts, round(g.GetWidth() / 1000)))
    return tuple(sorted(out))


def unify_footprints(board):
    """Within each footprint name, copy the most common graphics set to all instances with identical pads.

    Gerbers do not say which assembly/courtyard stroke belongs to which part where parts overlap, so a few
    instances pick up a neighbour's stroke. Instances whose *pads* differ get a variant name instead."""
    groups = collections.defaultdict(list)
    for fp in board.GetFootprints():
        groups[str(fp.GetFPID().GetLibItemName())].append(fp)
    canon = {}
    fixed = renamed = 0
    for name, fps in groups.items():
        norm = {fp.GetReference(): normalized(fp) for fp in fps}
        by_pads = collections.defaultdict(list)
        for fp in fps:
            by_pads[pad_sig(norm[fp.GetReference()])].append(fp)
        variants = sorted(by_pads.values(), key=len, reverse=True)
        for vi, members in enumerate(variants):
            vname = name if vi == 0 else f'{name}_v{vi + 1}'
            if vi:
                for fp in members:
                    fp.SetFPID(pcbnew.LIB_ID(NAME, vname))
                renamed += len(members)
            gs = collections.Counter(gfx_sig(norm[fp.GetReference()]) for fp in members)
            best = gs.most_common(1)[0][0]
            ref_fp = min((fp for fp in members if gfx_sig(norm[fp.GetReference()]) == best),
                         key=lambda fp: (fp.IsDNP(), fp.GetReference()))
            canon[vname] = ref_fp.GetReference()
            src = norm[ref_fp.GetReference()]
            for fp in members:
                if gfx_sig(norm[fp.GetReference()]) == best:
                    continue
                for g in list(fp.GraphicalItems()):
                    if g.GetClass() == 'PCB_SHAPE':
                        fp.Remove(g)
                t = pcbnew.FOOTPRINT(src)
                if fp.IsFlipped():
                    t.Flip(t.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
                t.SetOrientation(fp.GetOrientation())
                t.SetPosition(fp.GetPosition())
                for g in t.GraphicalItems():
                    if g.GetClass() == 'PCB_SHAPE':
                        fp.Add(g.Duplicate().Cast())
                fixed += 1
    print(f'footprint instances normalised: {fixed} graphics replaced, {renamed} moved to pad variants')
    return canon


def write_stackup(path):
    s = open(path).read()
    stack = '''		(stackup
			(layer "F.SilkS" (type "Top Silk Screen"))
			(layer "F.Paste" (type "Top Solder Paste"))
			(layer "F.Mask" (type "Top Solder Mask") (thickness 0.01))
			(layer "F.Cu" (type "copper") (thickness 0.035))
			(layer "dielectric 1" (type "prepreg") (thickness 0.119) (material "FR4") (epsilon_r 4.5) (loss_tangent 0.02))
			(layer "In1.Cu" (type "copper") (thickness 0.035))
			(layer "dielectric 2" (type "core") (thickness 1.2) (material "FR4") (epsilon_r 4.5) (loss_tangent 0.02))
			(layer "In2.Cu" (type "copper") (thickness 0.035))
			(layer "dielectric 3" (type "prepreg") (thickness 0.119) (material "FR4") (epsilon_r 4.5) (loss_tangent 0.02))
			(layer "B.Cu" (type "copper") (thickness 0.035))
			(layer "B.Mask" (type "Bottom Solder Mask") (thickness 0.01))
			(layer "B.Paste" (type "Bottom Solder Paste"))
			(layer "B.SilkS" (type "Bottom Silk Screen"))
			(dielectric_constraints no)
		)
'''
    if '(stackup' not in s:
        s = s.replace('\t(setup\n', '\t(setup\n' + stack, 1)
        open(path, 'w').write(s)


if __name__ == '__main__':
    main()
