#!/usr/bin/env python3
"""Fix a Vector Map Builder lanelet2 export (campus_ost) for Autoware, and check the result.

Run in the Autoware container (needs the lanelet2 Python bindings), from the repo root:
  python3 scripts/fix_lanelet_map.py <map_dir>            check, fix, write <map_dir>/lanelet2_map.osm
                                                           (the original is kept as lanelet2_map.osm.orig)
  python3 scripts/fix_lanelet_map.py <map_dir> --check    only report
The fix always starts from lanelet2_map.osm.orig if it exists, so running it twice gives the same map.

Fixes (see docs/car-integration.md, "Lanelet map"):
1. Narrow two-way roads modelled as two opposite lanelets over the same full-width strip (no middle line):
   split at the middle. Both lanelets share the new middle line (type virtual, like the map's other middle lines),
   so Autoware's avoidance can use the oncoming lane (use_lane_type: opposite_direction_lane needs a shared line).
   Lanelets before and after the strip that started at its outer edge are moved onto the middle node.
2. Opposite lanelets whose middle lines are separate ways (0.2-1 m apart, crossing each other in curves): replaced
   by one shared middle line halfway between them; the end nodes are merged.
3. Turning loops whose inner bound is a closed ring (start = end point, Autoware finds no route into them):
   split into two lanelets.
4. one_way=no on a lanelet that has its own opposite lanelet next to it: one_way=yes.
"""
import argparse
import math
import os
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np
import lanelet2
import lanelet2.geometry as G
from lanelet2.io import Origin
from lanelet2.projection import UtmProjector

MERGE_MAX_GAP_M = 1.2        # 2.: facing middle lines closer than this (median) ...
MERGE_MAX_OVERLAP_M = 2.5    #     ... or, if the two lanelets overlap (conflicting), this ...
MERGE_MIN_COVERAGE = 0.5     # ... over at least this share of both lines ...
MERGE_MAX_END_GAP_M = 2.0    # ... whose ends are at most this far apart
SAME_STRIP_TOL_M = 0.3       # 1.: bounds of the two lanelets coincide within this, or ...
STRIP_END_TOL_M = 2.5        #     they overlap and cover the same road section (centerline ends this close)
REUSE_NODE_TOL_M = 0.6       # 1.: an existing node this close to the new middle end is reused
MIDLINE_SPACING_M = 1.0
SIMPLIFY_TOL_M = 0.03


# ---------------------------------------------------------------- geometry helpers (local metric frame)
class Frame:
    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0
        self.kx = math.radians(1.0) * 6378137.0 * math.cos(math.radians(lat0))
        self.ky = math.radians(1.0) * 6378137.0

    def xy(self, lat, lon):
        return np.array([(lon - self.lon0) * self.kx, (lat - self.lat0) * self.ky])

    def latlon(self, p):
        return self.lat0 + p[1] / self.ky, self.lon0 + p[0] / self.kx


def arclen(p):
    return np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))))


def resample_frac(p, n):
    s = arclen(p)
    t = np.linspace(0.0, s[-1], n)
    return np.column_stack((np.interp(t, s, p[:, 0]), np.interp(t, s, p[:, 1])))


def dist_to_polyline(pts, line):
    a, b = line[:-1], line[1:]
    d = b - a
    L = np.maximum((d * d).sum(1), 1e-12)
    u = np.clip(((pts[:, None, :] - a[None]) * d[None]).sum(2) / L[None], 0.0, 1.0)
    c = a[None] + u[..., None] * d[None]
    return np.sqrt(((pts[:, None, :] - c) ** 2).sum(2)).min(1)


def simplify(p, tol):
    """Douglas-Peucker, keeps the end points."""
    if len(p) < 3:
        return p
    a, b = p[0], p[-1]
    ab = b - a
    n = np.linalg.norm(ab)
    d = (np.abs(np.cross(ab, p - a)) / n) if n > 1e-9 else np.linalg.norm(p - a, axis=1)
    i = int(np.argmax(d))
    if d[i] <= tol:
        return np.array([a, b])
    return np.vstack((simplify(p[:i + 1], tol)[:-1], simplify(p[i:], tol)))


# ---------------------------------------------------------------- OSM document
class Osm:
    def __init__(self, path):
        self.tree = ET.parse(path)
        self.root = self.tree.getroot()
        self.nodes = {n.get("id"): n for n in self.root.findall("node")}
        self.ways = {w.get("id"): w for w in self.root.findall("way")}
        self.rels = {r.get("id"): r for r in self.root.findall("relation")}
        self.next_id = min(int(i) for i in list(self.nodes) + list(self.ways) + list(self.rels)) - 1

    def new_id(self):
        i = self.next_id
        self.next_id -= 1
        return str(i)

    def refs(self, wid):
        return [nd.get("ref") for nd in self.ways[wid].findall("nd")]

    def set_refs(self, wid, refs):
        w = self.ways[wid]
        for nd in w.findall("nd"):
            w.remove(nd)
        tags = list(w)
        for t in tags:
            w.remove(t)
        for r in refs:
            ET.SubElement(w, "nd", ref=r)
        for t in tags:
            w.append(t)

    def tag(self, el, k, v):
        for t in el.findall("tag"):
            if t.get("k") == k:
                t.set("v", v)
                return
        ET.SubElement(el, "tag", k=k, v=v)

    def add_node(self, lat, lon, like=None):
        nid = self.new_id()
        n = ET.Element("node", id=nid, lat=f"{lat:.11f}", lon=f"{lon:.11f}")
        if like is not None:  # VMB keeps local_x/local_y/ele tags; interpolate them like lat/lon
            for t in like:
                ET.SubElement(n, "tag", k=t[0], v=f"{t[1]:.4f}")
        self.root.append(n)
        self.nodes[nid] = n
        return nid

    def move_node(self, nid, lat, lon, local=None):
        n = self.nodes[nid]
        n.set("lat", f"{lat:.11f}")
        n.set("lon", f"{lon:.11f}")
        if local:
            for k, v in local:
                self.tag(n, k, f"{v:.4f}")

    def local_tags(self, nid):
        out = []
        for t in self.nodes[nid].findall("tag"):
            try:
                out.append((t.get("k"), float(t.get("v"))))
            except ValueError:
                pass
        return out

    def add_way(self, refs, tags):
        wid = self.new_id()
        w = ET.Element("way", id=wid)
        for r in refs:
            ET.SubElement(w, "nd", ref=r)
        for k, v in tags.items():
            ET.SubElement(w, "tag", k=k, v=v)
        # ways must come before relations in the file
        first_rel = next(iter(self.rels.values()), None)
        idx = list(self.root).index(first_rel) if first_rel is not None else len(self.root)
        self.root.insert(idx, w)
        self.ways[wid] = w
        return wid

    def bound(self, lid, role):
        return next(m.get("ref") for m in self.rels[lid].findall("member") if m.get("role") == role)

    def set_bound(self, lid, role, wid):
        next(m for m in self.rels[lid].findall("member") if m.get("role") == role).set("ref", wid)

    def replace_node_everywhere(self, old, new):
        for w in self.ways.values():
            for nd in w.findall("nd"):
                if nd.get("ref") == old:
                    nd.set("ref", new)

    def drop_unused(self):
        used_w = {m.get("ref") for r in self.rels.values() for m in r.findall("member") if m.get("type") == "way"}
        for wid in [w for w in self.ways if w not in used_w and not self.ways[w].findall("tag")]:
            self.root.remove(self.ways.pop(wid))
        used_n = {nd.get("ref") for w in self.ways.values() for nd in w.findall("nd")}
        used_n |= {m.get("ref") for r in self.rels.values() for m in r.findall("member") if m.get("type") == "node"}
        for nid in [n for n in self.nodes if n not in used_n]:
            self.root.remove(self.nodes.pop(nid))

    def write(self, path):
        ET.indent(self.tree, space="  ")
        self.tree.write(path, encoding="UTF-8", xml_declaration=True)


# ---------------------------------------------------------------- lanelet2 view of the map
def load(path, origin):
    if not path.endswith(".osm"):  # lanelet2 only reads *.osm (the backup is *.osm.orig)
        tmp = os.path.join(os.path.dirname(path), ".fix_lanelet_map_tmp.osm")
        shutil.copy2(path, tmp)
        try:
            return load(tmp, origin)
        finally:
            os.remove(tmp)
    m, errors = lanelet2.io.loadRobust(path, UtmProjector(Origin(*origin)))
    rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                          lanelet2.traffic_rules.Participants.Vehicle)
    return m, lanelet2.routing.RoutingGraph(m, rules), errors


def pts(ls):
    return np.array([[p.x, p.y] for p in ls])


def ids(ls):
    return [str(p.id) for p in ls]


def width(ll):
    left, right = pts(ll.leftBound), pts(ll.rightBound)
    return float(np.median(dist_to_polyline(left, right)))


def heading_diff(a, b):
    ca, cb = pts(a.centerline), pts(b.centerline)
    da, db = ca[-1] - ca[0], cb[-1] - cb[0]
    return abs(math.degrees(math.atan2(np.cross(da, db), np.dot(da, db))))


def bound_users(m):
    """way id -> lanelets using it as left or right bound (lanelet2's findUsages misses inverted bounds)."""
    users = {}
    for ll in m.laneletLayer:
        for b in (ll.leftBound, ll.rightBound):
            users.setdefault(b.id, []).append(ll)
    return users


def opposite_conflicts(g, m):
    out = set()
    for ll in m.laneletLayer:
        for c in g.conflicting(ll):
            o = c.lanelet() if hasattr(c, "lanelet") else c
            if heading_diff(ll, o) > 150.0:
                out.add(tuple(sorted((ll.id, o.id))))
    return out


def in_junction(g, ll):
    """Part of a fork or merge: another lanelet leaves the same predecessor or enters the same successor."""
    return (any(len(g.following(p)) > 1 for p in g.previous(ll))
            or any(len(g.previous(f)) > 1 for f in g.following(ll)))


def report(m, g, errors, title):
    lls = list(m.laneletLayer)
    # opposite lanes lying on each other; inside junctions turning lanelets overlap by design
    both = sorted(opposite_conflicts(g, m))
    junction = [p for p in both if all(in_junction(g, m.laneletLayer[x]) for x in p)]
    parallel = [p for p in both if p not in junction]
    degenerate = [ll.id for ll in lls for b in (ll.leftBound, ll.rightBound) if b[0].id == b[-1].id]
    two_way = [ll.id for ll in lls if "one_way" in ll.attributes and ll.attributes["one_way"] == "no"]
    users = bound_users(m)
    shared_opposite = sum(1 for ll in lls if any(o.id != ll.id for o in users[ll.leftBound.id]))
    widths = [width(ll) for ll in lls]
    reach = min(len(g.reachableSet(ll, 5000.0, 0)) for ll in lls)
    print(f"--- {title}")
    print(f"  lanelets {len(lls)}, load errors {len(errors)}, graph errors {len(g.checkValidity())}")
    print(f"  opposite lanelets lying on each other outside junctions: {len(parallel)} {parallel}")
    print(f"  ... inside junctions (turning lanelets, expected): {len(junction)}")
    print(f"  bounds with start = end point: {degenerate}")
    print(f"  two-way lanelets: {two_way}")
    print(f"  lanelets whose left bound is shared with an opposite lane (avoidance can use it): {shared_opposite}")
    print(f"  width: min {min(widths):.2f} m, median {np.median(widths):.2f} m; smallest reachable set {reach}")
    return dict(parallel=parallel, degenerate=degenerate, two_way=two_way)


# ---------------------------------------------------------------- fixes
def fix(osm, m, g, frame, phase):
    """One pass: phase "one_way", "strips", "merge" or "loops" (the map is reloaded between passes)."""
    # VMB writes local_x / local_y / ele on every node: an affine fit lat/lon -> local gives them to new nodes
    all_ids = [i for i in osm.nodes if {"local_x", "local_y"} <= set(dict(osm.local_tags(i)))]
    X = np.array([[*frame.xy(float(osm.nodes[i].get("lat")), float(osm.nodes[i].get("lon"))), 1.0] for i in all_ids])
    Y = np.array([[dict(osm.local_tags(i))["local_x"], dict(osm.local_tags(i))["local_y"]] for i in all_ids])
    affine = np.linalg.lstsq(X, Y, rcond=None)[0] if len(all_ids) >= 3 else None

    def local_of(p):
        if affine is None:
            return None
        lx, ly = np.array([p[0], p[1], 1.0]) @ affine
        return [("local_x", lx), ("local_y", ly), ("ele", 0.0)]

    def P(nid):
        n = osm.nodes[nid]
        return frame.xy(float(n.get("lat")), float(n.get("lon")))

    def mid_node(a, b, reuse=None):
        p = 0.5 * (P(a) + P(b))
        if reuse:
            for cand in reuse:
                if np.linalg.norm(P(cand) - p) < REUSE_NODE_TOL_M:
                    return cand
        return osm.add_node(*frame.latlon(p), like=local_of(p))

    def midline(left_ids, right_ids, start, end):
        """Node ids of a line halfway between two polylines given in the same direction."""
        L = np.array([P(i) for i in left_ids])
        R = np.array([P(i) for i in right_ids])
        n = max(2, int(max(arclen(L)[-1], arclen(R)[-1]) / MIDLINE_SPACING_M) + 1)
        mid = simplify(0.5 * (resample_frac(L, n) + resample_frac(R, n)), SIMPLIFY_TOL_M)
        inner = [osm.add_node(*frame.latlon(p), like=local_of(p)) for p in mid[1:-1]]
        return [start] + inner + [end]

    lls = {str(ll.id): ll for ll in m.laneletLayer}
    users = bound_users(m)
    overlapping = {frozenset(str(x) for x in pair) for pair in opposite_conflicts(g, m)}
    log = []

    # 4. one_way=no next to its own opposite lane
    for lid, ll in (lls.items() if phase == "one_way" else ()):
        if "one_way" not in ll.attributes or ll.attributes["one_way"] != "no":
            continue
        partners = [o for o in users[ll.leftBound.id] if o.id != ll.id]
        if any(heading_diff(ll, o) > 150.0 for o in partners):
            osm.tag(osm.rels[lid], "one_way", "yes")
            log.append(f"one_way=yes on {lid} (opposite lane {[o.id for o in partners]} shares its left bound)")

    # 1. narrow two-way roads: two opposite lanelets on the same strip
    strips, done, seen_pairs = [], set(), set()
    for lid, a in (lls.items() if phase == "strips" else ()):
        for oid, b in lls.items():
            if oid == lid or frozenset((lid, oid)) in seen_pairs:
                continue
            seen_pairs.add(frozenset((lid, oid)))
            if lid in done or oid in done or heading_diff(a, b) < 150.0:
                continue
            same = {a.leftBound.id, a.rightBound.id} == {b.leftBound.id, b.rightBound.id}
            if not same and frozenset((lid, oid)) in overlapping:
                ca, cb = pts(a.centerline), pts(b.centerline)
                same = (np.linalg.norm(ca[0] - cb[-1]) < STRIP_END_TOL_M
                        and np.linalg.norm(ca[-1] - cb[0]) < STRIP_END_TOL_M)
            if not same:
                d1 = np.median(dist_to_polyline(pts(a.leftBound), pts(b.rightBound)))
                d2 = np.median(dist_to_polyline(pts(a.rightBound), pts(b.leftBound)))
                c1 = (dist_to_polyline(pts(b.rightBound), pts(a.leftBound)) < SAME_STRIP_TOL_M).mean()
                same = d1 < SAME_STRIP_TOL_M and d2 < SAME_STRIP_TOL_M and c1 > 0.8
            if same:
                strips.append((lid, oid))
                done |= {lid, oid}

    # connectivity by node ids, in lanelet direction: (left, right) start / end node of every lanelet
    ends = {lid: ((ids(ll.leftBound)[0], ids(ll.rightBound)[0]), (ids(ll.leftBound)[-1], ids(ll.rightBound)[-1]))
            for lid, ll in lls.items()}
    split_ids = {x for pair in strips for x in pair}
    mid_cache = {}

    def strip_end_node(left_end, right_end, neighbours):
        key = frozenset((left_end, right_end))
        if key not in mid_cache:
            reuse = [n for nb in neighbours for n in (ends[nb][0] + ends[nb][1])]
            mid_cache[key] = mid_node(left_end, right_end, reuse)
        return mid_cache[key]

    def bound_node_ids(lid, role):
        """Node ids of a bound in lanelet direction (lanelet2 may have inverted the OSM way)."""
        return ids(lls[lid].leftBound if role == "left" else lls[lid].rightBound)

    def repoint(lid, role, old, new):
        """Move the start/end of a neighbour's bound from the strip's outer edge to the middle node."""
        wid = osm.bound(lid, role)
        refs = osm.refs(wid)
        if refs[0] == old:
            refs[0] = new
        elif refs[-1] == old:
            refs[-1] = new
        else:
            return False
        osm.set_refs(wid, refs)  # in place: a shared middle line stays shared (its other user moves along)
        return True

    for a_id, b_id in strips:
        a = lls[a_id]
        # the road lies between the two outer edges: a's right and b's right (in a's direction: left edge).
        # On an exact strip b's right is a's left.
        a_left_old = bound_node_ids(a_id, "left")
        left, right = bound_node_ids(b_id, "right")[::-1], bound_node_ids(a_id, "right")
        preds_a = [x for x in lls if ends[x][1] == ends[a_id][0] and x not in split_ids]
        succs_a = [x for x in lls if ends[x][0] == ends[a_id][1] and x not in split_ids]
        preds_b = [x for x in lls if ends[x][1] == ends[b_id][0] and x not in split_ids]
        succs_b = [x for x in lls if ends[x][0] == ends[b_id][1] and x not in split_ids]
        m_start = strip_end_node(left[0], right[0], preds_a + succs_b)
        m_end = strip_end_node(left[-1], right[-1], succs_a + preds_b)
        mid = midline(left, right, m_start, m_end)
        mid_way = osm.add_way(mid, {"type": "virtual"})
        osm.set_bound(a_id, "left", mid_way)
        osm.set_bound(b_id, "left", mid_way)
        moved = []
        # a: its left edge becomes the middle. Neighbours in a's direction must follow.
        for x in preds_a:
            moved += [x] if repoint(x, "left", a_left_old[0], m_start) else []
        for x in succs_a:
            moved += [x] if repoint(x, "left", a_left_old[-1], m_end) else []
        # b: its left edge was a's right edge
        b_left_old = bound_node_ids(b_id, "left")
        for x in preds_b:
            moved += [x] if repoint(x, "left", b_left_old[0], m_end) else []
        for x in succs_b:
            moved += [x] if repoint(x, "left", b_left_old[-1], m_start) else []
        log.append(f"split strip {a_id} / {b_id} ({G.length2d(a):.0f} m, {width(a):.2f} m wide): "
                   f"middle line {mid_way}, neighbours moved onto the middle: {moved}")

    # 2. opposite lanelets with separate, nearly coincident middle lines
    merged = set()
    for a_id, a in (lls.items() if phase == "merge" else ()):
        for b_id, b in lls.items():
            if b_id <= a_id or a_id in split_ids or b_id in split_ids or {a_id, b_id} & merged:
                continue
            if a.leftBound.id == b.leftBound.id or heading_diff(a, b) < 150.0:
                continue
            la, lb = pts(a.leftBound), pts(b.leftBound)[::-1]  # b's left in a's direction
            if np.linalg.norm(la.mean(0) - lb.mean(0)) > 100.0:
                continue
            dab, dba = dist_to_polyline(la, lb), dist_to_polyline(lb, la)
            gap = MERGE_MAX_OVERLAP_M if frozenset((a_id, b_id)) in overlapping else MERGE_MAX_GAP_M
            if (np.median(dab) > gap or (dab < gap).mean() < MERGE_MIN_COVERAGE
                    or (dba < gap).mean() < MERGE_MIN_COVERAGE):
                continue
            end_gap = max(np.linalg.norm(la[0] - lb[0]), np.linalg.norm(la[-1] - lb[-1]))
            if end_gap > MERGE_MAX_END_GAP_M:
                continue
            a_left, b_left = bound_node_ids(a_id, "left"), bound_node_ids(b_id, "left")[::-1]
            starts, stops = (a_left[0], b_left[0]), (a_left[-1], b_left[-1])
            for keep, drop in (starts, stops):
                if keep != drop:
                    p = 0.5 * (P(keep) + P(drop))
                    osm.move_node(keep, *frame.latlon(p), local_of(p))
                    osm.replace_node_everywhere(drop, keep)
            mid_way = osm.add_way(midline(a_left, b_left, starts[0], stops[0]), {"type": "virtual"})
            osm.set_bound(a_id, "left", mid_way)
            osm.set_bound(b_id, "left", mid_way)
            merged |= {a_id, b_id}
            log.append(f"shared middle line for {a_id} / {b_id}: were {np.median(dab):.2f} m apart (max "
                       f"{dab.max():.2f} m), ends {end_gap:.2f} m apart -> way {mid_way}")

    # 3. closed inner bound (turning loop): split the lanelet in two
    for lid, ll in (lls.items() if phase == "loops" else ()):
        for role in ("left", "right"):
            b = ids(ll.leftBound if role == "left" else ll.rightBound)
            if b[0] != b[-1]:
                continue
            other_role = "right" if role == "left" else "left"
            o = bound_node_ids(lid, other_role)
            O = np.array([P(i) for i in o])
            j = int(np.argmin(np.abs(arclen(O) - arclen(O)[-1] / 2.0)))
            B = np.array([P(i) for i in b])
            i = int(np.argmin(np.linalg.norm(B[1:-1] - O[j], axis=1))) + 1
            tags = {t.get("k"): t.get("v") for t in osm.ways[osm.bound(lid, role)].findall("tag")}
            otags = {t.get("k"): t.get("v") for t in osm.ways[osm.bound(lid, other_role)].findall("tag")}
            w1, w2 = osm.add_way(b[:i + 1], tags), osm.add_way(b[i:], tags)
            o1, o2 = osm.add_way(o[:j + 1], otags), osm.add_way(o[j:], otags)
            osm.set_bound(lid, role, w1)
            osm.set_bound(lid, other_role, o1)
            new = ET.Element("relation", id=osm.new_id())
            for r, w in ((role, w2), (other_role, o2)):
                ET.SubElement(new, "member", type="way", role=r, ref=w)
            for t in osm.rels[lid].findall("tag"):
                ET.SubElement(new, "tag", k=t.get("k"), v=t.get("v"))
            osm.root.append(new)
            osm.rels[new.get("id")] = new
            log.append(f"split turning loop {lid} (closed {role} bound) into {lid} + {new.get('id')}")
            break

    osm.drop_unused()
    return log


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map_dir")
    ap.add_argument("--check", action="store_true", help="only report, write nothing")
    a = ap.parse_args()
    path = os.path.join(a.map_dir, "lanelet2_map.osm")
    orig = path + ".orig"
    src = orig if os.path.exists(orig) else path
    origin = None
    with open(os.path.join(a.map_dir, "map_projector_info.yaml")) as f:
        vals = {}
        for line in f:
            line = line.split("#")[0]
            if ":" in line:
                k, v = line.split(":", 1)
                vals[k.strip()] = v.strip().strip('"')
        origin = (float(vals["latitude"]), float(vals["longitude"]))

    m, g, errors = load(src, origin)
    before = report(m, g, errors, f"before ({src})")
    if a.check:
        return 0

    tmp = os.path.join(a.map_dir, "lanelet2_map.fixed.osm")
    cur, mc, gc = src, m, g
    for phase in ("one_way", "strips", "merge", "loops"):
        osm = Osm(cur)
        for line in fix(osm, mc, gc, Frame(*origin), phase):
            print("  " + line)
        osm.write(tmp)
        cur = tmp
        mc, gc, _ = load(cur, origin)

    m2, g2, errors2 = load(tmp, origin)
    after = report(m2, g2, errors2, "after")
    # every successor link of the original map must still exist (a split loop's links go through its new half)
    lost = []
    for ll in m.laneletLayer:
        if ll.id not in m2.laneletLayer:
            continue
        nxt = list(g2.following(m2.laneletLayer[ll.id]))
        reach2 = {x.id for x in nxt} | {y.id for x in nxt for y in g2.following(x)}  # 2 steps: split loops
        lost += [(ll.id, f.id) for f in g.following(ll) if f.id not in reach2]
    print(f"  successor links lost: {lost}")
    if errors2 or lost or after["parallel"] or after["degenerate"]:
        print("NOT written: the fixed map still has problems (see above). Kept as " + tmp)
        return 1
    if not os.path.exists(orig):
        shutil.copy2(path, orig)
    os.replace(tmp, path)
    print(f"written {path} (original: {orig})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
