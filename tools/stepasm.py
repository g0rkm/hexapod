# -*- coding: utf-8 -*-
"""Minimal STEP AP214 assembly parser: product tree with resolved placements."""
import re, math

WANT = ('PRODUCT(', 'PRODUCT_DEFINITION(', 'PRODUCT_DEFINITION_FORMATION',
        'PRODUCT_DEFINITION_SHAPE(', 'NEXT_ASSEMBLY_USAGE_OCCURRENCE(',
        'CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(', 'ITEM_DEFINED_TRANSFORMATION(',
        'REPRESENTATION_RELATIONSHIP', 'AXIS2_PLACEMENT_3D(')

SIMPLE = re.compile(r"^#(\d+)\s*=\s*([A-Z_0-9]+)\s*\((.*)\)\s*;\s*$", re.S)
COMPLEX = re.compile(r"^#(\d+)\s*=\s*\((.*)\)\s*;\s*$", re.S)


def _iter_stmts(path):
    buf = ''
    with open(path, 'r', encoding='utf-8', errors='replace') as f:
        for line in f:
            buf += line.strip()
            if buf.endswith(';'):
                yield buf
                buf = ''


def load(path, want=WANT):
    ents = {}
    for s in _iter_stmts(path):
        if not s.startswith('#'):
            continue
        if want and not any(w in s for w in want):
            continue
        m = SIMPLE.match(s)
        if m:
            ents[int(m.group(1))] = (m.group(2), m.group(3))
            continue
        m = COMPLEX.match(s)
        if m:
            ents[int(m.group(1))] = ('COMPLEX', m.group(2))
    return ents


def fetch(path, need):
    """Second pass: pull specific entity ids (points/directions)."""
    got = {}
    for s in _iter_stmts(path):
        if not s.startswith('#'):
            continue
        i = s.find('=')
        try:
            eid = int(s[1:i].strip())
        except ValueError:
            continue
        if eid in need:
            m = SIMPLE.match(s)
            if m:
                got[eid] = (m.group(2), m.group(3))
                if len(got) == len(need):
                    break
    return got


def refs(arg):
    return [int(x) for x in re.findall(r"#(\d+)", arg)]


def split_top(arg):
    out, depth, cur, q = [], 0, '', False
    for ch in arg:
        if ch == "'":
            q = not q
        if not q:
            if ch == '(':
                depth += 1
            elif ch == ')':
                depth -= 1
            elif ch == ',' and depth == 0:
                out.append(cur.strip())
                cur = ''
                continue
        cur += ch
    out.append(cur.strip())
    return out


def name_of(arg):
    m = re.search(r"'((?:[^']|'')*)'", arg)
    return m.group(1) if m else ''


# ---- linear algebra (3x4 matrices as tuple of 3 rows of 4) ----

def mat_from_axis(loc, zdir, xdir):
    if zdir is None:
        zdir = (0.0, 0.0, 1.0)
    if xdir is None:
        xdir = (1.0, 0.0, 0.0)
    z = _norm(zdir)
    x = _norm(_sub(xdir, _scale(z, _dot(xdir, z))))
    y = _cross(z, x)
    return ((x[0], y[0], z[0], loc[0]),
            (x[1], y[1], z[1], loc[1]),
            (x[2], y[2], z[2], loc[2]))


def mat_mul(a, b):
    out = []
    for i in range(3):
        row = []
        for j in range(3):
            row.append(sum(a[i][k] * b[k][j] for k in range(3)))
        row.append(sum(a[i][k] * b[k][3] for k in range(3)) + a[i][3])
        out.append(tuple(row))
    return tuple(out)


def mat_inv(a):
    out = []
    for i in range(3):
        row = [a[0][i], a[1][i], a[2][i]]
        row.append(-(a[0][i] * a[0][3] + a[1][i] * a[1][3] + a[2][i] * a[2][3]))
        out.append(tuple(row))
    return tuple(out)


def apply(m, p):
    return tuple(m[i][0] * p[0] + m[i][1] * p[1] + m[i][2] * p[2] + m[i][3] for i in range(3))


IDENT = ((1.0, 0, 0, 0), (0, 1.0, 0, 0), (0, 0, 1.0, 0))


def _sub(a, b):
    return tuple(a[i] - b[i] for i in range(3))


def _dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _scale(a, s):
    return tuple(v * s for v in a)


def _norm(a):
    n = math.sqrt(_dot(a, a)) or 1.0
    return tuple(v / n for v in a)
