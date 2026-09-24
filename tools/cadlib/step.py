"""Minimal STEP AP214 ayrıştırıcı: yalnızca montaj ağacı için gereken varlıklar.

Tam bir STEP okuyucu değil. Metni satır satır okuyup montaj ağacını
(PRODUCT, NEXT_ASSEMBLY_USAGE_OCCURRENCE, ...) ve yerleşim eksenlerini
(AXIS2_PLACEMENT_3D) çeker; geometriye (B-spline yüzeyler) bakmaz.
Montaj çözümü assembly.py'de.
"""

import re

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
