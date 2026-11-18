"""Minimal .xlsx reader using only the standard library.

Piπd reads the team's BOM straight out of Excel. Pulling in pandas/openpyxl for
a handful of sheets is not worth the dependency, and this keeps
`pip install -e .` to numpy/scipy/matplotlib.
"""

from __future__ import annotations

import re
import zipfile
import xml.etree.ElementTree as ET

MAIN_NS = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"
REL_NS = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"


def _col_index(ref: str) -> int:
    letters = re.match(r"([A-Z]+)", ref)
    if not letters:
        return 0
    idx = 0
    for ch in letters.group(1):
        idx = idx * 26 + (ord(ch) - ord("A") + 1)
    return idx - 1


def read_workbook(path: str) -> dict[str, list[list[str]]]:
    """Return {sheet name: rows of cell strings}."""
    with zipfile.ZipFile(path) as z:
        names = set(z.namelist())
        shared: list[str] = []
        if "xl/sharedStrings.xml" in names:
            root = ET.fromstring(z.read("xl/sharedStrings.xml"))
            for si in root.findall(f"{{{MAIN_NS}}}si"):
                shared.append("".join(t.text or "" for t in si.iter(f"{{{MAIN_NS}}}t")))

        wb = ET.fromstring(z.read("xl/workbook.xml"))
        rels = ET.fromstring(z.read("xl/_rels/workbook.xml.rels"))
        relmap = {r.get("Id"): r.get("Target") for r in rels}

        out: dict[str, list[list[str]]] = {}
        for sheet in wb.find(f"{{{MAIN_NS}}}sheets"):
            name = sheet.get("name")
            rid = sheet.get(f"{{{REL_NS}}}id")
            target = relmap.get(rid, "")
            target = target.lstrip("/")
            if not target.startswith("xl/"):
                target = "xl/" + target
            if target not in names:
                continue
            ws = ET.fromstring(z.read(target))
            rows: list[list[str]] = []
            for row in ws.iter(f"{{{MAIN_NS}}}row"):
                cells: dict[int, str] = {}
                for c in row.findall(f"{{{MAIN_NS}}}c"):
                    ref = c.get("r", "A1")
                    t = c.get("t")
                    value = ""
                    if t == "inlineStr":
                        value = "".join(x.text or "" for x in c.iter(f"{{{MAIN_NS}}}t"))
                    else:
                        v = c.find(f"{{{MAIN_NS}}}v")
                        if v is not None and v.text is not None:
                            value = v.text
                            if t == "s":
                                try:
                                    value = shared[int(value)]
                                except (ValueError, IndexError):
                                    value = ""
                    cells[_col_index(ref)] = value
                width = max(cells) + 1 if cells else 0
                rows.append([cells.get(i, "") for i in range(width)])
            out[name] = rows
        return out


def find_rows(rows: list[list[str]], needle: str) -> list[list[str]]:
    """Rows whose concatenated text contains `needle` (case-insensitive)."""
    n = needle.lower()
    return [r for r in rows if n in " ".join(r).lower()]
