# SPDX-License-Identifier: MIT
"""The lift fixing text follows P-03 item 30 A3 (D-20, D-21): four foot tab holes on 260 x 30 mm into tapped MEC-019.
The 224 x 40 mm pattern is internal to the DOLD assembly and must never be described as the fixing again (it was, under item 28)."""
import os
import re

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, os.pardir)
REPO = os.path.join(PKG, os.pardir, os.pardir, os.pardir)


def text(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def column_note():
    with open(os.path.join(PKG, "config", "datums.yaml"), encoding="utf-8") as f:
        return yaml.safe_load(f)["datums"]["lift_column_section"]["note"]


def test_the_datum_note_gives_the_260_x_30_tab_holes_and_calls_224_x_40_internal():
    note = column_note()
    assert "260 x 30 mm" in note and "item 30 A3" in note
    assert re.search(r"224 x 40 mm pattern is internal to the DOLD assembly", note)
    assert "four fixing screws on a 224 x 40" not in note


def test_no_document_describes_224_x_40_as_the_screw_pattern():
    for rel in ("README.md", "CHANGELOG.md"):
        path = os.path.join(REPO, rel) if rel == "CHANGELOG.md" else os.path.join(PKG, rel)
        body = text(path)
        assert not re.search(r"screws on (a )?224 x 40", body), rel
    assert "260 x 30 mm" in text(os.path.join(PKG, "README.md"))
    assert "260 x 30 mm" in text(os.path.join(REPO, "CHANGELOG.md"))
