#!/usr/bin/env python3
"""Generate the 82-piece LDraw reconstruction of MOC-194603."""
from __future__ import annotations

import json
import math
from collections import Counter
from pathlib import Path

BASE = Path(__file__).resolve().parent
OUT_LDR = BASE / "brougham_exact_model.ldr"
OUT_JSON = BASE / "brougham_exact_model.json"

COLOR = {
    "Black": 0,
    "White": 15,
    "Dark Red": 320,
    "Trans-Clear": 47,
    "Reddish Brown": 70,
    "Trans-Yellow": 46,
}


def mmul(a, b):
    return [
        sum(a[r * 3 + k] * b[k * 3 + c] for k in range(3))
        for r in range(3) for c in range(3)
    ]


def rx(deg):
    a = math.radians(deg)
    c, s = round(math.cos(a), 9), round(math.sin(a), 9)
    return [1, 0, 0, 0, c, -s, 0, s, c]


def ry(deg):
    a = math.radians(deg)
    c, s = round(math.cos(a), 9), round(math.sin(a), 9)
    return [c, 0, s, 0, 1, 0, -s, 0, c]


parts = []


def add(step, part, color, x, y, z, yaw=0, matrix=None, note=""):
    parts.append({
        "id": f"s{step:02d}-p{len(parts) + 1:03d}",
        "step": step,
        "part": part,
        "color": color,
        "ldraw_color": COLOR[color],
        "position": [x, y, z],
        "matrix": matrix or ry(yaw),
        "note": note,
    })


# X is left/right, Z is rear/front (horse is +Z), Y points down.
# The two 4x4 chassis plates occupy Z=-80..0 and Z=0..80.

# PDF 1-6: rear cabin end.
add(1, "3031", "Black", 0, 0, -40)
add(2, "3010", "Black", 0, -24, -70)
for x in (-20, 20):
    add(3, "3021", "Black", x, -32, -50, yaw=90)
add(4, "3004", "Black", 0, -56, -70)
for x in (-30, 30):
    add(4, "3622", "Black", x, -56, -50, yaw=90)
add(5, "51266", "Trans-Clear", 0, -128, -70)
add(5, "51239", "Black", 0, -128, -70)
for x, yaw in ((-30, 90), (30, 270)):
    add(6, "2362b", "Black", x, -128, -30, yaw=yaw)

# PDF 7-20: front chassis, window and coachman's seats.
add(7, "3031", "Black", 0, 0, 40)
for x in (-30, 30):
    add(8, "88292", "Black", x, -48, 30)
for x in (-30, 30):
    add(9, "3023", "Black", x, -56, 50, yaw=90)
add(10, "3747b", "Black", 0, -24, 70)
for x in (-10, 10):
    add(11, "3005", "Black", x, -48, 30)
add(12, "3021", "Black", 0, -56, 50)
add(13, "51266", "Trans-Clear", 0, -128, 10)
add(13, "51239", "Black", 0, -128, 10)
add(15, "3020", "Black", 0, -64, 60)
add(15, "3020", "Black", 0, -72, 60)
add(16, "3709b", "Black", 0, -80, 60)
add(17, "48336", "Black", -20, -88, 50)
add(17, "48336", "Black", 20, -88, 70, yaw=180)
add(18, "54200", "Black", -20, -88, 70, yaw=180)
add(18, "54200", "Black", 20, -88, 50, yaw=180)
add(19, "3024", "Black", -20, -88, 50)
add(19, "3024", "Black", 20, -88, 70)
add(20, "3070", "Black", -20, -96, 50)
add(20, "3070", "Black", 20, -96, 70)

# PDF 21-27: separate driver box / splash board, installed at the front.
add(21, "3021", "Black", 0, -48, 90, yaw=90)
for x in (-10, 10):
    add(22, "3023", "Black", x, -56, 100, yaw=90)
    add(22, "25269", "Black", x, -56, 70,
        yaw=90 if x < 0 else 180)
add(23, "3021", "Black", 0, -64, 90, yaw=90)
add(24, "99207", "Black", 0, -72, 110, yaw=180)
add(25, "3023", "Black", 0, -88, 136, matrix=rx(90),
    note="vertical spacer on the driver-box bracket")
add(25, "15068", "Black", 0, -88, 152,
    matrix=rx(90), note="vertical curved splash board across the carriage")
for x in (-10, 10):
    add(26, "3070", "Black", x, -80, 70)

# PDF 28-35: running gear and the integral horse shafts.
add(28, "2926", "Black", 0, 32, 70)
add(28, "4489b", "Reddish Brown", -50, 37, 70, yaw=-90)
add(28, "4489b", "Reddish Brown", 50, 37, 70, yaw=90)
add(29, "30236", "Black", 0, 8, 70, yaw=90)
add(30, "11476", "Black", 0, 0, 70, yaw=90)
add(31, "2397", "Black", 0, -16, 130,
    note="large local -Z end is centered on the front axle")
add(32, "2460", "Black", 0, 0, 70)
add(34, "3010", "Black", 0, 8, -70)
add(35, "2926", "Black", 0, 32, -70)
add(35, "4489b", "Reddish Brown", -50, 37, -70, yaw=-90)
add(35, "4489b", "Reddish Brown", 50, 37, -70, yaw=90)

# PDF 36-40: passenger seat, doors and side windows.
add(36, "3023", "Dark Red", 0, -64, -30, yaw=90)
add(36, "15068", "Dark Red", 0, -64, -30, yaw=180)
add(38, "3188", "Black", -40, -56, -50)
add(38, "3189", "Black", 40, -56, -50)
for x, yaw in ((-40, 90), (40, 270)):
    add(39, "60603", "Trans-Clear", x, -128, -30, yaw=yaw)
    add(39, "60594", "Black", x, -128, -30, yaw=yaw)

# PDF 41-47: roof perimeter, cornice, brackets, lamps and roof plate.
for x in (-40, 40):
    add(41, "41740", "Black", x, -136, -30, yaw=90)
add(41, "3069", "Black", 0, -136, -70)
add(41, "3069", "Black", -20, -136, 10)
add(41, "3069", "Black", 20, -136, 10)
add(42, "3023", "Black", 0, -144, 20)
for x in (-30, 30):
    add(42, "2555", "Black", x, -144, 20)
add(43, "3023", "Black", 0, -152, 20)
add(43, "93273", "Black", 0, -152, 20, yaw=90)
for x, yaw in ((-50, 90), (50, 270)):
    add(44, "36840", "Black", x, -144, 10, yaw=yaw)
for x in (-55, 55):
    add(45, "4589", "Trans-Yellow", x, -126, 10)
    add(45, "37776", "Black", x, -144, 10)
add(47, "3032", "Black", 0, -144, -30, yaw=90,
    note="4x6 roof plate seated directly on the step-41 side rails")

# PDF 48: horse and the white 1x2 back brick.
add(48, "4493c01pb04", "White", 0, 22, 180, yaw=180,
    note="printed horse; official-library fallback geometry is 4493c01")
add(48, "3004", "White", 0, -2, 180,
    note="1x2 brick attached only to the two horse-back studs")


def audit():
    bom = json.loads((BASE / "brougham_exact_bom.json").read_text())
    expected = Counter((p["part"], p["color"]) for p in bom["parts"]
                       for _ in range(p["count"]))
    actual = Counter((p["part"], p["color"]) for p in parts)
    errors = []
    for key in sorted(set(expected) | set(actual)):
        if expected[key] != actual[key]:
            errors.append((key, expected[key], actual[key]))
    if errors:
        raise RuntimeError("model/BOM mismatch: " + repr(errors))


def main():
    audit()
    model = {
        "model_status": "verified-against-pdf-key-steps",
        "source": "MOC-194603_brickbicycle_brougham.pdf",
        "units": {"ldu_per_stud": 20, "ldu_per_plate": 8},
        "axes": {"x": "left-right", "y": "down", "z": "rear-to-horse"},
        "geometry_fallbacks": [{
            "part": "4493c01pb04",
            "geometry": "4493c01",
            "reason": "pb04 is not present in the official complete LDraw library",
            "render_adjustment": "fixed tack color 6 remapped to Dark Orange",
        }],
        "part_count": len(parts),
        "parts": parts,
    }
    OUT_JSON.write_text(json.dumps(model, indent=2) + "\n")

    lines = [
        "0 FILE brougham_exact_model.ldr",
        "0 Victorian Era Horse-Drawn Brougham",
        "0 !LICENSE Redistributable under CCAL version 2.0 : see CAreadme.txt",
        "0 // Piece-level reconstruction from the 48 PDF instruction steps",
    ]
    last_step = None
    for p in parts:
        if p["step"] != last_step:
            lines.append(f"0 STEP // PDF {p['step']}")
            last_step = p["step"]
        x, y, z = p["position"]
        vals = " ".join(f"{v:g}" for v in p["matrix"])
        part_name = "4493c01" if p["part"] == "4493c01pb04" else p["part"]
        lines.append(
            f"0 // {p['id']} {p['note']}".rstrip() + "\n"
            f"1 {p['ldraw_color']} {x:g} {y:g} {z:g} {vals} {part_name}.dat")
    OUT_LDR.write_text("\n".join(lines) + "\n")
    print(f"wrote {OUT_JSON.name}, {OUT_LDR.name}: {len(parts)} parts")


if __name__ == "__main__":
    main()
