#!/usr/bin/env python3
"""
Generate merged experiment_geometry + waypoint_state_machine diagram as drawio XML.
All font sizes use CSS px values, never HTML <font size=N>.
"""
import xml.etree.ElementTree as ET

W, H = 1050, 880

def make_style(**kw):
    return ";".join(f"{k}={v}" for k, v in kw.items())

def box(fill="#d5e8d4", stroke="#82b366", fc="#000000", fs=11):
    return make_style(rounded="1", absoluteArcSize="1", html="1", arcSize="10",
                      fillColor=fill, strokeColor=stroke, fontColor=fc,
                      fontFamily="Helvetica", fontSize=str(fs), whiteSpace="wrap")

def ellipse(fill, stroke, dashed=False):
    s = {"ellipse": "1", "html": "1", "fillColor": fill, "strokeColor": stroke,
         "fontFamily": "Helvetica", "fontSize": "11", "whiteSpace": "wrap"}
    if dashed: s["dashed"] = "1"
    return make_style(**s)

def txt(fs=11, color="#000", align="center", bg="none"):
    return make_style(text="html", html="1", fontSize=str(fs),
                      fontFamily="Helvetica", fontColor=color,
                      align=align, verticalAlign="middle",
                      whiteSpace="wrap", labelBackgroundColor=bg)

def edge(stroke="#333", w=2, dash=False, endArrow="classic"):
    return make_style(edgeStyle="orthogonalEdgeStyle", html="1", rounded="0",
                      strokeColor=stroke, strokeWidth=str(w),
                      endArrow=endArrow, endFill="1" if endArrow != "none" else "0",
                      fontFamily="Helvetica", fontSize="11",
                      labelBackgroundColor="#ffffff",
                      dashed="1" if dash else "0")

# ---- Build XML ----
mxfile = ET.Element("mxfile", {"host": "app.diagrams.net", "agent": "Mozilla/5.0"})
diagram = ET.SubElement(mxfile, "diagram", {"name": "Experiment Overview (merged)", "id": "merged-exp"})
model = ET.SubElement(diagram, "mxGraphModel",
    {"grid": "1", "page": "1", "gridSize": "10", "pageWidth": str(W), "pageHeight": str(H),
     "background": "#ffffff", "math": "0", "shadow": "0"})
root = ET.SubElement(model, "root")

ET.SubElement(root, "mxCell", {"id": "0"})
ET.SubElement(root, "mxCell", {"id": "1", "parent": "0"})

nid = [100]

def add_cell(value="", style="", vertex=None, edge=None, parent="1",
             x=0, y=0, w=100, h=40, source=None, target=None,
             points=None, ex=None, ey=None, enx=None, eny=None):
    cid = str(nid[0]); nid[0] += 1
    a = {"id": cid, "parent": parent}
    if value: a["value"] = value
    if style: a["style"] = style
    if vertex: a["vertex"] = "1"
    if edge: a["edge"] = "1"
    if source: a["source"] = source
    if target: a["target"] = target
    el = ET.SubElement(root, "mxCell", a)
    geo = ET.SubElement(el, "mxGeometry",
        {"x": str(x), "y": str(y), "width": str(w), "height": str(h), "as": "geometry"})
    if edge and points:
        arr = ET.SubElement(geo, "Array", {"as": "points"})
        for px, py in points:
            ET.SubElement(arr, "mxPoint", {"x": str(px), "y": str(py)})
    if ex is not None: geo.set("exitX", str(ex)); geo.set("exitY", str(ey or 0.5))
    if enx is not None: geo.set("entryX", str(enx)); geo.set("entryY", str(eny or 0.5))
    return cid

def sm_edge(src, dst, label="", style=None):
    s = style or ("edgeStyle=orthogonalEdgeStyle;html=1;rounded=0;strokeColor=#333;"
                  "strokeWidth=1.5;endArrow=classic;endFill=1;fontSize=10;")
    a = {"id": str(nid[0]), "parent": "1", "edge": "1", "source": src, "target": dst, "style": s}
    if label: a["value"] = label
    nid[0] += 1
    el = ET.SubElement(root, "mxCell", a)
    ET.SubElement(el, "mxGeometry", {"relative": "1", "as": "geometry"})

# ===== BACKGROUND =====
add_cell("Collision Sweep Experiment", box("#fafafa", "#ccc", "#333", 14),
         vertex=True, x=20, y=20, w=W-40, h=H-40)

# ===== ENU AXES =====
add_cell("+Y (North) ⬆", txt(10, "#999"), vertex=True, x=25, y=28, w=90, h=14)
add_cell("+X (East)  →", txt(10, "#999"), vertex=True, x=25, y=44, w=90, h=14)

# ===== COLUMN (center) =====
add_cell("Column\n9 cm", ellipse("#e74c3c", "#c0392b"), vertex=True, x=320, y=360, w=60, h=60)
add_cell("Cage perimeter\n35.8 cm", ellipse("none", "#e67e22", True), vertex=True, x=265, y=305, w=170, h=170)

# ===== IMPACT ZONE ANNOTATION =====
add_cell('<span style="color:#e74c3c;font-weight:bold">⚡ IMPACT ZONE</span>',
         txt(11, "#e74c3c"), vertex=True, x=290, y=485, w=120, h=26)

# ===== WAYPOINTS =====
wp = box("#2ecc71", "#27ae60", "#000", 11)
wp_sm = box("#2ecc71", "#27ae60", "#000", 10)

wp1 = add_cell("WP1 / WP_STAGE\nU-turn, Y≈1.200 m", wp, vertex=True, x=430, y=55, w=150, h=46)
wp2 = add_cell("WP2 — GATE (Start-Point)\nY≈0.950–1.100 m", wp, vertex=True, x=50, y=180, w=160, h=46)
wp3 = add_cell("WP3 — RECOVERY (End-Point)\nY≈−1.200 m", wp, vertex=True, x=50, y=630, w=160, h=46)
wp4 = add_cell("WP4 — PAUSE &amp; LOOP\nHold + battery check", wp, vertex=True, x=430, y=720, w=160, h=46)

# ===== APPROACH ANGLE LABEL =====
add_cell('<span style="font-weight:bold">θ = 75° or 45°</span><br><span style="font-size:10px">'
         'approach angle<br>velocity vs column normal</span>',
         box("#fef9e7", "#f39c12", "#333", 11), vertex=True, x=50, y=400, w=125, h=55)

# ===== COLLISION DETAIL =====
add_cell('<span style="color:#e74c3c;font-weight:bold">⚡ COLLISION HERE</span><br>'
         '<span style="font-size:10px">Cage contacts column.<br>WP2_SWEEP active state.<br>'
         'IMU captures peak data.</span>',
         box("#fff5f5", "#e74c3c", "#333", 11), vertex=True, x=410, y=275, w=125, h=68)

# ===== FLIGHT PATHS =====
add_cell('<span style="color:#e67e22">← Transit leg</span>',
         edge("#e67e22"), edge=True, source=wp1, target=wp2, ex=0.3, ey=1, enx=1, eny=0)

add_cell('<span style="color:#3498db"><b>SWEEP LEG</b><br>'
         '<span style="font-size:10px">0.30–0.50 m/s constant, ~2.15 m</span></span>',
         edge("#3498db", 3, endArrow="block"), edge=True, source=wp2, target=wp3, ex=0.5, ey=1, enx=0.4, eny=0)

add_cell('<span style="color:#e67e22">→ Recovery transit</span>',
         edge("#e67e22"), edge=True, source=wp3, target=wp4, ex=1, ey=0.3, enx=0, eny=0.8)

add_cell('<span style="color:#2ecc71"><b>LOOP BACK</b><br>'
         '<span style="font-size:10px">(bypass TAKEOFF)</span></span>',
         edge("#2ecc71", 2, True), edge=True, source=wp4, target=wp1,
         ex=0.5, ey=0, enx=0.5, eny=1, points=[(500, 600), (500, 180)])

# ===== STATE MACHINE PANEL (right side) =====
PX, PY, PW = 720, 55, 280
add_cell("FLIGHT STATE MACHINE", box("#f0f4ff", "#4472c4", "#333", 12),
         vertex=True, x=PX, y=PY, w=PW, h=H-110)

SM = box("#d5e8d4", "#82b366", "#000", 10)
EM = box("#f8cecc", "#b85450", "#000", 10)
LP = box("#ffe6cc", "#d79b00", "#000", 10)

SW, SH = 120, 28
C1X = PX + 20
C2X = PX + 160
RS = [PY+35, PY+95, PY+155, PY+215, PY+275, PY+335, PY+395, PY+455]

s_dis = add_cell("DISARMED", SM, vertex=True, x=C1X, y=RS[0], w=SW, h=SH)
s_tko = add_cell("TAKEOFF", SM, vertex=True, x=C1X, y=RS[1], w=SW, h=SH)
s_stg = add_cell("WP_STAGE", SM, vertex=True, x=C1X, y=RS[2], w=SW, h=SH)
s_g1  = add_cell("WP1_GATE", SM, vertex=True, x=C1X, y=RS[3], w=SW, h=SH)
s_swp = add_cell("WP2_SWEEP", SM, vertex=True, x=C1X, y=RS[4], w=SW, h=SH)
s_rec = add_cell("WP3_RECOVER", SM, vertex=True, x=C1X, y=RS[5], w=SW, h=SH)
s_pau = add_cell("WP4_PAUSE", LP, vertex=True, x=C1X, y=RS[6], w=SW, h=SH)
s_ld  = add_cell("LANDING", SM, vertex=True, x=C2X, y=RS[7], w=SW, h=SH)
s_em  = add_cell("EMERGENCY", EM, vertex=True, x=C2X, y=RS[3], w=SW, h=SH)

ARROW = ("edgeStyle=orthogonalEdgeStyle;html=1;rounded=0;strokeColor=#333;"
         "strokeWidth=1.5;endArrow=classic;endFill=1;fontSize=10;")

for src, dst in [(s_dis, s_tko), (s_tko, s_stg), (s_stg, s_g1),
                 (s_g1, s_swp), (s_swp, s_rec), (s_rec, s_pau)]:
    sm_edge(src, dst)

sm_edge(s_pau, s_ld, "exit →")

# Loop back
add_cell('<span style="color:#2ecc71"><b>↻ LOOP</b></span>',
         "edgeStyle=orthogonalEdgeStyle;html=1;rounded=0;dashed=1;"
         "strokeColor=#2ecc71;strokeWidth=1.5;endArrow=classic;endFill=1;fontSize=10;",
         edge=True, source=s_pau, target=s_g1,
         points=[(PX+80, RS[6]+14), (PX+80, RS[3]+14)])

# Failsafe connector
add_cell('<span style="color:#e74c3c">failsafe</span>',
         "edgeStyle=orthogonalEdgeStyle;html=1;rounded=0;dashed=1;"
         "strokeColor=#e74c3c;strokeWidth=1.5;endArrow=open;endFill=0;fontSize=10;",
         edge=True, source=s_g1, target=s_em)

# Failsafe triggers
add_cell('<span style="font-size:10px;color:#b85450"><b>TRIGGERS:</b><br>'
         '• MoCap tracking lost<br>'
         '• Geofence breached<br>'
         '• Spacebar kill-switch<br>'
         '• Low battery (&lt;20.4 V)</span>',
         txt(10, "#b85450", "left"), vertex=True, x=C2X-5, y=RS[3]+SH+5, w=135, h=65)

# Loop counter
add_cell('<span style="font-size:10px;color:#888">Each loop = 1 data pass<br>'
         'Max ~8–12 passes per battery</span>',
         txt(10, "#888"), vertex=True, x=PX+15, y=RS[6]+SH+5, w=130, h=28)

# ===== WRITE =====
path = "/home/dorten/MasterThesisDrone/thesis/diagrams/experiment_overview.drawio"
tree = ET.ElementTree(mxfile)
tree.write(path, encoding="utf-8", xml_declaration=True)
print(f"✅ Written: {path}  ({nid[0] - 100} cells)")
