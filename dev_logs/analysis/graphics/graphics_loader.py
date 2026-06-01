import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def load_drone_svg_elements(svg_path='/home/dorten/pi_drone_sshfs/dev_logs/analysis/graphics/drone_top.svg'):
    """
    Parses the drone_top.svg file and extracts all structural geometric elements
    (lines, circles, polygons) from the #drone-graphics-layer group.
    """
    import os
    if svg_path == '/home/dorten/pi_drone_sshfs/dev_logs/analysis/graphics/drone_top.svg' or not os.path.exists(svg_path):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        resolved_path = os.path.join(script_dir, 'drone_top.svg')
        if os.path.exists(resolved_path):
            svg_path = resolved_path
        else:
            # fallback to relative path search from workspace
            for root, dirs, files in os.walk('/home/dorten/MasterThesisDrone'):
                if 'drone_top.svg' in files:
                    svg_path = os.path.join(root, 'drone_top.svg')
                    break
    try:
        tree = ET.parse(svg_path)
    except Exception as e:
        print(f"[WARN] Could not parse drone SVG at {svg_path}: {e}")
        return []

    root = tree.getroot()
    namespaces = {'svg': 'http://www.w3.org/2000/svg'}
    
    # Locate the graphics group
    layer = root.find(".//svg:g[@id='drone-graphics-layer']", namespaces)
    if layer is None:
        layer = root.find(".//g[@id='drone-graphics-layer']")
    if layer is None:
        layer = root  # fallback to parsing entire root
        
    elements = []
    
    for child in layer:
        tag = child.tag.split('}')[-1]  # strip namespace if present
        attribs = child.attrib
        
        # We ignore the central helper gridlines that span y1=50 to y2=550
        if tag == 'line':
            x1, y1 = float(attribs.get('x1', 0)), float(attribs.get('y1', 0))
            x2, y2 = float(attribs.get('x2', 0)), float(attribs.get('y2', 0))
            if abs(x1 - x2) < 0.1 and abs(y1 - 50) < 0.1:  # helper line
                continue
            if abs(y1 - y2) < 0.1 and abs(x1 - 50) < 0.1:  # helper line
                continue
            elements.append({
                'type': 'line',
                'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                'class': attribs.get('class', ''),
                'stroke_width': float(attribs.get('stroke-width', 1))
            })
            
        elif tag == 'circle':
            cx, cy = float(attribs.get('cx', 0)), float(attribs.get('cy', 0))
            r = float(attribs.get('r', 0))
            elements.append({
                'type': 'circle',
                'cx': cx, 'cy': cy, 'r': r,
                'class': attribs.get('class', ''),
                'fill': attribs.get('fill', '')
            })
            
        elif tag == 'polygon':
            pts_str = attribs.get('points', '')
            pts = []
            for p in pts_str.strip().split():
                if ',' in p:
                    px, py = map(float, p.split(','))
                    pts.append([px, py])
            elements.append({
                'type': 'polygon',
                'points': np.array(pts),
                'class': attribs.get('class', '')
            })
            
    return elements

def draw_vector_drone(ax, cx_val, cy_val, cage_radius, rotation_deg=0, color_mode='full', alpha_multiplier=1.0):
    """
    Draws the parsed SVG drone elements in Matplotlib as vector objects.
    Scales the SVG coordinate frame (centered at 300, 300, with cage radius = 180)
    to match the physical cage_radius, and rotates it by rotation_deg.
    """
    elements = load_drone_svg_elements()
    if not elements:
        # Fallback to drawing a simple circle and arm cross if SVG not found
        cage_circle = plt.Circle((cx_val, cy_val), cage_radius, color='gray' if color_mode == 'gray' else '#1F77B4', alpha=0.15 * alpha_multiplier)
        ax.add_patch(cage_circle)
        return

    # Scale factor from SVG coordinates (cage R = 180) to physical meters
    scale = cage_radius / 180.0
    theta = np.radians(rotation_deg)
    cos_t, sin_t = np.cos(theta), np.sin(theta)

    def transform(x, y):
        # Translate SVG coordinates (origin top-left) relative to center (300, 300)
        # Note: SVG y increases downwards, so we invert y to align with ENU
        dx = (x - 300.0) * scale
        dy = -(y - 300.0) * scale
        # Rotate coordinates
        rx = dx * cos_t - dy * sin_t
        ry = dx * sin_t + dy * cos_t
        # Translate to target position
        return cx_val + rx, cy_val + ry

    for el in elements:
        # Resolve styling colors based on class and color mode
        cls = el.get('class', '')
        fill_color = el.get('fill', '')
        
        # Defaults for 'full' color mode
        stroke_color = '#1e293b'
        fill_val = 'none'
        lw = 1.5
        ls = '-'
        opacity = 1.0
        zorder = 7

        if 'cad-line-primary' in cls:
            stroke_color = '#475569'  # Slim, professional slate grey carbon fiber arms
            lw = el.get('stroke_width', 2.2) * 0.38  # Slimmed down to prevent muddy overlapping
            zorder = 4  # Drawn behind plates
        elif 'cad-line-secondary' in cls:
            stroke_color = '#64748b'
            lw = el.get('stroke_width', 1.5) * 0.7
            zorder = 4  # Drawn behind plates
        elif 'cad-cage-line' in cls:
            stroke_color = '#94a3b8'  # Softer cage boundary line
            lw = 1.0
            ls = ':'
            zorder = 3
        elif 'cad-plate-fill' in cls:
            stroke_color = '#1e293b'
            fill_val = '#f8fafc'      # Pristine clean off-white plate fill
            lw = 1.2
            zorder = 7  # Drawn on top of arms for crisp structural outlines
        elif 'cad-prop-fill' in cls:
            stroke_color = '#cbd5e1'  # Soft light grey stroke
            fill_val = 'none'        # Transparent sweeps
            lw = 0.5                 # Thin crisp circular sweep
            ls = (0, (4, 4))         # Elegant dashed style
            opacity = 0.18           # Subtle premium opacity
            zorder = 8               # Drawn on top of everything for pure visibility
        elif fill_color == '#1e293b':  # shaft center
            stroke_color = '#1e293b'
            fill_val = '#1e293b'
            lw = 1.0

        # Adjust for 'gray' mode
        if color_mode == 'gray':
            stroke_color = '#94a3b8'
            if 'cad-prop-fill' in cls:
                fill_val = 'none'
                stroke_color = '#cbd5e1'
                lw = 0.4
                ls = (0, (4, 4))
                opacity = 0.15 * alpha_multiplier
            else:
                if fill_val != 'none':
                    fill_val = '#f8fafc'
                opacity = 0.20 * alpha_multiplier
            zorder = 6
        else:
            opacity = opacity * alpha_multiplier

        # Render element
        if el['type'] == 'line':
            x1, y1 = transform(el['x1'], el['y1'])
            x2, y2 = transform(el['x2'], el['y2'])
            ax.plot([x1, x2], [y1, y2], color=stroke_color, linewidth=lw, linestyle=ls, alpha=opacity, zorder=zorder)
            
        elif el['type'] == 'circle':
            cx, cy = transform(el['cx'], el['cy'])
            r = el['r'] * scale
            
            circle_patch = patches.Circle(
                (cx, cy), r,
                edgecolor=stroke_color,
                facecolor=fill_val,
                linewidth=lw,
                linestyle=ls,
                alpha=opacity,
                zorder=zorder
            )
            ax.add_patch(circle_patch)
            
        elif el['type'] == 'polygon':
            transformed_pts = [transform(pt[0], pt[1]) for pt in el['points']]
            poly_patch = patches.Polygon(
                transformed_pts,
                edgecolor=stroke_color,
                facecolor=fill_val,
                linewidth=lw,
                alpha=opacity,
                zorder=zorder
            )
            ax.add_patch(poly_patch)

def get_drone_tikz(cx_val, cy_val, cage_radius, rotation_deg=0, is_grayed_out=False):
    """
    Generates inline LaTeX TikZ vector statements to draw the drone structure.
    Integrates the elements parsed from drone_top.svg natively!
    """
    elements = load_drone_svg_elements()
    if not elements:
        # Fallback simple drone
        a_len = 0.08
        fallback = []
        c_style = "dashed, gray!60" if is_grayed_out else "blue!50!black, thick"
        f_style = "gray!5" if is_grayed_out else "blue!5"
        fallback.append(f"  \\draw[{c_style}, fill={f_style}, fill opacity=0.3] ({cx_val:.3f}, {cy_val:.3f}) circle ({cage_radius:.3f});")
        return fallback

    scale = cage_radius / 180.0
    theta = np.radians(rotation_deg)
    cos_t, sin_t = np.cos(theta), np.sin(theta)

    def transform(x, y):
        dx = (x - 300.0) * scale
        dy = -(y - 300.0) * scale
        rx = dx * cos_t - dy * sin_t
        ry = dx * sin_t + dy * cos_t
        return cx_val + rx, cy_val + ry

    tikz = []
    
    # Styling classes translated into TikZ colors & widths
    p_color = "gray!60" if is_grayed_out else "black!80"
    s_color = "gray!40" if is_grayed_out else "gray"
    fill_plate = "gray!5" if is_grayed_out else "white!95!black"
    fill_prop = "gray!3" if is_grayed_out else "gray!8"
    prop_opacity = "0.2" if is_grayed_out else "0.4"
    
    for el in elements:
        cls = el.get('class', '')
        fill_color = el.get('fill', '')
        
        if el['type'] == 'line':
            x1, y1 = transform(el['x1'], el['y1'])
            x2, y2 = transform(el['x2'], el['y2'])
            
            if 'cad-line-primary' in cls:
                opts = f"draw={p_color}, line width=0.45pt, line cap=round"
            elif 'cad-line-secondary' in cls:
                opts = f"draw={s_color}, line width=0.35pt, line cap=round"
            else:
                opts = f"draw={s_color}, line width=0.2pt, dotted"
                
            tikz.append(f"  \\draw[{opts}] ({x1:.3f}, {y1:.3f}) -- ({x2:.3f}, {y2:.3f});")
            
        elif el['type'] == 'circle':
            cx, cy = transform(el['cx'], el['cy'])
            r = el['r'] * scale
            
            if 'cad-cage-line' in cls:
                opts = f"draw={p_color}, line width=0.6pt, dashed"
            elif 'cad-prop-fill' in cls:
                opts = f"draw={s_color}, fill=none, line width=0.35pt, dash pattern=on 2pt off 2pt"
            elif 'cad-plate-fill' in cls:
                opts = f"draw={p_color}, fill={fill_plate}, line width=0.45pt"
            elif fill_color == '#1e293b':  # shaft
                opts = f"draw={p_color}, fill={p_color}, line width=0.2pt"
            else:
                opts = f"draw={s_color}, line width=0.6pt"
                
            tikz.append(f"  \\draw[{opts}] ({cx:.3f}, {cy:.3f}) circle ({r:.3f});")
            
        elif el['type'] == 'polygon':
            transformed_pts = [transform(pt[0], pt[1]) for pt in el['points']]
            pts_str = " -- ".join([f"({p[0]:.3f}, {p[1]:.3f})" for p in transformed_pts])
            
            opts = f"draw={p_color}, fill={fill_plate}, line width=1.0pt, line join=round"
            tikz.append(f"  \\filldraw[{opts}] {pts_str} -- cycle;")
            
    return "\n".join(tikz)
