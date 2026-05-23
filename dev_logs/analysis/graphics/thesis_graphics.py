import os

def generate_trajectory_tikz(df_mocap, wps, target_flight_path, flight_folder_name, 
                           column_x, column_y, column_radius, cage_radius,
                           closest_t, closest_x, closest_y, closest_clearance,
                           wp2_x=None, wp2_y=None, wp3_x=None, wp3_y=None):
    """
    Generates standalone LaTeX TikZ vector graphics code for the drone's trajectory.
    Rotated 90 degrees counter-clockwise (x_new = -y, y_new = x) as required for portrait layouts.
    Downsamples the trajectory for performance and saves a .tikz file next to the flight folder.
    """
    print("💡 LATEX TIKZ GENERATOR ACTIVE (ROTATED 90° CCW)\n")
    print("This function generates standalone LaTeX TikZ vector graphics to paste directly")
    print("into your Overleaf document for professional, crisp publications.")
    print("===========================================================================\n")

    # Downsample trajectory for high-performance LaTeX compiling (keeps every 25th coordinate)
    df_ds = df_mocap.iloc[::25]
    a_len = 0.08
    cage_diameter = cage_radius * 2.0

    tikz_code = []
    tikz_code.append("% standalone TikZ vector path generated automatically from flight telemetry (Rotated 90 deg CCW)")
    tikz_code.append("\\begin{tikzpicture}[scale=2.5]")
    tikz_code.append("  % Draw grid lines (limits rotated: X from -y_max to -y_min, Y from x_min to x_max)")
    tikz_code.append("  \\draw[very thin, gray!20, step=0.5] (-1.6,-0.5) grid (1.6,1.0);\n")
    tikz_code.append("  % Draw coordinate axes")
    tikz_code.append("  \\draw[->, thick] (-1.6,0) -- (1.6,0) node[right] {$-y$ (ENU, m)};")
    tikz_code.append("  \\draw[->, thick] (0,-0.5) -- (0,1.0) node[above] {$x$ (ENU, m)};\n")

    # Draw column to scale
    tikz_code.append(f"  % Central Column core obstacle (scale 1:1)")
    tikz_code.append(f"  \\draw[fill=yellow!20, draw=yellow!80!black, thick] ({-column_y:.3f}, {column_x:.3f}) circle ({column_radius:.3f});")
    tikz_code.append(f"  \\node at ({-column_y:.3f}, {column_x:.3f}) [font=\\scriptsize] {{Column}};\n")

    # Draw Waypoints & Command Path
    tikz_code.append("  % Waypoint coordinates & Ideal Command Path")
    for i, wp in enumerate(wps, 1):
        tikz_code.append(f"  \\draw[red, fill=white, thick] ({-wp[1]:.3f}, {wp[0]:.3f}) circle (0.04) node[red, above right, font=\\scriptsize\\bfseries] {{WP{i}}};")
    
    # WP1 -> WP2 (1 arrow)
    tikz_code.append(f"  \\draw[red, dotted, thick, -latex] ({-wps[0][1]:.3f}, {wps[0][0]:.3f}) -- ({-wps[1][1]:.3f}, {wps[1][0]:.3f});")
    # WP2 -> WP3 (Split into 3 segments for regular arrows)
    tikz_code.append(f"  \\draw[red, dotted, thick, -latex] ({-wps[1][1]:.3f}, {wps[1][0]:.3f}) -- (-0.400, {wps[1][0]:.3f});")
    tikz_code.append(f"  \\draw[red, dotted, thick, -latex] (-0.400, {wps[1][0]:.3f}) -- (0.400, {wps[1][0]:.3f});")
    tikz_code.append(f"  \\draw[red, dotted, thick, -latex] (0.400, {wps[1][0]:.3f}) -- ({-wps[2][1]:.3f}, {wps[2][0]:.3f});")
    # WP3 -> WP4 (1 arrow)
    tikz_code.append(f"  \\draw[red, dotted, thick, -latex] ({-wps[2][1]:.3f}, {wps[2][0]:.3f}) -- ({-wps[3][1]:.3f}, {wps[3][0]:.3f});")

    # Draw grayed-out drone at WP2 if coordinates are provided
    if wp2_x is not None and wp2_y is not None:
        tikz_code.append(f"  % Drone envelope grayed out at WP2")
        from .graphics_loader import get_drone_tikz
        tikz_code.append(get_drone_tikz(-wp2_y, wp2_x, cage_radius, rotation_deg=0, is_grayed_out=True) + "\n")

    # Draw grayed-out drone at WP3 if coordinates are provided
    if wp3_x is not None and wp3_y is not None:
        tikz_code.append(f"  % Drone envelope grayed out at WP3")
        from .graphics_loader import get_drone_tikz
        tikz_code.append(get_drone_tikz(-wp3_y, wp3_x, cage_radius, rotation_deg=0, is_grayed_out=True) + "\n")

    # Draw Trajectory
    tikz_code.append("\n  % Flight trajectory path")
    path_coords = " -- ".join([f"({-row['y']:.3f}, {row['x']:.3f})" for _, row in df_ds.iterrows()])
    tikz_code.append(f"  \\draw[blue!80!black, thick] {path_coords};\n")

    # Draw closest approach drone schematic and cage to scale
    tikz_code.append(f"  % Drone envelope at closest approach (t = {closest_t:.2f}s)")
    from .graphics_loader import get_drone_tikz
    tikz_code.append(get_drone_tikz(-closest_y, closest_x, cage_radius, rotation_deg=0, is_grayed_out=False))
    tikz_code.append(f"  \\fill[black] ({-closest_y:.3f}, {closest_x:.3f}) circle (0.02) node[above right, font=\\scriptsize] {{Drone}};\n")

    # Separation vector
    tikz_code.append(f"  % Separation vector")
    tikz_code.append(f"  \\draw[purple, dotted, thick] ({-closest_y:.3f}, {closest_x:.3f}) -- ({-column_y:.3f}, {column_x:.3f});")
    tikz_code.append(f"  \\node at ({- (closest_y + column_y) / 2:.3f}, {(closest_x + column_x) / 2:.3f}) [below left, font=\\tiny, purple] {{clearance: {closest_clearance*100:.1f}cm}};\n")

    tikz_code.append("\\end{tikzpicture}")

    tikz_str = "\n".join(tikz_code)

    # Save as standalone LaTeX file next to the notebook
    latex_path = os.path.join(target_flight_path, f"{flight_folder_name}_trajectory.tikz")
    try:
        with open(latex_path, 'w') as f:
            f.write(tikz_str)
        print(f"💾 Standalone TikZ code saved successfully as: \n   {latex_path}\n")
    except Exception as e:
        print(f"[WARN] Failed to write TikZ file: {e}")

    print("📋 LaTeX Code Snippet:")
    print("---------------------------------------------------------------------------")
    print(tikz_str)
    print("---------------------------------------------------------------------------")


