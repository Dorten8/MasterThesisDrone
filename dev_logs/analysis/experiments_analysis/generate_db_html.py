#!/usr/bin/env python3
import sqlite3
import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
DB_PATH = os.path.abspath(os.path.join(current_dir, "..", "experiments_summary.db"))
HTML_PATH = os.path.abspath(os.path.join(current_dir, "..", "db_inspector.html"))

def generate_html():
    if not os.path.exists(DB_PATH):
        print(f"❌ Database not found at {DB_PATH}")
        sys.exit(1)

    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    
    # Query all records
    cursor.execute("""
        SELECT flight_name, condition, sweep_speed, battery_at_start,
               impact_speed, before_impact_accel, impact_accel, impact_angle,
               avg_dev_after, max_dev_after, recovery_area, closest_clearance,
               impact_detected, 
               nom_sp_x, nom_sp_y, nom_sp_z, 
               nom_ep_x, nom_ep_y, nom_ep_z, 
               act_sp_x, act_sp_y, act_sp_z, 
               act_ep_x, act_ep_y, act_ep_z,
               timestamp
        FROM flights_summary
        ORDER BY timestamp DESC
    """)
    rows = cursor.fetchall()
    conn.close()

    total_runs = len(rows)
    impact_runs = sum(1 for r in rows if r[12] == 1)
    impact_rate = (impact_runs / total_runs * 100.0) if total_runs > 0 else 0.0
    
    # Build Table rows dynamically
    table_rows_html = []
    for r in rows:
        f_name = r[0]
        cond = r[1]
        sweep_speed = f"{r[2]:.2f}" if r[2] is not None else "N/A"
        battery = f"{r[3]:.1f}%" if r[3] is not None else "N/A"
        imp_speed = f"{r[4]:.2f}" if r[4] is not None else "N/A"
        before_acc = f"{r[5]:.2f}" if r[5] is not None else "N/A"
        imp_acc = f"{r[6]:.2f}" if r[6] is not None else "N/A"
        imp_angle = f"{r[7]:.1f}°" if r[7] is not None else "N/A"
        avg_dev = f"{r[8]:.1f}" if r[8] is not None else "N/A"
        max_dev = f"{r[9]:.1f}" if r[9] is not None else "N/A"
        rec_area = f"{r[10]:.1f}" if r[10] is not None else "N/A"
        clearance = f"{r[11]:.1f}" if r[11] is not None else "N/A"
        
        # Tags formatting
        cond_class = "cond-fixed" if "fixed" in cond.lower() else "cond-rotating"
        impact_tag = '<span class="tag tag-impact">Yes 💥</span>' if r[12] == 1 else '<span class="tag tag-no-impact">No ⏭️</span>'
        
        nom_sp = f"({r[13]:.3f}, {r[14]:.3f}, {r[15]:.3f})" if r[13] is not None else "N/A"
        nom_ep = f"({r[16]:.3f}, {r[17]:.3f}, {r[18]:.3f})" if r[16] is not None else "N/A"
        act_sp = f"({r[19]:.3f}, {r[20]:.3f}, {r[21]:.3f})" if r[19] is not None else "N/A"
        act_ep = f"({r[22]:.3f}, {r[23]:.3f}, {r[24]:.3f})" if r[22] is not None else "N/A"
        ts = r[25]

        row_html = f"""
        <tr data-condition="{cond.lower()}" data-impact="{r[12]}">
            <td class="bold text-white">{f_name}</td>
            <td><span class="tag {cond_class}">{cond}</span></td>
            <td>{sweep_speed} m/s</td>
            <td>{battery}</td>
            <td>{impact_tag}</td>
            <td class="bold text-accent">{imp_speed} m/s</td>
            <td>{before_acc}</td>
            <td>{imp_acc}</td>
            <td>{imp_angle}</td>
            <td>{avg_dev} mm</td>
            <td>{max_dev} mm</td>
            <td>{rec_area} cm²</td>
            <td class="bold">{clearance} cm</td>
            <td class="coords">{nom_sp}</td>
            <td class="coords text-highlight">{act_sp}</td>
            <td class="coords">{nom_ep}</td>
            <td class="coords text-highlight">{act_ep}</td>
            <td class="time">{ts}</td>
        </tr>
        """
        table_rows_html.append(row_html)

    table_body = "\n".join(table_rows_html)

    html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Master Thesis: Collision Experiments Dashboard</title>
    <link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&display=swap" rel="stylesheet">
    <style>
        :root {{
            --bg-color: #0d0f12;
            --panel-bg: rgba(20, 24, 33, 0.7);
            --border-color: rgba(255, 255, 255, 0.08);
            --text-main: #b0b8c6;
            --text-white: #ffffff;
            --accent: #ff4a5a;
            --accent-glow: rgba(255, 74, 90, 0.15);
            --highlight: #00f0ff;
            --highlight-glow: rgba(0, 240, 255, 0.15);
            --rotating-color: #00ffd5;
            --fixed-color: #ffaa00;
        }}

        * {{
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }}

        body {{
            background-color: var(--bg-color);
            background-image: radial-gradient(circle at 50% 0%, #151a24 0%, #0d0f12 70%);
            color: var(--text-main);
            font-family: 'Inter', sans-serif;
            min-height: 100vh;
            padding: 2rem;
            line-height: 1.5;
        }}

        header {{
            margin-bottom: 2rem;
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 1px solid var(--border-color);
            padding-bottom: 1.5rem;
        }}

        h1 {{
            font-size: 1.8rem;
            font-weight: 700;
            color: var(--text-white);
            letter-spacing: -0.5px;
            background: linear-gradient(135deg, #ffffff 0%, #7d8b9e 100%);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }}

        .subtitle {{
            font-size: 0.9rem;
            color: #7d8b9e;
            margin-top: 0.25rem;
        }}

        .stats-grid {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
            gap: 1.5rem;
            margin-bottom: 2.5rem;
        }}

        .stat-card {{
            background: var(--panel-bg);
            backdrop-filter: blur(16px);
            border: 1px solid var(--border-color);
            border-radius: 12px;
            padding: 1.5rem;
            box-shadow: 0 8px 32px 0 rgba(0, 0, 0, 0.37);
            transition: transform 0.3s ease, border-color 0.3s ease;
        }}

        .stat-card:hover {{
            transform: translateY(-4px);
            border-color: rgba(255, 255, 255, 0.15);
        }}

        .stat-label {{
            font-size: 0.85rem;
            font-weight: 500;
            color: #7d8b9e;
            text-transform: uppercase;
            letter-spacing: 1px;
        }}

        .stat-value {{
            font-size: 2.2rem;
            font-weight: 700;
            color: var(--text-white);
            margin-top: 0.5rem;
            letter-spacing: -1px;
        }}

        .stat-value.accent {{
            color: var(--accent);
            text-shadow: 0 0 12px var(--accent-glow);
        }}

        .stat-value.highlight {{
            color: var(--highlight);
            text-shadow: 0 0 12px var(--highlight-glow);
        }}

        .controls {{
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 1.5rem;
            gap: 1rem;
            flex-wrap: wrap;
        }}

        .search-box {{
            background: var(--panel-bg);
            border: 1px solid var(--border-color);
            border-radius: 8px;
            padding: 0.75rem 1rem;
            color: var(--text-white);
            font-family: inherit;
            width: 300px;
            font-size: 0.9rem;
            outline: none;
            transition: border-color 0.3s ease, box-shadow 0.3s ease;
        }}

        .search-box:focus {{
            border-color: var(--highlight);
            box-shadow: 0 0 12px var(--highlight-glow);
        }}

        .filter-buttons {{
            display: flex;
            gap: 0.5rem;
        }}

        .btn {{
            background: rgba(255, 255, 255, 0.03);
            border: 1px solid var(--border-color);
            padding: 0.6rem 1.2rem;
            color: var(--text-main);
            border-radius: 8px;
            font-size: 0.85rem;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.2s ease;
        }}

        .btn:hover, .btn.active {{
            background: var(--text-white);
            color: var(--bg-color);
            border-color: var(--text-white);
            box-shadow: 0 4px 12px rgba(255, 255, 255, 0.2);
        }}

        .table-container {{
            background: var(--panel-bg);
            backdrop-filter: blur(16px);
            border: 1px solid var(--border-color);
            border-radius: 12px;
            overflow-x: auto;
            box-shadow: 0 8px 32px 0 rgba(0, 0, 0, 0.37);
        }}

        table {{
            width: 100%;
            border-collapse: collapse;
            text-align: left;
            font-size: 0.85rem;
        }}

        th {{
            background: rgba(255, 255, 255, 0.02);
            color: var(--text-white);
            font-weight: 600;
            padding: 1.2rem 1rem;
            border-bottom: 1px solid var(--border-color);
            white-space: nowrap;
            cursor: pointer;
            user-select: none;
        }}

        th:hover {{
            background: rgba(255, 255, 255, 0.05);
        }}

        td {{
            padding: 1.2rem 1rem;
            border-bottom: 1px solid rgba(255, 255, 255, 0.04);
            white-space: nowrap;
        }}

        tr:last-child td {{
            border-bottom: none;
        }}

        tr:hover td {{
            background: rgba(255, 255, 255, 0.015);
        }}

        .bold {{
            font-weight: 600;
        }}

        .text-white {{
            color: var(--text-white);
        }}

        .text-accent {{
            color: var(--accent);
        }}

        .text-highlight {{
            color: var(--highlight);
        }}

        .tag {{
            display: inline-block;
            padding: 0.25rem 0.6rem;
            border-radius: 6px;
            font-size: 0.75rem;
            font-weight: 600;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }}

        .cond-fixed {{
            background: rgba(255, 170, 0, 0.1);
            color: var(--fixed-color);
            border: 1px solid rgba(255, 170, 0, 0.2);
        }}

        .cond-rotating {{
            background: rgba(0, 255, 213, 0.1);
            color: var(--rotating-color);
            border: 1px solid rgba(0, 255, 213, 0.2);
        }}

        .tag-impact {{
            background: rgba(255, 74, 90, 0.1);
            color: var(--accent);
            border: 1px solid rgba(255, 74, 90, 0.2);
        }}

        .tag-no-impact {{
            background: rgba(176, 184, 198, 0.05);
            color: var(--text-main);
            border: 1px solid rgba(176, 184, 198, 0.1);
        }}

        .coords {{
            font-family: monospace;
            font-size: 0.8rem;
            color: #7d8b9e;
        }}

        .time {{
            color: #5b697c;
            font-size: 0.8rem;
        }}
    </style>
</head>
<body>
    <header>
        <div>
            <h1>Collision Experiments Database</h1>
            <div class="subtitle">Master Thesis Dynamic Slicing & Telemetry Verification Pipeline</div>
        </div>
        <div class="filter-buttons">
            <button class="btn active" onclick="filterCondition('all', this)">All Runs</button>
            <button class="btn" onclick="filterCondition('fixed', this)">Fixed Cage</button>
            <button class="btn" onclick="filterCondition('rotating', this)">Rotating Cage</button>
        </div>
    </header>

    <div class="stats-grid">
        <div class="stat-card">
            <div class="stat-label">Total Experiment Passes</div>
            <div class="stat-value highlight">{total_runs}</div>
        </div>
        <div class="stat-card">
            <div class="stat-label">Impact Rate</div>
            <div class="stat-value accent">{impact_rate:.1f}%</div>
        </div>
        <div class="stat-card">
            <div class="stat-label">Impact Count</div>
            <div class="stat-value">{impact_runs}</div>
        </div>
    </div>

    <div class="controls">
        <input type="text" class="search-box" id="searchInput" placeholder="Search flights..." onkeyup="searchFlights()">
    </div>

    <div class="table-container">
        <table id="expTable">
            <thead>
                <tr>
                    <th onclick="sortTable(0)">Flight Name & Pass</th>
                    <th onclick="sortTable(1)">Condition</th>
                    <th onclick="sortTable(2)">Nominal Speed</th>
                    <th onclick="sortTable(3)">Battery</th>
                    <th onclick="sortTable(4)">Impact?</th>
                    <th onclick="sortTable(5)">Impact Speed</th>
                    <th onclick="sortTable(6)">Before Accel</th>
                    <th onclick="sortTable(7)">Impact Accel</th>
                    <th onclick="sortTable(8)">Impact Angle</th>
                    <th onclick="sortTable(9)">Avg Dev</th>
                    <th onclick="sortTable(10)">Max Dev</th>
                    <th onclick="sortTable(11)">Recovery Area</th>
                    <th onclick="sortTable(12)">Min Clearance</th>
                    <th>Nominal SP</th>
                    <th>Actual SP</th>
                    <th>Nominal EP</th>
                    <th>Actual EP</th>
                    <th onclick="sortTable(17)">Recorded Timestamp</th>
                </tr>
            </thead>
            <tbody>
                {table_body}
            </tbody>
        </table>
    </div>

    <script>
        let currentFilter = 'all';

        function filterCondition(type, btn) {{
            currentFilter = type;
            document.querySelectorAll('.filter-buttons .btn').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            applyFilters();
        }}

        function searchFlights() {{
            applyFilters();
        }}

        function applyFilters() {{
            const searchVal = document.getElementById('searchInput').value.toLowerCase();
            const rows = document.querySelectorAll('#expTable tbody tr');
            
            rows.forEach(row => {{
                const name = row.cells[0].textContent.toLowerCase();
                const cond = row.getAttribute('data-condition');
                
                const matchesSearch = name.includes(searchVal);
                const matchesFilter = (currentFilter === 'all') || (cond.includes(currentFilter));
                
                if (matchesSearch && matchesFilter) {{
                    row.style.display = '';
                }} else {{
                    row.style.display = 'none';
                }}
            }});
        }}

        function sortTable(n) {{
            const table = document.getElementById("expTable");
            let rows, switching, i, x, y, shouldSwitch, dir, switchcount = 0;
            switching = true;
            dir = "asc";
            while (switching) {{
                switching = false;
                rows = table.rows;
                for (i = 1; i < (rows.length - 1); i++) {{
                    shouldSwitch = false;
                    x = rows[i].getElementsByTagName("TD")[n];
                    y = rows[i + 1].getElementsByTagName("TD")[n];
                    let valX = x.textContent || x.innerText;
                    let valY = y.textContent || y.innerText;
                    
                    // Parse numbers for sorting if applicable
                    let numX = parseFloat(valX.replace(/[^\d\.-]/g, ''));
                    let numY = parseFloat(valY.replace(/[^\d\.-]/g, ''));
                    
                    if (!isNaN(numX) && !isNaN(numY)) {{
                        valX = numX;
                        valY = numY;
                    }}
                    
                    if (dir == "asc") {{
                        if (valX > valY) {{
                            shouldSwitch = true;
                            break;
                        }}
                    }} else if (dir == "desc") {{
                        if (valX < valY) {{
                            shouldSwitch = true;
                            break;
                        }}
                    }}
                }}
                if (shouldSwitch) {{
                    rows[i].parentNode.insertBefore(rows[i + 1], rows[i]);
                    switching = true;
                    switchcount++;
                }} else {{
                    if (switchcount == 0 && dir == "asc") {{
                        dir = "desc";
                        switching = true;
                    }}
                }}
            }}
        }}
    </script>
</body>
</html>
"""

    with open(HTML_PATH, "w") as f:
        f.write(html_content)
    print(f"🎉 Successfully exported beautiful HTML dashboard to: {HTML_PATH}")

if __name__ == "__main__":
    generate_html()
