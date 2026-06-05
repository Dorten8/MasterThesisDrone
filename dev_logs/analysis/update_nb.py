import nbformat

nb_path = 'experiments_analysis.ipynb'
with open(nb_path, 'r') as f:
    nb = nbformat.read(f, as_version=4)

# We want to keep Cell 0 (imports).
new_cells = [nb.cells[0]]

# Cell 1: Unified Sweep Markdown
cell1_md = """### 📐 2. UNIFIED SWEEP COLLISION EXPERIMENTS
This runs the complete analytics and visualization pipeline for the unified collision sweep experiments.
It processes both Fixed and Rotating cage passes, creating stacked comparative plots for tracking, velocities, and motor outputs."""
new_cells.append(nbformat.v4.new_markdown_cell(cell1_md))

# Cell 2: Code for running unified pipeline
cell2_code = """# Define the flights to process
flights_rotating_cage = [
    "flight_20260529-1210_45°_column_collision_loop_rotating_cage - Pass-02",
    "flight_20260526-0931_75°_column_collision_loop_rotating_cage - Pass-04"
]
flights_fixed_cage = [
    "flight_20260528-1655_45°_column_collision_loop_fixed_cage - Pass-01",
    "flight_20260524-1904_75°_column_collision_loop_fixed_cage - Pass-01"
]

results = run(
    label="Collision Sweep",
    angle_deg=None,
    column_x=0.408,
    column_y=0.358,
    flights_rotating_cage=flights_rotating_cage,
    flights_fixed_cage=flights_fixed_cage,
    representative_rotating_cage=0,
    representative_fixed_cage=0,
    project_root=project_root
)
"""
new_cells.append(nbformat.v4.new_code_cell(cell2_code))

# Cell 3: Aggregate Analysis Markdown
cell3_md = """### 📈 3. STATISTICAL AGGREGATE PERFORMANCE ANALYSIS
This section queries the SQLite database to generate comparative boxplots, scatterplots, and heatmaps across all passes, forming the primary quantitative findings of the thesis."""
new_cells.append(nbformat.v4.new_markdown_cell(cell3_md))

# Cell 4: Aggregate Analysis Code
cell4_code = """import sqlite3
import pandas as pd

# Load data from the database
conn = sqlite3.connect(os.path.join(analysis_dir, "experiments_summary.db"))
df_flights = pd.read_sql_query("SELECT * FROM flights_summary", conn)
conn.close()

# Generate Aggregate Plots
# TODO: Implement aggregated visualizations of motors output fixed vs rotating
# TODO: Implement IMU Collision Dynamic heatmaps

print(f"Loaded {len(df_flights)} passes for aggregate analysis.")
"""
new_cells.append(nbformat.v4.new_code_cell(cell4_code))

nb.cells = new_cells

with open(nb_path, 'w') as f:
    nbformat.write(nb, f)

print("Notebook updated successfully.")
