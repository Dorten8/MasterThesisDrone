import os
import sys

# Ensure this directory is in the import path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import core diagnostic functions from renamed files
from dia_analyze_flight import main as analyze_flight
from dia_analyze_deep_dissect import run_deep_dissection as analyze_deep_dissection

__all__ = [
    'analyze_flight',
    'analyze_deep_dissection'
]

print("📊 Thesis Flight Diagnostics Dashboard (diagnostics_main) Loaded!")
print("Available programmatic call handles:")
print("  - analyze_flight(target=None)                    -> Full Dynamic Trajectory and Sag profile")
print("  - analyze_deep_dissection(bag=None, ulog=None)   -> High-frequency gap & fusion rate dissection")
print("=========================================================================================\n")
