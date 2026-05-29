import json
import os

def refactor_notebook(path):
    print(f"Refactoring {path}...")
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    changed = False
    for cell in data.get('cells', []):
        if 'source' in cell:
            new_source = []
            for line in cell['source']:
                orig = line
                # Reload cache guard key replacement
                line = line.replace("'experiments_analysis'", "'dev_logs.analysis'")
                line = line.replace('"experiments_analysis"', '"dev_logs.analysis"')
                
                # Direct imports
                line = line.replace("from experiments_analysis import run, compare_all_angles", 
                                    "from dev_logs.analysis.database import run\nfrom dev_logs.analysis.kinematics import compare_all_angles")
                
                # Sub-module imports
                line = line.replace("experiments_analysis.exa_loader", "dev_logs.analysis.database.db_loader")
                line = line.replace("experiments_analysis.exa_kinematics", "dev_logs.analysis.kinematics.kin_calculator")
                line = line.replace("experiments_analysis.exa_plot_kinematics", "dev_logs.analysis.kinematics.kin_plot_kinematics")
                line = line.replace("experiments_analysis.exa_plot_trajectory", "dev_logs.analysis.kinematics.kin_plot_trajectory")
                line = line.replace("experiments_analysis.exa_plot_statistics", "dev_logs.analysis.kinematics.kin_plot_statistics")
                line = line.replace("experiments_analysis.exa_database", "dev_logs.analysis.database.db_manager")
                line = line.replace("experiments_analysis.exa_pipeline", "dev_logs.analysis.database.db_pipeline")

                # Warning path sanitization in outputs / execution logs
                line = line.replace("experiments_analysis/exa_loader.py", "database/db_loader.py")
                line = line.replace("experiments_analysis/exa_database.py", "database/db_manager.py")
                line = line.replace("experiments_analysis/exa_pipeline.py", "database/db_pipeline.py")
                line = line.replace("experiments_analysis/exa_kinematics.py", "kinematics/kin_calculator.py")
                line = line.replace("experiments_analysis/exa_plot_kinematics.py", "kinematics/kin_plot_kinematics.py")
                line = line.replace("experiments_analysis/exa_plot_trajectory.py", "kinematics/kin_plot_trajectory.py")
                line = line.replace("experiments_analysis/exa_plot_statistics.py", "kinematics/kin_plot_statistics.py")

                if line != orig:
                    changed = True
                new_source.append(line)
            cell['source'] = new_source
            
        if 'outputs' in cell:
            for out in cell['outputs']:
                if 'text' in out:
                    new_text = []
                    for line in out['text']:
                        orig = line
                        line = line.replace("experiments_analysis/exa_loader.py", "database/db_loader.py")
                        line = line.replace("experiments_analysis/exa_database.py", "database/db_manager.py")
                        line = line.replace("experiments_analysis/exa_pipeline.py", "database/db_pipeline.py")
                        line = line.replace("experiments_analysis/exa_kinematics.py", "kinematics/kin_calculator.py")
                        line = line.replace("experiments_analysis/exa_plot_kinematics.py", "kinematics/kin_plot_kinematics.py")
                        line = line.replace("experiments_analysis/exa_plot_trajectory.py", "kinematics/kin_plot_trajectory.py")
                        line = line.replace("experiments_analysis/exa_plot_statistics.py", "kinematics/kin_plot_statistics.py")
                        if line != orig:
                            changed = True
                        new_text.append(line)
                    out['text'] = new_text

    if changed:
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=1, ensure_ascii=False)
        print(f"Refactored successfully: {path}")
    else:
        print(f"No changes required for {path}")

# Run for both notebooks
refactor_notebook("/home/dorten/pi_drone_sshfs/dev_logs/analysis/experiments_analysis.ipynb")
refactor_notebook("/home/dorten/pi_drone_sshfs/dev_logs/analysis/diagnostics.ipynb")
