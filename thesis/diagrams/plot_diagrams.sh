#!/bin/bash
set -e

# Directory where the script is located
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

# Ensure npx is available
if ! command -v npx &> /dev/null; then
    echo "Error: npx is required but not installed. Please install Node.js and npm."
    exit 1
fi

echo "Scanning for Mermaid diagrams in .md files..."

for md_file in *.md; do
    # Skip standard non-diagram markdown files if needed
    if [ "$md_file" = "diagrams_skill.md" ]; then
        continue
    fi
    
    # Check if the file contains a mermaid block
    if grep -q '```mermaid' "$md_file" || head -n 1 "$md_file" | grep -q '^```mermaid'; then
        echo "Processing $md_file..."
        base_name="${md_file%.md}"
        mmd_file="${base_name}.mmd"
        png_file="${base_name}.png"
        
        # Check if it's already just a raw mermaid file disguised as .md (no text outside block)
        # Or if it's a normal markdown file with a block.
        # We handle both by using awk to extract lines between ```mermaid and ```
        # If the file starts with ```mermaid but misses the ending ```, we handle it as well.
        awk '
            /^```mermaid/ { flag=1; next }
            /^```/ { if(flag) { flag=0; next } }
            flag { print }
        ' "$md_file" > "$mmd_file"
        
        # Fallback: if the file starts with ```mermaid and doesn't have an ending ```, 
        # the awk above will capture until EOF. Let's make sure it handles files 
        # where there are no fences at all, or just top fences.
        if [ ! -s "$mmd_file" ]; then
            # Maybe the file didn't have ```mermaid, but starts with graph or flowchart?
            # Or maybe it has no ``` end block.
            if grep -Eq '^(graph|flowchart|sequenceDiagram|classDiagram|stateDiagram|pie|gantt)' "$md_file"; then
                 # Remove any leading ```mermaid if it exists
                 sed -e 's/^```mermaid//' -e 's/^```//' "$md_file" > "$mmd_file"
            fi
        fi

        # If extraction was successful and the file is not empty
        if [ -s "$mmd_file" ]; then
            # Run mermaid-cli to generate PNG
            npx --yes @mermaid-js/mermaid-cli -i "$mmd_file" -o "$png_file"
            echo "Successfully generated $png_file"
        else
            echo "Warning: No valid mermaid content extracted from $md_file"
        fi
        
        # Clean up temporary .mmd file
        rm -f "$mmd_file"
    fi
done

echo "Done!"
