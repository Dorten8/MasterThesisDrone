#!/bin/bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

# Ensure mmdc is available
if ! command -v mmdc &> /dev/null; then
    echo "Error: mmdc (Mermaid CLI) is required. Install via: npm install -g @mermaid-js/mermaid-cli"
    exit 1
fi

OUTPUT_DIR="$DIR/../Figures"
mkdir -p "$OUTPUT_DIR"

echo "Rendering all .mmd diagrams to PDF..."

for mmd_file in *.mmd; do
    [ -f "$mmd_file" ] || continue
    base_name="${mmd_file%.mmd}"
    pdf_file="$OUTPUT_DIR/${base_name}.pdf"

    echo "  Rendering $mmd_file -> $pdf_file ..."
    mmdc -i "$mmd_file" -o "$pdf_file" -f
    echo "  OK $base_name.pdf generated"
done

echo "Done! All diagrams rendered to $OUTPUT_DIR"
