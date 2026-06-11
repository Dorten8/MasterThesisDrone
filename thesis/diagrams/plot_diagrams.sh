#!/bin/bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

# Ensure mmdc is available
if ! command -v mmdc &> /dev/null; then
    echo "Error: mmdc (Mermaid CLI) is required. Install via: npm install -g @mermaid-js/mermaid-cli"
    exit 1
fi

PDF_DIR="$DIR/../Figures"
SVG_DIR="$DIR"
mkdir -p "$PDF_DIR"

echo "Rendering all .mmd diagrams to SVG (diagrams/) + PDF (Figures/)..."

for mmd_file in *.mmd; do
    [ -f "$mmd_file" ] || continue
    base_name="${mmd_file%.mmd}"

    # SVG — preview in diagrams folder
    svg_file="$SVG_DIR/${base_name}.svg"
    echo "  SVG:  $mmd_file -> ${base_name}.svg"
    mmdc -i "$mmd_file" -o "$svg_file" -b transparent
    echo "    OK"

    # PDF — publication vector format for LaTeX Figures/
    pdf_file="$PDF_DIR/${base_name}.pdf"
    echo "  PDF:  $mmd_file -> ../Figures/${base_name}.pdf"
    mmdc -i "$mmd_file" -o "$pdf_file" -b white
    echo "    OK"
done

echo ""
echo "✅ All done! SVGs: $SVG_DIR  |  PDFs: $PDF_DIR"
