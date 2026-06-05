#!/usr/bin/env python3
import os
import glob
import sys
try:
    from PIL import Image
except ImportError:
    print("[ERROR] Pillow is required. Please install it using 'pip install Pillow'")
    sys.exit(1)

def stack_pass_plots(flight_dir, metric_keyword):
    """
    Finds all pass-specific plots in a flight directory matching the metric keyword
    and stitches them vertically into a single massive image for easy comparison.
    """
    if not os.path.isdir(flight_dir):
        print(f"[ERROR] Directory not found: {flight_dir}")
        return

    # Find all matching PNGs
    # e.g. pass01_kinetic_profile.png, pass02_kinetic_profile.png
    search_pattern = os.path.join(flight_dir, f"pass*_{metric_keyword}.png")
    image_paths = sorted(glob.glob(search_pattern))

    if not image_paths:
        print(f"[WARN] No plots found matching '{metric_keyword}' in {os.path.basename(flight_dir)}")
        return

    print(f"[INFO] Found {len(image_paths)} '{metric_keyword}' plots to stack.")

    images = [Image.open(p) for p in image_paths]
    
    # Calculate the total height and maximum width
    widths, heights = zip(*(i.size for i in images))
    max_width = max(widths)
    total_height = sum(heights)

    # Create a new blank image with white background
    stacked_image = Image.new('RGB', (max_width, total_height), color='white')

    # Paste the images vertically
    y_offset = 0
    for img in images:
        stacked_image.paste(img, (0, y_offset))
        y_offset += img.size[1]

    output_filename = f"stacked_{metric_keyword}.png"
    output_path = os.path.join(flight_dir, output_filename)
    stacked_image.save(output_path)
    print(f"[SUCCESS] Saved stacked plot to: {output_path}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Stack pass metric plots vertically.")
    parser.add_argument("flight_dir", help="Path to the flight directory containing pass plots.")
    parser.add_argument("metric_keyword", help="Keyword to match, e.g. 'kinetic_profile' or 'imu_dynamics'")
    
    args = parser.parse_args()
    
    if args.flight_dir.lower() == 'all':
        flights_dir = "/home/dorten/MasterThesisDrone/dev_logs/flights"
        for folder in os.listdir(flights_dir):
            flight_path = os.path.join(flights_dir, folder)
            if os.path.isdir(flight_path) and folder.startswith("flight_"):
                stack_pass_plots(flight_path, args.metric_keyword)
    else:
        stack_pass_plots(args.flight_dir, args.metric_keyword)
