import sys
import numpy as np
from PIL import Image

def image_to_map(image_path, map_size_x, map_size_y, cells_per_meter, output_path):
    # Open the image and convert it to grayscale
    img = Image.open(image_path).convert('L')
    
    # Resize the image to fit the desired map size in cells
    width_in_cells = int(map_size_x * cells_per_meter)
    height_in_cells = int(map_size_y * cells_per_meter)
    img = img.resize((width_in_cells, height_in_cells), Image.LANCZOS)
    
    # Convert image to numpy array
    img_array = np.array(img)
    
    # Normalize pixel values to log-odds
    log_odds_map = -(img_array / 255.0) * 254 + 127
    log_odds_map = log_odds_map.astype(int)
    
    # Create the header for the map file
    global_origin_x = -map_size_x / 2.0
    global_origin_y = -map_size_y / 2.0
    header = f"{global_origin_x} {global_origin_y} {width_in_cells} {height_in_cells} {1/cells_per_meter}\n"
    
    # Write the map to the file
    with open(output_path, 'w') as f:
        f.write(header)
        for row in log_odds_map:
            row_str = " ".join(map(str, row))
            f.write(row_str + '\n')

if __name__ == "__main__":
    if len(sys.argv) < 5:
        print("Usage: python generate_map.py <input_image> <map_size_x> <map_size_y> <cells_per_meter> <output_map>")
        sys.exit(1)
    
    input_image = sys.argv[1]
    map_size_x = float(sys.argv[2])
    map_size_y = float(sys.argv[3])
    cells_per_meter = float(sys.argv[4])
    output_map = sys.argv[5]

    image_to_map(input_image, map_size_x, map_size_y, cells_per_meter, output_map)
