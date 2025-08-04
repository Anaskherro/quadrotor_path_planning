import numpy as np

# Read the map.pgm file
def read_pgm(file_path):
    with open(file_path, 'r') as file:
        # Read the first three lines (P2, width, height) and ignore them
        for _ in range(3):
            file.readline()
        
        # Read the rest of the file
        lines = file.readlines()
        
        # Convert the lines into a 2D array of integers
        data = np.array([[int(value) for value in line.split()] for line in lines])
    
    return data

# def downsample_map(original_map, downsampling_factor):
#     # Compute dimensions of the downscaled map
#     new_width = original_map.shape[1] // downsampling_factor
#     new_height = original_map.shape[0] // downsampling_factor
    
#     # Initialize downscaled map
#     downscaled_map = np.zeros((new_height, new_width), dtype=np.uint8)
    
#     # Perform downsampling using average pooling
#     for i in range(0, new_height):
#         for j in range(0, new_width):
#             block = original_map[i * downsampling_factor: (i + 1) * downsampling_factor,
#                                   j * downsampling_factor: (j + 1) * downsampling_factor]
#             downscaled_map[i, j] = np.mean(block)
    
#     return downscaled_map

def downsample_map(original_map, downsampling_factor):
    # Compute dimensions of the downscaled map
    new_width = original_map.shape[1] // downsampling_factor
    new_height = original_map.shape[0] // downsampling_factor
    
    # Initialize downscaled map
    downscaled_map = np.zeros((new_height, new_width), dtype=np.uint8)
    
    # Perform downsampling
    for i in range(new_height):
        for j in range(new_width):
            block = original_map[i * downsampling_factor: (i + 1) * downsampling_factor,
                                  j * downsampling_factor: (j + 1) * downsampling_factor]
            # Check if all pixels in the block are white (255)
            if np.all(block == 255):
                downscaled_map[i, j] = 255  # Keep white
            else:
                downscaled_map[i, j] = 0  # Convert to black
    
    return downscaled_map

# Read the original map from map.pgm
file_path = 'map.pgm'
original_map = read_pgm(file_path)

# Define downsampling factor
downsampling_factor = 8  # Reduce map size by half in both dimensions

# Perform downsampling
downscaled_map = downsample_map(original_map, downsampling_factor)

# Save downscaled map to a new PGM file
def save_pgm(file_path, data):
    with open(file_path, 'w') as file:
        file.write("P2\n")
        file.write(f"{data.shape[1]} {data.shape[0]}\n")  # width height
        file.write("255\n")  # max gray value
        for row in data:
            for value in row:
                file.write(f"{value} ")
            file.write("\n")

# Save downscaled map to a new PGM file
output_file_path = 'downscaled_map.pgm'
save_pgm(output_file_path, downscaled_map)

print(f"Downscaled map saved to {output_file_path}")
