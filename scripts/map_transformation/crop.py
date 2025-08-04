def read_pgm(file_name):
    with open(file_name, 'r') as f:
        # Read the magic number
        magic_number = f.readline().strip()
        if magic_number != 'P2':
            raise ValueError("Only plain text PGM files (P2) are supported.")
        
        # Read the image dimensions
        width, height = map(int, f.readline().split())
        
        # Read the maximum gray value
        max_gray_value = int(f.readline())
        
        # Read the image data
        image_data = []
        for _ in range(height):
            row = list(map(int, f.readline().split()))
            image_data.extend(row)
        
    return width, height, image_data

def find_black_region(image_matrix):
    height = len(image_matrix)
    width = len(image_matrix[0])
    left, upper, right, lower = width, height, 0, 0

    for y in range(height):
        for x in range(width):
            if image_matrix[y][x] == 0:  # Black
                left = min(left, x)
                upper = min(upper, y)
                right = max(right, x)
                lower = max(lower, y)

    return left, upper, right, lower

def crop_pgm(image_path, output_path):
    # Read the PGM file
    width, height, image_data = read_pgm(image_path)
    
    # Convert the image data to a matrix
    image_matrix = [image_data[i:i+width] for i in range(0, len(image_data), width)]
    
    # Find the bounding box of the black region
    left, upper, right, lower = find_black_region(image_matrix)
    
    # Crop the image matrix using the bounding box
    cropped_matrix = [row[left:right+1] for row in image_matrix[upper:lower+1]]
    
    # Write the cropped image matrix to a new PGM file
    with open(output_path, 'w') as f:
        # Write the header
        f.write('P2\n')
        f.write(f"{right - left + 1} {lower - upper + 1}\n")
        f.write("255\n")
        
        # Write the image data
        for row in cropped_matrix:
            f.write(' '.join(map(str, row)) + '\n')

# Example usage
input_path = "downscaled_map.pgm"
output_path = "cropped_map_1.pgm"

crop_pgm(input_path, output_path)
