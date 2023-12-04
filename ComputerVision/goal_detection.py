import cv2
import numpy as np
import os

def divide_image(image, rows, cols):
    # Get image dimensions
    height, width = image.shape[:2]

    # Calculate the height and width of each grid cell
    cell_height = height // rows
    cell_width = width // cols

    # Divide the image into a grid of cells
    cells = [image[i * cell_height:(i + 1) * cell_height, j * cell_width:(j + 1) * cell_width] for i in range(rows) for j in range(cols)]

    return cells

def calculate_light_pixel_concentration(image_part):
    # Convert the image part to grayscale
    gray_part = cv2.cvtColor(image_part, cv2.COLOR_BGR2GRAY)

    # Create a histogram of pixel intensities
    histogram = cv2.calcHist([gray_part], [0], None, [256], [0, 256])

    # Calculate the concentration of light pixels (higher intensity values)
    light_pixel_concentration = np.sum(histogram[200:]) / np.sum(histogram)

    return light_pixel_concentration

def find_light_interest_regions(image, rows, cols, num_regions):
    # Divide the image into a grid of cells
    image_cells = divide_image(image, rows, cols)

    # Calculate light pixel concentration for each cell
    concentrations = [calculate_light_pixel_concentration(cell) for cell in image_cells]

    # Find the indices of the top N cells with the highest light pixel concentration
    top_indices = np.argsort(concentrations)[-num_regions:]

    return top_indices, [image_cells[i] for i in top_indices]

def main():

    script_dir = os.path.dirname(__file__) 
    rel_path = "Img\\goal.jpg"
    image_path = os.path.join(script_dir, rel_path)
    image = cv2.imread(image_path)

    # Number of rows and columns to divide the image into
    num_rows = 6
    num_cols = 6

    # Number of top regions to select
    num_top_regions = 8

    # Find the top N regions with the highest light pixel concentration
    top_indices, top_regions = find_light_interest_regions(image, num_rows, num_cols, num_top_regions)

    # Calculate the center of the goal posts
    goal_centers = []
    for index, region in zip(top_indices, top_regions):
        # Convert the region to grayscale
        gray_region = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)
        
        # Threshold the region to create a binary mask
        _, binary_mask = cv2.threshold(gray_region, 200, 255, cv2.THRESH_BINARY)
        
        # Find contours in the binary mask
        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Offset the contours to the correct position in the original image
        for contour in contours:
            contour[:, 0, 0] += (index % num_cols) * (image.shape[1] // num_cols)
            contour[:, 0, 1] += (index // num_cols) * (image.shape[0] // num_rows)

            # Calculate the center of the contour using moments
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                goal_centers.append((cX, cY))

        # Draw the contours on the original image
        cv2.drawContours(image, contours, -1, (0, 255, 0), 2)

    # Calculate the average center of the goal posts
    if goal_centers:
        avg_center = tuple(np.mean(goal_centers, axis=0, dtype=int))

        # Print the average center coordinates
        print(f"Avg Center Coordinates: {avg_center}")

        # Draw a marker at the average center
        cv2.drawMarker(image, avg_center, (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)

        # Display the result
        cv2.imshow("Goal contours and center", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("No goal posts detected.")

if __name__ == "__main__":
    main()
