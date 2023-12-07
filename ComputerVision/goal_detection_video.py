import cv2
import numpy as np
import os

def divide_image(image, rows, cols, top_border):
    # Get image dimensions
    height, width = image.shape[:2]

    # Calculate the height and width of each grid cell
    cell_height = (height - top_border) // rows
    cell_width = width // cols

    # Divide the image into a grid of cells, starting from the top_border
    cells = [
        image[
            top_border + i * cell_height: top_border + (i + 1) * cell_height,
            j * cell_width: (j + 1) * cell_width,
        ]
        for i in range(rows)
        for j in range(cols)
    ]

    #  # Create an image to visualize the cells
    # cell_visualization = np.zeros_like(image)

    # # Draw rectangles around each cell
    # for i in range(rows):
    #     for j in range(cols):
    #         x1 = j * cell_width
    #         y1 = top_border + i * cell_height
    #         x2 = (j + 1) * cell_width
    #         y2 = top_border + (i + 1) * cell_height
    #         cv2.rectangle(cell_visualization, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # # Display the visualization
    # cv2.imshow("Cell Visualization", cell_visualization)

    return cells

def calculate_light_pixel_concentration(image_part):
    # Convert the image part to grayscale
    gray_part = cv2.cvtColor(image_part, cv2.COLOR_BGR2GRAY)

    # Create a histogram of pixel intensities
    histogram = cv2.calcHist([gray_part], [0], None, [256], [0, 256])

    # Calculate the concentration of light pixels (higher intensity values)
    light_pixel_concentration = np.sum(histogram[200:]) / np.sum(histogram)

    return light_pixel_concentration

def find_light_interest_regions(image, rows, cols, num_regions, top_border):
    # Divide the image into a grid of cells
    image_cells = divide_image(image, rows, cols, top_border)

    # print(len(image_cells))

    # Calculate light pixel concentration for each cell
    concentrations = [calculate_light_pixel_concentration(cell) for cell in image_cells]

    # Find the indices of the top N cells with the highest light pixel concentration
    top_indices = np.argsort(concentrations)[-num_regions:]

      # Display the top regions with rectangles
    display_top_regions(image, rows, cols, top_indices, top_border)

    return top_indices, [image_cells[i] for i in top_indices]

def find_top_border(image):
    # Convert the image to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for the green color
    lower_green = np.array([32, 112, 16])  # Adjust as needed
    upper_green = np.array([101, 255, 255])

    # Threshold the image to extract the green color
    green_mask = cv2.inRange(hsv, lower_green, upper_green)

     # Display the green mask for visualization
    cv2.imshow("Green Mask", green_mask)

    # Find contours in the green mask
    contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Find the contour with the minimum y-coordinate (top border)
        min_y = min(contours, key=lambda c: cv2.boundingRect(c)[1])[0][0][1]
        return min_y
    else:
        return 0  # Default to 0 if no contours are foun


# Debug functions
def display_top_regions(image, rows, cols, top_indices, top_border):
    # Draw rectangles around the top regions for visualization
    for index in top_indices:
        i, j = divmod(index, cols)

        # Calculate the coordinates in the original image
        x = j * (image.shape[1] // cols)
        y = top_border + i * (image.shape[0] - top_border) // rows
        w = image.shape[1] // cols
        h = (image.shape[0] - top_border) // rows

        # Draw the rectangle
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

def draw_contours(image, contours, index, num_cols, num_rows, top_border):
    offset_x = (index % num_cols) * (image.shape[1] // num_cols)
    offset_y = top_border + (index // num_cols) * (image.shape[0] - top_border) // num_rows
    
    # Draw the contours on the original image with correct offset
    for contour in contours:
        contour_with_offset = contour + (offset_x, offset_y)
        cv2.drawContours(image, [contour_with_offset], -1, (0, 0, 255), 2)

def calculate_centers(contours, index, num_cols, num_rows, top_border, goal_centers, image):
    offset_x = (index % num_cols) * (image.shape[1] // num_cols)
    offset_y = top_border + (index // num_cols) * (image.shape[0] - top_border) // num_rows
    for contour in contours:
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            goal_centers.append((cX + offset_x, cY + offset_y))

    return goal_centers

def main():
    # Open a connection to the webcam (use 0 for default webcam)
    cap = cv2.VideoCapture(0)

    # Number of rows and columns to divide the image into
    num_rows = 6
    num_cols = 6

    # Number of top regions to select
    num_top_regions = 12

    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()
        if not ret:
            break

        # Find the top border of the grass
        top_border = find_top_border(frame)

        # Display the top border for visualization
        cv2.line(frame, (0, top_border), (frame.shape[1], top_border), (0, 255, 255), 2)

        # Find the top N regions with the highest light pixel concentration, discarding the part above the top border
        top_indices, top_regions = find_light_interest_regions(frame, num_rows, num_cols, num_top_regions, top_border)

        # Calculate the center of the goal posts
        goal_centers = []
        for index, region in zip(top_indices, top_regions):
            gray_region = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)
            _, binary_mask = cv2.threshold(gray_region, 150, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Draw the contours on the original image with correct offset
            draw_contours(frame, contours, index, num_cols, num_rows, top_border)
            goal_centers = calculate_centers(contours, index, num_cols, num_rows, top_border, goal_centers, frame)

        # Calculate the average center of the goal posts
        if goal_centers:
            avg_center = tuple(np.mean(goal_centers, axis=0, dtype=int))

            # Print the average center coordinates
            print(f"Avg Center Coordinates: {avg_center}")

            # Draw a marker at the average center
            cv2.drawMarker(frame, avg_center, (255, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)

        # # Display the result
        cv2.imshow("Goal contours and center", frame)

        # Exit when 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()