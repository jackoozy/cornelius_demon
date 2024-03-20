def contours_to_svg(contours, width=1000, height=1000, stroke_width=2, stroke_color="black"):
    # Start the SVG document
    svg_header = f'<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg">'
    svg_paths = ""
    svg_footer = '</svg>'
    
    # Process each contour
    for contour in contours:
        path_data = "M"  # Move to the starting point
        for i, point in enumerate(contour):
            if i == 0:
                path_data += f"{point[0]},{point[1]}"  # Starting point
            else:
                path_data += f" L{point[0]},{point[1]}"  # Line to for subsequent points
        path_data += " Z"  # Close the path for a closed contour
        # Include stroke-width and stroke-color in the path element
        svg_paths += f'<path d="{path_data}" fill="none" stroke="{stroke_color}" stroke-width="{stroke_width}"/>\n'
    
    # Combine parts to form the final SVG
    svg_content = svg_header + svg_paths + svg_footer
    return svg_content



# Example usage with no fill and adjusted stroke width
contours = [
    [(100, 100), (150, 100), (150, 150), (100, 150)],  # A square contour
    [(200, 200), (250, 200), (250, 250)]  # An open triangular contour
]
# Adjust the stroke width to 3 pixels and the stroke color to blue
svg_content = contours_to_svg(contours, stroke_width=3, stroke_color="blue")
print(svg_content)

# file_name = "src/selfie_drawing_robot/src/SVG_test/contours2.svg"
file_name = "src/SVG_test/contours2.svg"
with open(file_name, "w") as file:
    file.write(svg_content)
print(f"SVG saved to {file_name}")
