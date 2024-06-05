#!/home/lucas/cornEnv/bin/python

import os
from svgpathtools import parse_path, Path, Line, CubicBezier, QuadraticBezier
import matplotlib.pyplot as plt
from scipy.spatial import distance_matrix
from scipy.optimize import linear_sum_assignment

class StrokePlanner:
    def __init__(self, svg_file_path):
        self.svg_file_path = svg_file_path

    def approximate_bezier_curve(self, bezier_curve, num_samples=100):
        return [bezier_curve.point(t/num_samples) for t in range(num_samples + 1)]

    def generate_robot_commands_from_path(self, path, move_command="MOVE TO", pen_up="PEN UP", pen_down="PEN DOWN"):
        commands = []
        coordinates = []  # This will store the (x, y) tuples

        for segment in path:
            if isinstance(segment, Line):
                start = segment.start
                end = segment.end
                commands.append(f"{pen_up}")
                coordinates.append((start.real, start.imag))
                commands.append(f"{pen_down}")
                coordinates.append((end.real, end.imag))
                commands.append(f"{pen_up}")
            elif isinstance(segment, (CubicBezier, QuadraticBezier)):
                points = self.approximate_bezier_curve(segment)
                commands.append(f"{pen_down}")
                for point in points:
                    x, y = point.real, point.imag
                    coordinates.append((x, y))
                commands.append(f"{pen_up}")

        return commands, coordinates

    def generate_shading_commands_from_region(self, region, move_command="MOVE TO", pen_up="PEN UP", pen_down="PEN DOWN", density=0.1):
        shading_commands = []
        shading_coordinates = []

        for y in range(int(region['y_min']), int(region['y_max']), int(1/density)):
            for x in range(int(region['x_min']), int(region['x_max']), int(1/density)):
                shading_commands.append(f"{pen_down}")
                shading_coordinates.append((x, y))
                shading_commands.append(f"{pen_up}")

        return shading_commands, shading_coordinates

    def parse_svg_file_and_generate_commands(self):
        print("Current working directory:", os.getcwd())
        print("Full path to SVG file:", os.path.abspath(self.svg_file_path))
        
        with open(self.svg_file_path, 'r') as file:
            svg_content = file.read().split('<path')
            
        path_data = [part.split('d="')[1].split('"')[0] for part in svg_content if 'd="' in part]
        paths = [parse_path(d) for d in path_data]
        
        all_commands = []
        all_coordinates = []
        for path in paths:
            commands, coordinates = self.generate_robot_commands_from_path(path)
            all_commands.extend(commands)
            all_coordinates.extend(coordinates)
        
        return all_commands, all_coordinates

    def commands_to_binary(self, commands):
        binary_mapping = {
            "PEN UP": "0",
            "PEN DOWN": "1"
        }
        
        binary_output = ''.join([binary_mapping.get(cmd, "0") for cmd in commands])
        
        return binary_output

    def generate_and_plot_commands(self):
        drawing_commands, coordinates = self.parse_svg_file_and_generate_commands()

        shading_region = {'x_min': 50, 'y_min': 50, 'x_max': 150, 'y_max': 150}
        # shading_commands, shading_coordinates = self.generate_shading_commands_from_region(shading_region)

        all_commands = drawing_commands # + shading_commands
        all_coordinates = coordinates # + shading_coordinates

        dist_matrix = distance_matrix(all_coordinates, all_coordinates)

        row_ind, col_ind = linear_sum_assignment(dist_matrix)

        ordered_commands = [all_commands[i] for i in col_ind]
        ordered_coordinates = [all_coordinates[i] for i in col_ind]

        binary_output = self.commands_to_binary(ordered_commands)

        x_vals, y_vals = zip(*ordered_coordinates)
        plt.figure(figsize=(10, 8))
        plt.plot(x_vals, y_vals, 'o-', markersize=2)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.gca().invert_yaxis()
        plt.title('Robot Stroke Plan Visualization (Optimized)')
        plt.xlabel('X coordinate')
        plt.ylabel('Y coordinate')
        plt.show()

        return binary_output

if __name__ == "__main__":
    path = "line_detect_data/test.svg"
    planner = StrokePlanner(path)
    planner.generate_and_plot_commands()
