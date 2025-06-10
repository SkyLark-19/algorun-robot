"""
Map Visualizer and Path Optimization Tool
Loads the map created by SLAM and provides path planning capabilities
FIXED VERSION - Added proper boundary constraints and obstacle inflation
"""

import json
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import heapq
import math
import os
import cv2


class MapOptimizer:
    def __init__(self, map_file="robot_exploration_map.json", robot_radius=0.05):
        
        """Load map data from JSON file"""
        with open(map_file, 'r') as f:
            self.map_data = json.load(f)
        
        self.occupancy_grid = np.array(self.map_data['occupancy_grid'])
        self.map_width = self.map_data['map_width']
        self.map_height = self.map_data['map_height']
        self.resolution = self.map_data['resolution']
        self.path_points = self.map_data['path_points']
        self.visited_cells = set(tuple(cell) for cell in self.map_data['visited_cells'])
        
        # Robot safety parameters
        self.robot_radius = robot_radius
        self.safety_margin_cells = max(1, int(robot_radius / self.resolution))
        
        # Create inflated occupancy grid for safe path planning
        self.safe_grid = self.create_safe_navigation_grid()
        
        print(f"Loaded map: {self.map_width}x{self.map_height} cells, resolution: {self.resolution}m/cell")
        print(f"Robot radius: {robot_radius}m, Safety margin: {self.safety_margin_cells} cells")
        
    def create_safe_navigation_grid(self):
        """Create an inflated occupancy grid that accounts for robot size and boundaries"""
        # Start with original grid
        safe_grid = self.occupancy_grid.copy()
        
        # Convert to binary format for inflation (0=obstacle/unknown, 1=free)
        binary_grid = np.zeros_like(safe_grid, dtype=np.uint8)
        binary_grid[safe_grid == 1] = 1  # Only free space is navigable
        
        # Inflate obstacles and boundaries
        if self.safety_margin_cells > 0:
            kernel_size = 2 * self.safety_margin_cells + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
            
            # Erode free space (equivalent to inflating obstacles)
            binary_grid = cv2.erode(binary_grid, kernel, iterations=1)
        
        # Add boundary constraints - mark boundary cells as obstacles
        boundary_margin = max(2, self.safety_margin_cells)
        binary_grid[:boundary_margin, :] = 0  # Top boundary
        binary_grid[-boundary_margin:, :] = 0  # Bottom boundary
        binary_grid[:, :boundary_margin] = 0  # Left boundary
        binary_grid[:, -boundary_margin:] = 0  # Right boundary
        
        return binary_grid
    
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        grid_x = int((x / self.resolution) + self.map_width // 2)
        grid_y = int((y / self.resolution) + self.map_height // 2)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates"""
        x = (grid_x - self.map_width // 2) * self.resolution
        y = (grid_y - self.map_height // 2) * self.resolution
        return x, y
    
    def is_valid_cell(self, x, y, use_safe_grid=True):
        """Check if cell is valid and safe for navigation"""
        # Check basic bounds
        if not (0 <= x < self.map_width and 0 <= y < self.map_height):
            return False
        
        # Use safe grid by default for path planning
        grid_to_check = self.safe_grid if use_safe_grid else self.occupancy_grid
        
        if use_safe_grid:
            return grid_to_check[y, x] == 1  # Safe free space
        else:
            return grid_to_check[y, x] == 1  # Original free space
    
    def is_position_safe(self, world_x, world_y):
        """Check if a world position is safe for the robot"""
        grid_x, grid_y = self.world_to_grid(world_x, world_y)
        return self.is_valid_cell(grid_x, grid_y, use_safe_grid=True)
    
    def get_neighbors(self, x, y):
        """Get valid neighboring cells (8-connected) using safe grid"""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if self.is_valid_cell(nx, ny, use_safe_grid=True):
                    # Cost is higher for diagonal moves
                    cost = math.sqrt(dx*dx + dy*dy)
                    neighbors.append((nx, ny, cost))
        return neighbors
    
    def heuristic(self, x1, y1, x2, y2):
        """Euclidean distance heuristic for A*"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def find_safe_nearby_position(self, world_x, world_y, search_radius=0.2):
        """Find a safe position near the given coordinates"""
        if self.is_position_safe(world_x, world_y):
            return world_x, world_y
        
        # Search in expanding circles
        max_search_cells = int(search_radius / self.resolution)
        center_x, center_y = self.world_to_grid(world_x, world_y)
        
        for radius in range(1, max_search_cells + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) == radius or abs(dy) == radius:  # Only check perimeter
                        test_x, test_y = center_x + dx, center_y + dy
                        if self.is_valid_cell(test_x, test_y, use_safe_grid=True):
                            return self.grid_to_world(test_x, test_y)
        
        return None  # No safe position found
    
    def a_star_path(self, start_world, goal_world):
        """Find optimal path using A* algorithm with safety constraints"""
        # Find safe positions near start and goal if needed
        safe_start = self.find_safe_nearby_position(start_world[0], start_world[1])
        safe_goal = self.find_safe_nearby_position(goal_world[0], goal_world[1])
        
        if safe_start is None:
            print(f"Cannot find safe start position near ({start_world[0]:.3f}, {start_world[1]:.3f})")
            return None
        
        if safe_goal is None:
            print(f"Cannot find safe goal position near ({goal_world[0]:.3f}, {goal_world[1]:.3f})")
            return None
        
        start_x, start_y = self.world_to_grid(*safe_start)
        goal_x, goal_y = self.world_to_grid(*safe_goal)
        
        print(f"Planning path from ({safe_start[0]:.3f}, {safe_start[1]:.3f}) to ({safe_goal[0]:.3f}, {safe_goal[1]:.3f})")
        
        # Priority queue: (f_score, g_score, x, y)
        open_set = [(0, 0, start_x, start_y)]
        came_from = {}
        g_score = {(start_x, start_y): 0}
        f_score = {(start_x, start_y): self.heuristic(start_x, start_y, goal_x, goal_y)}
        closed_set = set()
        
        while open_set:
            current_f, current_g, current_x, current_y = heapq.heappop(open_set)
            
            if (current_x, current_y) in closed_set:
                continue
            
            closed_set.add((current_x, current_y))
            
            if current_x == goal_x and current_y == goal_y:
                # Reconstruct path
                path = []
                while (current_x, current_y) in came_from:
                    world_x, world_y = self.grid_to_world(current_x, current_y)
                    path.append((world_x, world_y))
                    current_x, current_y = came_from[(current_x, current_y)]
                
                # Add start position
                world_x, world_y = self.grid_to_world(start_x, start_y)
                path.append((world_x, world_y))
                path.reverse()
                
                return path
            
            for next_x, next_y, move_cost in self.get_neighbors(current_x, current_y):
                if (next_x, next_y) in closed_set:
                    continue
                
                tentative_g = g_score[(current_x, current_y)] + move_cost
                
                if (next_x, next_y) not in g_score or tentative_g < g_score[(next_x, next_y)]:
                    came_from[(next_x, next_y)] = (current_x, current_y)
                    g_score[(next_x, next_y)] = tentative_g
                    f_score[(next_x, next_y)] = tentative_g + self.heuristic(next_x, next_y, goal_x, goal_y)
                    heapq.heappush(open_set, (f_score[(next_x, next_y)], tentative_g, next_x, next_y))
        
        print("No safe path found!")
        return None
    
    def find_unexplored_frontiers(self):
        """Find frontier cells (boundaries between free and unknown space)"""
        frontiers = []
        
        # Use original grid for frontier detection, but check safety
        for y in range(1, self.map_height - 1):
            for x in range(1, self.map_width - 1):
                if self.occupancy_grid[y, x] == 1:  # Free cell in original grid
                    # Check if it borders unknown space
                    has_unknown_neighbor = False
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            if self.occupancy_grid[y + dy, x + dx] == 0:  # Unknown
                                has_unknown_neighbor = True
                                break
                        if has_unknown_neighbor:
                            break
                    
                    if has_unknown_neighbor:
                        # Check if this frontier is safely reachable
                        if self.is_valid_cell(x, y, use_safe_grid=True):
                            world_x, world_y = self.grid_to_world(x, y)
                            frontiers.append((world_x, world_y))
        
        return frontiers
    
    def optimize_exploration_path(self, current_position):
        """Find optimal path to explore remaining unknown areas"""
        frontiers = self.find_unexplored_frontiers()
        
        if not frontiers:
            print("No unexplored frontiers found!")
            return None
        
        # Find closest reachable frontier
        best_frontier = None
        best_path = None
        min_distance = float('inf')
        
        print(f"Checking {min(len(frontiers), 20)} frontiers for optimal path...")
        
        for i, frontier in enumerate(frontiers[:20]):  # Limit to 20 closest for performance
            path = self.a_star_path(current_position, frontier)
            if path:
                distance = len(path)
                if distance < min_distance:
                    min_distance = distance
                    best_frontier = frontier
                    best_path = path
                print(f"Frontier {i+1}: Path found, length {distance}")
            else:
                print(f"Frontier {i+1}: No safe path found")
        
        if best_path:
            return best_path, best_frontier
        else:
            print("No reachable frontiers found!")
            return None
    
    def visualize_map(self, path=None, frontiers=None, save_file=None):
        """Visualize the occupancy grid map with safety zones"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
        
        # Original map
        self._plot_map(ax1, self.occupancy_grid, "Original Map", path, frontiers)
        
        # Safe navigation map
        # Convert safe_grid back to occupancy format for visualization
        safe_display_grid = np.zeros_like(self.occupancy_grid)
        safe_display_grid[self.safe_grid == 1] = 1  # Free space
        safe_display_grid[self.safe_grid == 0] = 2  # Obstacles/unsafe areas
        
        self._plot_map(ax2, safe_display_grid, "Safe Navigation Map", path, frontiers)
        
        if save_file:
            plt.savefig(save_file, dpi=300, bbox_inches='tight')
            print(f"Map visualization saved to {save_file}")
        
        plt.tight_layout()
        plt.show()
    
    def _plot_map(self, ax, grid, title, path=None, frontiers=None):
        """Helper method to plot a single map"""
        # Create color map: unknown=gray, free=white, occupied=black
        display_grid = np.zeros((self.map_height, self.map_width, 3))
        
        for y in range(self.map_height):
            for x in range(self.map_width):
                if grid[y, x] == 0:  # Unknown/Unsafe
                    display_grid[y, x] = [0.5, 0.5, 0.5]  # Gray
                elif grid[y, x] == 1:  # Free
                    display_grid[y, x] = [1.0, 1.0, 1.0]  # White
                elif grid[y, x] == 2:  # Occupied
                    display_grid[y, x] = [0.0, 0.0, 0.0]  # Black
        
        ax.imshow(display_grid, origin='lower')
        
        # Plot original exploration path
        if self.path_points:
            path_x = []
            path_y = []
            for point in self.path_points[::5]:  # Every 5th point for clarity
                grid_x, grid_y = self.world_to_grid(point[0], point[1])
                if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                    path_x.append(grid_x)
                    path_y.append(grid_y)
            ax.plot(path_x, path_y, 'b-', alpha=0.5, linewidth=1, label='Exploration Path')
        
        # Plot optimized path if provided
        if path:
            opt_x = []
            opt_y = []
            for point in path:
                grid_x, grid_y = self.world_to_grid(point[0], point[1])
                opt_x.append(grid_x)
                opt_y.append(grid_y)
            ax.plot(opt_x, opt_y, 'r-', linewidth=3, label='Optimized Path')
            if opt_x:
                ax.plot(opt_x[0], opt_y[0], 'go', markersize=10, label='Start')
                ax.plot(opt_x[-1], opt_y[-1], 'ro', markersize=10, label='Goal')
        
        # Plot frontiers if provided
        if frontiers:
            frontier_x = []
            frontier_y = []
            for frontier in frontiers:
                grid_x, grid_y = self.world_to_grid(frontier[0], frontier[1])
                if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                    frontier_x.append(grid_x)
                    frontier_y.append(grid_y)
            ax.scatter(frontier_x, frontier_y, c='yellow', s=20, alpha=0.7, label='Frontiers')
        
        ax.set_title(title)
        ax.set_xlabel('Grid X')
        ax.set_ylabel('Grid Y')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    def generate_waypoints(self, path, waypoint_distance=0.1):
        """Generate waypoints from path for robot navigation"""
        if not path:
            return []
        
        waypoints = [path[0]]  # Start with first point
        
        for i in range(1, len(path)):
            # Calculate distance from last waypoint
            last_wp = waypoints[-1]
            current_point = path[i]
            
            distance = math.sqrt((current_point[0] - last_wp[0])**2 + 
                               (current_point[1] - last_wp[1])**2)
            
            if distance >= waypoint_distance:
                waypoints.append(current_point)
        
        # Always include the final point
        if waypoints[-1] != path[-1]:
            waypoints.append(path[-1])
        
        return waypoints
    
    def save_optimized_path(self, path, filename="optimized_path.json"):
        """Save optimized path for robot controller"""
        waypoints = self.generate_waypoints(path)
        
        path_data = {
            'full_path': path,
            'waypoints': waypoints,
            'total_distance': sum(math.sqrt((path[i+1][0] - path[i][0])**2 + 
                                          (path[i+1][1] - path[i][1])**2) 
                                for i in range(len(path)-1)),
            'num_waypoints': len(waypoints),
            'safety_margin': self.safety_margin_cells * self.resolution
        }
        
        with open(filename, 'w') as f:
            json.dump(path_data, f, indent=2)
        
        print(f"Optimized path saved to {filename}")
        print(f"Total distance: {path_data['total_distance']:.3f}m")
        print(f"Number of waypoints: {path_data['num_waypoints']}")
        print(f"Safety margin: {path_data['safety_margin']:.3f}m")
        
        return waypoints

# Example usage
if __name__ == "__main__":
    # Load and analyze the map with robot radius
    robot_radius = 0.05  # 5cm robot radius - adjust based on your robot
    optimizer = MapOptimizer("robot_exploration_map.json", robot_radius=robot_radius)
    
    # Get current robot position (last known position)
    if optimizer.path_points:
        current_pos = (optimizer.path_points[-1][0], optimizer.path_points[-1][1])
        print(f"Current robot position: ({current_pos[0]:.3f}, {current_pos[1]:.3f})")
    else:
        current_pos = (0.0, 0.0)  # Default start position
    
    # Check if current position is safe
    if not optimizer.is_position_safe(current_pos[0], current_pos[1]):
        print("Warning: Current position is not safe! Finding nearby safe position...")
        safe_pos = optimizer.find_safe_nearby_position(current_pos[0], current_pos[1])
        if safe_pos:
            current_pos = safe_pos
            print(f"Using safe position: ({current_pos[0]:.3f}, {current_pos[1]:.3f})")
        else:
            print("Cannot find safe starting position!")
    
    # Find unexplored frontiers
    frontiers = optimizer.find_unexplored_frontiers()
    print(f"Found {len(frontiers)} safe unexplored frontier points")
    
    # Find optimal exploration path
    if frontiers:
        result = optimizer.optimize_exploration_path(current_pos)
        if result:
            best_path, best_frontier = result
            print(f"Optimal safe path found to frontier at ({best_frontier[0]:.3f}, {best_frontier[1]:.3f})")
            print(f"Path length: {len(best_path)} points")
            
            # Save optimized path
            waypoints = optimizer.save_optimized_path(best_path)
            
            # Visualize everything
            optimizer.visualize_map(path=best_path, frontiers=frontiers[:50], 
                                  save_file="safe_optimized_map.png")
        else:
            print("No safe path found to any frontier")
            optimizer.visualize_map(frontiers=frontiers[:50], save_file="current_safe_map.png")
    else:
        print("No safe unexplored areas found - exploration complete or all areas unsafe!")
        optimizer.visualize_map(save_file="complete_safe_map.png")