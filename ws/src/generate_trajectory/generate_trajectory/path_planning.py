import numpy as np
import heapq

class PathPlanner:
    def __init__(self, workspace_size=(640, 480), resolution=5):
        self.width = int(workspace_size[0] / resolution)
        self.height = int(workspace_size[1] / resolution)
        self.res = resolution
        
    def _to_grid(self, x, y):
        grid_x = int(np.clip(x / self.res, 0, self.width - 1))
        grid_y = int(np.clip(y / self.res, 0, self.height - 1))
        return grid_x, grid_y

    def _to_world(self, grid_x, grid_y):
        return grid_x * self.res, grid_y * self.res

    def create_grid_map(self, obstacles, safety_margin_mm=20):
        grid = np.zeros((self.width, self.height))
        for obs in obstacles:
            obs_x, obs_y = obs['x'], obs['y']
            total_radius = obs['radius'] + safety_margin_mm
            min_gx, min_gy = self._to_grid(obs_x - total_radius, obs_y - total_radius)
            max_gx, max_gy = self._to_grid(obs_x + total_radius, obs_y + total_radius)
            for gx in range(min_gx, max_gx + 1):
                for gy in range(min_gy, max_gy + 1):
                    wx, wy = self._to_world(gx, gy)
                    if np.hypot(wx - obs_x, wy - obs_y) <= total_radius:
                        grid[gx, gy] = 1
        return grid

    def a_star(self, start, goal, grid):
        start_g = self._to_grid(*start)
        goal_g = self._to_grid(*goal)
        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        open_set = []
        heapq.heappush(open_set, (0, start_g))
        came_from = {}
        g_score = {start_g: 0}
        f_score = {start_g: np.hypot(start_g[0]-goal_g[0], start_g[1]-goal_g[1])}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            if current == goal_g:
                path = []
                while current in came_from:
                    path.append(self._to_world(*current))
                    current = came_from[current]
                path.append(self._to_world(*start_g))
                return path[::-1]
                
            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height:
                    if grid[neighbor[0], neighbor[1]] == 1:
                        continue
                    weight = np.hypot(dx, dy)
                    tentative_g_score = g_score[current] + weight
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        h_score = np.hypot(neighbor[0] - goal_g[0], neighbor[1] - goal_g[1])
                        f_score[neighbor] = tentative_g_score + h_score
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return None

    def smooth_path(self, path, weight_data=0.5, weight_smooth=0.1, tolerance=0.00001):
        if not path:
            return path
        new_path = np.copy(path).astype(float)
        change = tolerance
        while change >= tolerance:
            change = 0.0
            for i in range(1, len(path) - 1):
                for j in range(2):
                    aux = new_path[i][j]
                    new_path[i][j] += weight_data * (path[i][j] - new_path[i][j]) + \
                                      weight_smooth * (new_path[i-1][j] + new_path[i+1][j] - 2.0 * new_path[i][j])
                    change += abs(aux - new_path[i][j])
        return new_path.tolist()
