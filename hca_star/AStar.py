import heapq

class AStar:
    def __init__(self, grid_size=8):
        self.size = grid_size

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_path(self, start, goal, obstacles, reserved_position=None):
        open_list = []
        heapq.heappush(open_list, (0 + self.heuristic(start, goal),
                                   0, (start, 0)))
        g_score = {(start, 0): 0}
        came_from = {}

        while open_list:
            f, g, (current, t) = heapq.heappop(open_list)
            if current == goal:
                path = []
                node = (current, t)
                while node in came_from:
                    path.append(node[0])
                    node = came_from[node]
                path.append(start)
                return path[::-1]

            # 这里加了一个动作：原地等待
            for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1), (0, 0)):
                nx = current[0] + dx
                ny = current[1] + dy
                neighbour = (nx, ny)
                nt = t + 1
                tentative_g_score = g + 1
                if 0 <= neighbour[0] < self.size and 0 <= neighbour[1] < self.size:
                    if neighbour in obstacles:
                        continue
                    if nt in reserved_position and neighbour in reserved_position[nt]:
                        continue

                    node_key = (neighbour, nt)
                    if node_key not in g_score or tentative_g_score < g_score[node_key]:
                        g_score[node_key] = tentative_g_score
                        f = tentative_g_score + self.heuristic(neighbour, goal)
                        heapq.heappush(open_list, (f, tentative_g_score, node_key))
                        came_from[node_key] = (current, t)

        return None



