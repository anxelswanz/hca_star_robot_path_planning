import time
from Robot import Robot
from AStar import AStar


class Simulation:
    def __init__(self):
        self.size = 8
        self.obstacles = set()
        self.robots = [
            Robot('A', [0, 0], priority=10),
            Robot('B', [7, 7], priority=5)
        ]
        self.astar = AStar()
        self.reserved_position = {}

    def _reserve_path(self, path, rid, max_horizon=100):
        """
        将给定路径记录到时空资源表（reserved_position 和 reserved_edges）中
        """
        last_t = len(path) - 1
        last_pos = path[-1]
        for t, pos in enumerate(path):
            # 记录位置占用 (Vertex Conflict)
            if t not in self.reserved_position:
                self.reserved_position[t] = {}
            self.reserved_position[t][pos] = rid

            # 记录路径边占用 (Edge Conflict)
            if t > 0:
                if t not in self.reserved_edges:
                    self.reserved_edges[t] = {}
                self.reserved_edges[t][(path[t - 1], path[t])] = rid
        # 这样比它晚到的高优先级机器人就会知道这个格子一直有人
        for t in range(last_t + 1, max_horizon):
            if t not in self.reserved_position:
                self.reserved_position[t] = {}
            self.reserved_position[t][last_pos] = rid

    def plan_paths(self, targets):
        # reserved_position 是Dict类型: 在reserved_position[t] 个时刻 (x, y) 这个pos被占用 => 解决vertex conflict
        self.reserved_position = {}
        # reserved_edges[t] 用于存储 (start_pos, end_pos)，代表t-1时刻有机器人从 start_pos 到 t 时刻的 end_pos
        self.reserved_edges = {}
        planned_paths = {}


        sorted_robots = sorted(self.robots, key=lambda r: r.priority, reverse=True)

        for robot in sorted_robots:
            rid = robot.rid  # 假设 Robot 类有 id 属性
            if rid not in targets: continue


            goal = targets[rid]
            # 默认起点是机器人的当前位置
            current_start = tuple(robot.pos)

            #清理之前的路径
            self._clear_robot_reservations(rid)
            # 处理冲突：如果我是高优先级，这一步可能会给低优先级机器人分配避让路径并预留资源
            self._handle_goal_conflict(rid, targets[rid], planned_paths)

            # 检查这个机器人是否已经有了路径（比如刚刚被高优先级机器人踢走的避让路径）
            existing_path = planned_paths.get(rid, [])

            if existing_path:
                # 如果已经有路径了，新的规划起点应该是避让路径的终点
                current_start = existing_path[-1]
                t_offset = len(existing_path) - 1
            else:
                t_offset = 0
            # 这里的 dynamic_obstacles 逻辑保持不变
            dynamic_obstacles = self.obstacles.copy()


            path = self.astar.get_path(current_start,
                                       goal,
                                       dynamic_obstacles,
                                       self.reserved_position,
                                       self.reserved_edges,
                                       t_offset)

            if path:
                planned_paths[rid] = path
                self._reserve_path(path, rid)
            else:
                print(f"Robot {rid} 无法找到路径")


        return planned_paths

    def _handle_goal_conflict(self, rid, goal, planned_paths):
        # 1. 查找当前机器人对象
        robot = next(r for r in self.robots if r.rid == rid)

        # 2. 查找是否有其他机器人停在我的目标点上
        conflict_robot = None
        for other in self.robots:
            if other.rid != rid and tuple(other.pos) == goal:
                conflict_robot = other
                break

        if not conflict_robot:
            return True

        if robot.priority > conflict_robot.priority:
            print(f"[调度] {rid}(P{robot.priority}) 优先级高，强制 {conflict_robot.rid} 避让")
            forbidden = self.obstacles.copy()
            forbidden.add(goal)

            for r in self.robots: forbidden.add(tuple(r.pos))
            eva_goal = self._find_nearest_safe_pos(tuple(conflict_robot.pos), forbidden)
            if eva_goal:
                b_path = self.astar.get_path(tuple(conflict_robot.pos), eva_goal, self.obstacles, self.reserved_position, self.reserved_edges)
                if b_path:
                    planned_paths[conflict_robot.rid] = b_path
                    self._reserve_path(b_path, conflict_robot.rid)
            return True
        else:
            #TODO 监听补位
            return True
        return 1

    def _clear_robot_reservations(self, rid):
        # 清除位置预约
        for t in list(self.reserved_position.keys()):
            # 过滤掉属于该 rid 的坐标
            self.reserved_position[t] = {
                pos: r for pos, r in self.reserved_position[t].items() if r != rid
            }

            if not self.reserved_position[t]:
                del self.reserved_position[t]

        # 清除边预约
        for t in list(self.reserved_edges.keys()):
            self.reserved_edges[t] = {
                edge: r for edge, r in self.reserved_edges[t].items() if r != rid
            }
            if not self.reserved_edges[t]:
                del self.reserved_edges[t]

    def _find_nearest_safe_pos(self, start_pos, forbidden_positions):
        """
        start_pos: 机器人当前位置 (x, y)
        forbidden_positions: 集合，包含障碍物、其他机器人的目标点等
        """
        # 搜索队列：(x, y, distance)
        queue = [(start_pos[0], start_pos[1], 0)]
        visited = {tuple(start_pos)}

        while queue:
            cx, cy, dist = queue.pop(0)

            # 检查当前点是否安全（不是障碍物，不在禁用名单，且不是起点本身）
            if (cx, cy) not in forbidden_positions and (cx, cy) != tuple(start_pos):
                # 边界检查
                if 0 <= cx < self.size and 0 <= cy < self.size:
                    return (cx, cy)

            # 向四周扩散搜索
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < self.size and 0 <= ny < self.size:
                    if (nx, ny) not in visited:
                        visited.add((nx, ny))
                        queue.append((nx, ny, dist + 1))
        return None  # 没找到

    def display(self, planned_paths=None):
        """
        打印网格状态，显示机器人位置
        """
        print("\n" + "=" * (self.size * 2))
        for y in range(self.size):
            line = ""  # 每一行开始前清空字符串
            for x in range(self.size):
                pos = (x, y)
                # 检查当前位置是否有机器人
                robot_here = None
                for robot in self.robots:
                    # 假设 Robot 类属性是 pos，如果是 position 请自行修改
                    if tuple(robot.pos) == pos:
                        robot_here = robot.rid
                        break

                if robot_here:
                    line += robot_here + " "
                elif planned_paths:
                    # 路径显示逻辑
                    in_path = any(pos in path for path in planned_paths.values())
                    line += "* " if in_path else "+ "
                else:
                    line += "+ "

            # 【关键点】在完成一整行 (x 循环) 的拼接后，才打印这一行并换行
            print(line)


    def execute_paths(self, planned_paths):
        """
         按时间步执行机器人路径，显示每一步
        """
        max_len = max(len(p) for p in planned_paths.values())
        print("\n" + f"maxlen= {max_len}")
        for t in range(max_len):
            for robot in self.robots:
                if robot.rid in planned_paths:
                    path = planned_paths[robot.rid]
                    if t < len(path):
                        # 更新机器人对象的位置属性
                        robot.pos = list(path[t])
            self.display()
            time.sleep(0.5)


if __name__ == "__main__":
    sim = Simulation()
    print("输入格式：机器人ID 目标X 目标Y (例如：A 2 3)")

    while True:
        sim.display()
        raw_input = input("\n 输入指令 (EXIT退出)：").upper().strip()

        if raw_input == 'EXIT': break

        user_input = raw_input.split()
        if len(user_input) % 3 != 0 or not user_input:
            print("格式错误，请按 'ID X Y' 格式输入")
            continue

        valid = True
        targets = {}
        # 提取当前所有机器人的 ID 用于校验
        existing_ids = [r.rid for r in sim.robots]

        for i in range(0, len(user_input), 3):
            rid = user_input[i]
            try:
                tx, ty = int(user_input[i + 1]), int(user_input[i + 2])
            except ValueError:
                print("坐标必须是数字")
                valid = False;
                break

            if not (0 <= tx < sim.size and 0 <= ty < sim.size):
                print(f"坐标 ({tx}, {ty}) 超出边界")
                valid = False;
                break

            if rid not in existing_ids:
                print(f"机器人 {rid} 不存在")
                valid = False;
                break

            targets[rid] = (tx, ty)

        if valid:
            paths = sim.plan_paths(targets)
            sim.execute_paths(paths)
