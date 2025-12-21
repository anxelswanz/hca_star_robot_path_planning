import time

from AStar import AStar


class Simulation:
    def __init__(self):
        self.size = 8
        self.obstacles = set()
        self.robots = {
            'A': [0, 0],
            'B': [7, 7]
        }
        self.astar = AStar()
        self.reserved_position = {}

    def display(self):
        print("\n" + "=" * 20)
        for y in range(0, self.size):
            line = ""
            for x in range(0, self.size):
                pos = (x, y)
                if [x, y] == self.robots['A']:
                    line += "A "
                elif [x, y] == self.robots['B']:
                    line += "B "
                else: line += "+ "
            print(line)

    def plan_paths(self, targets):
        self.reserved_position = {}
        planned_paths = {}

        for rid in sorted(targets.keys()):
            start = tuple(self.robots[rid])
            goal = targets[rid]
            #将其他机器人当前的位置看作障碍
            dynamic_obstacles = self.obstacles.copy()
            for orid, pos in self.robots.items():
                if orid != rid:
                    dynamic_obstacles.add(tuple(pos))
            path = self.astar.get_path(start, goal, dynamic_obstacles, self.reserved_position)
            if path:
                planned_paths[rid] = path
                # 更新reserved_position table, 避免时间冲突
                for t, pos in enumerate(path):
                    if t not in self.reserved_position:
                        self.reserved_position[t] = set()
                    self.reserved_position[t].add(pos)
            else:
                print(f"Robot {rid} 无法找到路径{goal}")
        return planned_paths

    def display(self, planned_paths=None):
        """
        打印网格状态，显示机器人位置
        planned_paths: 可选，显示每个机器人规划路径
        """
        print("\n" + "=" * (self.size * 2))
        for y in range(self.size):
            line = ""
            for x in range(self.size):
                pos = (x, y)
                # 显示机器人
                robot_here = None
                for rid, rpos in self.robots.items():
                    if tuple(rpos) == pos:
                        robot_here = rid
                        break
                if robot_here:
                    line += robot_here + " "
                elif planned_paths:
                    # 显示路径（用*表示）
                    in_path = False
                    for path in planned_paths.values():
                        if pos in path:
                            in_path = True
                            break
                    line += "* " if in_path else "+ "
                else:
                    line += "+ "
            print(line)


    def move_robot(self, robot_id, target_pos):
        start = (self.robots[robot_id][0], self.robots[robot_id][1])

        # 把所有在场机器人都算入 obstacle
        dynamic_obstacles = self.obstacles.copy()
        for rid, pos in self.robots.items():
            if rid != robot_id: dynamic_obstacles.add(tuple(pos))
        path = self.astar.get_path(start, target_pos, dynamic_obstacles)
        print("path =>" + str(path))
        if path:
            self.robots[robot_id] = list(path[0])

    def execute_paths(self, planned_paths):
        max_len = max(len(p) for p in planned_paths.values())
        """
         按时间步执行机器人路径，显示每一步
        """
        max_len = max(len(p) for p in planned_paths.values())
        for t in range(max_len):
            for rid, path in planned_paths.items():
                if t < len(path):
                    self.robots[rid] = list(path[t])
            self.display()
            time.sleep(1) #暂停0.3秒
        
        
if __name__ == "__main__":
    sim = Simulation()
    print("输入格式：机器人ID 目标X 目标Y (例如：A 2 3)")
    while True:
        sim.display()
        user_input = input("\n 输入指令：").upper().split()

        if user_input == 'EXIT': break
        if len(user_input) % 3 != 0:
            print("格式错误，请重新输入")
            continue
        valid =True
        targets = {}
        for i in range(0, len(user_input), 3):
            rid = user_input[i]
            try:
                tx = int(user_input[i + 1])
                ty = int(user_input[i + 2])
            except ValueError:
                print("位置必须是数字")
                valid = False
                break
            if not (0 <= tx < sim.size and 0 <= ty < sim.size):
                print("pos must be within the grid")
                valid = False
                continue
            if rid not in sim.robots:
                print("robot id not exist")
                valid = False
                continue
            targets[rid] = (tx, ty)
        if not valid:
            continue
        planned_paths = sim.plan_paths(targets)
        sim.execute_paths(planned_paths)
