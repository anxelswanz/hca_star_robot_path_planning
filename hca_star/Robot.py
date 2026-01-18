from enum import Enum, auto

class Robot:
    def __init__(self, rid, start_pos, priority):
        self.rid = rid
        self.pos = list(start_pos)
        self.priority = priority
        self.status = RobotState.IDLE # 状态：IDLE (空闲), WORKING (任务中), PENDING (挂起)
        self.target = None  # 记录当前的目标点
    def __repre__(self):
        return f"Robot({self.rid}, pos={self.pos}, priority={self.priority})"




class RobotState(Enum):
    IDLE = auto()      # 空闲：没有任务，原地待命
    WORKING = auto()   # 任务中：正在按照规划路径移动
    PENDING = auto()   # 挂起：有任务但被阻塞，正在“排队位”等待补位