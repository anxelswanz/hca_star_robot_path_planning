class Robot:
    def __init__(self, rid, start_pos, priority):
        self.rid = rid
        self.pos = list(start_pos)
        self.priority = priority
    def __repre__(self):
        return f"Robot({self.rid}, pos={self.pos}, priority={self.priority})"