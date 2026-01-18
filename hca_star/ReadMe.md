
HCA* v1 重要的思想
(1) 寻路算法还是使用的A*, A*重要的思想就是heruistc function, 将预测的Manhattan Distance 拉入到考虑的范围内, f(n) = g(n) + h(n)
(2) 
a. Hierachical Cooperative a* 属于multi agent path planning思想，将二维的地图变为三维，把时间维度考虑进去，引入了重要的table, 
reserved_table, 意味着在(1, 1)是不够的，必须在t=1的时刻在(1, 1)
b. Hierachical Copperative a* 同时基于 Priority-Based, 按照robot的优先级来进行路径规划，高优先级先规划路径
c. Robot除了有移动，同时还有等待 (0, 0), 这里有一个重要的一点:
    之前是g_score[neighbour], 现在是[node_key], 其中node_key为((nx, ny), t), 这不仅仅是简单意义上在二维的层面进行搜索，而是在三维
    的层面进行搜索，回溯你可能会得到：(2,2,t=3) -> (2,2,t=4) -> (2,2,t=5) -> (3,2,t=6)


# 2026.01.17 解决agent目标点和占用点问题 1. 占用点agent自动避让
# 2026.01.18 升级预约表数据结构