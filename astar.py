from Cell import Cell


def get_heuristic(cell, goal_cell):
    x_final = abs(cell.x - goal_cell.x)
    y_final = abs(cell.y - goal_cell.y)
    dist = x_final + y_final
    return dist
