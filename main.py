# Project 1
import random
import astar
from Cell import Cell
from BinaryHeap import BinaryHeap
import os
import matplotlib.pyplot as plt
import time
print('Test4')

# reads maze files and converts them to a 2d array
def create_grid(filename):
    # for filename in os.listdir(directory):
    # if filename.endswith(".txt"):
    file = open("arrs/randGrid/" + filename)
    lines = [line.split() for line in file]
    return lines


# picks the starting node, recursively calls until it finds an unblocked cell
def point_picker(arr):

    while True:

        start_row = random.randint(0, len(arr) - 1)
        start_col = random.randint(0, len(arr[0]) - 1)

        goal_row = random.randint(0, len(arr) - 1)
        goal_col = random.randint(0, len(arr[0]) - 1)

        if not(int(arr[start_row][start_col]) or int(arr[goal_row][goal_col])):
            break

    #(start_row, start_col)
    #print(goal_row, goal_col)

    # initial heuristic
    goal_cell = Cell(goal_row, goal_col,0)
    start_cell = Cell(start_row, start_col,0)
    #print("Heuristic: " + str(astar.get_heuristic(start_cell, goal_cell)))
    return [start_cell, goal_cell]


def arr_contains_pair(expanded, cell):
    for i in expanded:
        if i.x == cell.x and i.y == cell.y:
            return 1
    return 0

def backtrace(cell):
    solution = []
    parent = cell
    while parent is not None:
        solution.insert(0, parent)
        parent = parent.parent
    return solution


def update_agent_vision(node):
    children = [Cell(node.x - 1, node.y, 0), Cell(node.x, node.y + 1, 0),
                Cell(node.x + 1, node.y, 0), Cell(node.x, node.y - 1, 0)]
    for child in children:
        if 0 <= child.x < len(maze) and 0 <= child.y < len(maze[0]):
            agent_vision[child.x][child.y] = maze[child.x][child.y]

def compute_path(min_heap, start_cell, goal_cell, mode):
    expanded = []
    start_cell.g = astar.get_heuristic(start_cell, goal_cell)
    start_cell.f = start_cell.g
    min_heap.insert(start_cell)
    if mode == "forward":
        children = [Cell(start_cell.x - 1, start_cell.y, 0), Cell(start_cell.x, start_cell.y + 1, 0),
                    Cell(start_cell.x + 1, start_cell.y, 0), Cell(start_cell.x, start_cell.y - 1, 0)]
    elif mode == "backward":
        children = [Cell(goal_cell.x - 1, goal_cell.y, 0), Cell(goal_cell.x, goal_cell.y + 1, 0),
                    Cell(goal_cell.x + 1, goal_cell.y, 0), Cell(goal_cell.x, goal_cell.y - 1, 0)]
    else:
        #print("Illegal Mode Argument in Compute_Path")
        return -1
    for child in children:
        if 0 <= child.x < len(maze) and 0 <= child.y < len(maze[0]):
            agent_vision[child.x][child.y] = maze[child.x][child.y]

    while min_heap.get_min() is not None:
        currNode = min_heap.extract_min()
        children = [Cell(currNode.x - 1, currNode.y, 0), Cell(currNode.x, currNode.y + 1, 0), Cell(currNode.x + 1, currNode.y, 0), Cell(currNode.x, currNode.y - 1, 0)]
        for child in children:
            if 0 <= child.x < len(maze) and 0 <= child.y < len(maze[0]) and arr_contains_pair(expanded, child) == 0 and int(agent_vision[child.x][child.y]) != 1 and arr_contains_pair(min_heap.heap, child) == 0:
                child.parent = currNode
                child.h = astar.get_heuristic(child, goal_cell)
                child.g = currNode.g + 1
                child.f = child.h + child.g
                min_heap.insert(child)
        expanded.append(currNode)
        if currNode.x == goal_cell.x and currNode.y == goal_cell.y:
            return backtrace(currNode)
    #print("Unreachable Goal")
    return -1


"""
Optimized Methods for Repeated A* only re-calculate A* once the agent has reached a wall.
Lazy Methods for repeated A* re-calculate A* after every singular movement of the agent in any direction.
"""


def repeated_forward_optimized(start_cell, goal_cell):
    end_node = start_cell
    while end_node.x != goal_cell.x or end_node.y != goal_cell.y:
        path = compute_path(BinaryHeap(), end_node, goal_cell, "forward")
        if path != -1:
            for node in path:
                if int(maze[node.x][node.y]) == 1:
                    break
                update_agent_vision(node)
                end_node = node
        else:
            return -1
    return backtrace(end_node)


def repeated_backward_optimized(start_cell, goal_cell):
    solution = [start_cell]
    while solution[-1].x != goal_cell.x or solution[-1].y != goal_cell.y:
        path = compute_path(BinaryHeap(), goal_cell, solution[-1], "backward")
        if path != -1:
            for node in reversed(path):
                if int(maze[node.x][node.y]) == 1:
                    break
                if node.x != solution[-1].x or node.y != solution[-1].y:
                    solution.append(Cell(node.x, node.y, 0))
                    update_agent_vision(node)
        else:
            return -1
    return solution


def repeated_forward_lazy(start_cell, goal_cell):
    solution = [start_cell]
    while solution[-1].x != goal_cell.x or solution[-1].y != goal_cell.y:
        path = compute_path(BinaryHeap(), solution[-1], goal_cell, "forward")
        if path != -1:
            solution.append(Cell(path[1].x, path[1].y, 0))
        else:
            return -1
    return solution


def repeated_backward_lazy(start_cell, goal_cell):
    solution = [start_cell]
    while solution[-1].x != solution[-1].x or solution[-1].y != goal_cell.y:
        path = compute_path(BinaryHeap(), goal_cell, solution[-1], "backward")
        if path != -1:
            solution.append(Cell(path[-2].x, path[-2].y, 0))
        else:
            return -1
    return solution

def draw_path(maze, x):
    int_maze = [list(map(int, i)) for i in maze]
    plt.figure()
    plt.imshow(int_maze, cmap=plt.cm.binary, interpolation='nearest')
    plt.xticks([])  # remove the tick marks by setting to an empty list
    plt.yticks([])
    plt.savefig("results/maze{0:0=2d}.png".format(x))
    plt.close()
    #plt.show()

if __name__ == "__main__":
    start_time = time.time()
    #maze = create_grid()
    #agent_vision = [[0 for x in range(len(maze[0]))] for y in range(len(maze))]
    #points = point_picker(maze)
    #points = [Cell(4, 74, 0), Cell(42, 15, 0)]
    #solution = repeated_forward_lazy(points[0], points[1])
    directory = 'arrs/randGrid'
    x = 0
    for filename in os.listdir(directory):
        if filename.endswith(".txt"):
            maze = create_grid(filename)
            agent_vision = [[0 for x in range(len(maze[0]))] for y in range(len(maze))]
            #print(filename)
            points = point_picker(maze)
            solution = repeated_forward_optimized(points[0], points[1])
            if solution != -1:
                #print("Path Cost = " + str(len(solution) - 1))
                for cell in solution:
                    maze[cell.x][cell.y] = '3'
                    #print("(" + str(cell.x) + ", " + str(cell.y) + ")")
                draw_path(maze, x)
        x = x + 1
        #time.sleep(0.8)
    end_time = time.time()
    print(end_time - start_time)

