# Project 1
import random
import sys

import astar
from Cell import Cell
from BinaryHeap import BinaryHeap
import os
import matplotlib.pyplot as plt
import time

print('Test4')


# reads maze files and converts them to a 2d array
def create_grid(filename):
    file = open("arrs/randGrid/" + filename)
    lines = [line.split() for line in file]
    return lines


# picks the starting node until it finds goal and start are unblocked cells
def point_picker(arr):
    while True:

        start_row = random.randint(0, len(arr) - 1)
        start_col = random.randint(0, len(arr[0]) - 1)

        goal_row = random.randint(0, len(arr) - 1)
        goal_col = random.randint(0, len(arr[0]) - 1)

        if not (int(arr[start_row][start_col]) or int(arr[goal_row][goal_col])):
            break

    print(start_row, start_col)
    print(goal_row, goal_col)

    # initial heuristic
    goal_cell = Cell(goal_row, goal_col, 0)
    start_cell = Cell(start_row, start_col, 0)
    # print("Heuristic: " + str(astar.get_heuristic(start_cell, goal_cell)))
    return [start_cell, goal_cell]


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
    expandedList = [[0 for x in range(len(maze[0]))] for y in range(len(maze))]
    if mode == "forward":
        start_cell.h = astar.get_heuristic(start_cell, goal_cell)
        start_cell.g = 0
        start_cell.f = start_cell.h
    if mode == "backward":
        start_cell.g = astar.get_heuristic(start_cell, goal_cell)
        start_cell.h = 0
        start_cell.f = start_cell.g
    min_heap.insert(start_cell)
    update_agent_vision(start_cell)
    while min_heap.get_min() is not None:
        currNode = min_heap.extract_min()

        while expandedList[currNode.x][currNode.y] == 1 and min_heap.get_min() is not None:
            currNode = min_heap.extract_min()

        children = [Cell(currNode.x - 1, currNode.y, 0), Cell(currNode.x, currNode.y + 1, 0),
                    Cell(currNode.x + 1, currNode.y, 0), Cell(currNode.x, currNode.y - 1, 0)]
        for child in children:
            if 0 <= child.x < len(maze) and 0 <= child.y < len(maze[0]) and expandedList[child.x][child.y] == 0 and int(
                    agent_vision[child.x][child.y]) != 1:
                child.parent = currNode
                if mode == "forward":
                    child.h = astar.get_heuristic(child, goal_cell)
                    child.g = currNode.g + 1
                if mode == "backward":
                    child.g = astar.get_heuristic(child, goal_cell)
                    child.h = currNode.g + 1
                child.f = child.h + child.g
                min_heap.insert(child)
        expandedList[currNode.x][currNode.y] = 1
        if currNode.x == goal_cell.x and currNode.y == goal_cell.y:
            return backtrace(currNode)
    # print("Unreachable Goal")
    return -1


def compute_path_adaptive(min_heap, start_cell, goal_cell, prev_expanded, prev_cost):
    expandedList = [[0 for x in range(len(maze[0]))] for y in range(len(maze))]
    expanded = [[None for x in range(len(maze[0]))] for y in range(len(maze))]
    if prev_expanded[start_cell.x][start_cell.y] is not None:
        start_cell.h = prev_cost - prev_expanded[start_cell.x][start_cell.y]
    else:
        start_cell.h = astar.get_heuristic(start_cell, goal_cell)
    start_cell.g = 0
    start_cell.f = start_cell.h
    min_heap.insert(start_cell)
    update_agent_vision(start_cell)
    while min_heap.get_min() is not None:
        currNode = min_heap.extract_min()
        while expandedList[currNode.x][currNode.y] == 1 and min_heap.get_min() is not None:
            currNode = min_heap.extract_min()
        children = [Cell(currNode.x - 1, currNode.y, 0), Cell(currNode.x, currNode.y + 1, 0),
                    Cell(currNode.x + 1, currNode.y, 0), Cell(currNode.x, currNode.y - 1, 0)]
        for child in children:
            if 0 <= child.x < len(maze) and 0 <= child.y < len(maze[0]) and expandedList[child.x][child.y] == 0 and int(
                    agent_vision[child.x][child.y]) != 1:

                child.parent = currNode
                child.h = astar.get_heuristic(child, goal_cell)
                if prev_expanded[child.x][child.y] is not None:
                    child.h = prev_cost - prev_expanded[child.x][child.y]
                child.g = currNode.g + 1
                child.f = child.h + child.g
                min_heap.insert(child)
        expandedList[currNode.x][currNode.y] = 1
        expanded[currNode.x][currNode.y] = currNode.g
        if currNode.x == goal_cell.x and currNode.y == goal_cell.y:
            return backtrace(currNode), expanded
    # print("Unreachable Goal")
    return -1, -1


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
    end_node = start_cell
    while end_node.x != goal_cell.x or end_node.y != goal_cell.y:
        path = compute_path(BinaryHeap(), end_node, goal_cell, "backward")
        if path != -1:
            for node in path:
                if int(maze[node.x][node.y]) == 1:
                    break
                update_agent_vision(node)
                end_node = node
        else:
            return -1
    return backtrace(end_node)


def repeated_adaptive(start_cell, goal_cell):
    end_node = start_cell
    expanded = [[None for x in range(len(maze[0]))] for y in range(len(maze))]
    path_cost = sys.maxsize
    while end_node.x != goal_cell.x or end_node.y != goal_cell.y:
        (path, expanded) = compute_path_adaptive(BinaryHeap(), end_node, goal_cell, expanded, path_cost)
        if path != -1:
            path_cost = path[-1].g
            for node in path:
                if int(maze[node.x][node.y]) == 1:
                    break
                update_agent_vision(node)
                end_node = node
        else:
            return -1
    return backtrace(end_node)


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


# draws path and saves to folder in directory, need to have results folder in project folder prior.
def draw_path(maze, x):
    int_maze = [list(map(int, i)) for i in maze]
    plt.figure()
    plt.imshow(int_maze, cmap=plt.cm.binary, interpolation='nearest')
    plt.xticks([])  # remove the tick marks by setting to an empty list
    plt.yticks([])
    plt.savefig("results/maze{0:0=2d}.png".format(x))
    plt.close()
    # plt.show()


if __name__ == "__main__":
    start_time = time.time()
    # maze = create_grid()
    # agent_vision = [[0 for x in range(len(maze[0]))] for y in range(len(maze))]
    # points = point_picker(maze)
    # points = [Cell(4, 74, 0), Cell(42, 15, 0)]
    # solution = repeated_forward_lazy(points[0], points[1])
    search_type = sys.argv[1]
    directory = 'arrs/randGrid'
    x = 0
    for filename in os.listdir(directory):
        if filename.endswith(".txt"):
            maze = create_grid(filename)
            agent_vision = [[0 for x in range(len(maze[0]))] for y in range(len(maze))]
            print(filename)
            points = point_picker(maze)

            # checks for which search type
            if search_type == 'b':
                solution = repeated_backward_optimized(points[0], points[1])
            elif search_type == 'f':
                solution = repeated_forward_optimized(points[0], points[1])
            elif search_type == 'a':
                solution = repeated_adaptive(points[0], points[1])
            if solution != -1:
                # print("Path Cost = " + str(len(solution) - 1))
                for cell in solution:
                    maze[cell.x][cell.y] = '3'
                    # print("(" + str(cell.x) + ", " + str(cell.y) + ")")
                draw_path(maze, x)
        x = x + 1
    end_time = time.time()
    print(end_time - start_time)
