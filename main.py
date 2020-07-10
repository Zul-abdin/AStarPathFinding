# Project 1
import random
import astar
from Cell import Cell
from BinaryHeap import BinaryHeap
import matplotlib.pyplot as plt
import os
import sys
import shutil

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

        if not (int(arr[start_row][start_col]) or int(arr[goal_row][goal_col])):
            break

    print(start_row, start_col)
    print(goal_row, goal_col)

    # initial heuristic
    goal_cell = Cell(goal_row, goal_col, 0)
    start_cell = Cell(start_row, start_col, 0)
    print("Heuristic: " + str(astar.get_heuristic(start_cell, goal_cell)))
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


def compute_path(min_heap, start_cell, goal_cell):
    expanded = []
    start_cell.g = astar.get_heuristic(start_cell, goal_cell)
    start_cell.f = start_cell.g
    min_heap.insert(start_cell)

    children = [Cell(start_cell.x - 1, start_cell.y, 0), Cell(start_cell.x, start_cell.y + 1, 0),
                Cell(start_cell.x + 1, start_cell.y, 0), Cell(start_cell.x, start_cell.y - 1, 0)]
    for child in children:
        if 0 <= child.x < len(maze) and 0 <= child.y < len(maze[0]) and arr_contains_pair(expanded, child) == 0:
            agent_vision[child.x][child.y] = maze[child.x][child.y]

    while min_heap.get_min() is not None:
        currNode = min_heap.extract_min()
        children = [Cell(currNode.x - 1, currNode.y, 0), Cell(currNode.x, currNode.y + 1, 0),
                    Cell(currNode.x + 1, currNode.y, 0), Cell(currNode.x, currNode.y - 1, 0)]
        for child in children:
            if 0 <= child.x < len(maze) and 0 <= child.y < len(maze[0]) and arr_contains_pair(expanded,
                                                                                              child) == 0 and int(
                    agent_vision[child.x][child.y]) != 1:
                child.parent = currNode
                child.h = astar.get_heuristic(child, goal_cell)
                child.g = currNode.g + 1
                child.f = child.h + child.g
                min_heap.insert(child)
        expanded.append(currNode)
        if currNode.x == goal_cell.x and currNode.y == goal_cell.y:
            return backtrace(currNode)
    print("Unreachable Goal")
    return -1


def repeated_forward(start_cell, goal_cell):
    end_node = start_cell
    while end_node.x != goal_cell.x or end_node.y != goal_cell.y:
        path = compute_path(BinaryHeap(), end_node, goal_cell)
        if path != -1:
            for node in path:
                if int(maze[node.x][node.y]) == 1:
                    break
                end_node = node
        else:
            return -1
    return backtrace(end_node)


def draw_path(maze):
    int_maze = [list(map(int, i)) for i in maze]
    plt.figure()
    plt.imshow(int_maze, cmap=plt.cm.binary, interpolation='nearest')
    plt.xticks([])  # remove the tick marks by setting to an empty list
    plt.yticks([])
    plt.savefig("pics/randGrid/path0")
    plt.show()

def solver(maze):
    agent_vision = [[0 for x in range(len(maze[0]))] for y in range(len(maze))]
    points = point_picker(maze)
    solution = repeated_forward(points[0], points[1])
    if solution != -1:
        print("Path Cost = " + str(len(solution) - 1))
        for cell in solution:
            maze[cell.x][cell.y] = '3'
            print("(" + str(cell.x) + ", " + str(cell.y) + ")")
        draw_path(maze)

if __name__ == "__main__":

    directory = 'arrs/randGrid'
    for filename in os.listdir(directory):
        if filename.endswith(".txt"):

            maze = create_grid(filename)
            agent_vision = [[0 for x in range(len(maze[0]))] for y in range(len(maze))]
            print(filename)
            points = point_picker(maze)
            solution = repeated_forward(points[0], points[1])
            if solution != -1:
                print("Path Cost = " + str(len(solution) - 1))
                for cell in solution:
                    maze[cell.x][cell.y] = '3'
                    print("(" + str(cell.x) + ", " + str(cell.y) + ")")
                draw_path(maze)
