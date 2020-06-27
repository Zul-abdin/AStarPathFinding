# Project 1
import random
import astar
from Cell import Cell

print('Test4')


# reads maze files and converts them to a 2d array
def create_grid():
    file = open("maze1.txt")
    lines = [line.split() for line in file]
    return lines


# picks the starting node, recursively calls until it finds an unblocked cell
def point_picker(arr):
    start_row = random.randint(0, len(arr))
    start_col = random.randint(0, len(arr[0]))

    goal_row = random.randint(0, len(arr))
    goal_col = random.randint(0, len(arr[0]))

    # print(arr[start_row][start_col], arr[goal_row][goal_col])
    if arr[start_row][start_col] and arr[goal_row][goal_col] == '1':
        point_picker(arr)

    print(start_row, start_col)
    print(goal_row, goal_col)
    goal_cell = Cell(goal_row,goal_col,0)
    start_cell = Cell(0,1,0)
    print("Heuristic: " + str(astar.get_heuristic(start_cell, goal_cell)))

if __name__ == "__main__":
    point_picker(create_grid())
