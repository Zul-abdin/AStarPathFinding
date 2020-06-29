#!/usr/bin/python

import sys


class BinaryHeap:
    def __init__(self):
        self.heap = []

    def size(self):
        return len(self.heap)

    #given index, returns the indexes parent
    def parent(self, i):
        return (i - 1) // 2

    # given index, returns the indexes left child
    def left_child(self, i):
        return 2 * i + 1
    # given index, returns the indexes right child
    def right_child(self, i):
        return 2 * i + 2

    def get(self, i):
        return self.heap[i]

    # returns the root node
    def get_min(self):
        if self.size() == 0:
            return None
        return self.heap[0]

    # pops out the root node, then calls max_heapify to arrange nods again
    def extract_min(self):
        if self.size() == 0:
            return None
        smallest = self.get_min()
        self.heap[0] = self.heap[-1]
        del self.heap[-1]
        self.min_heapify(0)
        return smallest

    # after extracting root node, it arranges the remaining nodes
    def min_heapify(self, i):
        left_node = self.left_child(i)
        right_node = self.right_child(i)
        smallest = i
        if left_node <= self.size() - 1 and self.get(left_node) < self.get(i):
            smallest = left_node
        else:
            largest = i
        if right_node <= self.size() - 1 and self.get(right_node) < self.get(smallest):
            smallest = right_node
        if smallest != i:
            self.swap(smallest, i)
            self.min_heapify(smallest)

    # pretty straightforward
    def swap(self, i, j):
        self.heap[i], self.heap[j] = self.heap[j], self.heap[i]

    # adds element to end of list, then compares if parents is smaller than child, if so swaps them
    def insert(self, key):
        index = self.size()
        self.heap.append(key)

        while (index != 0):
            p = self.parent(index)
            if self.get(p) > self.get(index):
                self.swap(p, index)
            index = p
    # prints the whole list but in binary heap way instead of straight list order.
    def print(self):
        for i in range(0, (len(self.heap) // 2)):
            print(" parent : " + str(self.heap[i]) +
                  " left child : " + str(self.heap[2 * i + 1]) +
                  " right child : " + str(self.heap[2 * i + 2]))
