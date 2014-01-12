import numpy as np
from collections import deque


class State:
    def __init__(self, permutation, orientation):
        self.permutation = np.array(permutation, dtype='int8')
        self.orientation = np.array(orientation, dtype='int8')

    def multiply(self, move):
        new_permutation = self.permutation[move.permutation]
        new_orientation = (self.orientation[move.permutation] + move.orientation) % 3
        return State(new_permutation, new_orientation)


origin = State([0, 1, 2, 3, 4, 5, 6, 7], [0, 0, 0, 0, 0, 0, 0, 0])
sample = State([0, 1, 2, 3, 4, 5, 7, 6], [0, 0, 0, 0, 0, 0, 0, 0])

_N_PERMUTATIONS = 40320
_N_ORIENTATIONS = 2187
_N_MOVE_TYPES = 6 * 3


class PocketCubeSolver:
    def __init__(self):
        moves = {'U': State([3, 0, 1, 2, 4, 5, 6, 7], [0, 0, 0, 0, 0, 0, 0, 0]),
                 'D': State([0, 1, 2, 3, 5, 6, 7, 4], [0, 0, 0, 0, 0, 0, 0, 0]),
                 'L': State([4, 1, 2, 0, 7, 5, 6, 3], [2, 0, 0, 1, 1, 0, 0, 2]),
                 'R': State([0, 2, 6, 3, 4, 1, 5, 7], [0, 1, 2, 0, 0, 2, 1, 0]),
                 'F': State([0, 1, 3, 7, 4, 5, 2, 6], [0, 0, 1, 2, 0, 0, 2, 1]),
                 'B': State([1, 5, 2, 3, 0, 4, 6, 7], [1, 2, 0, 0, 2, 1, 0, 0])}

        move_names = []
        for face_name in moves.keys():
            move_names += [face_name, face_name + '2', face_name + '\'']
            moves[face_name + '2'] = moves[face_name].multiply(moves[face_name])
            moves[face_name + '\''] = moves[face_name].multiply(moves[face_name]).multiply(moves[face_name])

        self.moves = moves
        self.move_names = move_names
        self.is_initialized = False
        pass

    def initialize(self):
        print('Start initializing')
        self.permutation_move = np.zeros([_N_PERMUTATIONS, _N_MOVE_TYPES])
        for i in xrange(_N_PERMUTATIONS):
            state = State(index2permutation(i, 8), [0, 0, 0, 0, 0, 0, 0, 0])
            for j in xrange(_N_MOVE_TYPES):
                self.permutation_move[i][j] = permutation2index(
                    state.multiply(self.moves[self.move_names[j]]).permutation)

        self.orientation_move = np.zeros([_N_ORIENTATIONS, _N_MOVE_TYPES])
        for i in xrange(_N_ORIENTATIONS):
            state = State([0, 1, 2, 3, 4, 5, 6, 7], index2orientation(i, 3, 8))
            for j in xrange(_N_MOVE_TYPES):
                self.orientation_move[i][j] = orientation2index(
                    state.multiply(self.moves[self.move_names[j]]).orientation)

        self.permutation_distance = -1 * np.ones(_N_PERMUTATIONS)
        self.permutation_distance[0] = 0
        self.orientation_distance = -1 * np.ones(_N_ORIENTATIONS)
        self.orientation_distance[0] = 0

        depth = 0
        while True:
            n_visited = 0
            for i in xrange(_N_PERMUTATIONS):
                if self.permutation_distance[i] == depth:
                    for j in xrange(_N_MOVE_TYPES):
                        next = self.permutation_move[i][j]
                        if self.permutation_distance[next] < 0:
                            self.permutation_distance[next] = depth + 1
                            n_visited += 1

            if n_visited == 0:
                break
            depth += 1

        depth = 0
        while True:
            n_visited = 0
            for i in xrange(_N_ORIENTATIONS):
                if self.orientation_distance[i] == depth:
                    for j in xrange(_N_MOVE_TYPES):
                        next = self.orientation_move[i][j]
                        if self.orientation_distance[next] < 0:
                            self.orientation_distance[next] = depth + 1
                            n_visited += 1

            if n_visited == 0:
                break
            depth += 1

        self.is_initialized = True
        print('Finish initializing')

    def solve(self, state):
        if not self.is_initialized:
            self.initialize()

        permutation_index = permutation2index(state.permutation)
        orientation_index = orientation2index(state.orientation, 3)

        solution = deque()
        depth = 0
        while True:
            if self.search(permutation_index, orientation_index, solution, depth, -1):
                return solution
            depth += 1


    def search(self, permutation_index, orientation_index, solution, depth, last_face):
        if depth == 0:
            return permutation_index == 0 and orientation_index == 0
        if (self.permutation_distance[permutation_index] <= depth
            and self.orientation_distance[orientation_index] <= depth):
            for i_move in xrange(len(self.move_names)):
                if i_move / 3 == last_face:
                    continue

                solution.append(self.move_names[i_move])
                if (self.search(self.permutation_move[permutation_index][i_move],
                                self.orientation_move[orientation_index][i_move],
                                solution,
                                depth - 1,
                                i_move / 3)):
                    return True
                solution.pop()

        return False


def permutation2index(permutation):
    index = 0
    length = len(permutation) # 8 if 2x2 cube
    for i in xrange(length):
        index *= length - i
        for j in xrange(i + 1, length):
            if permutation[i] > permutation[j]:
                index += 1

    return index


def index2permutation(index, length):
    permutation = np.zeros(length)
    for i in xrange(length - 2, -1, -1):
        permutation[i] = index % (length - i)
        index /= (length - i)
        for j in xrange(i + 1, length):
            if permutation[j] >= permutation[i]:
                permutation[j] += 1
    return permutation


def orientation2index(orientation, n_values=3):
    index = 0
    length = len(orientation)
    for i in xrange(length - 1):
        index = n_values * index + orientation[i]
    return index


def index2orientation(index, n_values, length):
    orientation = np.zeros(length)
    for i in xrange(length - 2, -1, -1):
        orientation[i] = index % n_values
        index /= n_values
    orientation[length - 1] = 3 - sum(orientation) % 3
    return orientation
