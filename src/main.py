import pocketcube

if __name__ == '__main__':
    target = pocketcube.State([0, 2, 1, 3, 4, 5, 6, 7], [0, 1, 2, 0, 0, 0, 0, 0])
    solver = pocketcube.PocketCubeSolver()
    solution = solver.solve(target)
    print(" ".join(solution))