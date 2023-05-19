from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import time



#Calculating the summation of Manhattan distance among all misplaced nodes
def heuristic_manhattan(cur_state, goal):
    width, height = cur_state.shape[1], cur_state.shape[0]

    manhattan_distance = 0
    for i in range(width):
        for j in range(height):
            index = goal[i][j]
            if cur_state[i][j] != index:
                manhattan_distance += abs(i - (index // width)) + abs(j - (index % width))

    return manhattan_distance

#Calculating the misplaced number of tiles with respect to final goal state
#given the current state of the 8-Puzzle
def heuristic_misplaced(state, goal):
    
    misplaced_tiles = 0
    for i in range(len(state)):
        for j in range(len(state[0])):
            if state[i][j] != goal[i][j]:
                misplaced_tiles += 1
    return misplaced_tiles

#Uniform Common Search
def heuristic_ucs(state, goal):
    return 0

#Checking the parity of the start, and end states are the same. The 8-puzzle problem is unsolvable 
#if the parities of the start and end states are different.
def check_parity(state1, state2):

    x, y = state1.shape 

    pair1, pair2 = 0, 0
    state1_copy = np.reshape(state1, x * y)
    state2_copy = np.reshape(state2, x * y)
    state1_copy = np.delete(state1_copy, list(state1_copy).index(0))
    state2_copy = np.delete(state2_copy, list(state2_copy).index(0))
    for i in range(x * y - 2):
        for j in range(i + 1, x * y - 1):
            if state1_copy[i] > state1_copy[j]:
                pair1 += 1
            if state2_copy[i] > state2_copy[j]:
                pair2 += 1

    return pair1 % 2 == pair2 % 2

#A-star function 
def astar_search(start_state, goal_state, heuristic):
    #Time calculation
    start_time = time.time()
    
    #Checking if the parity of start state and goal_state are not the same
    if not check_parity(start_state, goal_state):
        return None
    #Initializing start state, goal state, and 
    start = start_state.tolist()
    goal = goal_state.tolist()
    start = tuple(tuple(row) for row in start)
    goal = tuple(tuple(row) for row in goal)
    frontier = deque([(start, 0)]) 

    previous = {start: None}
    explored = set()
    expand = 0

    while frontier:
        s, cost = frontier.popleft()
        expand += 1
        #checking if final goal state is reached
        if s == goal:
            end_time = time.time()
            return previous, expand, end_time - start_time

        explored.add(s)
        s_list = [list(row) for row in s]

        # Finding the position of the new tile
        i_blank, j_blank = [(i, j) for i in range(3) for j in range(3) if s_list[i][j] == 0][0]

        #Defining the blank tiles possible moves
        pos_moves = [(0, -1), (0, 1), (-1, 0), (1, 0)]  # Left, right, up, down

        for dx, dy in pos_moves:
            i_new, j_new = i_blank + dx, j_blank + dy

            # Checking if the move is valid
            if 0 <= i_new < 3 and 0 <= j_new < 3:
                new_state = [list(row) for row in s_list]

                # Swapping blank tile with the new location tile
                new_state[i_blank][j_blank], new_state[i_new][j_new] = new_state[i_new][j_new], new_state[i_blank][j_blank]
                
                new_state_tuple = tuple(tuple(row) for row in new_state)

                if new_state_tuple not in explored:
                    newcost = cost + 1 + heuristic(np.array(new_state), np.array(goal))
                    frontier.append((new_state_tuple, newcost))
                    previous[new_state_tuple] = s

    return None

#Printing iterations of the 
def print_iterations(moves, goal, puzzle_len, mode):
  tmp_goal = goal.copy()
  goal = tuple(goal.reshape(puzzle_len * puzzle_len))

  res = []
  while goal is not None:
    res.append(goal)
    goal = moves.get(goal, None)
  for i, sta in enumerate(reversed(res)):
    tmp = list(sta)
    hn = {
      1: heuristic_ucs,
      2: heuristic_misplaced,
      3: heuristic_manhattan,
    }[mode]
    print(f"State: {tmp}, cost of heuristic function: {hn(tmp, tmp_goal)}")

if __name__ == "__main__":
    start = np.array([[1,3,6], [5,0,6], [4,7,8]])
    goal = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 0]])
    x,y = start.shape
    #print("Shape "+" "+str(x)+" "+str(y))
    # start = np.array([2, 8, 1, 0, 4, 7, 6, 5, 3]).reshape((3, 3))
    # goal = np.array([1, 2, 3, 4, 5, 6, 7, 8, 0]).reshape((3, 3))

    # Selecting the heuristic algorithm
    print("Select an algorithm:")
    print("1:Uniform Cost Search")
    print("2:Misplaced Tile Heuristic")
    print("3:Manhattan Distance Heuristic")

    algorithm = int(input())

    # Dictionary storing heuristics types
    heuristics = {
        1: heuristic_ucs,
        2: heuristic_misplaced,
        3: heuristic_manhattan,
    }


    heuristic = heuristics[algorithm]

    # Run the A* search
    previous, expand, time_taken = astar_search(start, goal, heuristic)

    # Printing the results
    if previous is not None:
        print("Previous:", previous)
        print("Expand:", expand)
        print("Time taken:", time_taken)
    else:
        print("No solution found.")
