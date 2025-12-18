import heapq
import random
from collections import deque
from copy import deepcopy

# ================================
# === 1. Mohamed Hamdy - BFS ===
# ================================
def solve_bfs(start, goal):
    queue = deque([(start, [start])])
    visited = set()
    visited.add(start)
    nodes_explored = 0
    
    while queue:
        current, path = queue.popleft()
        nodes_explored += 1
        
        if current == goal:
            return len(path) - 1, path, nodes_explored
        
        x, y = [(i,j) for i in range(3) for j in range(3) if current[i][j]==0][0]
        moves = [(-1,0),(1,0),(0,-1),(0,1)]
        for dx, dy in moves:
            nx, ny = x+dx, y+dy
            if 0<=nx<3 and 0<=ny<3:
                new_state_list = [list(row) for row in current]
                new_state_list[x][y], new_state_list[nx][ny] = new_state_list[nx][ny], new_state_list[x][y]
                new_state = tuple(tuple(row) for row in new_state_list)
                
                if new_state not in visited:
                    visited.add(new_state)
                    queue.append((new_state, path + [new_state]))
    return -1, [], nodes_explored

# ================================
# === 2. Ahmed Hashem - DFS ===
# ================================
def solve_dfs(start, goal):
    stack = [(start, [start])]
    visited = set()
    visited.add(start)
    nodes_explored = 0
    
    while stack:
        current, path = stack.pop()
        nodes_explored += 1
        
        if current == goal:
            return len(path) - 1, path, nodes_explored
            
        x, y = [(i,j) for i in range(3) for j in range(3) if current[i][j]==0][0]
        moves = [(-1,0),(1,0),(0,-1),(0,1)]
        for dx, dy in moves:
            nx, ny = x+dx, y+dy
            if 0<=nx<3 and 0<=ny<3:
                new_state_list = [list(row) for row in current]
                new_state_list[x][y], new_state_list[nx][ny] = new_state_list[nx][ny], new_state_list[x][y]
                new_state = tuple(tuple(row) for row in new_state_list)
                
                if new_state not in visited:
                    visited.add(new_state)
                    stack.append((new_state, path + [new_state]))
    return -1, [], nodes_explored

# ================================
# === 3. Fares Nageh - IDS ===
# ================================
def solve_ids(start, goal, max_depth=50):
    def dls(node, goal, depth, path, visited_in_path):
        if depth == 0 and node == goal:
            return path
        if depth > 0:
            if node == goal:
                return path
            
            x, y = [(i,j) for i in range(3) for j in range(3) if node[i][j]==0][0]
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = x+dx, y+dy
                if 0<=nx<3 and 0<=ny<3:
                    new_list = [list(row) for row in node]
                    new_list[x][y], new_list[nx][ny] = new_list[nx][ny], new_list[x][y]
                    new_node = tuple(tuple(row) for row in new_list)
                    
                    if new_node not in visited_in_path:
                        res = dls(new_node, goal, depth-1, path + [new_node], visited_in_path | {new_node})
                        if res: return res
        return None

    total_nodes = 0
    for d in range(max_depth):
        result = dls(start, goal, d, [start], {start})
        total_nodes += (d * 2) 
        if result:
            return len(result)-1, result, total_nodes
            
    return -1, [], total_nodes

# ================================
# === 4. Ahmed Aglan - A* Search ===
# ================================
class PuzzleStateAStar:
    def __init__(self, board, parent=None, move="", g=0, h=0):
        self.board = board
        self.parent = parent
        self.g = g
        self.h = h
        self.f = g + h

    def __lt__(self, other):
        return self.f < other.f

def heuristic_manhattan(board, goal):
    dist = 0
    for i in range(3):
        for j in range(3):
            val = board[i][j]
            if val != 0:
                for gi in range(3):
                    for gj in range(3):
                        if goal[gi][gj] == val:
                            dist += abs(i - gi) + abs(j - gj)
    return dist

def solve_astar(start, goal):
    initial = PuzzleStateAStar(start, None, "", 0, heuristic_manhattan(start, goal))
    open_set = []
    heapq.heappush(open_set, initial)
    closed_set = set()
    nodes_explored = 0
    
    while open_set:
        current = heapq.heappop(open_set)
        nodes_explored += 1
        
        if current.board == goal:
            path = []
            curr = current
            while curr:
                path.append(curr.board)
                curr = curr.parent
            return len(path)-1, path[::-1], nodes_explored
            
        closed_set.add(current.board)
        
        x, y = [(i,j) for i in range(3) for j in range(3) if current.board[i][j]==0][0]
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = x+dx, y+dy
            if 0<=nx<3 and 0<=ny<3:
                new_list = [list(row) for row in current.board]
                new_list[x][y], new_list[nx][ny] = new_list[nx][ny], new_list[x][y]
                new_board = tuple(tuple(row) for row in new_list)
                
                if new_board in closed_set:
                    continue
                    
                h = heuristic_manhattan(new_board, goal)
                new_node = PuzzleStateAStar(new_board, current, "", current.g + 1, h)
                heapq.heappush(open_set, new_node)
                
    return -1, [], nodes_explored

# ========================================
# === 5. Mohamed Ibrahim - Hill Climbing ===
# ========================================
def solve_hill_climbing(start, goal):
    current = start
    path = [current]
    steps = 0
    
    while True:
        current_h = heuristic_manhattan(current, goal)
        if current_h == 0:
            return steps, path, steps 
            
        neighbors = []
        x, y = [(i,j) for i in range(3) for j in range(3) if current[i][j]==0][0]
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = x+dx, y+dy
            if 0<=nx<3 and 0<=ny<3:
                new_list = [list(row) for row in current]
                new_list[x][y], new_list[nx][ny] = new_list[nx][ny], new_list[x][y]
                new_state = tuple(tuple(row) for row in new_list)
                neighbors.append(new_state)
        
        if not neighbors: break
        
        best_neighbor = min(neighbors, key=lambda s: heuristic_manhattan(s, goal))
        best_h = heuristic_manhattan(best_neighbor, goal)
        
        if best_h >= current_h:
            return -1, path, steps
            
        current = best_neighbor
        path.append(current)
        steps += 1
        
    return -1, path, steps