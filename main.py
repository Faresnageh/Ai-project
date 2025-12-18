import time
import solvers
START_STATE = (
    (1, 2, 3),
    (4, 6, 8),
    (7, 0, 5)
)
GOAL_STATE = (
    (1, 2, 3),
    (4, 5, 6),
    (7, 8, 0)
)

def print_board(state):
    for row in state:
        print(row)
    print()

def main():
    print("Target Goal:")
    print_board(GOAL_STATE)
    print("Start State:")
    print_board(START_STATE)
    
    algorithms = [
        ("BFS", solvers.solve_bfs),
        ("DFS", solvers.solve_dfs),
        ("IDS", solvers.solve_ids),
        ("A*", solvers.solve_astar),
        ("Hill Climbing", solvers.solve_hill_climbing)
    ]
    
    results = []
    
    for name, algo_func in algorithms:
        print(f"Running {name}")
        start_time = time.time()
        
        try:
            steps, path, nodes = algo_func(START_STATE, GOAL_STATE)
            end_time = time.time()
            elapsed = end_time - start_time
            
            status = "Solved" if steps != -1 else "Failed"
            steps_str = str(steps) if steps != -1 else "N/A"
            time_str = f"{elapsed:.4f} s"
            
            results.append([name, status, steps_str, time_str, nodes])
                    
        except Exception as e:
            print(f"Error in {name}: {e}")
            results.append([name, "Error", "-", "-", "-"])

    print()
    print(f"{'Algorithm':<20}  {'Status':<10}  {'Steps':<8}  {'Time':<12}  {'Nodes Exp'}")
    
    for row in results:
        print(f"{row[0]:<20}  {row[1]:<10}  {row[2]:<8}  {row[3]:<12}  {row[4]}")

main()