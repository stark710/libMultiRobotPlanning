import random
import yaml

def generate_map(dimensions, num_obstacles):
    obstacles = set()
    while len(obstacles) < num_obstacles:
        row = random.randint(0, dimensions[0] - 1)
        col = random.randint(0, dimensions[1] - 1)
        if (row, col) not in obstacles:
            obstacles.add((row, col))
    return {'dimensions': dimensions, 'obstacles': list(obstacles)}

def generate_tasks(dimensions, obstacles, num_agents):
    
    potential_goals = []
    goal_time_windows = []
    
    
    for n in range(num_agents):
        num_goals = random.randint(1, 5)
        potential_goal = []
        time_window = []
        for i in range(num_goals):
            goal = None
            while not goal or goal in obstacles:
                goal = [random.randint(0, dimensions[0] - 1), random.randint(0, dimensions[1] - 1)]
            potential_goal.append(goal)
            window_start = random.randint(0, 20)
            window_end = random.randint(window_start + 1, 50)
            time_window.append([window_start, window_end])
        
        potential_goals.append(potential_goal)
        goal_time_windows.append(time_window)
        
    return potential_goals, goal_time_windows
    

def generate_agent(name, dimensions, obstacles, potential_goals, time_windows):
    start = None
    while not start or start in obstacles:
        start = [random.randint(0, dimensions[0] - 1), random.randint(0, dimensions[1] - 1)]
    
    return {'name': name, 'start': start, 'potentialGoals': potential_goals, 'goalTimeWindows': time_windows}

def generate_yaml(num_agents, min_map_size, max_map_size):
    dimensions = [random.randint(min_map_size, max_map_size), random.randint(min_map_size, max_map_size)]
    num_obstacles = random.randint(0, min(dimensions[0], dimensions[1]) ** 2 // 2)
    obstacles = generate_map(dimensions, num_obstacles)['obstacles']
    agents = []
    potential_goals, time_windows = generate_tasks(dimensions, obstacles, num_agents)
    for i in range(num_agents):
        agent_name = f'agent{i}'
        agent = generate_agent(agent_name, dimensions, obstacles, potential_goals, time_windows)
        agents.append(agent)
    map_data = generate_map(dimensions, num_obstacles)
    data = {'agents': agents, 'map': map_data}
    return yaml.safe_dump(data)

if __name__ == '__main__':
    
    MIN_MAP_SIZE = 3 # minimum map size
    MAX_MAP_SIZE = 10 # maximum map size
    
    for i in range(50):
        num_agents = random.randint(0, 50)
        yaml_data = generate_yaml(num_agents, MIN_MAP_SIZE, MAX_MAP_SIZE)

        filename = f'./example/random_maps/map_{i}.yaml'
        try:
            with open(filename, 'w') as f:
                f.write(yaml_data)
        except Exception as e:
            print(f"Error writing to file {filename}: {e}")