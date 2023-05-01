import random
from ruamel import yaml

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
    durations = []
    
    for n in range(num_agents):
        num_goals = random.randint(1, 5)
        potential_goal = []
        time_window = []
        duration = []
        for i in range(num_goals):
            goal = None
            while not goal or goal in obstacles:
                goal = [random.randint(0, dimensions[0] - 1), random.randint(0, dimensions[1] - 1)]
            potential_goal.append(goal)
            window_start = random.randint(0, 20)
            window_end = random.randint(window_start + 1, 50)
            # time_window.append([window_start, window_end])
            time_window.append([0, 100])
            duration.append(0)
            
        
        potential_goals.append(potential_goal)
        goal_time_windows.append(time_window)
        durations.append(duration)
        
    return potential_goals, goal_time_windows, durations
    

def generate_agent(name, dimensions, obstacles, potential_goals, time_windows, durations, start_locs):
    start = None
    start_valid = False
    while start_valid is False:
        start_valid = True
        start = [random.randint(0, dimensions[0] - 1), random.randint(0, dimensions[1] - 1)]
        if tuple(start) in obstacles:
            start_valid = False
        if start in start_locs:
            start_valid = False
        print("inside while loop")
    return {'name': name, 'start': start, 'potentialGoals': potential_goals, 'goalTimeWindows': time_windows, 'goalDurations':durations}, start

def generate_yaml(num_agents, min_map_size, max_map_size):
    dim_x = random.randint(min_map_size, max_map_size)
    dimensions = [dim_x, dim_x]
    num_obstacles = random.randint(0, min(dimensions[0], dimensions[1]) ** 2 // 5)
    map_data = generate_map(dimensions, num_obstacles)
    map_data['obstacles'] = []
    obstacles = map_data['obstacles']
   
    agents = []
    start_locs = []
    potential_goals, time_windows, durations = generate_tasks(dimensions, obstacles, num_agents)
    for i in range(num_agents):
        agent_name = f'agent{i}'
        agent, start_loc = generate_agent(agent_name, dimensions, obstacles, potential_goals, time_windows, durations, start_locs)
        agents.append(agent)
        start_locs.append(start_loc)
    
    data = {'agents': agents, 'map': map_data}
    return yaml.safe_dump(data)

if __name__ == '__main__':
    
    MIN_MAP_SIZE = 3 # minimum map size
    MAX_MAP_SIZE = 30 # maximum map size
    
    for i in range(10):
        # num_agents = random.randint(2, 5)
        num_agents = 20
        MIN_MAP_SIZE = 10
        MAX_MAP_SIZE = 15 
        yaml_data = generate_yaml(num_agents, MIN_MAP_SIZE, MAX_MAP_SIZE)

        filename = f'random_maps/20agents_map_{i}.yaml'
        try:
            with open(filename, 'w') as f:
                f.write(yaml_data)
        except Exception as e:
            print(f"Error writing to file {filename}: {e}")