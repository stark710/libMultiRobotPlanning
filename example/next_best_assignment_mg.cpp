#include <fstream>
#include <iostream>
#include <regex>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <libMultiRobotPlanning/next_best_assignment.hpp>

using libMultiRobotPlanning::NextBestAssignment;

struct Location {
  Location() = default;
  Location(int x, int y) : x(x), y(y) {}
  int x;
  int y;

  bool operator<(const Location& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const Location& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const Location& c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
};

struct State {
  State(int time, int x, int y) : time(time), x(x), y(y) {}

  bool operator==(const State& s) const {
    return time == s.time && x == s.x && y == s.y;
  }

  bool equalExceptTime(const State& s) const { return x == s.x && y == s.y; }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << s.time << ": (" << s.x << "," << s.y << ")";
    // return os << "(" << s.x << "," << s.y << ")";
  }

  int time ;
  int x;
  int y;
};

namespace std {
template <>
struct hash<Location> {
  size_t operator()(const Location& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};

template <>
struct hash<State> {
  size_t operator()(const State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};


}  // namespace std
#include "shortest_path_heuristic.hpp"
typedef std::map<std::string, std::vector<std::vector<int>>> t_map;
  
t_map createMultiGoalCostMatrix(std::string inputFile, NextBestAssignment<std::string, std::string> &assignment){
  YAML::Node config = YAML::LoadFile(inputFile);

  std::unordered_set<Location> obstacles;
  std::vector<std::unordered_set<Location> > goals;
  std::vector<State> startStates;
  t_map task_definition; 
  
  const auto& dim = config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  for (const auto& node : config["map"]["obstacles"]) {
    obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
  }
  int agent_id=0;
  int task_id = 0;
  for (const auto& node : config["agents"]) {
    std::vector<std::vector<int>> agent_potential_goals;
    ShortestPathHeuristic m_heuristic(dimx, dimy, obstacles);

    const auto& start = node["start"];
    startStates.emplace_back(State(0, start[0].as<int>(), start[1].as<int>()));
    goals.resize(goals.size() + 1);
    task_id = 0;
    for (const auto& goal : node["potentialGoals"]) {

      std::vector<std::vector<int>> all_goals = goal.as<std::vector<std::vector<int>>>();
      task_definition[std::to_string(task_id)] = all_goals;

      
      std::vector<int> goal_last_element = goal.as<std::vector<std::vector<int>>>().back();
      int cost = m_heuristic.getValue(Location(startStates[agent_id].x,startStates[agent_id].y), Location(goal_last_element[0], goal_last_element[1]));
      assignment.setCost(std::to_string(agent_id), std::to_string(task_id), cost);
      task_id++;
    }
    agent_id++;
  }


    // sanity check: no identical start states
  std::unordered_set<State> startStatesSet;
  for (const auto& s : startStates) {
    if (startStatesSet.find(s) != startStatesSet.end()) {
      std::cout << "Identical start states detected -> no solution!" << std::endl;
      return {};
    }
    startStatesSet.insert(s);
  }

  return task_definition; 
}

// Create a time window dependent cost matrix

std::vector<t_map> createMultiGoalTimedCostMatrix(std::string inputFile, NextBestAssignment<std::string, std::string> &assignment){
  YAML::Node config = YAML::LoadFile(inputFile);
  std::vector<t_map> task_goal_full_defintion;
  std::unordered_set<Location> obstacles;
  std::vector<std::unordered_set<Location> > goals;
  std::vector<State> startStates;
  t_map task_definition; 
  t_map task_time_window_definition;
  
  
  const auto& dim = config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  for (const auto& node : config["map"]["obstacles"]) {
    obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
  }
  int agent_id=0;
  int task_id = 0;
  for (const auto& node : config["agents"]) {
    std::vector<std::vector<int>> agent_potential_goals;
    ShortestPathHeuristic m_heuristic(dimx, dimy, obstacles);

    const auto& start = node["start"];
    startStates.emplace_back(State(0, start[0].as<int>(), start[1].as<int>()));
    goals.resize(goals.size() + 1);
    task_id = 0;

    // Iterate through potential goals and their time windows
    for (int i=0; i<node["potentialGoals"].size(); i++){

      const auto& goal = node["potentialGoals"][i];
      const auto& time_window = node["goalTimeWindows"][i];
      const auto& goal_duration = node["goalDurations"][i];

      std::vector<std::vector<int>> all_goals = goal.as<std::vector<std::vector<int>>>();
      std::vector<std::vector<int>> all_time_windows = time_window.as<std::vector<std::vector<int>>>();
      std::vector<int> all_durations = goal_duration.as<std::vector<int>>();

      int num_goals = all_goals.size();
      task_definition[std::to_string(task_id)] = all_goals;
      task_time_window_definition[std::to_string(task_id)] = all_time_windows;

      std::vector<int> goal_last_element = goal.as<std::vector<std::vector<int>>>().back();
      
      // Find sum of costs to all goals 
      int sum_cost = 0;
      int curr_cost = 0;
      int curr_time_cost = 0;
      sum_cost = m_heuristic.getValue(Location(startStates[agent_id].x,startStates[agent_id].y), Location(all_goals[0][0], all_goals[0][1])) + all_durations[0];
      for (int j=1; j<all_goals.size(); j++){

          int prev_goal_x = all_goals[j-1][0];
          int prev_goal_y = all_goals[j-1][1];
          int curr_goal_x = all_goals[j][0];
          int curr_goal_y = all_goals[j][1];
          curr_cost = m_heuristic.getValue(Location(prev_goal_x, prev_goal_y), Location(curr_goal_x, curr_goal_y));
          curr_time_cost = all_durations[j];
          sum_cost += curr_cost + curr_time_cost;
      }

      // If the sum of costs is less than the time window, then the agent can reach all goals in time
      int last_goal_deadline = all_time_windows[num_goals-1][1];
      if (sum_cost < last_goal_deadline){
        assignment.setCost(std::to_string(agent_id), std::to_string(task_id), sum_cost);
      }

      // If the sum of costs is greater than the time window, then the agent cannot reach all goals in time
      else{
        assignment.setCost(std::to_string(agent_id), std::to_string(task_id), 1000000);
      }
      
      task_id++;
    }

    agent_id++;
  }


  // sanity check: no identical start states
  std::unordered_set<State> startStatesSet;
  for (const auto& s : startStates) {
    if (startStatesSet.find(s) != startStatesSet.end()) {
      std::cout << "Identical start states detected -> no solution!" << std::endl;
      return {};
    }
    startStatesSet.insert(s);
  }

  task_goal_full_defintion.push_back(task_definition);
  task_goal_full_defintion.push_back(task_time_window_definition);

  return task_goal_full_defintion; 
}


int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile1;
  std::string outputFile2;

  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->default_value("../benchmark/custom/mapfta4.yaml"),
      "input cost (txt)")("output-1,o",
                          po::value<std::string>(&outputFile1)->default_value(
                               "output_ecbsta_untimed.yaml"),
                          "output file (YAML)"),("output-2,o",
                          po::value<std::string>(&outputFile2)->default_value(
                               "output_ecbsta_timed.yaml"),
                          "output file (YAML)") ;


  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  std::map<std::string, std::string> solution;
  NextBestAssignment<std::string, std::string> assignment_1;
  NextBestAssignment<std::string, std::string> assignment_2;
  t_map task_definition = createMultiGoalCostMatrix(inputFile, assignment_1);
  std::vector<t_map> task_time_window_definition = createMultiGoalTimedCostMatrix(inputFile, assignment_2);
  
  // Solve the assignment problem untimed 
  // assignment_1.solve(task_definition);
  // assignment_1.nextSolution(solution, task_definition);

  // Solve the assignment problem timed
  assignment_2.solve(task_time_window_definition[0]);
  assignment_2.nextSolution(solution, task_time_window_definition[0]);

  return 0;
}
