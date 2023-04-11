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
  State() = default;
  State(int time, int x, int y) : time(time), location(x, y) {}
  int time;
  Location location;

  bool operator<(const State& other) const {
    return std::tie(time, location) < std::tie(other.time, other.location);
  }

  bool operator==(const State& other) const {
    return std::tie(time, location) == std::tie(other.time, other.location);
  }

  friend std::ostream& operator<<(std::ostream& os, const State& c) {
    return os << "(" << c.time << "," << c.location << ")";
  }
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
}  // namespace std
#include "shortest_path_heuristic.hpp"

void createMultiGoalCostMatrix(std::string inputFile, NextBestAssignment<std::string, std::string> &assignment){

  YAML::Node config = YAML::LoadFile(inputFile);

  std::unordered_set<Location> obstacles;
  std::vector<std::unordered_set<Location> > goals;
  std::vector<State> startStates;
  
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
      std::vector<int> goal_last_element = goal.as<std::vector<std::vector<int>>>().back();
      int cost = m_heuristic.getValue(startStates[agent_id].location, Location(goal_last_element[0], goal_last_element[1]));
      assignment.setCost(std::to_string(agent_id), std::to_string(task_id), cost);
      task_id++;
    }
    agent_id++;
  }

  return; 
}

int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->default_value("../benchmark/custom/mapfta1.yaml"),
      "input cost (txt)")("output,o",
                          po::value<std::string>(&outputFile)->default_value(
                               "output_ecbsta.yaml"),
                          "output file (YAML)");

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


  // std::ifstream input(inputFile);
  // std::regex re("(\\w+)\\s*->\\s*(\\w+)\\s*:\\s*(\\d+)");
  // for (std::string line; getline(input, line);) {
  //   std::smatch match;
  //   if (std::regex_search(line, match, re) && match.size() == 4) {
  //     std::string agent = match.str(1);
  //     std::string task = match.str(2);
  //     int cost = std::stoi(match.str(3));
  //     assignment.setCost(agent, task, cost);
  //   } else {
  //     std::cerr << "Couldn't match line \"" << line << "\"!" << match.size()
  //               << std::endl;
  //   }
  // }

  // const size_t numAgents = 4;
  // const size_t numTasks = 4;

  // const int64_t cost[numAgents][numTasks] = {{90, 76, 75, 80},
  //                                            {35, 85, 55, 65},
  //                                            {125, 95, 90, 105},
  //                                            {45, 110, 95, 115}};

  // for (size_t i = 0; i < numAgents; ++i) {
  //   for (size_t j = 0; j < numTasks; ++j) {
  //     a.setCost("a" + std::to_string(i), "t" + std::to_string(j),
  //     cost[i][j]);
  //   }
  // }
  std::map<std::string, std::string> solution;
  NextBestAssignment<std::string, std::string> assignment;
  createMultiGoalCostMatrix(inputFile, assignment);
  assignment.solve();

  std::ofstream out(outputFile);
  out << "solutions:" << std::endl;

  for (size_t i = 0;; ++i) {
    int64_t c = assignment.nextSolution(solution);
    if (solution.empty()) {
      if (i == 0) {
        out << "  []" << std::endl;
      }
      break;
    }
    out << "  - cost: " << c << std::endl;
    out << "    assignment:" << std::endl;
    for (const auto& s : solution) {
      out << "      " << s.first << ": " << s.second << std::endl;
    }
  }

  return 0;
}
