#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <libMultiRobotPlanning/ecbs_ta_mla.hpp>
#include <libMultiRobotPlanning/next_best_assignment.hpp>
#include "timer.hpp"

using libMultiRobotPlanning::ECBSTA;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::NextBestAssignment;

struct State {
  State() = default;
  State(int time, int x, int y) : time(time), x(x), y(y) {}
  State(int time, int x, int y, int label) : time(time), x(x), y(y), label(label) {}

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
  int label;
  
};

namespace std {
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

  struct StateHasher2 {
      std::size_t operator()(const State& state) const {
        std::size_t seed = 0;
        boost::hash_combine(seed, state.x);
        boost::hash_combine(seed, state.y);
        boost::hash_combine(seed, state.label);
        return seed;
      }
    };


}  // namespace std

///
enum class Action {
  Up,
  Down,
  Left,
  Right,
  Wait,
};

std::ostream& operator<<(std::ostream& os, const Action& a) {
  switch (a) {
    case Action::Up:
      os << "Up";
      break;
    case Action::Down:
      os << "Down";
      break;
    case Action::Left:
      os << "Left";
      break;
    case Action::Right:
      os << "Right";
      break;
    case Action::Wait:
      os << "Wait";
      break;
  }
  return os;
}

///

struct Conflict {
  enum Type {
    Vertex,
    Edge,
  };

  int time;
  size_t agent1;
  size_t agent2;
  Type type;

  int x1;
  int y1;
  int x2;
  int y2;

  friend std::ostream& operator<<(std::ostream& os, const Conflict& c) {
    switch (c.type) {
      case Vertex:
        return os << c.time << ": Vertex(" << c.x1 << "," << c.y1 << ")";
      case Edge:
        return os << c.time << ": Edge(" << c.x1 << "," << c.y1 << "," << c.x2
                  << "," << c.y2 << ")";
    }
    return os;
  }
};

struct VertexConstraint {
  VertexConstraint(int time, int x, int y) : time(time), x(x), y(y) {}
  int time;
  int x;
  int y;

  bool operator<(const VertexConstraint& other) const {
    return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
  }

  bool operator==(const VertexConstraint& other) const {
    return std::tie(time, x, y) == std::tie(other.time, other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const VertexConstraint& c) {
    return os << "VC(" << c.time << "," << c.x << "," << c.y << ")";
  }
};

namespace std {
template <>
struct hash<VertexConstraint> {
  size_t operator()(const VertexConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

struct EdgeConstraint {
  EdgeConstraint(int time, int x1, int y1, int x2, int y2)
      : time(time), x1(x1), y1(y1), x2(x2), y2(y2) {}
  int time;
  int x1;
  int y1;
  int x2;
  int y2;

  bool operator<(const EdgeConstraint& other) const {
    return std::tie(time, x1, y1, x2, y2) <
           std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
  }

  bool operator==(const EdgeConstraint& other) const {
    return std::tie(time, x1, y1, x2, y2) ==
           std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
  }

  friend std::ostream& operator<<(std::ostream& os, const EdgeConstraint& c) {
    return os << "EC(" << c.time << "," << c.x1 << "," << c.y1 << "," << c.x2
              << "," << c.y2 << ")";
  }
};

namespace std {
template <>
struct hash<EdgeConstraint> {
  size_t operator()(const EdgeConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x1);
    boost::hash_combine(seed, s.y1);
    boost::hash_combine(seed, s.x2);
    boost::hash_combine(seed, s.y2);
    return seed;
  }
};
}  // namespace std

struct Constraints {
  std::unordered_set<VertexConstraint> vertexConstraints;
  std::unordered_set<EdgeConstraint> edgeConstraints;

  void add(const Constraints& other) {
    vertexConstraints.insert(other.vertexConstraints.begin(),
                             other.vertexConstraints.end());
    edgeConstraints.insert(other.edgeConstraints.begin(),
                           other.edgeConstraints.end());
  }

  bool overlap(const Constraints& other) const {
    for (const auto& vc : vertexConstraints) {
      if (other.vertexConstraints.count(vc) > 0) {
        return true;
      }
    }
    for (const auto& ec : edgeConstraints) {
      if (other.edgeConstraints.count(ec) > 0) {
        return true;
      }
    }
    return false;
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraints& c) {
    for (const auto& vc : c.vertexConstraints) {
      os << vc << std::endl;
    }
    for (const auto& ec : c.edgeConstraints) {
      os << ec << std::endl;
    }
    return os;
  }
};

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
typedef std::map<std::string, std::vector<std::vector<int>>> t_map;
 

///
class Environment {
 public:
  Environment(size_t dimx, size_t dimy,
              const std::unordered_set<Location>& obstacles,
              const std::vector<State>& startStates,
              const std::vector<std::unordered_set<Location> >& goals,
              size_t maxTaskAssignments,
              NextBestAssignment<std::string, std::string> assignment,
              t_map task_definition)
      : m_dimx(dimx),
        m_dimy(dimy),
        m_obstacles(obstacles),
        m_agentIdx(0),
        m_goal(nullptr),
        m_constraints(nullptr),
        m_lastGoalConstraint(-1),
        m_maxTaskAssignments(maxTaskAssignments),
        m_numTaskAssignments(0),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0),
        m_heuristic(dimx, dimy, obstacles),
        m_assignment(assignment),
        m_task_definition(task_definition) {
    
  std::map<std::string, std::string> solution;
  m_assignment.solve(task_definition);
  std::cout << " Found initial solution!" << std::endl;
  }

  void setLowLevelContext(size_t agentIdx, const Constraints* constraints,
                          std::map<std::string, std::vector<State>> agent_goals_map) {
    assert(constraints);
    m_agentIdx = agentIdx;
    m_goals = agent_goals_map[std::to_string(m_agentIdx)];
    m_goal_label = 0;
    m_num_of_goals = m_goals.size();
    m_constraints = constraints;
    m_lastGoalConstraint = -1;
    if (m_goal != nullptr) {
      for (const auto& vc : constraints->vertexConstraints) {
        if (vc.x == m_goal->x && vc.y == m_goal->y) {
          m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
        }
      }
    } else {
      for (const auto& vc : constraints->vertexConstraints) {
        m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
      }
    }
    // std::cout << "setLLCtx: " << agentIdx << " " << m_lastGoalConstraint <<
    // std::endl;
  }

  // int admissibleHeuristic(const State& s) {
  //   if (m_goal != nullptr) {
  //     return m_heuristic.getValue(Location(s.x, s.y), *m_goal);
  //   } else {
  //     return 0;
  //   }
  // }

  // // low-level
  // int focalStateHeuristic(
  //     const State& s, int /*gScore*/,
  //     const std::vector<PlanResult<State, Action, int> >& solution) {
  //   int numConflicts = 0;
  //   for (size_t i = 0; i < solution.size(); ++i) {
  //     if (i != m_agentIdx && solution[i].states.size() > 0) {
  //       State state2 = getState(i, solution, s.time);
  //       if (s.equalExceptTime(state2)) {
  //         ++numConflicts;
  //       }
  //     }
  //   }
  //   return numConflicts;
  // }

  // // low-level
  // int focalTransitionHeuristic(
  //     const State& s1a, const State& s1b, int /*gScoreS1a*/, int /*gScoreS1b*/,
  //     const std::vector<PlanResult<State, Action, int> >& solution) {
  //   int numConflicts = 0;
  //   for (size_t i = 0; i < solution.size(); ++i) {
  //     if (i != m_agentIdx && solution[i].states.size() > 0) {
  //       State s2a = getState(i, solution, s1a.time);
  //       State s2b = getState(i, solution, s1b.time);
  //       if (s1a.equalExceptTime(s2b) && s1b.equalExceptTime(s2a)) {
  //         ++numConflicts;
  //       }
  //     }
  //   }
  //   return numConflicts;
  // }

  // // Count all conflicts
  int focalHeuristic(
      const std::vector<PlanResult<State, Action, int> >& solution) {
    int numConflicts = 0;

    int max_t = 0;
    for (size_t i = 0; i < solution.size(); ++i) {
      max_t = std::max<int>(max_t, solution[i].states.size() - 1);
    }

    for (int t = 0; t < max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2 = getState(j, solution, t);
          if (state1.equalExceptTime(state2)) {
            ++numConflicts;
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1a = getState(i, solution, t);
        State state1b = getState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2a = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1);
          if (state1a.equalExceptTime(state2b) &&
              state1b.equalExceptTime(state2a)) {
            ++numConflicts;
          }
        }
      }
    }
    return numConflicts;
  }


  // ML-A* 
  int m_goal_label = 0;
  // int admissibleHeuristic(const State& s) {
  //   return std::abs(s.x - m_goals[m_goal_label].x) + std::abs(s.y - m_goals[m_goal_label].y);
  // }
  // heuristic = dist(state, current goal) + pairwise sum of dist(current goal, next goal) from current goal to last goal

  int admissibleHeuristic(const State& s) {
    int h = 0;
    int current_goal = m_goal_label;
    int next_goal = current_goal + 1;
    h += std::abs(s.x - m_goals[current_goal].x) + std::abs(s.y - m_goals[current_goal].y);
    while (next_goal < m_num_of_goals) {
      h += std::abs(m_goals[current_goal].x - m_goals[next_goal].x) + std::abs(m_goals[current_goal].y - m_goals[next_goal].y);
      current_goal = next_goal;
      next_goal++;
    }
    return h;
  }

  bool isSolution(const State& s, int current_goal_label, int env_goal_label) {
    bool atGoal = true;
    State current_goal = m_goals[env_goal_label];

    atGoal = s.x == current_goal.x && s.y == current_goal.y;
    if(atGoal && s.time > m_lastGoalConstraint && current_goal_label == env_goal_label) {
      std::cout << "Goal: " << s.x << ", " << s.y << " at time: " << s.time << std::endl;
    }

    return atGoal && s.time > m_lastGoalConstraint;
  }

  // bool isSolution(const State& s) { return s == m_goals[m_goal_label]; }
  void getNeighbors(const State& s,
                    std::vector<Neighbor<State, Action, int> >& neighbors, bool reachedGoal) {
    neighbors.clear();
    int new_label = s.label;
    if(reachedGoal) {
      new_label++;
    }
    State up(s.time+1, s.x, s.y + 1, new_label);
    if (stateValid(up)) {
      neighbors.emplace_back(Neighbor<State, Action, int>(up, Action::Up, 1));
    }
    State down(s.time+1, s.x, s.y - 1, new_label);
    if (stateValid(down)) {
      neighbors.emplace_back(
          Neighbor<State, Action, int>(down, Action::Down, 1));
    }
    State left(s.time+1, s.x - 1, s.y, new_label);
    if (stateValid(left)) {
      neighbors.emplace_back(
          Neighbor<State, Action, int>(left, Action::Left, 1));
    }
    State right(s.time+1, s.x + 1, s.y, new_label);
    if (stateValid(right)) {
      neighbors.emplace_back(
          Neighbor<State, Action, int>(right, Action::Right, 1));
    }
  }

  

  // void getNeighbors(const State& s,
  //                   std::vector<Neighbor<State, Action, int> >& neighbors) {
  //   // std::cout << "#VC " << constraints.vertexConstraints.size() << std::endl;
  //   // for(const auto& vc : constraints.vertexConstraints) {
  //   //   std::cout << "  " << vc.time << "," << vc.x << "," << vc.y <<
  //   //   std::endl;
  //   // }
  //   neighbors.clear();
  //   {
  //     State n(s.time + 1, s.x, s.y);
  //     if (stateValid(n) && transitionValid(s, n)) {
  //       bool atGoal = true;
  //       if (m_goal != nullptr) {
  //         atGoal = s.x == m_goal->x && s.y == m_goal->y;
  //       }
  //       neighbors.emplace_back(
  //           Neighbor<State, Action, int>(n, Action::Wait, atGoal ? 0 : 1));
  //     }
  //   }
  //   {
  //     State n(s.time + 1, s.x - 1, s.y);
  //     if (stateValid(n) && transitionValid(s, n)) {
  //       neighbors.emplace_back(
  //           Neighbor<State, Action, int>(n, Action::Left, 1));
  //     }
  //   }
  //   {
  //     State n(s.time + 1, s.x + 1, s.y);
  //     if (stateValid(n) && transitionValid(s, n)) {
  //       neighbors.emplace_back(
  //           Neighbor<State, Action, int>(n, Action::Right, 1));
  //     }
  //   }
  //   {
  //     State n(s.time + 1, s.x, s.y + 1);
  //     if (stateValid(n) && transitionValid(s, n)) {
  //       neighbors.emplace_back(Neighbor<State, Action, int>(n, Action::Up, 1));
  //     }
  //   }
  //   {
  //     State n(s.time + 1, s.x, s.y - 1);
  //     if (stateValid(n) && transitionValid(s, n)) {
  //       neighbors.emplace_back(
  //           Neighbor<State, Action, int>(n, Action::Down, 1));
  //     }
  //   }
  // }

  bool getFirstConflict(
      const std::vector<PlanResult<State, Action, int> >& solution,
      Conflict& result) {
    int max_t = 0;
    for (const auto& sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size());
    }

    for (int t = 0; t < max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2 = getState(j, solution, t);
          if (state1.equalExceptTime(state2)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.x1 = state1.x;
            result.y1 = state1.y;
            // std::cout << "VC " << t << "," << state1.x << "," << state1.y <<
            // std::endl;
            return true;
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1a = getState(i, solution, t);
        State state1b = getState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2a = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1);
          if (state1a.equalExceptTime(state2b) &&
              state1b.equalExceptTime(state2a)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.x1 = state1a.x;
            result.y1 = state1a.y;
            result.x2 = state1b.x;
            result.y2 = state1b.y;
            return true;
          }
        }
      }
    }

    return false;
  }

  void createConstraintsFromConflict(
      const Conflict& conflict, std::map<size_t, Constraints>& constraints) {
    if (conflict.type == Conflict::Vertex) {
      Constraints c1;
      c1.vertexConstraints.emplace(
          VertexConstraint(conflict.time, conflict.x1, conflict.y1));
      constraints[conflict.agent1] = c1;
      constraints[conflict.agent2] = c1;
    } else if (conflict.type == Conflict::Edge) {
      Constraints c1;
      c1.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, conflict.x1, conflict.y1, conflict.x2, conflict.y2));
      constraints[conflict.agent1] = c1;
      Constraints c2;
      c2.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, conflict.x2, conflict.y2, conflict.x1, conflict.y1));
      constraints[conflict.agent2] = c2;
    }
  }

  std::map<std::string, std::vector<State>> taskToGoals(std::map<std::string, std::string> agent_task_map){
    
    std::map<std::string, std::vector<State>> agent_goals_map;
    for(auto it = agent_task_map.begin(); it != agent_task_map.end(); ++it){
      std::string task_id = it->second;
      std::vector<std::vector<int>> task_goals = m_task_definition[task_id];
      std::vector<State> state_vector;
      for (int i=0; i<task_goals.size(); i++){
        State state;
        state.x = task_goals[i][0];
        state.y = task_goals[i][1];
        state.time = 0;
        state_vector.push_back(state);
      }
      agent_goals_map.insert(std::make_pair(it->first, state_vector));
    }

    return agent_goals_map;
  }

  void nextTaskAssignment(std::map<std::string, std::string> tasks, 
  std::map<std::string, std::vector<State>> &agent_goals_map) {
    if (m_numTaskAssignments > m_maxTaskAssignments) {
      return;
    }

    int64_t cost = m_assignment.nextSolution(tasks, m_task_definition);
    agent_goals_map = taskToGoals(tasks);

    if (!tasks.empty()) {
      std::cout << "nextTaskAssignment: cost: " << cost << std::endl;
      for (const auto& s : tasks) {
        std::cout << s.first << "->" << s.second << std::endl;
      }

      ++m_numTaskAssignments;
    }
  }

  void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

  void onExpandLowLevelNode(const State& /*s*/, int /*fScore*/,
                            int /*gScore*/) {
    m_lowLevelExpanded++;
  }

  int highLevelExpanded() { return m_highLevelExpanded; }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }

  size_t numTaskAssignments() const { return m_numTaskAssignments; }
  int m_num_of_goals = 1;
 private:
  State getState(size_t agentIdx,
                 const std::vector<PlanResult<State, Action, int> >& solution,
                 size_t t) {
    assert(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size()) {
      return solution[agentIdx].states[t].first;
    }
    assert(!solution[agentIdx].states.empty());
    return solution[agentIdx].states.back().first;
  }

  bool stateValid(const State& s) {
    assert(m_constraints);
    const auto& con = m_constraints->vertexConstraints;
    return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
           m_obstacles.find(Location(s.x, s.y)) == m_obstacles.end() &&
           con.find(VertexConstraint(s.time, s.x, s.y)) == con.end();
  }

  bool transitionValid(const State& s1, const State& s2) {
    assert(m_constraints);
    const auto& con = m_constraints->edgeConstraints;
    return con.find(EdgeConstraint(s1.time, s1.x, s1.y, s2.x, s2.y)) ==
           con.end();
  }
  
  int m_dimx;
  int m_dimy;
  std::unordered_set<Location> m_obstacles;
  size_t m_agentIdx;
  const Location* m_goal;
  
  const Constraints* m_constraints;
  int m_lastGoalConstraint;
  // NextBestAssignment<size_t, Location> m_assignment;
  std::map<std::string, std::string> solution;
  NextBestAssignment<std::string, std::string> m_assignment;
  t_map m_task_definition;
  size_t m_maxTaskAssignments;
  size_t m_numTaskAssignments;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
  ShortestPathHeuristic m_heuristic;
  size_t m_numAgents;
  // std::unordered_set<Location> m_goals;
  std::vector<State> m_goals;
};

int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  float w;
  size_t maxTaskAssignments;

  // Added ../benchmark/custom/mapfta1.yaml as a default input file

  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->default_value("../benchmark/custom/mapfta1.yaml"),
      "input file (YAML)")("output,o",
                           po::value<std::string>(&outputFile)->default_value(
                               "output_ecbsta.yaml"),
                           "output file (YAML)")(
      "suboptimality,w", po::value<float>(&w)->default_value(1.0),
      "suboptimality bound")(
      "maxTaskAssignments",
      po::value<size_t>(&maxTaskAssignments)->default_value(1e9),
      "maximum number of task assignments to try");

  // Read the command line options
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
  YAML::Node config = YAML::LoadFile(inputFile);
  NextBestAssignment<std::string, std::string> assignment;
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
  // assignment.nextSolution(task,task_definition);
  std::unordered_set<State> startStatesSet;
  for (const auto& s : startStates) {
    if (startStatesSet.find(s) != startStatesSet.end()) {
      std::cout << "Identical start states detected -> no solution!" << std::endl;
      return {};
    }
    startStatesSet.insert(s);
  }

  Environment mapf(dimx, dimy, obstacles, startStates, goals,
                   maxTaskAssignments, assignment, task_definition);


  ECBSTA<State, Action, int, Conflict, Constraints, Location,
         Environment>
      cbs(mapf, w);
  std::vector<PlanResult<State, Action, int> > solution;

  Timer timer;
  bool success = cbs.search(startStates, solution);
  timer.stop();

  if (success) {
    std::cout << "Planning successful! " << std::endl;
    int64_t cost = 0;
    int64_t makespan = 0;
    for (const auto& s : solution) {
      cost += s.cost;
      makespan = std::max<int64_t>(makespan, s.cost);
    }

    std::ofstream out(outputFile);
    out << "statistics:" << std::endl;
    out << "  cost: " << cost << std::endl;
    out << "  makespan: " << makespan << std::endl;
    out << "  runtime: " << timer.elapsedSeconds() << std::endl;
    out << "  highLevelExpanded: " << mapf.highLevelExpanded() << std::endl;
    out << "  lowLevelExpanded: " << mapf.lowLevelExpanded() << std::endl;
    out << "  numTaskAssignments: " << mapf.numTaskAssignments() << std::endl;
    out << "schedule:" << std::endl;
    for (size_t a = 0; a < solution.size(); ++a) {
      // std::cout << "Solution for: " << a << std::endl;
      // for (size_t i = 0; i < solution[a].actions.size(); ++i) {
      //   std::cout << solution[a].states[i].second << ": " <<
      //   solution[a].states[i].first << "->" << solution[a].actions[i].first
      //   << "(cost: " << solution[a].actions[i].second << ")" << std::endl;
      // }
      // std::cout << solution[a].states.back().second << ": " <<
      // solution[a].states.back().first << std::endl;

      out << "  agent" << a << ":" << std::endl;
      for (const auto& state : solution[a].states) {
        out << "    - x: " << state.first.x << std::endl
            << "      y: " << state.first.y << std::endl
            << "      t: " << state.second << std::endl;
      }
    }
  } else {
    std::cout << "Planning NOT successful!" << std::endl;
  }

  return 0;
}
