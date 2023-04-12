#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

// #include <libMultiRobotPlanning/a_star.hpp>
#include <libMultiRobotPlanning/ml_a_star.hpp>

// using libMultiRobotPlanning::ml_AStar;
using libMultiRobotPlanning::ml_AStar;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;

struct State {
  State(int x, int y) : x(x), y(y) {}

  State(const State&) = default;
  State(State&&) = default;
  State& operator=(const State&) = default;
  State& operator=(State&&) = default;

  bool operator==(const State& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << "(" << s.x << "," << s.y << ")";
  }

  int x;
  int y;
};

namespace std {
template <>
struct hash<State> {
  size_t operator()(const State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

enum class Action {
  Up,
  Down,
  Left,
  Right,
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
  }
  return os;
}

class Environment {
 public:
  Environment(size_t dimx, size_t dimy, std::unordered_set<State> obstacles,
              std::vector<State> goals, int num_of_goals)
      : m_dimx(dimx),
        m_dimy(dimy),
        m_obstacles(std::move(obstacles)),
        m_goals(std::move(goals)), m_num_of_goals(num_of_goals) // NOLINT
  {}
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
  

  bool isSolution(const State& s) { return s == m_goals[m_goal_label]; }

  void getNeighbors(const State& s,
                    std::vector<Neighbor<State, Action, int> >& neighbors) {
    neighbors.clear();

    State up(s.x, s.y + 1);
    if (stateValid(up)) {
      neighbors.emplace_back(Neighbor<State, Action, int>(up, Action::Up, 1));
    }
    State down(s.x, s.y - 1);
    if (stateValid(down)) {
      neighbors.emplace_back(
          Neighbor<State, Action, int>(down, Action::Down, 1));
    }
    State left(s.x - 1, s.y);
    if (stateValid(left)) {
      neighbors.emplace_back(
          Neighbor<State, Action, int>(left, Action::Left, 1));
    }
    State right(s.x + 1, s.y);
    if (stateValid(right)) {
      neighbors.emplace_back(
          Neighbor<State, Action, int>(right, Action::Right, 1));
    }
  }

  void onExpandNode(const State& /*s*/, int /*fScore*/, int /*gScore*/) {}

  void onDiscover(const State& /*s*/, int /*fScore*/, int /*gScore*/) {}

 public:
  bool stateValid(const State& s) {
    return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
           m_obstacles.find(s) == m_obstacles.end();
  }
  int m_num_of_goals = 0;
  std::vector<State> m_goals;
 private:
  int m_dimx;
  int m_dimy;
  std::unordered_set<State> m_obstacles;
  
};

int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  int startX, startY, goalX1, goalY1, goalX2, goalY2;
  std::string mapFile;
  std::string outputFile;
  desc.add_options()("help", "produce help message")(
      "startX", po::value<int>(&startX)->required(),
      "start position x-component")("startY",
                                    po::value<int>(&startY)->required(),
                                    "start position y-component")(
      "goalX1", po::value<int>(&goalX1)->required(), "goal position 1 x-component")(
      "goalY1", po::value<int>(&goalY1)->required(), "goal position 1 y-component")(
      "goalX2", po::value<int>(&goalX2)->required(), "goal position 2 x-component")(
      "goalY2", po::value<int>(&goalY2)->required(), "goal position 2 y-component")(
      "map,m", po::value<std::string>(&mapFile)->required(), "input map (txt)")(
      "output,o", po::value<std::string>(&outputFile)->required(),
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

  std::unordered_set<State> obstacles;

  std::ifstream map(mapFile);
  int dimX = 0;
  int y = 0;
  while (map.good()) {
    std::string line;
    std::getline(map, line);
    int x = 0;
    for (char c : line) {
      if (c == '#') {
        obstacles.insert(State(x, y));
      }
      ++x;
    }
    dimX = std::max(dimX, x);
    ++y;
  }
  std::cout << dimX << " " << y << std::endl;

  bool success = false;

  std::vector<State> goals;
  // Add code to push back vector of goals
  State goal1(goalX1, goalY1);
  State goal2(goalX2, goalY2);
  goals.push_back(goal1);
  goals.push_back(goal2);
  State start(startX, startY);
  Environment env(dimX, y - 1, obstacles, goals, goals.size());

  ml_AStar<State, Action, int, Environment> ml_astar(env);

  PlanResult<State, Action, int> solution;

  if (env.stateValid(start)) {
    success = ml_astar.search(start, solution);
  }

  std::ofstream out(outputFile);
  if (success) {
    std::cout << "Planning successful! Total cost: " << solution.cost
              << std::endl;
    // print goal locations:
    // std::cout<<"Goal location 1: "<<goal1.x<<","<<goal1.y<<std::endl;
    // std::cout<<"Goal location 2: "<<goal2.x<<","<<goal2.y<<std::endl;
    for (size_t i = 0; i < solution.actions.size(); ++i) {
      std::cout << solution.states[i].second << ": " << solution.states[i].first
                << "->" << solution.actions[i].first
                << "(cost: " << solution.actions[i].second << ")" << std::endl;
      if (solution.states[i].first.x == goal1.x && solution.states[i].first.y == goal1.y){
        std::cout<<"Reached goal location 1!"<<std::endl;
      }
 
    }
    std::cout << solution.states.back().second << ": "
              << solution.states.back().first << std::endl;
    std::cout<<"Reached goal location 2!"<<std::endl;
    out << "schedule:" << std::endl;
    out << "  agent1:" << std::endl;
    for (size_t i = 0; i < solution.states.size(); ++i) {
      out << "    - x: " << solution.states[i].first.x << std::endl
          << "      y: " << solution.states[i].first.y << std::endl
          << "      t: " << i << std::endl;
      
    }
  } else {
    std::cout << "Planning NOT successful!" << std::endl;
  }

  return 0;
}
