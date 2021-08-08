#include "planner_2d.h"
#include <cmath>
#include <iostream>
#include <srrg_config/configurable_command.h>
#include <srrg_geometry/geometry2d.h>
#include <srrg_messages/messages/grid_map_message.h>
#include <srrg_pcl/point.h>
#include <srrg_system_utils/chrono.h>
#include <srrg_viewer/drawable_base.h>
#include <unistd.h>

namespace srrg2_navigation_2d {
  using namespace srrg2_core;
  using namespace std;

  
  Planner2D::SearchStatus Planner2D::doAstar(StdVectorEigenVector2i& path_,
                                             Vector2i& goal,
                                             const Vector2i& start,
                                             PathMatrix& pmap) {
    path_.clear();
    // bb create a priority queue with the starting point
    PathSearchQueue queue;
    // bb retrieve 8 neighbors displacements
    const int* neighbor_offsets = pmap.eightNeighborOffsets();
    // bb frontier of the expansion
    // bb priority is sum of traveled path and heuristic
    int num_expansions = 0;
    int max_expansions = 10000;
    for (auto& cell : pmap) {
      cell.parent    = nullptr;
      cell.heuristic = cell.cost;
      cell.cost      = std::numeric_limits<float>::max();
    }

    auto& initial  = pmap.at(start);
    initial.parent = &initial;
    initial.cost   = 0;
    queue.push(PathSearchEntry(initial.heuristic, &initial));
    // bb while current point is not on border and is not a neighbor of the goal expand it
    SearchStatus status=GoalNotFound;
    PathMatrixCell* current = nullptr;
    while (!queue.empty() && status==GoalNotFound && num_expansions < max_expansions) {
      PathSearchEntry entry = queue.top();
      current               = entry.cell;
      queue.pop();
      Vector2i current_pos = pmap.pos(current);
      if (pmap.onBorder(current_pos)) {
        status = GoalFound;
        break;
      }
      if (current->heuristic == 0) {
        status = GoalInWindow;
        break;
      }
      for (int i = 0; i < 8; ++i) {
        auto neighbor         = current + neighbor_offsets[i];
        Vector2i neighbor_pos = pmap.pos(neighbor);
        int d2                = (neighbor_pos - current_pos).squaredNorm();
        float linear_distance = 1;
        if (d2 > 1) {
          linear_distance = sqrt(2);
        }
        float arc_cost = _dijkstra_search.traversalCost(linear_distance, neighbor->distance);
        float cost_till_neighbor = current->cost + arc_cost;
        if (cost_till_neighbor < neighbor->cost) {
          neighbor->cost = cost_till_neighbor;

          neighbor->parent    = current;
          float expected_cost = neighbor->cost + neighbor->heuristic;
          queue.push(PathSearchEntry(expected_cost, neighbor));
        }
      }
      ++num_expansions;
    }
    //cerr << "astar, num_expansions: " << num_expansions << endl;
    //cerr << "astar, start: " << start.transpose() << endl;
    path_.clear();
    _cost_to_global_goal=-1;
    if (status==GoalNotFound)
      return status;
    _cost_to_global_goal=current->heuristic+current->cost;
    float delta_heuristic=initial.heuristic-current->heuristic;
    if (delta_heuristic<0) {
      cerr << "**********************************************************" << endl;
      cerr << "DELTA H<0 (" << delta_heuristic << ") ! NEED GLOBAL REPLAN" << endl;
      cerr << "**********************************************************" << endl;
      return HeuristicMismatch;
    }

    goal      = pmap.pos(current->parent->parent);
    auto cell = current;
    while (cell != cell->parent) {
      Vector2i pos = pmap.pos(cell);
      path_.push_back(pos);
      cell = cell->parent;
    }
    std::reverse(path_.begin(), path_.end());
    //cerr << "astar path_size: " << _local_path_pxl.size() << endl;
    return status;
  }

  Planner2D::SearchStatus Planner2D::doDijkstra(const Vector2i& goal, const Vector2i& start, PathMatrix& pmap) {
    _dijkstra_search.setPathMatrix(&pmap);
    Point2iVectorCloud goals;
    Point2i goal_pt;
    goal_pt.coordinates() = goal;
    goals.push_back(goal_pt);
    _dijkstra_search.reset();
    int num_good_goals = _dijkstra_search.setGoals(goals);
    if (!num_good_goals) {
      return GoalNotFound;
    } 
    _dijkstra_search.compute();
    if (pmap.at(start).cost==_dijkstra_search.param_max_cost.constValue()) {
      return GoalNotFound;
    }
    return GoalFound;
  }
  

} // namespace srrg2_navigation_2d
