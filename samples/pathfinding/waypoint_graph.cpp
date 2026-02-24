/*
 * Copyright (c) 2026 libtcod contributors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "waypoint_graph.hpp"

#include <algorithm>
#include <cmath>

WaypointGraph::WaypointGraph()
    : goal_x_(0), goal_y_(0), goal_waypoint_(-1), goal_to_wp_dist_(INFINITY_DIST), initialized_(false) {
  // Initialize distance matrix to infinity
  for (int i = 0; i < NUM_WAYPOINTS; ++i) {
    for (int j = 0; j < NUM_WAYPOINTS; ++j) {
      wp_distances_[i][j] = (i == j) ? 0.0f : INFINITY_DIST;
    }
  }
}

void WaypointGraph::initialize() {
  compute_all_pairs_shortest_paths();
  precompute_cell_distances();
  initialized_ = true;
}

void WaypointGraph::compute_all_pairs_shortest_paths() {
  // Initialize with direct edge weights
  for (int i = 0; i < NUM_EDGES; ++i) {
    int from = EDGES[i].from;
    int to = EDGES[i].to;
    float dist = EDGES[i].distance;

    // Edges are bidirectional
    wp_distances_[from][to] = dist;
    wp_distances_[to][from] = dist;
  }

  // Floyd-Warshall algorithm
  for (int k = 0; k < NUM_WAYPOINTS; ++k) {
    for (int i = 0; i < NUM_WAYPOINTS; ++i) {
      for (int j = 0; j < NUM_WAYPOINTS; ++j) {
        if (wp_distances_[i][k] != INFINITY_DIST && wp_distances_[k][j] != INFINITY_DIST) {
          float through_k = wp_distances_[i][k] + wp_distances_[k][j];
          if (through_k < wp_distances_[i][j]) {
            wp_distances_[i][j] = through_k;
          }
        }
      }
    }
  }
}

void WaypointGraph::precompute_cell_distances() {
  cell_to_wp_dist_.resize(MAP_WIDTH * MAP_HEIGHT, INFINITY_DIST);

  for (int y = 0; y < MAP_HEIGHT; ++y) {
    for (int x = 0; x < MAP_WIDTH; ++x) {
      int idx = y * MAP_WIDTH + x;

      if (!is_walkable(x, y)) {
        cell_to_wp_dist_[idx] = INFINITY_DIST;
        continue;
      }

      int wp = find_nearest_waypoint(x, y);
      if (wp >= 0 && wp < NUM_WAYPOINTS) {
        // Simple Euclidean distance to waypoint center
        float dx = static_cast<float>(x - WAYPOINTS[wp].x);
        float dy = static_cast<float>(y - WAYPOINTS[wp].y);
        cell_to_wp_dist_[idx] = std::sqrt(dx * dx + dy * dy);
      }
    }
  }
}

void WaypointGraph::set_goal(int goal_x, int goal_y) {
  goal_x_ = goal_x;
  goal_y_ = goal_y;

  goal_waypoint_ = find_nearest_waypoint(goal_x, goal_y);
  if (goal_waypoint_ >= 0 && goal_waypoint_ < NUM_WAYPOINTS) {
    float dx = static_cast<float>(goal_x - WAYPOINTS[goal_waypoint_].x);
    float dy = static_cast<float>(goal_y - WAYPOINTS[goal_waypoint_].y);
    goal_to_wp_dist_ = std::sqrt(dx * dx + dy * dy);
  } else {
    goal_to_wp_dist_ = INFINITY_DIST;
  }
}

float WaypointGraph::heuristic(int x, int y) const {
  if (!initialized_ || goal_waypoint_ < 0) {
    // Fall back to Euclidean if not properly initialized
    float dx = static_cast<float>(goal_x_ - x);
    float dy = static_cast<float>(goal_y_ - y);
    return std::sqrt(dx * dx + dy * dy);
  }

  int wp = find_nearest_waypoint(x, y);
  if (wp < 0 || wp >= NUM_WAYPOINTS) {
    // Cell not in any room, use Euclidean fallback
    float dx = static_cast<float>(goal_x_ - x);
    float dy = static_cast<float>(goal_y_ - y);
    return std::sqrt(dx * dx + dy * dy);
  }

  // Heuristic = dist_to_my_wp + wp_to_goal_wp + goal_wp_to_goal
  float dist_to_wp = get_distance_to_waypoint(x, y);
  float wp_to_goal_wp = wp_distances_[wp][goal_waypoint_];
  float goal_from_wp = goal_to_wp_dist_;

  // If waypoints aren't connected, fall back to Euclidean
  if (wp_to_goal_wp == INFINITY_DIST) {
    float dx = static_cast<float>(goal_x_ - x);
    float dy = static_cast<float>(goal_y_ - y);
    return std::sqrt(dx * dx + dy * dy);
  }

  return dist_to_wp + wp_to_goal_wp + goal_from_wp;
}

float WaypointGraph::heuristic_callback(int x, int y, int goal_x, int goal_y, void* user_data) {
  auto* self = static_cast<WaypointGraph*>(user_data);
  // Note: goal is already set via set_goal(), we don't update it here
  // to avoid overhead. The caller must call set_goal() before pathfinding.
  (void)goal_x;
  (void)goal_y;
  return self->heuristic(x, y);
}

float WaypointGraph::get_waypoint_distance(int wp1, int wp2) const {
  if (wp1 < 0 || wp1 >= NUM_WAYPOINTS || wp2 < 0 || wp2 >= NUM_WAYPOINTS) {
    return INFINITY_DIST;
  }
  return wp_distances_[wp1][wp2];
}

int WaypointGraph::get_waypoint_for_room(Room room) const {
  for (int i = 0; i < NUM_WAYPOINTS; ++i) {
    if (WAYPOINTS[i].room == room) {
      return i;
    }
  }
  return -1;
}

int WaypointGraph::get_nearest_waypoint(int x, int y) const { return find_nearest_waypoint(x, y); }

float WaypointGraph::get_distance_to_waypoint(int x, int y) const {
  if (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) {
    return INFINITY_DIST;
  }
  return cell_to_wp_dist_[y * MAP_WIDTH + x];
}
