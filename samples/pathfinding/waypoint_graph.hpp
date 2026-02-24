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
/**
 * @file waypoint_graph.hpp
 * @brief Precomputed waypoint graph for domain-knowledge heuristic.
 *
 * This demonstrates the key insight: the real value of custom heuristics
 * isn't geometric formulas (anyone can write abs(dx)+abs(dy)). The value
 * is INJECTING DOMAIN KNOWLEDGE into the search.
 *
 * The waypoint graph precomputes shortest paths between room centers.
 * The heuristic then uses: dist_to_waypoint + graph_distance + dist_from_waypoint
 * This guides A* toward the actual shortest path through room connectivity.
 */
#pragma once
#ifndef WAYPOINT_GRAPH_HPP_
#define WAYPOINT_GRAPH_HPP_

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "monastery_map.h"

/**
 * Precomputed waypoint graph data structure.
 *
 * Stores:
 * - Shortest path distances between all waypoint pairs (Floyd-Warshall)
 * - For each map cell, the nearest waypoint and distance to it
 * - Goal waypoint information for heuristic calculation
 */
class WaypointGraph {
 public:
  static constexpr float INFINITY_DIST = std::numeric_limits<float>::infinity();

  WaypointGraph();

  /**
   * Initialize the waypoint graph from the map data.
   * Must be called before using the heuristic.
   */
  void initialize();

  /**
   * Set the goal position for heuristic calculations.
   * Call this before each A* search.
   */
  void set_goal(int goal_x, int goal_y);

  /**
   * Custom heuristic function for A*.
   * Estimates distance from (x,y) to the goal using waypoint graph.
   *
   * Returns: dist_to_nearest_wp + wp_graph_dist_to_goal_wp + goal_dist_from_wp
   */
  float heuristic(int x, int y) const;

  /**
   * Static wrapper for TCOD_heuristic_func_t callback.
   * user_data must point to a WaypointGraph instance.
   */
  static float heuristic_callback(int x, int y, int goal_x, int goal_y, void* user_data);

  /**
   * Get the precomputed distance between two waypoints.
   */
  float get_waypoint_distance(int wp1, int wp2) const;

  /**
   * Get the waypoint index for a given room.
   */
  int get_waypoint_for_room(Room room) const;

  /**
   * Get the nearest waypoint to a position.
   */
  int get_nearest_waypoint(int x, int y) const;

  /**
   * Get distance from a cell to its nearest waypoint.
   */
  float get_distance_to_waypoint(int x, int y) const;

 private:
  // Floyd-Warshall all-pairs shortest paths between waypoints
  std::array<std::array<float, NUM_WAYPOINTS>, NUM_WAYPOINTS> wp_distances_;

  // For each cell, distance to its room's waypoint (precomputed)
  std::vector<float> cell_to_wp_dist_;

  // Goal information
  int goal_x_, goal_y_;
  int goal_waypoint_;
  float goal_to_wp_dist_;

  bool initialized_;

  /**
   * Compute all-pairs shortest paths using Floyd-Warshall.
   */
  void compute_all_pairs_shortest_paths();

  /**
   * Precompute distances from each cell to its room's waypoint.
   */
  void precompute_cell_distances();
};

/**
 * Euclidean heuristic wrapper that counts nodes expanded.
 * Used for comparison metrics.
 */
class CountingEuclideanHeuristic {
 public:
  CountingEuclideanHeuristic() : nodes_expanded_(0), goal_x_(0), goal_y_(0) {}

  void set_goal(int goal_x, int goal_y) {
    goal_x_ = goal_x;
    goal_y_ = goal_y;
  }

  void reset_count() { nodes_expanded_ = 0; }
  int get_count() const { return nodes_expanded_; }

  float heuristic(int x, int y) {
    ++nodes_expanded_;
    float dx = static_cast<float>(goal_x_ - x);
    float dy = static_cast<float>(goal_y_ - y);
    return std::sqrt(dx * dx + dy * dy);
  }

  static float heuristic_callback(int x, int y, int goal_x, int goal_y, void* user_data) {
    auto* self = static_cast<CountingEuclideanHeuristic*>(user_data);
    // Update goal in case it changed (though typically it doesn't)
    self->goal_x_ = goal_x;
    self->goal_y_ = goal_y;
    return self->heuristic(x, y);
  }

 private:
  mutable int nodes_expanded_;
  int goal_x_, goal_y_;
};

/**
 * Waypoint heuristic wrapper that counts nodes expanded.
 */
class CountingWaypointHeuristic {
 public:
  CountingWaypointHeuristic() : nodes_expanded_(0) {}

  void initialize() { graph_.initialize(); }

  void set_goal(int goal_x, int goal_y) { graph_.set_goal(goal_x, goal_y); }

  void reset_count() { nodes_expanded_ = 0; }
  int get_count() const { return nodes_expanded_; }

  float heuristic(int x, int y) {
    ++nodes_expanded_;
    return graph_.heuristic(x, y);
  }

  static float heuristic_callback(int x, int y, int goal_x, int goal_y, void* user_data) {
    (void)goal_x;
    (void)goal_y;
    auto* self = static_cast<CountingWaypointHeuristic*>(user_data);
    return self->heuristic(x, y);
  }

  WaypointGraph& get_graph() { return graph_; }

 private:
  WaypointGraph graph_;
  mutable int nodes_expanded_;
};

#endif  // WAYPOINT_GRAPH_HPP_
