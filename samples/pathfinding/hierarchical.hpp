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
 * @file hierarchical.hpp
 * @brief Hierarchical (minimap) heuristic for A*.
 *
 * The hierarchical heuristic technique:
 * 1. Create a scaled-down "minimap" version of the map (e.g., 5:1 or 4:1)
 * 2. Precompute Dijkstra distances on the minimap from the goal region
 * 3. For each A* node, look up minimap distance and scale it
 *
 * This provides near-optimal guidance with minimal precomputation.
 * The minimap naturally captures room connectivity because blocked
 * regions stay blocked at reduced resolution.
 *
 * Trade-off: Less accurate than waypoint graph (minimap cells blend
 * walkable/blocked), but requires no manual room annotation.
 */
#pragma once
#ifndef HIERARCHICAL_HPP_
#define HIERARCHICAL_HPP_

#include <libtcod.h>

#include <cmath>
#include <limits>
#include <vector>

#include "monastery_map.h"

/**
 * Hierarchical heuristic using a scaled-down map.
 */
class HierarchicalHeuristic {
 public:
  static constexpr int SCALE = 4;  // 4:1 reduction
  static constexpr float INFINITY_DIST = std::numeric_limits<float>::infinity();

  HierarchicalHeuristic();
  ~HierarchicalHeuristic();

  /**
   * Initialize the minimap and Dijkstra solver.
   * Must be called before using the heuristic.
   */
  void initialize();

  /**
   * Set the goal position and precompute minimap distances.
   * Call this before each A* search.
   */
  void set_goal(int goal_x, int goal_y);

  /**
   * Custom heuristic function for A*.
   * Returns minimap distance * SCALE.
   */
  float heuristic(int x, int y) const;

  /**
   * Static wrapper for TCOD_heuristic_func_t callback.
   */
  static float heuristic_callback(int x, int y, int goal_x, int goal_y, void* user_data);

  /**
   * Get the minimap dimensions.
   */
  int get_mini_width() const { return mini_width_; }
  int get_mini_height() const { return mini_height_; }

  /**
   * Get the minimap distance at a minimap coordinate.
   */
  float get_mini_distance(int mx, int my) const;

  /**
   * Check if a minimap cell is walkable.
   */
  bool is_mini_walkable(int mx, int my) const;

 private:
  int mini_width_, mini_height_;
  TCOD_Map* mini_map_;
  TCOD_Dijkstra* mini_dijkstra_;
  std::vector<float> mini_distances_;

  int goal_x_, goal_y_;
  bool initialized_;

  /**
   * Create the scaled-down map.
   * A mini-cell is walkable if ANY of its source cells are walkable.
   */
  void create_minimap();

  /**
   * Compute Dijkstra on the minimap from the goal region.
   */
  void compute_minimap_dijkstra();
};

/**
 * Counting wrapper for hierarchical heuristic.
 */
class CountingHierarchicalHeuristic {
 public:
  CountingHierarchicalHeuristic() : nodes_expanded_(0) {}

  void initialize() { heuristic_.initialize(); }

  void set_goal(int goal_x, int goal_y) { heuristic_.set_goal(goal_x, goal_y); }

  void reset_count() { nodes_expanded_ = 0; }
  int get_count() const { return nodes_expanded_; }

  float heuristic(int x, int y) {
    ++nodes_expanded_;
    return heuristic_.heuristic(x, y);
  }

  static float heuristic_callback(int x, int y, int goal_x, int goal_y, void* user_data) {
    (void)goal_x;
    (void)goal_y;
    auto* self = static_cast<CountingHierarchicalHeuristic*>(user_data);
    return self->heuristic(x, y);
  }

  HierarchicalHeuristic& get_heuristic() { return heuristic_; }

 private:
  HierarchicalHeuristic heuristic_;
  mutable int nodes_expanded_;
};

#endif  // HIERARCHICAL_HPP_
