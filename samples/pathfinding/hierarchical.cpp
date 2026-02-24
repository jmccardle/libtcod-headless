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
#include "hierarchical.hpp"

#include <algorithm>

HierarchicalHeuristic::HierarchicalHeuristic()
    : mini_width_((MAP_WIDTH + SCALE - 1) / SCALE),
      mini_height_((MAP_HEIGHT + SCALE - 1) / SCALE),
      mini_map_(nullptr),
      mini_dijkstra_(nullptr),
      goal_x_(0),
      goal_y_(0),
      initialized_(false) {}

HierarchicalHeuristic::~HierarchicalHeuristic() {
  if (mini_dijkstra_) {
    TCOD_dijkstra_delete(mini_dijkstra_);
    mini_dijkstra_ = nullptr;
  }
  if (mini_map_) {
    TCOD_map_delete(mini_map_);
    mini_map_ = nullptr;
  }
}

void HierarchicalHeuristic::initialize() {
  create_minimap();
  mini_distances_.resize(mini_width_ * mini_height_, INFINITY_DIST);
  initialized_ = true;
}

void HierarchicalHeuristic::create_minimap() {
  mini_map_ = TCOD_map_new(mini_width_, mini_height_);

  // A mini-cell is walkable if ANY of its source cells are walkable
  // This ensures connectivity is preserved
  for (int my = 0; my < mini_height_; ++my) {
    for (int mx = 0; mx < mini_width_; ++mx) {
      bool walkable = false;
      bool transparent = false;

      // Check all cells in this mini-cell's region
      for (int dy = 0; dy < SCALE && !walkable; ++dy) {
        for (int dx = 0; dx < SCALE && !walkable; ++dx) {
          int x = mx * SCALE + dx;
          int y = my * SCALE + dy;

          if (x < MAP_WIDTH && y < MAP_HEIGHT) {
            if (is_walkable(x, y)) {
              walkable = true;
              transparent = true;
            }
          }
        }
      }

      TCOD_map_set_properties(mini_map_, mx, my, transparent, walkable);
    }
  }

  // Create Dijkstra solver for the minimap
  mini_dijkstra_ = TCOD_dijkstra_new(mini_map_, 1.41f);
}

void HierarchicalHeuristic::set_goal(int goal_x, int goal_y) {
  goal_x_ = goal_x;
  goal_y_ = goal_y;
  compute_minimap_dijkstra();
}

void HierarchicalHeuristic::compute_minimap_dijkstra() {
  if (!mini_dijkstra_) return;

  // Convert goal to minimap coordinates
  int mini_goal_x = goal_x_ / SCALE;
  int mini_goal_y = goal_y_ / SCALE;

  // Clamp to valid range
  mini_goal_x = std::max(0, std::min(mini_width_ - 1, mini_goal_x));
  mini_goal_y = std::max(0, std::min(mini_height_ - 1, mini_goal_y));

  // Compute Dijkstra from the goal
  TCOD_dijkstra_compute(mini_dijkstra_, mini_goal_x, mini_goal_y);

  // Extract distances into our array for fast lookup
  for (int my = 0; my < mini_height_; ++my) {
    for (int mx = 0; mx < mini_width_; ++mx) {
      float dist = TCOD_dijkstra_get_distance(mini_dijkstra_, mx, my);
      // TCOD returns -1 for unreachable cells
      if (dist < 0) {
        mini_distances_[my * mini_width_ + mx] = INFINITY_DIST;
      } else {
        mini_distances_[my * mini_width_ + mx] = dist;
      }
    }
  }
}

float HierarchicalHeuristic::heuristic(int x, int y) const {
  if (!initialized_) {
    // Fallback to Euclidean
    float dx = static_cast<float>(goal_x_ - x);
    float dy = static_cast<float>(goal_y_ - y);
    return std::sqrt(dx * dx + dy * dy);
  }

  // Convert to minimap coordinates
  int mx = x / SCALE;
  int my = y / SCALE;

  // Clamp to valid range
  mx = std::max(0, std::min(mini_width_ - 1, mx));
  my = std::max(0, std::min(mini_height_ - 1, my));

  float mini_dist = mini_distances_[my * mini_width_ + mx];

  if (mini_dist == INFINITY_DIST) {
    // Unreachable in minimap, use Euclidean
    float dx = static_cast<float>(goal_x_ - x);
    float dy = static_cast<float>(goal_y_ - y);
    return std::sqrt(dx * dx + dy * dy);
  }

  // Scale the minimap distance
  // This is an admissible heuristic because the minimap can only have
  // shorter paths (it ignores fine-grained obstacles)
  return mini_dist * static_cast<float>(SCALE);
}

float HierarchicalHeuristic::heuristic_callback(int x, int y, int goal_x, int goal_y, void* user_data) {
  auto* self = static_cast<HierarchicalHeuristic*>(user_data);
  (void)goal_x;
  (void)goal_y;
  return self->heuristic(x, y);
}

float HierarchicalHeuristic::get_mini_distance(int mx, int my) const {
  if (mx < 0 || mx >= mini_width_ || my < 0 || my >= mini_height_) {
    return INFINITY_DIST;
  }
  return mini_distances_[my * mini_width_ + mx];
}

bool HierarchicalHeuristic::is_mini_walkable(int mx, int my) const {
  if (!mini_map_ || mx < 0 || mx >= mini_width_ || my < 0 || my >= mini_height_) {
    return false;
  }
  return TCOD_map_is_walkable(mini_map_, mx, my);
}
