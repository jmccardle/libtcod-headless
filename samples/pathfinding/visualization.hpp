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
 * @file visualization.hpp
 * @brief Visualization utilities for pathfinding demo.
 *
 * Provides:
 * - Heat map rendering (cells colored by visit order or distance)
 * - Path highlighting
 * - Waypoint graph overlay
 * - Minimap overlay
 * - Dijkstra gradient visualization
 * - Metrics display
 */
#pragma once
#ifndef VISUALIZATION_HPP_
#define VISUALIZATION_HPP_

#include <libtcod.hpp>
#include <string>
#include <vector>

#include "hierarchical.hpp"
#include "monastery_map.h"
#include "waypoint_graph.hpp"

/**
 * Pathfinding metrics for display.
 */
struct PathMetrics {
  int nodes_expanded = 0;
  int path_length = 0;
  float computation_time_ms = 0.0f;
  std::string algorithm_name;
};

/**
 * Demo mode enumeration.
 */
enum class DemoMode {
  EUCLIDEAN_ASTAR = 1,
  WAYPOINT_ASTAR = 2,
  HIERARCHICAL_ASTAR = 3,
  MULTI_GOAL_DIJKSTRA = 4,
  FLEE_MAP = 5,
};

/**
 * Visualization state and rendering functions.
 */
class Visualizer {
 public:
  Visualizer(int console_width, int console_height);

  /**
   * Render the base map (walls and floors).
   */
  void render_map(tcod::Console& console);

  /**
   * Render a heat map overlay showing node expansion order or distances.
   * Values are normalized to [0,1] range for coloring.
   */
  void render_heat_map(tcod::Console& console, const std::vector<float>& values, float max_value);

  /**
   * Render Dijkstra distance map as a gradient.
   * Useful for flee map and multi-goal visualization.
   */
  void render_dijkstra_gradient(tcod::Console& console, TCOD_Dijkstra* dijkstra, bool inverted = false);

  /**
   * Render a computed path.
   */
  void render_path(tcod::Console& console, TCOD_path_t path);

  /**
   * Render the waypoint graph overlay (waypoints and edges).
   */
  void render_waypoint_graph(tcod::Console& console, const WaypointGraph& graph);

  /**
   * Render the hierarchical minimap overlay.
   */
  void render_minimap(tcod::Console& console, const HierarchicalHeuristic& hier, int offset_x, int offset_y);

  /**
   * Render start position marker.
   */
  void render_start(tcod::Console& console, int x, int y);

  /**
   * Render goal position marker.
   */
  void render_goal(tcod::Console& console, int x, int y);

  /**
   * Render treasure markers for multi-goal demo.
   */
  void render_treasures(tcod::Console& console);

  /**
   * Render threat markers for flee demo.
   */
  void render_threats(tcod::Console& console);

  /**
   * Render an entity (player, monster, etc.).
   */
  void render_entity(tcod::Console& console, int x, int y, int ch, TCOD_ColorRGB color);

  /**
   * Render metrics panel.
   */
  void render_metrics(tcod::Console& console, const PathMetrics& metrics, DemoMode mode);

  /**
   * Render help text showing controls.
   */
  void render_help(tcod::Console& console);

  /**
   * Render mode indicator.
   */
  void render_mode_indicator(tcod::Console& console, DemoMode mode);

  /**
   * Color schemes for heat maps.
   */
  static TCOD_ColorRGB heat_color(float t);  // Blue -> Red gradient
  static TCOD_ColorRGB gradient_color(float t);  // Green -> Yellow -> Red
  static TCOD_ColorRGB flee_color(float t);  // Red (danger) -> Green (safe)
  static TCOD_ColorRGB distance_color(float t);  // White -> Dark blue

 private:
  int console_width_, console_height_;

  // Color constants
  static constexpr TCOD_ColorRGB WALL_COLOR = {40, 40, 50};
  static constexpr TCOD_ColorRGB FLOOR_COLOR = {100, 90, 80};
  static constexpr TCOD_ColorRGB DOOR_COLOR = {139, 90, 43};
  static constexpr TCOD_ColorRGB PATH_COLOR = {255, 255, 0};
  static constexpr TCOD_ColorRGB START_COLOR = {0, 255, 0};
  static constexpr TCOD_ColorRGB GOAL_COLOR = {255, 0, 0};
  static constexpr TCOD_ColorRGB TREASURE_COLOR = {255, 215, 0};
  static constexpr TCOD_ColorRGB THREAT_COLOR = {255, 0, 128};
  static constexpr TCOD_ColorRGB WAYPOINT_COLOR = {0, 200, 255};
  static constexpr TCOD_ColorRGB EDGE_COLOR = {0, 150, 200};
  static constexpr TCOD_ColorRGB TEXT_COLOR = {255, 255, 255};
  static constexpr TCOD_ColorRGB TEXT_BG_COLOR = {0, 0, 0};
};

/**
 * Get the name of a demo mode.
 */
const char* get_mode_name(DemoMode mode);

/**
 * Get a brief description of a demo mode.
 */
const char* get_mode_description(DemoMode mode);

#endif  // VISUALIZATION_HPP_
