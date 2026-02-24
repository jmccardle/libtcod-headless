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
#include "visualization.hpp"

#include <algorithm>
#include <cmath>

Visualizer::Visualizer(int console_width, int console_height)
    : console_width_(console_width), console_height_(console_height) {}

void Visualizer::render_map(tcod::Console& console) {
  for (int y = 0; y < MAP_HEIGHT && y < console_height_; ++y) {
    for (int x = 0; x < MAP_WIDTH && x < console_width_; ++x) {
      char c = get_map_char(x, y);
      TCOD_ColorRGB bg;
      TCOD_ColorRGB fg = {200, 200, 200};
      int ch = c;

      switch (c) {
        case '#':
          bg = WALL_COLOR;
          fg = {60, 60, 70};
          ch = '#';
          break;
        case '+':
          bg = DOOR_COLOR;
          fg = {180, 130, 80};
          ch = '+';
          break;
        case '.':
        default:
          bg = FLOOR_COLOR;
          fg = {80, 70, 60};
          ch = '.';
          break;
      }

      console.at(x, y).ch = ch;
      console.at(x, y).fg = {fg.r, fg.g, fg.b, 255};
      console.at(x, y).bg = {bg.r, bg.g, bg.b, 255};
    }
  }
}

void Visualizer::render_heat_map(tcod::Console& console, const std::vector<float>& values, float max_value) {
  if (max_value <= 0) return;

  for (int y = 0; y < MAP_HEIGHT && y < console_height_; ++y) {
    for (int x = 0; x < MAP_WIDTH && x < console_width_; ++x) {
      if (is_wall(x, y)) continue;

      int idx = y * MAP_WIDTH + x;
      if (idx >= static_cast<int>(values.size())) continue;

      float v = values[idx];
      if (v < 0) continue;  // Not visited

      float t = std::min(1.0f, v / max_value);
      TCOD_ColorRGB color = heat_color(t);

      // Blend with existing background
      auto& tile = console.at(x, y);
      tile.bg = {color.r, color.g, color.b, 255};
    }
  }
}

void Visualizer::render_dijkstra_gradient(tcod::Console& console, TCOD_Dijkstra* dijkstra, bool inverted) {
  if (!dijkstra) return;

  // Find max distance for normalization
  float max_dist = 0;
  for (int y = 0; y < MAP_HEIGHT; ++y) {
    for (int x = 0; x < MAP_WIDTH; ++x) {
      float d = TCOD_dijkstra_get_distance(dijkstra, x, y);
      if (d >= 0 && d < 1000000) {
        max_dist = std::max(max_dist, d);
      }
    }
  }

  if (max_dist <= 0) return;

  for (int y = 0; y < MAP_HEIGHT && y < console_height_; ++y) {
    for (int x = 0; x < MAP_WIDTH && x < console_width_; ++x) {
      if (is_wall(x, y)) continue;

      float d = TCOD_dijkstra_get_distance(dijkstra, x, y);
      if (d < 0 || d > 1000000) continue;  // Unreachable

      float t = d / max_dist;
      TCOD_ColorRGB color = inverted ? flee_color(t) : distance_color(t);

      auto& tile = console.at(x, y);
      tile.bg = {color.r, color.g, color.b, 255};
    }
  }
}

void Visualizer::render_path(tcod::Console& console, TCOD_path_t path) {
  if (!path) return;

  int path_size = TCOD_path_size(path);
  for (int i = 0; i < path_size; ++i) {
    int x, y;
    TCOD_path_get(path, i, &x, &y);

    if (x >= 0 && x < console_width_ && y >= 0 && y < console_height_) {
      auto& tile = console.at(x, y);
      tile.ch = '*';
      tile.fg = {PATH_COLOR.r, PATH_COLOR.g, PATH_COLOR.b, 255};
    }
  }
}

void Visualizer::render_waypoint_graph(tcod::Console& console, const WaypointGraph& graph) {
  (void)graph;  // Uses global WAYPOINTS and EDGES constants
  // Draw edges first (under waypoints)
  for (int i = 0; i < NUM_EDGES; ++i) {
    int from = EDGES[i].from;
    int to = EDGES[i].to;

    int x1 = WAYPOINTS[from].x;
    int y1 = WAYPOINTS[from].y;
    int x2 = WAYPOINTS[to].x;
    int y2 = WAYPOINTS[to].y;

    // Simple line drawing (Bresenham-like)
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int sx = x1 < x2 ? 1 : -1;
    int sy = y1 < y2 ? 1 : -1;
    int err = dx - dy;

    int x = x1, y = y1;
    while (true) {
      if (x >= 0 && x < console_width_ && y >= 0 && y < console_height_) {
        if (!is_wall(x, y)) {
          auto& tile = console.at(x, y);
          // Don't overwrite waypoint markers
          if (tile.ch != 'W') {
            tile.ch = '.';
            tile.fg = {EDGE_COLOR.r, EDGE_COLOR.g, EDGE_COLOR.b, 255};
          }
        }
      }

      if (x == x2 && y == y2) break;

      int e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        x += sx;
      }
      if (e2 < dx) {
        err += dx;
        y += sy;
      }
    }
  }

  // Draw waypoints
  for (int i = 0; i < NUM_WAYPOINTS; ++i) {
    int x = WAYPOINTS[i].x;
    int y = WAYPOINTS[i].y;

    if (x >= 0 && x < console_width_ && y >= 0 && y < console_height_) {
      auto& tile = console.at(x, y);
      tile.ch = 'W';
      tile.fg = {WAYPOINT_COLOR.r, WAYPOINT_COLOR.g, WAYPOINT_COLOR.b, 255};
      tile.bg = {20, 20, 40, 255};
    }
  }
}

void Visualizer::render_minimap(tcod::Console& console, const HierarchicalHeuristic& hier, int offset_x, int offset_y) {
  int mw = hier.get_mini_width();
  int mh = hier.get_mini_height();

  // Draw minimap border
  for (int my = -1; my <= mh; ++my) {
    for (int mx = -1; mx <= mw; ++mx) {
      int cx = offset_x + mx;
      int cy = offset_y + my;

      if (cx < 0 || cx >= console_width_ || cy < 0 || cy >= console_height_) continue;

      if (mx == -1 || mx == mw || my == -1 || my == mh) {
        // Border
        console.at(cx, cy).ch = '#';
        console.at(cx, cy).fg = {100, 100, 100, 255};
        console.at(cx, cy).bg = {50, 50, 50, 255};
      } else {
        // Minimap cell
        bool walkable = hier.is_mini_walkable(mx, my);
        float dist = hier.get_mini_distance(mx, my);

        TCOD_ColorRGB bg;
        if (!walkable) {
          bg = {30, 30, 40};
        } else if (dist < 0 || dist > 1000000) {
          bg = {60, 30, 30};  // Unreachable
        } else {
          // Color by distance
          float t = std::min(1.0f, dist / 20.0f);
          bg = distance_color(t);
        }

        console.at(cx, cy).ch = walkable ? '.' : '#';
        console.at(cx, cy).fg = {150, 150, 150, 255};
        console.at(cx, cy).bg = {bg.r, bg.g, bg.b, 255};
      }
    }
  }
}

void Visualizer::render_start(tcod::Console& console, int x, int y) {
  if (x >= 0 && x < console_width_ && y >= 0 && y < console_height_) {
    auto& tile = console.at(x, y);
    tile.ch = '@';
    tile.fg = {START_COLOR.r, START_COLOR.g, START_COLOR.b, 255};
  }
}

void Visualizer::render_goal(tcod::Console& console, int x, int y) {
  if (x >= 0 && x < console_width_ && y >= 0 && y < console_height_) {
    auto& tile = console.at(x, y);
    tile.ch = 'X';
    tile.fg = {GOAL_COLOR.r, GOAL_COLOR.g, GOAL_COLOR.b, 255};
  }
}

void Visualizer::render_treasures(tcod::Console& console) {
  for (int i = 0; i < NUM_TREASURES; ++i) {
    int x = TREASURE_X[i];
    int y = TREASURE_Y[i];

    if (x >= 0 && x < console_width_ && y >= 0 && y < console_height_) {
      auto& tile = console.at(x, y);
      tile.ch = '$';
      tile.fg = {TREASURE_COLOR.r, TREASURE_COLOR.g, TREASURE_COLOR.b, 255};
    }
  }
}

void Visualizer::render_threats(tcod::Console& console) {
  for (int i = 0; i < NUM_THREATS; ++i) {
    int x = THREAT_X[i];
    int y = THREAT_Y[i];

    if (x >= 0 && x < console_width_ && y >= 0 && y < console_height_) {
      auto& tile = console.at(x, y);
      tile.ch = '!';
      tile.fg = {THREAT_COLOR.r, THREAT_COLOR.g, THREAT_COLOR.b, 255};
    }
  }
}

void Visualizer::render_entity(tcod::Console& console, int x, int y, int ch, TCOD_ColorRGB color) {
  if (x >= 0 && x < console_width_ && y >= 0 && y < console_height_) {
    auto& tile = console.at(x, y);
    tile.ch = ch;
    tile.fg = {color.r, color.g, color.b, 255};
  }
}

void Visualizer::render_metrics(tcod::Console& console, const PathMetrics& metrics, DemoMode mode) {
  int y = MAP_HEIGHT + 1;

  std::string mode_str = get_mode_name(mode);
  std::string desc_str = get_mode_description(mode);

  tcod::print(console, {1, y}, tcod::stringf("Mode %d: %s", static_cast<int>(mode), mode_str.c_str()), TEXT_COLOR, {});
  y++;

  tcod::print(console, {1, y}, desc_str, TCOD_ColorRGB{180, 180, 180}, std::nullopt);
  y += 2;

  tcod::print(
      console,
      {1, y},
      tcod::stringf("Nodes expanded: %d   Path length: %d", metrics.nodes_expanded, metrics.path_length),
      TEXT_COLOR,
      {});
  y++;

  if (metrics.computation_time_ms > 0) {
    tcod::print(console, {1, y}, tcod::stringf("Time: %.2f ms", metrics.computation_time_ms), TEXT_COLOR, {});
  }
}

void Visualizer::render_help(tcod::Console& console) {
  int y = console_height_ - 3;

  tcod::print(
      console,
      {1, y},
      "1-5: Change mode   W: Waypoints   H: Minimap   R: Reset   Q: Quit",
      TCOD_ColorRGB{150, 150, 150},
      std::nullopt);
}

void Visualizer::render_mode_indicator(tcod::Console& console, DemoMode mode) {
  const char* name = get_mode_name(mode);
  tcod::print(
      console, {console_width_ - 20, 1}, tcod::stringf("[%d] %s", static_cast<int>(mode), name), TEXT_COLOR, {});
}

TCOD_ColorRGB Visualizer::heat_color(float t) {
  // Blue -> Cyan -> Green -> Yellow -> Red
  t = std::max(0.0f, std::min(1.0f, t));

  if (t < 0.25f) {
    float u = t / 0.25f;
    return {0, static_cast<uint8_t>(255 * u), 255};
  } else if (t < 0.5f) {
    float u = (t - 0.25f) / 0.25f;
    return {0, 255, static_cast<uint8_t>(255 * (1 - u))};
  } else if (t < 0.75f) {
    float u = (t - 0.5f) / 0.25f;
    return {static_cast<uint8_t>(255 * u), 255, 0};
  } else {
    float u = (t - 0.75f) / 0.25f;
    return {255, static_cast<uint8_t>(255 * (1 - u)), 0};
  }
}

TCOD_ColorRGB Visualizer::gradient_color(float t) {
  // Green -> Yellow -> Red
  t = std::max(0.0f, std::min(1.0f, t));

  if (t < 0.5f) {
    float u = t / 0.5f;
    return {static_cast<uint8_t>(255 * u), 255, 0};
  } else {
    float u = (t - 0.5f) / 0.5f;
    return {255, static_cast<uint8_t>(255 * (1 - u)), 0};
  }
}

TCOD_ColorRGB Visualizer::flee_color(float t) {
  // Red (near, dangerous) -> Green (far, safe)
  t = std::max(0.0f, std::min(1.0f, t));

  return {static_cast<uint8_t>(255 * (1 - t)), static_cast<uint8_t>(200 * t), 0};
}

TCOD_ColorRGB Visualizer::distance_color(float t) {
  // White -> Blue -> Dark blue
  t = std::max(0.0f, std::min(1.0f, t));

  if (t < 0.5f) {
    float u = t / 0.5f;
    return {static_cast<uint8_t>(255 * (1 - u)), static_cast<uint8_t>(255 * (1 - u)), 255};
  } else {
    float u = (t - 0.5f) / 0.5f;
    return {0, 0, static_cast<uint8_t>(255 * (1 - 0.5f * u))};
  }
}

const char* get_mode_name(DemoMode mode) {
  switch (mode) {
    case DemoMode::EUCLIDEAN_ASTAR:
      return "Euclidean A*";
    case DemoMode::WAYPOINT_ASTAR:
      return "Waypoint A*";
    case DemoMode::HIERARCHICAL_ASTAR:
      return "Hierarchical A*";
    case DemoMode::MULTI_GOAL_DIJKSTRA:
      return "Multi-Goal Dijkstra";
    case DemoMode::FLEE_MAP:
      return "Flee Map";
    default:
      return "Unknown";
  }
}

const char* get_mode_description(DemoMode mode) {
  switch (mode) {
    case DemoMode::EUCLIDEAN_ASTAR:
      return "Standard A* with Euclidean distance heuristic";
    case DemoMode::WAYPOINT_ASTAR:
      return "A* with waypoint graph heuristic (domain knowledge)";
    case DemoMode::HIERARCHICAL_ASTAR:
      return "A* with minimap (hierarchical) heuristic";
    case DemoMode::MULTI_GOAL_DIJKSTRA:
      return "Dijkstra with multiple goal positions";
    case DemoMode::FLEE_MAP:
      return "Inverted Dijkstra for flee/safety behavior";
    default:
      return "";
  }
}
