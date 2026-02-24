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
 * @file main.cpp
 * @brief Pathfinding demonstration comparing heuristic strategies.
 *
 * This demo shows the power of domain-knowledge heuristics vs naive geometric
 * heuristics. The monastery map is designed so that Euclidean distance misleads
 * A*, causing it to explore the Great Hall when the optimal path is through
 * the Long Corridor.
 *
 * Demo modes:
 * 1. Euclidean A*      - Shows baseline behavior with standard heuristic
 * 2. Waypoint A*       - Uses precomputed room connectivity graph
 * 3. Hierarchical A*   - Uses minimap (reduced resolution) distances
 * 4. Multi-Goal Dijkstra - Multiple treasures, find nearest
 * 5. Flee Map          - Inverted Dijkstra for AI flee behavior
 *
 * Controls:
 *   1-5: Change demo mode
 *   W: Toggle waypoint graph overlay
 *   H: Toggle hierarchical minimap overlay
 *   R: Reset/recompute paths
 *   Q: Quit
 */
#define SDL_MAIN_USE_CALLBACKS
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <chrono>
#include <libtcod.hpp>
#include <libtcod/timer.hpp>

#include "hierarchical.hpp"
#include "monastery_map.h"
#include "visualization.hpp"
#include "waypoint_graph.hpp"

// Console dimensions
static constexpr int CONSOLE_WIDTH = MAP_WIDTH;
static constexpr int CONSOLE_HEIGHT = MAP_HEIGHT + 8;  // Extra space for metrics

// Global state
static tcod::Console g_console{CONSOLE_WIDTH, CONSOLE_HEIGHT};
static tcod::Context g_context;
static auto g_timer = tcod::Timer();

static DemoMode g_mode = DemoMode::EUCLIDEAN_ASTAR;
static PathMetrics g_metrics;
static Visualizer g_viz(CONSOLE_WIDTH, CONSOLE_HEIGHT);

// Pathfinding state
static TCOD_Map* g_map = nullptr;
static TCOD_path_t g_path = nullptr;
static TCOD_Dijkstra* g_dijkstra = nullptr;

// Heuristic wrappers (with node counting)
static CountingEuclideanHeuristic g_euclidean;
static CountingWaypointHeuristic g_waypoint;
static CountingHierarchicalHeuristic g_hierarchical;

// Overlay toggles
static bool g_show_waypoints = false;
static bool g_show_minimap = false;

// Entity positions (for flee demo)
static int g_monster_x = 30;
static int g_monster_y = 20;

/**
 * Walk cost function for A* pathfinding.
 */
static float walk_cost(int x1, int y1, int x2, int y2, void* user_data) {
  (void)user_data;
  (void)x1;
  (void)y1;

  if (!is_walkable(x2, y2)) {
    return 0.0f;  // Blocked
  }
  return 1.0f;  // Uniform cost
}

/**
 * Initialize the pathfinding structures.
 */
static void init_pathfinding() {
  // Create TCOD map from our monastery map
  g_map = TCOD_map_new(MAP_WIDTH, MAP_HEIGHT);
  for (int y = 0; y < MAP_HEIGHT; ++y) {
    for (int x = 0; x < MAP_WIDTH; ++x) {
      bool walkable = is_walkable(x, y);
      TCOD_map_set_properties(g_map, x, y, walkable, walkable);
    }
  }

  // Initialize heuristics
  g_waypoint.initialize();
  g_hierarchical.initialize();

  // Create Dijkstra solver
  g_dijkstra = TCOD_dijkstra_new(g_map, 1.41f);
}

/**
 * Clean up pathfinding structures.
 */
static void cleanup_pathfinding() {
  if (g_path) {
    TCOD_path_delete(g_path);
    g_path = nullptr;
  }
  if (g_dijkstra) {
    TCOD_dijkstra_delete(g_dijkstra);
    g_dijkstra = nullptr;
  }
  if (g_map) {
    TCOD_map_delete(g_map);
    g_map = nullptr;
  }
}

/**
 * Compute path using current mode's heuristic.
 */
static void compute_path() {
  // Clean up previous path
  if (g_path) {
    TCOD_path_delete(g_path);
    g_path = nullptr;
  }

  g_metrics = PathMetrics{};
  g_metrics.algorithm_name = get_mode_name(g_mode);

  auto start_time = std::chrono::high_resolution_clock::now();

  switch (g_mode) {
    case DemoMode::EUCLIDEAN_ASTAR: {
      g_euclidean.set_goal(GOAL_X, GOAL_Y);
      g_euclidean.reset_count();

      g_path = TCOD_path_new_using_function_ex(
          MAP_WIDTH, MAP_HEIGHT, walk_cost, CountingEuclideanHeuristic::heuristic_callback, &g_euclidean, 1.41f, 1.0f);

      TCOD_path_compute(g_path, START_X, START_Y, GOAL_X, GOAL_Y);
      g_metrics.nodes_expanded = g_euclidean.get_count();
      break;
    }

    case DemoMode::WAYPOINT_ASTAR: {
      g_waypoint.set_goal(GOAL_X, GOAL_Y);
      g_waypoint.reset_count();

      g_path = TCOD_path_new_using_function_ex(
          MAP_WIDTH, MAP_HEIGHT, walk_cost, CountingWaypointHeuristic::heuristic_callback, &g_waypoint, 1.41f, 1.0f);

      TCOD_path_compute(g_path, START_X, START_Y, GOAL_X, GOAL_Y);
      g_metrics.nodes_expanded = g_waypoint.get_count();
      break;
    }

    case DemoMode::HIERARCHICAL_ASTAR: {
      g_hierarchical.set_goal(GOAL_X, GOAL_Y);
      g_hierarchical.reset_count();

      g_path = TCOD_path_new_using_function_ex(
          MAP_WIDTH,
          MAP_HEIGHT,
          walk_cost,
          CountingHierarchicalHeuristic::heuristic_callback,
          &g_hierarchical,
          1.41f,
          1.0f);

      TCOD_path_compute(g_path, START_X, START_Y, GOAL_X, GOAL_Y);
      g_metrics.nodes_expanded = g_hierarchical.get_count();
      break;
    }

    case DemoMode::MULTI_GOAL_DIJKSTRA: {
      // Compute Dijkstra from multiple treasure positions
      TCOD_dijkstra_compute_multi(g_dijkstra, NUM_TREASURES, TREASURE_X, TREASURE_Y);
      g_metrics.nodes_expanded = MAP_WIDTH * MAP_HEIGHT;  // Dijkstra visits all reachable
      break;
    }

    case DemoMode::FLEE_MAP: {
      // Compute Dijkstra from threat positions, then invert
      TCOD_dijkstra_compute_multi(g_dijkstra, NUM_THREATS, THREAT_X, THREAT_Y);
      TCOD_dijkstra_invert(g_dijkstra);
      g_metrics.nodes_expanded = MAP_WIDTH * MAP_HEIGHT;
      break;
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  g_metrics.computation_time_ms = std::chrono::duration<float, std::milli>(end_time - start_time).count();

  if (g_path) {
    g_metrics.path_length = TCOD_path_size(g_path);
  }
}

/**
 * Render the current state.
 */
static void render() {
  // Clear console
  for (int y = 0; y < CONSOLE_HEIGHT; ++y) {
    for (int x = 0; x < CONSOLE_WIDTH; ++x) {
      g_console.at(x, y) = {' ', {255, 255, 255, 255}, {0, 0, 0, 255}};
    }
  }

  // Render base map
  g_viz.render_map(g_console);

  // Mode-specific rendering
  switch (g_mode) {
    case DemoMode::EUCLIDEAN_ASTAR:
    case DemoMode::WAYPOINT_ASTAR:
    case DemoMode::HIERARCHICAL_ASTAR:
      // Render path
      if (g_path) {
        g_viz.render_path(g_console, g_path);
      }
      // Render start/goal
      g_viz.render_start(g_console, START_X, START_Y);
      g_viz.render_goal(g_console, GOAL_X, GOAL_Y);
      break;

    case DemoMode::MULTI_GOAL_DIJKSTRA:
      // Render Dijkstra gradient
      g_viz.render_dijkstra_gradient(g_console, g_dijkstra, false);
      // Render treasures
      g_viz.render_treasures(g_console);
      // Render an entity that could move toward nearest treasure
      g_viz.render_entity(g_console, START_X, START_Y, '@', {0, 255, 0});
      break;

    case DemoMode::FLEE_MAP:
      // Render inverted Dijkstra (flee gradient)
      g_viz.render_dijkstra_gradient(g_console, g_dijkstra, true);
      // Render threats
      g_viz.render_threats(g_console);
      // Render monster that's fleeing
      g_viz.render_entity(g_console, g_monster_x, g_monster_y, 'M', {200, 0, 200});
      break;
  }

  // Overlays
  if (g_show_waypoints && g_mode == DemoMode::WAYPOINT_ASTAR) {
    g_viz.render_waypoint_graph(g_console, g_waypoint.get_graph());
  }

  if (g_show_minimap && g_mode == DemoMode::HIERARCHICAL_ASTAR) {
    g_viz.render_minimap(g_console, g_hierarchical.get_heuristic(), CONSOLE_WIDTH - 20, 2);
  }

  // Metrics and help
  g_viz.render_metrics(g_console, g_metrics, g_mode);
  g_viz.render_help(g_console);

  // Present
  g_context.present(g_console);
}

/**
 * Update flee demo (monster moves away from threats).
 */
static void update_flee_demo(float delta) {
  static float move_timer = 0.0f;
  move_timer += delta;

  if (move_timer >= 0.3f) {
    move_timer = 0.0f;

    // Use gradient descent to find best escape direction
    int next_x, next_y;
    if (TCOD_dijkstra_get_descent(g_dijkstra, g_monster_x, g_monster_y, &next_x, &next_y)) {
      g_monster_x = next_x;
      g_monster_y = next_y;
    }
  }
}

SDL_AppResult SDL_AppIterate(void*) {
  float delta = g_timer.sync(30);  // 30 FPS target

  // Update flee demo if active
  if (g_mode == DemoMode::FLEE_MAP) {
    update_flee_demo(delta);
  }

  render();

  return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void*, SDL_Event* event) {
  switch (event->type) {
    case SDL_EVENT_QUIT:
      return SDL_APP_SUCCESS;

    case SDL_EVENT_KEY_DOWN:
      switch (event->key.scancode) {
        case SDL_SCANCODE_Q:
        case SDL_SCANCODE_ESCAPE:
          return SDL_APP_SUCCESS;

        case SDL_SCANCODE_1:
          g_mode = DemoMode::EUCLIDEAN_ASTAR;
          compute_path();
          break;

        case SDL_SCANCODE_2:
          g_mode = DemoMode::WAYPOINT_ASTAR;
          compute_path();
          break;

        case SDL_SCANCODE_3:
          g_mode = DemoMode::HIERARCHICAL_ASTAR;
          compute_path();
          break;

        case SDL_SCANCODE_4:
          g_mode = DemoMode::MULTI_GOAL_DIJKSTRA;
          compute_path();
          break;

        case SDL_SCANCODE_5:
          g_mode = DemoMode::FLEE_MAP;
          g_monster_x = 30;
          g_monster_y = 20;
          compute_path();
          break;

        case SDL_SCANCODE_W:
          g_show_waypoints = !g_show_waypoints;
          break;

        case SDL_SCANCODE_H:
          g_show_minimap = !g_show_minimap;
          break;

        case SDL_SCANCODE_R:
          if (g_mode == DemoMode::FLEE_MAP) {
            g_monster_x = 30;
            g_monster_y = 20;
          }
          compute_path();
          break;

        case SDL_SCANCODE_RETURN:
        case SDL_SCANCODE_RETURN2:
        case SDL_SCANCODE_KP_ENTER:
          if (event->key.mod & SDL_KMOD_ALT) {
            if (auto window = g_context.get_sdl_window(); window) {
              SDL_SetWindowFullscreen(window, (SDL_GetWindowFlags(window) & SDL_WINDOW_FULLSCREEN) == 0);
            }
          }
          break;

        default:
          break;
      }
      break;

    default:
      break;
  }

  return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppInit(void**, int argc, char** argv) {
  // Load tileset
  auto tileset = tcod::load_tilesheet("data/fonts/terminal8x8_gs_tc.png", {32, 8}, tcod::CHARMAP_TCOD);

  // Create context
  TCOD_ContextParams params{};
  params.argc = argc;
  params.argv = argv;
  params.console = g_console.get();
  params.tileset = tileset.get();
  params.window_title = "Pathfinding Heuristic Demo";
  params.sdl_window_flags = SDL_WINDOW_RESIZABLE;
  params.vsync = true;

  g_context = tcod::Context(params);

  // Initialize pathfinding
  init_pathfinding();

  // Compute initial path
  compute_path();

  return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void*, SDL_AppResult) { cleanup_pathfinding(); }
