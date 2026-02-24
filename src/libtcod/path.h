/* BSD 3-Clause License
 *
 * Copyright © 2008-2026, Jice and the libtcod contributors.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/// @file path.h
/// Libtcod A* and Dijkstra pathfinders.
#pragma once
#ifndef TCOD_PATH_H_
#define TCOD_PATH_H_

#include <stdint.h>

#include "fov_types.h"
#include "list.h"
#include "portability.h"
/// @defgroup Pathfinding Pathfinding (C)
/// @{
#ifdef __cplusplus
extern "C" {
#endif
typedef float (*TCOD_path_func_t)(int xFrom, int yFrom, int xTo, int yTo, void* user_data);

/**
 *  Custom A* heuristic function type.
 *
 *  @param x Current cell x coordinate.
 *  @param y Current cell y coordinate.
 *  @param goal_x Goal cell x coordinate.
 *  @param goal_y Goal cell y coordinate.
 *  @param user_data User-provided data pointer.
 *  @return Estimated cost from (x,y) to (goal_x, goal_y). Must be admissible (never overestimate).
 *  @versionadded{Unreleased}
 */
typedef float (*TCOD_heuristic_func_t)(int x, int y, int goal_x, int goal_y, void* user_data);

struct TCOD_Path;
typedef struct TCOD_Path* TCOD_path_t;

TCODLIB_API TCOD_path_t TCOD_path_new_using_map(TCOD_Map* map, float diagonalCost);
TCODLIB_API TCOD_path_t
TCOD_path_new_using_function(int map_width, int map_height, TCOD_path_func_t func, void* user_data, float diagonalCost);

/**
 *  Create a pathfinder with custom walk cost and heuristic functions.
 *
 *  @param map_width Width of the map.
 *  @param map_height Height of the map.
 *  @param func Walk cost function (required).
 *  @param heuristic_func Custom A* heuristic function (NULL = Euclidean distance).
 *  @param user_data User data passed to both functions.
 *  @param diagonalCost Cost multiplier for diagonal moves (0 = disallow diagonals).
 *  @param heuristic_weight Weight for heuristic (1.0 = optimal A*, >1.0 = weighted A*).
 *  @return New pathfinder or NULL on error.
 *  @versionadded{Unreleased}
 */
TCODLIB_API TCOD_path_t TCOD_path_new_using_function_ex(
    int map_width,
    int map_height,
    TCOD_path_func_t func,
    TCOD_heuristic_func_t heuristic_func,
    void* user_data,
    float diagonalCost,
    float heuristic_weight);

/**
 *  Set or change the heuristic function for an existing pathfinder.
 *
 *  @param path The pathfinder to modify.
 *  @param heuristic_func Custom heuristic function (NULL = Euclidean distance).
 *  @param heuristic_weight Weight for heuristic (1.0 = optimal A*, >1.0 = weighted A*).
 *  @versionadded{Unreleased}
 */
TCODLIB_API void TCOD_path_set_heuristic(
    TCOD_path_t path, TCOD_heuristic_func_t heuristic_func, float heuristic_weight);

/* Built-in heuristic functions */
/** Euclidean distance heuristic (default). @versionadded{Unreleased} */
TCODLIB_API float TCOD_heuristic_euclidean(int x, int y, int goal_x, int goal_y, void* user_data);
/** Manhattan distance heuristic (only admissible when diagonal moves are disabled). @versionadded{Unreleased} */
TCODLIB_API float TCOD_heuristic_manhattan(int x, int y, int goal_x, int goal_y, void* user_data);
/** Chebyshev distance heuristic (max of dx, dy). @versionadded{Unreleased} */
TCODLIB_API float TCOD_heuristic_chebyshev(int x, int y, int goal_x, int goal_y, void* user_data);
/** Diagonal distance heuristic (optimal for uniform cost with diagonal moves). @versionadded{Unreleased} */
TCODLIB_API float TCOD_heuristic_diagonal(int x, int y, int goal_x, int goal_y, void* user_data);
/** Zero heuristic (converts A* to Dijkstra). @versionadded{Unreleased} */
TCODLIB_API float TCOD_heuristic_zero(int x, int y, int goal_x, int goal_y, void* user_data);

TCODLIB_API bool TCOD_path_compute(TCOD_path_t path, int ox, int oy, int dx, int dy);
TCODLIB_API bool TCOD_path_walk(TCOD_path_t path, int* x, int* y, bool recalculate_when_needed);
TCODLIB_API bool TCOD_path_is_empty(TCOD_path_t path);
TCODLIB_API int TCOD_path_size(TCOD_path_t path);
TCODLIB_API void TCOD_path_reverse(TCOD_path_t path);
TCODLIB_API void TCOD_path_get(TCOD_path_t path, int index, int* x, int* y);
TCODLIB_API void TCOD_path_get_origin(TCOD_path_t path, int* x, int* y);
TCODLIB_API void TCOD_path_get_destination(TCOD_path_t path, int* x, int* y);
TCODLIB_API void TCOD_path_delete(TCOD_path_t path);

/* Dijkstra stuff - by Mingos*/
/**
 *  Dijkstra data structure
 *
 *  All attributes are considered private.
 */
typedef struct TCOD_Dijkstra {
  int diagonal_cost;
  int width, height, nodes_max;
  TCOD_Map* map; /* a TCODMap with walkability data */
  TCOD_path_func_t func;
  void* user_data;
  unsigned int* distances; /* distances grid */
  unsigned int* nodes; /* the processed nodes */
  TCOD_list_t path;
} TCOD_Dijkstra;
typedef struct TCOD_Dijkstra* TCOD_dijkstra_t;

TCODLIB_API TCOD_Dijkstra* TCOD_dijkstra_new(TCOD_Map* map, float diagonalCost);
TCODLIB_API TCOD_Dijkstra* TCOD_dijkstra_new_using_function(
    int map_width, int map_height, TCOD_path_func_t func, void* user_data, float diagonalCost);
TCODLIB_API void TCOD_dijkstra_compute(TCOD_Dijkstra* dijkstra, int root_x, int root_y);

/**
 *  Compute Dijkstra distances from multiple root/goal positions.
 *
 *  @param dijkstra The Dijkstra pathfinder.
 *  @param n_roots Number of root positions.
 *  @param roots_x Array of x coordinates for root positions.
 *  @param roots_y Array of y coordinates for root positions.
 *  @versionadded{Unreleased}
 */
TCODLIB_API void TCOD_dijkstra_compute_multi(
    TCOD_Dijkstra* dijkstra, int n_roots, const int* roots_x, const int* roots_y);

/**
 *  Compute Dijkstra distances using a mask of goal positions.
 *
 *  @param dijkstra The Dijkstra pathfinder.
 *  @param mask Array of width*height uint8_t values (non-zero = goal).
 *  @versionadded{Unreleased}
 */
TCODLIB_API void TCOD_dijkstra_compute_masked(TCOD_Dijkstra* dijkstra, const uint8_t* mask);

/**
 *  Invert the Dijkstra distance map for flee/safety calculations.
 *
 *  Transforms distances so that cells near goals become high values and
 *  cells far from goals become low values. Useful for AI flee behavior.
 *
 *  @param dijkstra The Dijkstra pathfinder (must have compute called first).
 *  @versionadded{Unreleased}
 */
TCODLIB_API void TCOD_dijkstra_invert(TCOD_Dijkstra* dijkstra);

/**
 *  Get the adjacent cell with the lowest distance (gradient descent).
 *
 *  @param dijkstra The Dijkstra pathfinder (must have compute called first).
 *  @param x Current x position.
 *  @param y Current y position.
 *  @param out_x Output: x coordinate of best neighbor.
 *  @param out_y Output: y coordinate of best neighbor.
 *  @return true if a valid neighbor was found, false if stuck or at goal.
 *  @versionadded{Unreleased}
 */
TCODLIB_API bool TCOD_dijkstra_get_descent(TCOD_Dijkstra* dijkstra, int x, int y, int* out_x, int* out_y);
TCODLIB_API float TCOD_dijkstra_get_distance(TCOD_Dijkstra* dijkstra, int x, int y);
TCODLIB_API bool TCOD_dijkstra_path_set(TCOD_Dijkstra* dijkstra, int x, int y);
TCODLIB_API bool TCOD_dijkstra_is_empty(TCOD_Dijkstra* path);
TCODLIB_API int TCOD_dijkstra_size(TCOD_Dijkstra* path);
TCODLIB_API void TCOD_dijkstra_reverse(TCOD_Dijkstra* path);
TCODLIB_API void TCOD_dijkstra_get(TCOD_Dijkstra* path, int index, int* x, int* y);
TCODLIB_API bool TCOD_dijkstra_path_walk(TCOD_Dijkstra* dijkstra, int* x, int* y);
TCODLIB_API void TCOD_dijkstra_delete(TCOD_Dijkstra* dijkstra);
#ifdef __cplusplus
}
#endif
/// @}
#endif  // TCOD_PATH_H_
