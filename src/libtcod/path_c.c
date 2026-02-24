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
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "libtcod_int.h"
#include "path.h"
enum { NORTH_WEST, NORTH, NORTH_EAST, WEST, NONE, EAST, SOUTH_WEST, SOUTH, SOUTH_EAST };
typedef unsigned char dir_t;

/* convert dir_t to dx,dy */
static const int dir_x[] = {-1, 0, 1, -1, 0, 1, -1, 0, 1};
static const int dir_y[] = {-1, -1, -1, 0, 0, 0, 1, 1, 1};
static const int invert_dir[] = {SOUTH_EAST, SOUTH, SOUTH_WEST, EAST, NONE, WEST, NORTH_EAST, NORTH, NORTH_WEST};

typedef struct TCOD_Path {
  int ox, oy; /* coordinates of the creature position */
  int dx, dy; /* coordinates of the creature's destination */
  TCOD_list_t path; /* list of dir_t to follow the path */
  int w, h; /* map size */
  float* grid; /* wxh dijkstra distance grid (covered distance) */
  float* heuristic; /* wxh A* score grid (covered distance + estimated remaining distance) */
  dir_t* prev; /* wxh 'previous' grid : direction to the previous cell */
  float diagonalCost;
  TCOD_list_t heap; /* min_heap used in the algorithm. stores the offset in grid/heuristic (offset=x+y*w) */
  TCOD_Map* map;
  TCOD_path_func_t func;
  void* user_data;
  TCOD_heuristic_func_t heuristic_func; /* custom A* heuristic (NULL = Euclidean) */
  void* heuristic_user_data; /* context passed to heuristic_func */
  float heuristic_weight; /* heuristic weight (1.0 = optimal A*) */
} TCOD_Path;

/* small layer on top of TCOD_list_t to implement a binary heap (min_heap) */
static void heap_sift_down(TCOD_Path* path, TCOD_list_t heap) {
  /* sift-down : move the first element of the heap down to its right place */
  const int end = TCOD_list_size(heap) - 1;
  int cur = 0;
  int child = 1;
  uintptr_t* array = (uintptr_t*)TCOD_list_begin(heap);
  while (child <= end) {
    const uintptr_t off_cur = array[cur];
    const float cur_dist = path->heuristic[off_cur];
    const uintptr_t off_child = array[child];
    const float child_dist = path->heuristic[off_child];
    int toSwap = cur;
    float swapValue = cur_dist;
    if (child_dist < cur_dist) {
      toSwap = child;
      swapValue = child_dist;
    }
    if (child < end) {
      /* get the min between child and child+1 */
      const uintptr_t off_child2 = array[child + 1];
      const float child2_dist = path->heuristic[off_child2];
      if (swapValue > child2_dist) {
        toSwap = child + 1;
        swapValue = child2_dist;
      }
    }
    if (toSwap != cur) {
      /* get down one level */
      const uintptr_t tmp = array[toSwap];
      array[toSwap] = array[cur];
      array[cur] = tmp;
      cur = toSwap;
    } else
      return;
    child = cur * 2 + 1;
  }
}

static void heap_sift_up(TCOD_Path* path, TCOD_list_t heap) {
  /* sift-up : move the last element of the heap up to its right place */
  const int end = TCOD_list_size(heap) - 1;
  int child = end;
  uintptr_t* array = (uintptr_t*)TCOD_list_begin(heap);
  while (child > 0) {
    const uintptr_t off_child = array[child];
    const float child_dist = path->heuristic[off_child];
    const int parent = (child - 1) / 2;
    const uintptr_t off_parent = array[parent];
    const float parent_dist = path->heuristic[off_parent];
    if (parent_dist > child_dist) {
      /* get up one level */
      uintptr_t tmp = array[child];
      array[child] = array[parent];
      array[parent] = tmp;
      child = parent;
    } else
      return;
  }
}

/* add a coordinate pair in the heap so that the heap root always contains the minimum A* score */
static void heap_add(TCOD_Path* path, TCOD_list_t heap, int x, int y) {
  /* append the new value to the end of the heap */
  const uintptr_t off = x + y * path->w;
  TCOD_list_push(heap, (void*)off);
  /* bubble the value up to its real position */
  heap_sift_up(path, heap);
}

/* get the coordinate pair with the minimum A* score from the heap */
static uint32_t heap_get(TCOD_Path* path, TCOD_list_t heap) {
  /* return the first value of the heap (minimum score) */
  uintptr_t* array = (uintptr_t*)TCOD_list_begin(heap);
  const int end = TCOD_list_size(heap) - 1;
  const uint32_t off = (uint32_t)(array[0]);
  /* take the last element and put it at first position (heap root) */
  array[0] = array[end];
  TCOD_list_pop(heap);
  /* and bubble it down to its real position */
  heap_sift_down(path, heap);
  return off;
}

/* this is the slow part, when we change the heuristic of a cell already in the heap */
static void heap_reorder(TCOD_Path* path, uint32_t offset) {
  uintptr_t* array = (uintptr_t*)TCOD_list_begin(path->heap);
  const uintptr_t* const end = (uintptr_t*)TCOD_list_end(path->heap);
  uintptr_t* cur = array;
  const int heap_size = TCOD_list_size(path->heap);
  /* find the node corresponding to offset ... SLOW !! */
  int idx = 0;
  while (cur != end) {
    if (*cur == offset) break;
    cur++;
    idx++;
  }
  if (cur == end) return;
  const uintptr_t off_idx = array[idx];
  const float value = path->heuristic[off_idx];
  if (idx > 0) {
    int parent = (idx - 1) / 2;
    /* compare to its parent */
    uintptr_t off_parent = array[parent];
    float parent_value = path->heuristic[off_parent];
    if (value < parent_value) {
      /* smaller. bubble it up */
      while (idx > 0 && value < parent_value) {
        /* swap with parent */
        array[parent] = off_idx;
        array[idx] = off_parent;
        idx = parent;
        if (idx > 0) {
          parent = (idx - 1) / 2;
          off_parent = array[parent];
          parent_value = path->heuristic[off_parent];
        }
      }
      return;
    }
  }
  /* compare to its sons */
  while (idx * 2 + 1 < heap_size) {
    const int child = idx * 2 + 1;
    const uintptr_t off_child = array[child];
    int toSwap = idx;
    float swapValue = value;
    if (path->heuristic[off_child] < value) {
      /* swap with son1 ? */
      toSwap = child;
      swapValue = path->heuristic[off_child];
    }
    const int child2 = child + 1;
    if (child2 < heap_size) {
      const uintptr_t off_child2 = array[child2];
      if (path->heuristic[off_child2] < swapValue) {
        /* swap with son2 */
        toSwap = child2;
      }
    }
    if (toSwap != idx) {
      /* bigger. bubble it down */
      const uintptr_t tmp = array[toSwap];
      array[toSwap] = array[idx];
      array[idx] = tmp;
      idx = toSwap;
    } else
      return;
  }
}

/* private functions */
static void TCOD_path_push_cell(TCOD_Path* path, int x, int y);
static void TCOD_path_get_cell(TCOD_Path* path, int* x, int* y, float* distance);
static void TCOD_path_set_cells(TCOD_Path* path);
static float TCOD_path_walk_cost(TCOD_Path* path, int xFrom, int yFrom, int xTo, int yTo);

static TCOD_Path* TCOD_path_new_intern(int w, int h) {
  TCOD_Path* path = calloc(1, sizeof(*path));
  path->w = w;
  path->h = h;
  path->grid = calloc(w * h, sizeof(*path->grid));
  path->heuristic = calloc(w * h, sizeof(*path->heuristic));
  path->prev = calloc(w * h, sizeof(*path->prev));
  if (!path->grid || !path->heuristic || !path->prev) {
    free(path->grid);
    free(path->heuristic);
    free(path->prev);
    free(path);
    TCOD_set_errorvf("Cannot allocate dijkstra grids of size {%d, %d}", w, h);
    return NULL;
  }
  path->path = TCOD_list_new();
  path->heap = TCOD_list_new();
  path->heuristic_func = NULL; /* default: Euclidean distance */
  path->heuristic_user_data = NULL;
  path->heuristic_weight = 1.0f; /* default: optimal A* */
  return path;
}

TCOD_Path* TCOD_path_new_using_map(TCOD_Map* map, float diagonalCost) {
  TCOD_IFNOT(map != NULL) return NULL;
  TCOD_Path* path = TCOD_path_new_intern(TCOD_map_get_width(map), TCOD_map_get_height(map));
  if (!path) {
    return NULL;
  }
  path->map = map;
  path->diagonalCost = diagonalCost;
  return path;
}

TCOD_Path* TCOD_path_new_using_function(
    int map_width, int map_height, TCOD_path_func_t func, void* user_data, float diagonalCost) {
  TCOD_IFNOT(func != NULL && map_width > 0 && map_height > 0) return NULL;
  TCOD_Path* path = TCOD_path_new_intern(map_width, map_height);
  if (!path) {
    return NULL;
  }
  path->func = func;
  path->user_data = user_data;
  path->diagonalCost = diagonalCost;
  return path;
}

TCOD_Path* TCOD_path_new_using_function_ex(
    int map_width,
    int map_height,
    TCOD_path_func_t func,
    TCOD_heuristic_func_t heuristic_func,
    void* user_data,
    float diagonalCost,
    float heuristic_weight) {
  TCOD_IFNOT(func != NULL && map_width > 0 && map_height > 0) return NULL;
  TCOD_Path* path = TCOD_path_new_intern(map_width, map_height);
  if (!path) {
    return NULL;
  }
  path->func = func;
  path->user_data = user_data;
  path->diagonalCost = diagonalCost;
  path->heuristic_func = heuristic_func;
  path->heuristic_user_data = user_data; /* heuristic shares the cost function's context by default */
  path->heuristic_weight = heuristic_weight > 0.0f ? heuristic_weight : 1.0f;
  return path;
}

void TCOD_path_set_heuristic(
    TCOD_path_t path, TCOD_heuristic_func_t heuristic_func, void* heuristic_user_data, float heuristic_weight) {
  TCOD_IFNOT(path != NULL) return;
  path->heuristic_func = heuristic_func;
  path->heuristic_user_data = heuristic_user_data;
  path->heuristic_weight = heuristic_weight > 0.0f ? heuristic_weight : 1.0f;
}

bool TCOD_path_compute(TCOD_Path* path, int ox, int oy, int dx, int dy) {
  TCOD_IFNOT(path != NULL) return false;
  path->ox = ox;
  path->oy = oy;
  path->dx = dx;
  path->dy = dy;
  TCOD_list_clear(path->path);
  TCOD_list_clear(path->heap);
  if (ox == dx && oy == dy) return true; /* trivial case */
  /* check that origin and destination are inside the map */
  TCOD_IFNOT((unsigned)ox < (unsigned)path->w && (unsigned)oy < (unsigned)path->h) return false;
  TCOD_IFNOT((unsigned)dx < (unsigned)path->w && (unsigned)dy < (unsigned)path->h) return false;
  /* initialize dijkstra grids */
  memset(path->grid, 0, sizeof(*path->grid) * path->w * path->h);
  memset(path->prev, NONE, sizeof(*path->prev) * path->w * path->h);
  path->heuristic[ox + oy * path->w] = 1.0f; /* anything != 0 */
  TCOD_path_push_cell(path, ox, oy); /* put the origin cell as a bootstrap */
  /* fill the dijkstra grid until we reach dx,dy */
  TCOD_path_set_cells(path);
  if (path->grid[dx + dy * path->w] == 0) return false; /* no path found */
  /* there is a path. retrieve it */
  do {
    /* walk from destination to origin, using the 'prev' array */
    const int step = path->prev[dx + dy * path->w];
    TCOD_list_push(path->path, (void*)(uintptr_t)step);
    dx -= dir_x[step];
    dy -= dir_y[step];
  } while (dx != ox || dy != oy);
  return true;
}

void TCOD_path_reverse(TCOD_Path* path) {
  TCOD_IFNOT(path != NULL) return;
  int tmp = path->ox;
  path->ox = path->dx;
  path->dx = tmp;
  tmp = path->oy;
  path->oy = path->dy;
  path->dy = tmp;
  for (int i = 0; i < TCOD_list_size(path->path); i++) {
    int d = (int)(uintptr_t)TCOD_list_get(path->path, i);
    d = invert_dir[d];
    TCOD_list_set(path->path, (void*)(uintptr_t)d, i);
  }
}

bool TCOD_path_walk(TCOD_Path* path, int* x, int* y, bool recalculate_when_needed) {
  TCOD_IFNOT(path != NULL) return false;
  if (TCOD_path_is_empty(path)) return false;
  const int d = (int)(uintptr_t)TCOD_list_pop(path->path);
  const int new_x = path->ox + dir_x[d];
  const int new_y = path->oy + dir_y[d];
  /* check if the path is still valid */
  if (TCOD_path_walk_cost(path, path->ox, path->oy, new_x, new_y) <= 0.0f) {
    /* path is blocked */
    if (!recalculate_when_needed) return false; /* don't walk */
    /* calculate a new path */
    if (!TCOD_path_compute(path, path->ox, path->oy, path->dx, path->dy)) return false; /* cannot find a new path */
    return TCOD_path_walk(path, x, y, true); /* walk along the new path */
  }
  if (x) *x = new_x;
  if (y) *y = new_y;
  path->ox = new_x;
  path->oy = new_y;
  return true;
}

bool TCOD_path_is_empty(TCOD_Path* path) {
  TCOD_IFNOT(path != NULL) return true;
  return TCOD_list_is_empty(path->path);
}

int TCOD_path_size(TCOD_Path* path) {
  TCOD_IFNOT(path != NULL) return 0;
  return TCOD_list_size(path->path);
}

void TCOD_path_get(TCOD_Path* path, int index, int* x, int* y) {
  TCOD_IFNOT(path != NULL) return;
  if (x) *x = path->ox;
  if (y) *y = path->oy;
  int pos = TCOD_list_size(path->path) - 1;
  do {
    const int step = (int)(uintptr_t)TCOD_list_get(path->path, pos);
    if (x) *x += dir_x[step];
    if (y) *y += dir_y[step];
    pos--;
    index--;
  } while (index >= 0);
}

void TCOD_path_delete(TCOD_Path* path) {
  TCOD_IFNOT(path != NULL) return;
  if (path->grid) free(path->grid);
  if (path->heuristic) free(path->heuristic);
  if (path->prev) free(path->prev);
  if (path->path) TCOD_list_delete(path->path);
  if (path->heap) TCOD_list_delete(path->heap);
  free(path);
}

/* private stuff */
/* add a new unvisited cells to the cells-to-treat list
 * the list is in fact a min_heap. Cell at index i has its sons at 2*i+1 and 2*i+2
 */
static void TCOD_path_push_cell(TCOD_Path* path, int x, int y) { heap_add(path, path->heap, x, y); }

/* get the best cell from the heap */
static void TCOD_path_get_cell(TCOD_Path* path, int* x, int* y, float* distance) {
  const uint32_t offset = heap_get(path, path->heap);
  *x = (offset % path->w);
  *y = (offset / path->w);
  *distance = path->grid[offset];
}
/* fill the grid, starting from the origin until we reach the destination */
static void TCOD_path_set_cells(TCOD_Path* path) {
  while (path->grid[path->dx + path->dy * path->w] == 0 && !TCOD_list_is_empty(path->heap)) {
    int x, y;
    float distance;
    TCOD_path_get_cell(path, &x, &y, &distance);
    const int i_max = (path->diagonalCost == 0.0f ? 4 : 8);
    for (int i = 0; i < i_max; i++) {
      /* convert i to dx,dy */
      static const int i_dir_x[] = {0, -1, 1, 0, -1, 1, -1, 1};
      static const int i_dir_y[] = {-1, 0, 0, 1, -1, -1, 1, 1};
      /* convert i to direction */
      static const dir_t previous_dirs[] = {NORTH, WEST, EAST, SOUTH, NORTH_WEST, NORTH_EAST, SOUTH_WEST, SOUTH_EAST};
      /* coordinate of the adjacent cell */
      const int cx = x + i_dir_x[i];
      const int cy = y + i_dir_y[i];
      if (cx >= 0 && cy >= 0 && cx < path->w && cy < path->h) {
        const float walk_cost = TCOD_path_walk_cost(path, x, y, cx, cy);
        if (walk_cost > 0.0f) {
          /* in of the map and walkable */
          const float covered = distance + walk_cost * (i >= 4 ? path->diagonalCost : 1.0f);
          const float previousCovered = path->grid[cx + cy * path->w];
          if (previousCovered == 0) {
            /* put a new cell in the heap */
            const int offset = cx + cy * path->w;
            /* A* heuristic : remaining distance (custom heuristic if set, else Euclidean) */
            const float remaining =
                path->heuristic_func
                    ? path->heuristic_func(cx, cy, path->dx, path->dy, path->heuristic_user_data)
                    : (float)sqrt((cx - path->dx) * (cx - path->dx) + (cy - path->dy) * (cy - path->dy));
            path->grid[offset] = covered;
            path->heuristic[offset] = covered + path->heuristic_weight * remaining;
            path->prev[offset] = previous_dirs[i];
            TCOD_path_push_cell(path, cx, cy);
          } else if (previousCovered > covered) {
            /* we found a better path to a cell already in the heap */
            const int offset = cx + cy * path->w;
            path->grid[offset] = covered;
            path->heuristic[offset] -= (previousCovered - covered); /* fix the A* score (weight already applied) */
            path->prev[offset] = previous_dirs[i];
            /* reorder the heap */
            heap_reorder(path, offset);
          }
        }
      }
    }
  }
}

/* check if a cell is walkable (from the pathfinder point of view) */
static float TCOD_path_walk_cost(TCOD_Path* path, int xFrom, int yFrom, int xTo, int yTo) {
  if (path->map) return TCOD_map_is_walkable(path->map, xTo, yTo) ? 1.0f : 0.0f;
  return path->func(xFrom, yFrom, xTo, yTo, path->user_data);
}

void TCOD_path_get_origin(TCOD_Path* path, int* x, int* y) {
  TCOD_IFNOT(path != NULL) return;
  if (x) *x = path->ox;
  if (y) *y = path->oy;
}

void TCOD_path_get_destination(TCOD_Path* path, int* x, int* y) {
  TCOD_IFNOT(path != NULL) return;
  if (x) *x = path->dx;
  if (y) *y = path->dy;
}

/* ------------------------------------------------------- *
 * Built-in A* Heuristic Functions                         *
 * ------------------------------------------------------- */

float TCOD_heuristic_euclidean(int x, int y, int goal_x, int goal_y, void* user_data) {
  (void)user_data; /* unused */
  const int dx = x - goal_x;
  const int dy = y - goal_y;
  return (float)sqrt(dx * dx + dy * dy);
}

float TCOD_heuristic_manhattan(int x, int y, int goal_x, int goal_y, void* user_data) {
  (void)user_data; /* unused */
  const int dx = x > goal_x ? x - goal_x : goal_x - x;
  const int dy = y > goal_y ? y - goal_y : goal_y - y;
  return (float)(dx + dy);
}

float TCOD_heuristic_chebyshev(int x, int y, int goal_x, int goal_y, void* user_data) {
  (void)user_data; /* unused */
  const int dx = x > goal_x ? x - goal_x : goal_x - x;
  const int dy = y > goal_y ? y - goal_y : goal_y - y;
  return (float)(dx > dy ? dx : dy);
}

float TCOD_heuristic_diagonal(int x, int y, int goal_x, int goal_y, void* user_data) {
  (void)user_data; /* unused */
  const int dx = x > goal_x ? x - goal_x : goal_x - x;
  const int dy = y > goal_y ? y - goal_y : goal_y - y;
  /* octile distance: max(dx, dy) + (sqrt(2) - 1) * min(dx, dy) */
  const int min_d = dx < dy ? dx : dy;
  const int max_d = dx > dy ? dx : dy;
  return (float)max_d + (1.41421356f - 1.0f) * (float)min_d;
}

float TCOD_heuristic_zero(int x, int y, int goal_x, int goal_y, void* user_data) {
  (void)x;
  (void)y;
  (void)goal_x;
  (void)goal_y;
  (void)user_data;
  return 0.0f;
}

/* ------------------------------------------------------- *
 * Dijkstra                                                *
 * written by Mingos                                       *
 * -----------------                                       *
 * A floodfill-like algo that will calculate all distances *
 * to all accessible cells (nodes) from a given root node. *
 * ------------------------------------------------------- */

/* create a Dijkstra object */
TCOD_Dijkstra* TCOD_dijkstra_new(TCOD_map_t map, float diagonalCost) {
  TCOD_IFNOT(map != NULL) return NULL;
  TCOD_Dijkstra* data = malloc(sizeof(*data));
  data->map = map;
  data->func = NULL;
  data->user_data = NULL;
  data->distances = malloc(TCOD_map_get_nb_cells(data->map) * sizeof(*data->distances));
  data->nodes = malloc(TCOD_map_get_nb_cells(data->map) * sizeof(*data->nodes));
  data->diagonal_cost = (int)((diagonalCost * 100.0f) + 0.1f); /* because (int)(1.41f*100.0f) == 140!!! */
  data->width = TCOD_map_get_width(data->map);
  data->height = TCOD_map_get_height(data->map);
  data->nodes_max = TCOD_map_get_nb_cells(data->map);
  data->path = TCOD_list_new();
  return data;
}

TCOD_Dijkstra* TCOD_dijkstra_new_using_function(
    int map_width, int map_height, TCOD_path_func_t func, void* user_data, float diagonalCost) {
  TCOD_IFNOT(func != NULL && map_width > 0 && map_height > 0) return NULL;
  TCOD_Dijkstra* data = malloc(sizeof(*data));
  data->map = NULL;
  data->func = func;
  data->user_data = user_data;
  data->distances = malloc(map_width * map_height * sizeof(*data->distances) * 4);
  data->nodes = malloc(map_width * map_height * sizeof(*data->nodes) * 4);
  data->diagonal_cost = (int)((diagonalCost * 100.0f) + 0.1f); /* because (int)(1.41f*100.0f) == 140!!! */
  data->width = map_width;
  data->height = map_height;
  data->nodes_max = map_width * map_height;
  data->path = TCOD_list_new();
  return data;
}

/* Internal Dijkstra loop: drain the queue from pre-initialized distances[]/nodes[]
 * arrays. The caller must mark every root in distances[] (distance 0) and place it
 * in nodes[], passing last_index pointing past the last initialized root node.
 * Shared by the single-root and multi-root entry points so the BFS is written once. */
static void dijkstra_compute_internal(TCOD_Dijkstra* data, unsigned int last_index) {
  /* map size data */
  const unsigned int mx = data->width;
  const unsigned int my = data->height;
  const unsigned int m_max = data->nodes_max;
  unsigned int* nodes = data->nodes; /* table of nodes to which the indices above apply */
  unsigned int* distances = data->distances;
  /* ok, here's the order of node processing: W, S, E, N, NW, NE, SE, SW */
  static const int dx[8] = {-1, 0, 1, 0, -1, 1, 1, -1};
  static const int dy[8] = {0, -1, 0, 1, -1, -1, 1, 1};
  /* and distances for each index */
  const int dd[8] = {
      100, 100, 100, 100, data->diagonal_cost, data->diagonal_cost, data->diagonal_cost, data->diagonal_cost};
  /* if diagonal_cost is 0, disallow diagonal moves */
  const int i_max = (data->diagonal_cost == 0 ? 4 : 8);
  unsigned int index = 0; /* the index of the first node in queue */
  /* and the loop */
  do {
    if (nodes[index] == 0xFFFFFFFF) {
      continue;
    }

    /* coordinates of currently processed node */
    const unsigned int x = nodes[index] % mx;
    const unsigned int y = nodes[index] / mx;

    /* check adjacent nodes */
    for (int i = 0; i < i_max; i++) {
      /* checked node's coordinates */
      const unsigned int tx = x + dx[i];
      const unsigned int ty = y + dy[i];
      if ((unsigned)tx < (unsigned)mx && (unsigned)ty < (unsigned)my) {
        /* otherwise, calculate distance, ... */
        unsigned int dt = distances[nodes[index]], new_node;
        float userDist = 0.0f;
        if (data->map)
          dt += dd[i];
        else {
          /* distance given by the user callback */
          userDist = data->func(x, y, tx, ty, data->user_data);
          dt += (unsigned int)(userDist * dd[i]);
        }
        /* ..., encode coordinates, ... */
        new_node = (ty * mx) + tx;
        /* and check if the node's eligible for queuing */
        if (distances[new_node] > dt) {
          /* if not walkable, don't process it */
          if (data->map && !TCOD_map_is_walkable(data->map, tx, ty))
            continue;
          else if (data->func && userDist <= 0.0f)
            continue;
          distances[new_node] = dt; /* set processed node's distance */
          /* place the processed node in the queue before the last queued node with greater distance */
          unsigned int j = last_index - 1;
          while (distances[nodes[j]] >= distances[new_node]) {
            /* this ensures that if the node has been queued previously, but with a higher distance, it's removed */
            if (nodes[j] == new_node) {
              int k = j;
              while ((unsigned)k <= last_index) {
                nodes[k] = nodes[k + 1];
                k++;
              }
              last_index--;
            } else
              nodes[j + 1] = nodes[j];
            j--;
          }
          last_index++; /* increase total indices count */
          nodes[j + 1] = new_node; /* and finally put the node where it belongs in the queue */
        }
      }
    }
  } while (m_max > ++index);
}

/* compute a Dijkstra grid from a single root */
void TCOD_dijkstra_compute(TCOD_Dijkstra* data, int root_x, int root_y) {
  TCOD_IFNOT(data != NULL) return;
  const unsigned int mx = data->width;
  const unsigned int m_max = data->nodes_max;
  TCOD_IFNOT((unsigned)root_x < (unsigned)mx && (unsigned)root_y < (unsigned)data->height) return;
  const unsigned int root = (root_y * mx) + root_x;
  memset(data->distances, 0xFFFFFFFF, m_max * sizeof(*data->distances));
  memset(data->nodes, 0xFFFFFFFF, m_max * sizeof(*data->nodes));
  data->distances[root] = 0;
  data->nodes[0] = root;
  dijkstra_compute_internal(data, 1);
}

/* compute a Dijkstra grid from multiple root positions simultaneously */
void TCOD_dijkstra_compute_multi(TCOD_Dijkstra* data, int n_roots, const int* roots_x, const int* roots_y) {
  TCOD_IFNOT(data != NULL && n_roots > 0 && roots_x != NULL && roots_y != NULL) return;
  const unsigned int mx = data->width;
  const unsigned int my = data->height;
  const unsigned int m_max = data->nodes_max;
  memset(data->distances, 0xFFFFFFFF, m_max * sizeof(*data->distances));
  memset(data->nodes, 0xFFFFFFFF, m_max * sizeof(*data->nodes));
  /* seed every in-bounds root with distance 0, skipping duplicates */
  unsigned int last_index = 0;
  for (int r = 0; r < n_roots; r++) {
    if ((unsigned)roots_x[r] < mx && (unsigned)roots_y[r] < my) {
      const unsigned int root = (roots_y[r] * mx) + roots_x[r];
      if (data->distances[root] == 0xFFFFFFFF) { /* not already seeded */
        data->distances[root] = 0;
        data->nodes[last_index++] = root;
      }
    }
  }
  dijkstra_compute_internal(data, last_index);
}

/* compute a Dijkstra grid using a bitmap mask of goal positions */
void TCOD_dijkstra_compute_masked(TCOD_Dijkstra* data, const uint8_t* mask) {
  TCOD_IFNOT(data != NULL && mask != NULL) return;
  const int n_cells = data->width * data->height;
  /* first pass: count non-zero mask entries to size the root arrays exactly */
  int n_roots = 0;
  for (int i = 0; i < n_cells; i++) {
    if (mask[i] != 0) n_roots++;
  }
  if (n_roots == 0) return;
  int* const roots_x = malloc(n_roots * sizeof(*roots_x));
  int* const roots_y = malloc(n_roots * sizeof(*roots_y));
  if (!roots_x || !roots_y) {
    free(roots_x);
    free(roots_y);
    return;
  }
  /* second pass: collect coordinates */
  int n = 0;
  for (int y = 0; y < data->height; y++) {
    for (int x = 0; x < data->width; x++) {
      if (mask[y * data->width + x] != 0) {
        roots_x[n] = x;
        roots_y[n] = y;
        n++;
      }
    }
  }
  TCOD_dijkstra_compute_multi(data, n_roots, roots_x, roots_y);
  free(roots_x);
  free(roots_y);
}

/* invert the distance map for flee/safety calculations */
void TCOD_dijkstra_invert(TCOD_Dijkstra* data) {
  TCOD_IFNOT(data != NULL) return;
  const unsigned int m_max = data->nodes_max;
  /* find the maximum reachable (non-infinity) distance */
  unsigned int max_dist = 0;
  for (unsigned int i = 0; i < m_max; i++) {
    if (data->distances[i] != 0xFFFFFFFF && data->distances[i] > max_dist) {
      max_dist = data->distances[i];
    }
  }
  /* invert reachable cells (new = max - old); unreachable cells stay at infinity */
  for (unsigned int i = 0; i < m_max; i++) {
    if (data->distances[i] != 0xFFFFFFFF) {
      data->distances[i] = max_dist - data->distances[i];
    }
  }
}

/* get the adjacent cell with the lowest distance (gradient descent) */
bool TCOD_dijkstra_get_descent(TCOD_Dijkstra* data, int x, int y, int* out_x, int* out_y) {
  TCOD_IFNOT(data != NULL && out_x != NULL && out_y != NULL) return false;
  TCOD_IFNOT((unsigned)x < (unsigned)data->width && (unsigned)y < (unsigned)data->height) return false;
  static const int dx[8] = {-1, 0, 1, 0, -1, 1, 1, -1};
  static const int dy[8] = {0, -1, 0, 1, -1, -1, 1, 1};
  const unsigned int current_dist = data->distances[y * data->width + x];
  if (current_dist == 0xFFFFFFFF) return false; /* unreachable cell */
  const int i_max = (data->diagonal_cost == 0 ? 4 : 8);
  unsigned int lowest_dist = current_dist;
  int lowest_index = -1;
  for (int i = 0; i < i_max; i++) {
    const int nx = x + dx[i];
    const int ny = y + dy[i];
    if ((unsigned)nx < (unsigned)data->width && (unsigned)ny < (unsigned)data->height) {
      const unsigned int dist = data->distances[ny * data->width + nx];
      if (dist < lowest_dist) {
        lowest_dist = dist;
        lowest_index = i;
      }
    }
  }
  if (lowest_index < 0) {
    /* already at a goal or no lower neighbor */
    *out_x = x;
    *out_y = y;
    return false;
  }
  *out_x = x + dx[lowest_index];
  *out_y = y + dy[lowest_index];
  return true;
}

/* get distance from source */
float TCOD_dijkstra_get_distance(TCOD_Dijkstra* data, int x, int y) {
  TCOD_IFNOT(data != NULL) return -1.0f;
  TCOD_IFNOT((unsigned)x < (unsigned)data->width && (unsigned)y < (unsigned)data->height) return -1.0f;
  if (data->distances[(y * data->width) + x] == 0xFFFFFFFF) return -1.0f;
  return ((float)data->distances[(y * data->width) + x] * 0.01f);
}

unsigned int dijkstra_get_int_distance(TCOD_Dijkstra* data, int x, int y) {
  return data->distances[(y * data->width) + x];
}

/* create a path */
bool TCOD_dijkstra_path_set(TCOD_Dijkstra* data, int x, int y) {
  static const int dx[9] = {-1, 0, 1, 0, -1, 1, 1, -1, 0};
  static const int dy[9] = {0, -1, 0, 1, -1, -1, 1, 1, 0};
  const int i_max = (data->diagonal_cost == 0 ? 4 : 8);
  TCOD_IFNOT(data != NULL) return false;
  TCOD_IFNOT((unsigned)x < (unsigned)data->width && (unsigned)y < (unsigned)data->height) return false;
  /* check that destination is reachable */
  if (dijkstra_get_int_distance(data, x, y) == 0xFFFFFFFF) return false;
  TCOD_list_clear(data->path);
  int px = x;
  int py = y;
  unsigned int distances[8] = {0};
  int lowest_index;
  do {
    TCOD_list_push(data->path, (const void*)(uintptr_t)((py * data->width) + px));
    for (int i = 0; i < i_max; i++) {
      const int cx = px + dx[i];
      const int cy = py + dy[i];
      if ((unsigned)cx < (unsigned)data->width && (unsigned)cy < (unsigned)data->height)
        distances[i] = dijkstra_get_int_distance(data, cx, cy);
      else
        distances[i] = 0xFFFFFFFF;
    }
    unsigned int lowest = dijkstra_get_int_distance(data, px, py);
    lowest_index = 8;
    for (int i = 0; i < i_max; i++)
      if (distances[i] < lowest) {
        lowest = distances[i];
        lowest_index = i;
      }
    px += dx[lowest_index];
    py += dy[lowest_index];
  } while (lowest_index != 8);
  /* remove the last step */
  TCOD_list_pop(data->path);
  return true;
}

void TCOD_dijkstra_reverse(TCOD_Dijkstra* data) {
  TCOD_IFNOT(data != NULL) return;
  TCOD_list_reverse(data->path);
}

/* walk the path */
bool TCOD_dijkstra_path_walk(TCOD_Dijkstra* data, int* x, int* y) {
  TCOD_IFNOT(data != NULL) return false;
  if (TCOD_list_is_empty(data->path))
    return false;
  else {
    const unsigned int node = (unsigned int)(uintptr_t)TCOD_list_pop(data->path);
    if (x) *x = (int)(node % data->width);
    if (y) *y = (int)(node / data->width);
  }
  return true;
}

/* delete a Dijkstra object */
void TCOD_dijkstra_delete(TCOD_Dijkstra* data) {
  TCOD_IFNOT(data != NULL) return;
  if (data->distances) free(data->distances);
  if (data->nodes) free(data->nodes);
  if (data->path) TCOD_list_delete(data->path);
  free(data);
}

bool TCOD_dijkstra_is_empty(TCOD_Dijkstra* data) {
  TCOD_IFNOT(data != NULL) return true;
  return TCOD_list_is_empty(data->path);
}

int TCOD_dijkstra_size(TCOD_Dijkstra* data) {
  TCOD_IFNOT(data != NULL) return 0;
  return TCOD_list_size(data->path);
}

void TCOD_dijkstra_get(TCOD_Dijkstra* data, int index, int* x, int* y) {
  TCOD_IFNOT(data != NULL) return;
  const unsigned int node = (unsigned int)(uintptr_t)TCOD_list_get(data->path, TCOD_list_size(data->path) - index - 1);
  if (x) *x = (int)(node % data->width);
  if (y) *y = (int)(node / data->width);
}
