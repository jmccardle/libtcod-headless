/* pathfinding_heuristics_demo.c
 *
 * A terminal (ASCII) explainer for the custom-heuristic / weighted-A* additions.
 * No SDL: every scene prints a grid and some numbers to stdout.
 *
 * Scene 1 - What a heuristic is FOR: search efficiency. The same optimal path is
 *           found with ZERO (== Dijkstra), EUCLIDEAN, and MANHATTAN heuristics on a
 *           4-connected grid, but the number of cells the search touches collapses as
 *           the heuristic gets tighter. The instrumentation lives in a counting
 *           heuristic whose OWN user_data (a counter + an "explored" grid) is separate
 *           from the pathfinder's map - only possible with the new set_heuristic(...,
 *           heuristic_user_data, ...) signature.
 *
 * Scene 2 - Variable terrain cost ("avoid the bog, but cross it if it's worth it").
 *           The walk-cost callback returns a per-tile entry cost; A* trades distance
 *           for cheaper ground. We pair it with a custom heuristic via the new
 *           TCOD_path_new_using_function_ex().
 *
 * Scene 3 - A domain heuristic: a precomputed waypoint corridor (think Voronoi-region
 *           waypoints joined by straight segments). A custom, deliberately INADMISSIBLE
 *           heuristic = dist-to-goal + penalty * dist-from-corridor steers fine-grained
 *           A* to hug the preferred route instead of taking the geometric shortest path.
 *
 * Build from the repository root (against a libtcod build of this branch):
 *   clang -std=c11 -O2 examples/pathfinding_heuristics_demo.c -I src -L <build> -ltcod \
 *       -lm -Wl,-rpath,<build> -o pfdemo && ./pfdemo
 */
#include <libtcod/fov.h>
#include <libtcod/heightmap.h>
#include <libtcod/mersenne.h>
#include <libtcod/path.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ helpers */

enum { H_ZERO, H_EUCLID, H_MANHATTAN, H_OCTILE };

static float metric(int base, int x, int y, int gx, int gy) {
  const int dx = abs(x - gx);
  const int dy = abs(y - gy);
  switch (base) {
    case H_ZERO:
      return 0.0f;
    case H_MANHATTAN:
      return (float)(dx + dy);
    case H_OCTILE: {
      const int lo = dx < dy ? dx : dy;
      const int hi = dx > dy ? dx : dy;
      return (float)hi + (1.41421356f - 1.0f) * (float)lo;
    }
    case H_EUCLID:
    default:
      return sqrtf((float)(dx * dx + dy * dy));
  }
}

/* Heuristic context (its own user_data, distinct from the pathfinder's map). */
typedef struct {
  int base; /* which metric to report */
  int w, h;
  int* explored; /* optional w*h flag grid, marked per heuristic evaluation */
  long calls; /* number of cells the search reached out to */
} HeurCtx;

static float counting_heuristic(int x, int y, int gx, int gy, void* user_data) {
  HeurCtx* c = (HeurCtx*)user_data;
  ++c->calls;
  if (c->explored && x >= 0 && y >= 0 && x < c->w && y < c->h) c->explored[y * c->w + x] = 1;
  return metric(c->base, x, y, gx, gy);
}

static void hline(int n) {
  for (int i = 0; i < n; ++i) putchar('-');
  putchar('\n');
}

/* ============================================================== SCENE 1 ==== */
/* A big open arena built in code: border walls plus one partial divider that
 * forces a detour. Open space is what lets a heuristic actually prune. */
static void scene1(void) {
  const int w = 44, h = 19;
  const int sx = 2, sy = 9, gx = 41, gy = 9;
  int* wall = calloc((size_t)w * h, sizeof(int));
  for (int x = 0; x < w; ++x) {
    wall[x] = 1;
    wall[(h - 1) * w + x] = 1;
  }
  for (int y = 0; y < h; ++y) {
    wall[y * w] = 1;
    wall[y * w + (w - 1)] = 1;
  }
  for (int y = 1; y <= 13; ++y) wall[y * w + 22] = 1; /* divider, gap along the bottom */

  TCOD_Map* map = TCOD_map_new(w, h);
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      const int wk = !wall[y * w + x];
      TCOD_map_set_properties(map, x, y, wk, wk);
    }

  printf("SCENE 1  -  a heuristic's job is to shrink the search\n");
  printf("4-connected open arena %dx%d, S -> G. Every row below finds an equally short\n", w, h);
  printf("path; what changes is how many cells A* opens. A tighter/greedier heuristic\n");
  printf("aims the search at the goal instead of flooding outward.\n\n");

  const struct {
    int base;
    float weight;
    const char* name;
  } runs[] = {
      {H_ZERO, 1.0f, "ZERO  (= Dijkstra)"},
      {H_EUCLID, 1.0f, "EUCLIDEAN"},
      {H_MANHATTAN, 1.0f, "MANHATTAN (tight on 4-conn)"},
      {H_MANHATTAN, 2.0f, "MANHATTAN, weight 2.0 (greedy)"}};

  printf("  %-32s %10s %10s\n", "heuristic", "cells seen", "path len");
  hline(56);
  int* explored_zero = calloc((size_t)w * h, sizeof(int));
  int* explored_greedy = calloc((size_t)w * h, sizeof(int));
  for (size_t r = 0; r < sizeof(runs) / sizeof(runs[0]); ++r) {
    HeurCtx ctx = {runs[r].base, w, h, NULL, 0};
    if (r == 0) ctx.explored = explored_zero;
    if (r == 3) ctx.explored = explored_greedy;
    TCOD_path_t path = TCOD_path_new_using_map(map, 0.0f); /* 0 diag => 4-connected */
    /* counting heuristic carries its OWN context (the new 4-arg set_heuristic) */
    TCOD_path_set_heuristic(path, counting_heuristic, &ctx, runs[r].weight);
    TCOD_path_compute(path, sx, sy, gx, gy);
    printf("  %-32s %10ld %10d\n", runs[r].name, ctx.calls, TCOD_path_size(path));
    TCOD_path_delete(path);
  }
  hline(56);

  printf("\nCells opened ('.' = touched)    ZERO / Dijkstra      vs    MANHATTAN w=2.0\n\n");
  for (int y = 0; y < h; ++y) {
    for (int side = 0; side < 2; ++side) {
      const int* ex = side == 0 ? explored_zero : explored_greedy;
      for (int x = 0; x < w; ++x) {
        if (wall[y * w + x])
          putchar('#');
        else if (x == sx && y == sy)
          putchar('S');
        else if (x == gx && y == gy)
          putchar('G');
        else
          putchar(ex[y * w + x] ? '.' : ' ');
      }
      if (side == 0) printf("   ");
    }
    putchar('\n');
  }
  free(explored_zero);
  free(explored_greedy);
  free(wall);
  TCOD_map_delete(map);
  printf("\n");
}

/* ============================================================== SCENE 2 ==== */
/* Terrain: '.'=open(1) ','=grass(2) '~'=bog(5) '='=water(12) '#'=wall(blocked) */
static const char* SCENE2[] = {
    "............................",
    ".S..,,,,,...........=====...",
    "....,,,,,,,.........=====...",
    ".....,,~~~~~,.......=====...",
    "......~~~~~~~~,.....,,,,....",
    ".......~~~~~~~~,,..........G.",
    "......,,~~~~~~,,...........  ",
    "........,,,,,.............. ",
    "...........................",
};
#define S2_H ((int)(sizeof(SCENE2) / sizeof(SCENE2[0])))

typedef struct {
  int w, h;
  const char** art;
} TerrainCtx;

static float terrain_tile_cost(char ch) {
  switch (ch) {
    case '#':
      return 0.0f; /* impassable */
    case '=':
      return 12.0f; /* deep water */
    case '~':
      return 5.0f; /* bog */
    case ',':
      return 2.0f; /* grass */
    default:
      return 1.0f; /* open */
  }
}

/* Walk-cost callback: cost of ENTERING (xTo,yTo). */
static float terrain_cost(int xFrom, int yFrom, int xTo, int yTo, void* user_data) {
  (void)xFrom;
  (void)yFrom;
  TerrainCtx* t = (TerrainCtx*)user_data;
  if (xTo < 0 || yTo < 0 || xTo >= t->w || yTo >= t->h) return 0.0f;
  char ch = t->art[yTo][xTo];
  if (ch == ' ') return 0.0f; /* ragged edge => treat as off-map */
  return terrain_tile_cost(ch);
}

static void scene2(void) {
  const int h = S2_H, w = (int)strlen(SCENE2[0]);
  int sx = 0, sy = 0, gx = 0, gy = 0;
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < (int)strlen(SCENE2[y]); ++x) {
      if (SCENE2[y][x] == 'S') {
        sx = x;
        sy = y;
      }
      if (SCENE2[y][x] == 'G') {
        gx = x;
        gy = y;
      }
    }
  TerrainCtx tctx = {w, h, SCENE2};

  printf("SCENE 2  -  variable terrain cost (open=1 grass=2 bog~=5 water==12)\n");
  printf("8-connected A* with a walk-cost callback; paired with a Euclidean heuristic\n");
  printf("through the new TCOD_path_new_using_function_ex().\n\n");

  HeurCtx hctx = {H_EUCLID, w, h, NULL, 0};
  /* The cost callback needs the terrain; the heuristic needs its own counter - two
     DIFFERENT user_data pointers. _ex shares one pointer (here the terrain) with both
     callbacks; the new 4-arg set_heuristic then gives the heuristic its own context. */
  TCOD_path_t path = TCOD_path_new_using_function_ex(w, h, terrain_cost, counting_heuristic, &tctx, 1.41f, 1.0f);
  TCOD_path_set_heuristic(path, counting_heuristic, &hctx, 1.0f);
  TCOD_path_compute(path, sx, sy, gx, gy);

  /* Mark path cells + total terrain cost actually paid. */
  int n = TCOD_path_size(path);
  char* over = malloc((size_t)w * h);
  memset(over, 0, (size_t)w * h);
  float paid = 0.0f;
  for (int i = 0; i < n; ++i) {
    int px, py;
    TCOD_path_get(path, i, &px, &py);
    over[py * w + px] = 1;
    paid += terrain_tile_cost(SCENE2[py][px]);
  }

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < (int)strlen(SCENE2[y]); ++x) {
      char ch = SCENE2[y][x];
      if (ch == 'S' || ch == 'G')
        putchar(ch);
      else if (over[y * w + x])
        putchar('*');
      else
        putchar(ch);
    }
    putchar('\n');
  }
  /* Baseline: a terrain-BLIND shortest path (uniform cost) over the same cells,
     then total up the terrain it would actually cost to walk. */
  TCOD_Map* umap = TCOD_map_new(w, h);
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < (int)strlen(SCENE2[y]); ++x) {
      const int wk = terrain_tile_cost(SCENE2[y][x]) > 0.0f;
      TCOD_map_set_properties(umap, x, y, wk, wk);
    }
  TCOD_path_t blind = TCOD_path_new_using_map(umap, 1.41f);
  TCOD_path_compute(blind, sx, sy, gx, gy);
  float blind_paid = 0.0f;
  for (int i = 0; i < TCOD_path_size(blind); ++i) {
    int px, py;
    TCOD_path_get(blind, i, &px, &py);
    blind_paid += terrain_tile_cost(SCENE2[py][px]);
  }

  printf("\nterrain-aware path: %d cells, terrain cost paid=%.1f (cells seen=%ld)\n", n, paid, hctx.calls);
  printf("terrain-BLIND path: %d cells, but would cost=%.1f to actually walk\n", TCOD_path_size(blind), blind_paid);
  printf("(the cost-aware '*' detours around bog/water; the blind shortest route pays more.)\n\n");
  free(over);
  TCOD_path_delete(blind);
  TCOD_map_delete(umap);
  TCOD_path_delete(path);
}

/* ============================================================== SCENE 3 ==== */
/* Domain heuristic: bias A* toward a precomputed waypoint corridor. */
typedef struct {
  int gx, gy; /* goal */
  int n; /* waypoint count */
  int wx[8], wy[8]; /* waypoints (corridor = polyline through them) */
  float penalty; /* how strongly to hug the corridor */
} CorridorCtx;

/* distance from point p to segment ab */
static float seg_dist(float px, float py, float ax, float ay, float bx, float by) {
  const float vx = bx - ax, vy = by - ay;
  const float wx = px - ax, wy = py - ay;
  const float len2 = vx * vx + vy * vy;
  float t = len2 > 0.0f ? (wx * vx + wy * vy) / len2 : 0.0f;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  const float cx = ax + t * vx, cy = ay + t * vy;
  const float dx = px - cx, dy = py - cy;
  return sqrtf(dx * dx + dy * dy);
}

static float corridor_heuristic(int x, int y, int gx, int gy, void* user_data) {
  CorridorCtx* c = (CorridorCtx*)user_data;
  (void)gx;
  (void)gy;
  float best = 1e9f;
  for (int i = 0; i + 1 < c->n; ++i) {
    float d = seg_dist((float)x, (float)y, (float)c->wx[i], (float)c->wy[i], (float)c->wx[i + 1], (float)c->wy[i + 1]);
    if (d < best) best = d;
  }
  const float to_goal = metric(H_EUCLID, x, y, c->gx, c->gy);
  return to_goal + c->penalty * best; /* inadmissible on purpose: pull toward corridor */
}

static void scene3(void) {
  const int w = 44, h = 15;
  int sx = 2, sy = 12, gx = 41, gy = 2;
  TCOD_Map* map = TCOD_map_new(w, h);
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) TCOD_map_set_properties(map, x, y, 1, 1); /* wide-open arena */

  CorridorCtx cc = {gx, gy, 4, {2, 14, 30, 41}, {12, 3, 11, 2}, 3.0f};

  printf("SCENE 3  -  a custom heuristic that follows a precomputed route\n");
  printf("Open arena. Waypoints 0..3 are CALLER-PLACED sites (Voronoi region centers, a\n");
  printf("hand-authored route, etc.) joined into a preferred corridor. The default heuristic\n");
  printf("takes the geometric shortest line; the corridor heuristic (dist-to-goal +\n");
  printf("%.0f*dist-to-corridor) hugs the route instead.\n\n", cc.penalty);

  /* baseline: plain Euclidean shortest path */
  char* base = malloc((size_t)w * h);
  char* corr = malloc((size_t)w * h);
  memset(base, 0, (size_t)w * h);
  memset(corr, 0, (size_t)w * h);

  TCOD_path_t p1 = TCOD_path_new_using_map(map, 1.41f);
  TCOD_path_compute(p1, sx, sy, gx, gy);
  for (int i = 0; i < TCOD_path_size(p1); ++i) {
    int x, y;
    TCOD_path_get(p1, i, &x, &y);
    base[y * w + x] = 1;
  }
  TCOD_path_delete(p1);

  TCOD_path_t p2 = TCOD_path_new_using_map(map, 1.41f);
  TCOD_path_set_heuristic(p2, corridor_heuristic, &cc, 1.0f);
  TCOD_path_compute(p2, sx, sy, gx, gy);
  for (int i = 0; i < TCOD_path_size(p2); ++i) {
    int x, y;
    TCOD_path_get(p2, i, &x, &y);
    corr[y * w + x] = 1;
  }
  TCOD_path_delete(p2);

  printf("LEFT: default heuristic (shortest)        RIGHT: corridor heuristic (preferred)\n");
  printf("  S=start G=goal  0-3=waypoints  *=path\n\n");
  for (int y = 0; y < h; ++y) {
    for (int side = 0; side < 2; ++side) {
      const char* mark = side == 0 ? base : corr;
      for (int x = 0; x < w; ++x) {
        int wp = -1;
        for (int i = 0; i < cc.n; ++i)
          if (cc.wx[i] == x && cc.wy[i] == y) wp = i;
        if (x == sx && y == sy)
          putchar('S');
        else if (x == gx && y == gy)
          putchar('G');
        else if (wp >= 0)
          putchar('0' + wp);
        else if (mark[y * w + x])
          putchar('*');
        else
          putchar('.');
      }
      if (side == 0) printf("   ");
    }
    putchar('\n');
  }
  printf("\n");
  free(base);
  free(corr);
  TCOD_map_delete(map);
}

/* ============================================================== SCENE 4 ==== */
/* Terrain straight from libtcod's EXISTING Voronoi generator. We do NOT roll our
 * own Voronoi: TCOD_heightmap_add_voronoi() blends each cell's distance to its two
 * nearest (internally-random) sites into a height field; the F2-F1 form is ~0 along
 * region boundaries and large in interiors. We threshold that into terrain costs and
 * let the cost-aware A* weave through. (McRogueFace exposes this as
 * HeightMap.add_voronoi(); see the recon notes for what it can and can't drive.) */
typedef struct {
  int w, h;
  const float* cost;
} GridCostCtx;

static float gridcost_cost(int xFrom, int yFrom, int xTo, int yTo, void* user_data) {
  (void)xFrom;
  (void)yFrom;
  const GridCostCtx* g = (const GridCostCtx*)user_data;
  if (xTo < 0 || yTo < 0 || xTo >= g->w || yTo >= g->h) return 0.0f;
  return g->cost[yTo * g->w + xTo];
}

static char terrain_glyph(float cost) {
  if (cost >= 12.0f) return '=';
  if (cost >= 5.0f) return '~';
  if (cost >= 2.0f) return ',';
  return '.';
}

static void scene4(void) {
  const int w = 40, h = 16;
  const int sx = 1, sy = 1, gx = w - 2, gy = h - 2;

  TCOD_heightmap_t* hm = TCOD_heightmap_new(w, h);
  TCOD_Random* rng = TCOD_random_new_from_seed(TCOD_RNG_MT, 1337u);
  const float coef[1] = {1.0f}; /* (squared) distance to the NEAREST site: ~0 in region cores */
  TCOD_heightmap_add_voronoi(hm, 9, 1, coef, rng);
  TCOD_heightmap_normalize(hm, 0.0f, 1.0f);

  /* height -> terrain cost band: region cores cheap, the far edges between regions
     thicken into bog then water. */
  float* cost = malloc((size_t)w * h * sizeof(float));
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      const float v = TCOD_heightmap_get_value(hm, x, y);
      float c = 1.0f; /* open */
      if (v > 0.80f)
        c = 12.0f; /* region edge -> water */
      else if (v > 0.58f)
        c = 5.0f; /* bog */
      else if (v > 0.32f)
        c = 2.0f; /* grass */
      cost[y * w + x] = c;
    }
  cost[sy * w + sx] = cost[gy * w + gx] = 1.0f; /* keep endpoints clear */

  printf("SCENE 4  -  terrain from libtcod's OWN Voronoi (TCOD_heightmap_add_voronoi)\n");
  printf("9 random sites -> a distance-to-nearest field; region edges thicken into bog/water.\n");
  printf("Cost-aware A* (open=1 grass=2 bog~=5 water==12) threads the cheap cores, no custom\n");
  printf("Voronoi code of our own.\n\n");

  GridCostCtx gctx = {w, h, cost};
  HeurCtx hctx = {H_OCTILE, w, h, NULL, 0};
  TCOD_path_t path = TCOD_path_new_using_function_ex(w, h, gridcost_cost, counting_heuristic, &gctx, 1.41f, 1.0f);
  TCOD_path_set_heuristic(path, counting_heuristic, &hctx, 1.0f);
  TCOD_path_compute(path, sx, sy, gx, gy);

  char* over = calloc((size_t)w * h, 1);
  float paid = 0.0f;
  for (int i = 0; i < TCOD_path_size(path); ++i) {
    int px, py;
    TCOD_path_get(path, i, &px, &py);
    over[py * w + px] = 1;
    paid += cost[py * w + px];
  }
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (x == sx && y == sy)
        putchar('S');
      else if (x == gx && y == gy)
        putchar('G');
      else if (over[y * w + x])
        putchar('*');
      else
        putchar(terrain_glyph(cost[y * w + x]));
    }
    putchar('\n');
  }
  printf("\npath terrain cost paid=%.1f over %d cells\n\n", paid, TCOD_path_size(path));

  free(over);
  free(cost);
  TCOD_path_delete(path);
  TCOD_random_delete(rng);
  TCOD_heightmap_delete(hm);
}

/* ============================================================== SCENE 5 ==== */
/* Landmark / ALT heuristic, built on libtcod's EXISTING Dijkstra. On a maze the
 * straight-line (Euclidean) heuristic is badly misled by walls, so A* explores dead
 * ends. Precomputing exact distances from a few landmarks (one Dijkstra flood each)
 * gives a much tighter, still-admissible heuristic:
 *   h(n) = max over landmarks L of | dist(L, goal) - dist(L, n) |   (triangle ineq). */
typedef struct {
  int w, h, gx, gy, nL;
  float* dist[4]; /* per-landmark distance field, w*h, -1 = unreachable */
  int* explored;
  long calls;
} AltCtx;

static float alt_heuristic(int x, int y, int gx, int gy, void* user_data) {
  AltCtx* a = (AltCtx*)user_data;
  (void)gx;
  (void)gy;
  ++a->calls;
  if (a->explored) a->explored[y * a->w + x] = 1;
  float best = 0.0f;
  for (int l = 0; l < a->nL; ++l) {
    const float dn = a->dist[l][y * a->w + x];
    const float dg = a->dist[l][a->gy * a->w + a->gx];
    if (dn < 0.0f || dg < 0.0f) continue;
    const float est = fabsf(dg - dn);
    if (est > best) best = est;
  }
  return best;
}

static void scene5(void) {
  /* A wide-open arena with one concave "trap": a box open only on the side facing the
   * start. The straight-line heuristic is lured inside (the interior looks closer to the
   * goal), floods the dead end, then backs out and goes around. The ALT heuristic knows
   * the true detour distance and never enters. */
  const int w = 48, h = 19;
  const int sx = 3, sy = 9, gx = w - 4, gy = 9;
  char* cells = malloc((size_t)w * h);
  memset(cells, '.', (size_t)w * h);
  for (int x = 0; x < w; ++x) {
    cells[x] = '#';
    cells[(h - 1) * w + x] = '#';
  }
  for (int y = 0; y < h; ++y) {
    cells[y * w] = '#';
    cells[y * w + (w - 1)] = '#';
  }
  const int bx0 = 14, bx1 = 34, by0 = 4, by1 = 14; /* trap box */
  for (int x = bx0; x <= bx1; ++x) {
    cells[by0 * w + x] = '#';
    cells[by1 * w + x] = '#';
  }
  for (int y = by0; y <= by1; ++y) cells[y * w + bx1] = '#'; /* right wall; left side is the mouth */

  TCOD_Map* map = TCOD_map_new(w, h);
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      const int wk = cells[y * w + x] != '#';
      TCOD_map_set_properties(map, x, y, wk, wk);
    }

  /* Precompute landmark distance fields with the EXISTING single-source Dijkstra. */
  const int lx[4] = {1, w - 2, 1, w - 2};
  const int ly[4] = {1, 1, h - 2, h - 2};
  AltCtx alt = {w, h, gx, gy, 4, {NULL, NULL, NULL, NULL}, NULL, 0};
  for (int l = 0; l < alt.nL; ++l) {
    /* snap a landmark to the nearest open cell if it sits in a wall */
    int lxx = lx[l], lyy = ly[l];
    if (!TCOD_map_is_walkable(map, lxx, lyy)) {
      lxx += (lxx < w / 2) ? 1 : -1;
      lyy += (lyy < h / 2) ? 1 : -1;
    }
    TCOD_Dijkstra* dj = TCOD_dijkstra_new(map, 1.0f);
    TCOD_dijkstra_compute(dj, lxx, lyy);
    alt.dist[l] = malloc((size_t)w * h * sizeof(float));
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) alt.dist[l][y * w + x] = TCOD_dijkstra_get_distance(dj, x, y);
    TCOD_dijkstra_delete(dj);
  }

  printf("SCENE 5  -  landmark (ALT) heuristic vs straight-line, around a concave trap\n");
  printf("4 corner landmarks, each one EXISTING Dijkstra flood. The straight-line heuristic\n");
  printf("floods the dead-end box (it 'looks' closer to G); ALT's precomputed graph\n");
  printf("distances reveal the detour, so it barely enters.\n\n");

  /* run A* with euclidean vs ALT, counting cells opened */
  int* ex_euc = calloc((size_t)w * h, sizeof(int));
  int* ex_alt = calloc((size_t)w * h, sizeof(int));

  HeurCtx he = {H_EUCLID, w, h, ex_euc, 0};
  TCOD_path_t pe = TCOD_path_new_using_map(map, 0.0f);
  TCOD_path_set_heuristic(pe, counting_heuristic, &he, 1.0f);
  TCOD_path_compute(pe, sx, sy, gx, gy);

  alt.explored = ex_alt;
  TCOD_path_t pa = TCOD_path_new_using_map(map, 0.0f);
  TCOD_path_set_heuristic(pa, alt_heuristic, &alt, 1.0f);
  TCOD_path_compute(pa, sx, sy, gx, gy);

  printf("  %-22s %10s %10s\n", "heuristic", "cells seen", "path len");
  hline(46);
  printf("  %-22s %10ld %10d\n", "EUCLIDEAN", he.calls, TCOD_path_size(pe));
  printf("  %-22s %10ld %10d\n", "ALT (4 landmarks)", alt.calls, TCOD_path_size(pa));
  hline(46);

  printf("\nCells opened ('.')      EUCLIDEAN              vs            ALT\n\n");
  for (int y = 0; y < h; ++y) {
    for (int side = 0; side < 2; ++side) {
      const int* ex = side == 0 ? ex_euc : ex_alt;
      for (int x = 0; x < w; ++x) {
        if (cells[y * w + x] == '#')
          putchar('#');
        else if (x == sx && y == sy)
          putchar('S');
        else if (x == gx && y == gy)
          putchar('G');
        else
          putchar(ex[y * w + x] ? '.' : ' ');
      }
      if (side == 0) printf("  ");
    }
    putchar('\n');
  }
  printf("\n");

  TCOD_path_delete(pe);
  TCOD_path_delete(pa);
  for (int l = 0; l < alt.nL; ++l) free(alt.dist[l]);
  free(ex_euc);
  free(ex_alt);
  free(cells);
  TCOD_map_delete(map);
}

int main(void) {
  printf("\n==== libtcod custom-heuristic / weighted-A* demo ====\n\n");
  scene1();
  hline(74);
  scene2();
  hline(74);
  scene3();
  hline(74);
  scene4();
  hline(74);
  scene5();
  return 0;
}
