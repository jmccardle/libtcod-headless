# Issue #60: A* Custom Heuristic Function

## Summary

**Issue**: [A* custom heuristic function #60](https://github.com/libtcod/libtcod/issues/60)
**Opened**: May 5, 2020 by MichaelMackus
**Status**: Open (enhancement)
**Last Activity**: August 2021

## Issue Description

The request is to add a custom heuristic function parameter to libtcod's A* pathfinding implementation. Currently, the A* implementation in `path_c.c` uses hardcoded Euclidean distance as its heuristic:

```c
/* A* heuristic : remaining distance */
float remaining = (float)sqrt((cx - path->dx) * (cx - path->dx) + (cy - path->dy) * (cy - path->dy));
```

The original proposal suggested:
```c
TCOD_path_t TCOD_path_new_using_map(TCOD_map_t map, float diagonalCost, TCOD_path_func_t heuristicFunc)
```

Additionally, MichaelMackus noted that with a custom heuristic, Dijkstra could be implemented as A* with `heuristic = NULL`, potentially unifying the codebases.

## Thread Perspectives

### HexDecimal (Maintainer)
- Working on a long-term pathfinder rewrite that would:
  - Remove the 2D limit (n-dimensional support)
  - Allow custom heuristic functions
  - Allow custom graph functions
- Notes that C++ templates in `src/libtcod/pathfinding` achieve the desired flexibility but need C99 porting
- Main challenge: lacking a good priority queue for C with a stable interface
- Missing a good C container for distance/path-cost arrays
- Index sizing ("what is the size of an index") complicates heap queue design

### MichaelMackus (Contributor)
- Implemented custom pathfinding in [simplerl](https://github.com/MichaelMackus/simplerl) (MIT licensed)
- Suggested functional approach: pass a neighbor-returning function instead of complex data structures
- Notes that priority queue implementation can be simple for pathfinding purposes

## Current Implementation Analysis

### Files Involved
- **`src/libtcod/path_c.c`** (674 lines) - Main A*/Dijkstra implementation
- **`src/libtcod/path.h`** (98 lines) - C API header
- **`src/libtcod/path.hpp`** (595 lines) - C++ wrapper
- **`src/libtcod/pathfinder.c`** (311 lines) - Modern n-dimensional pathfinder
- **`src/libtcod/pathfinder.h`** (85 lines) - Modern pathfinder header

### Current Architecture

The legacy A* (`TCOD_Path`) uses:
- Custom binary heap on top of `TCOD_list_t`
- Fixed 2D grid with 8-directional movement
- Separate `grid` (covered distance) and `heuristic` (f-score) arrays
- Hardcoded Euclidean heuristic at line 406:
  ```c
  float remaining = (float)sqrt((cx - path->dx) * (cx - path->dx) + (cy - path->dy) * (cy - path->dy));
  ```

The modern pathfinder (`TCOD_Pathfinder`) offers:
- N-dimensional support (up to 4 dimensions)
- User-provided distance arrays
- Generic integer type support
- But currently NO heuristic support (pure Dijkstra)

### Heuristic Location in Code

The heuristic is applied at `path_c.c:406-408`:
```c
path->grid[offset] = covered;
path->heuristic[offset] = covered + remaining;
```

## Modern Best Practices Research

### Standard A* Heuristics (from [Stanford Theory](http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html))

| Movement Type | Heuristic | Formula |
|---------------|-----------|---------|
| 4-directional | Manhattan | `D * (|dx| + |dy|)` |
| 8-directional | Diagonal | `D * (dx + dy) + (D2 - 2*D) * min(dx, dy)` |
| Any-angle | Euclidean | `D * sqrt(dx² + dy²)` |

### Key Requirements for Heuristics

1. **Admissibility**: Never overestimate actual cost (guarantees optimal paths)
2. **Consistency**: `h(x) <= cost(x,y) + h(y)` for all nodes x,y
3. **Scale Matching**: Heuristic output must match cost function units

### Modern C++ Implementations

Popular patterns from [daancode/a-star](https://github.com/daancode/a-star) and [Rikora/A-star](https://github.com/Rikora/A-star):
```cpp
// Configuration method
astar.setHeuristic(pf::heuristic::euclidean);

// Or pass directly to findPath
astar.findPath(start, end, pf::heuristic::manhattan, diagonalCost);
```

## Recommended Implementation

### Phase 1: Add Heuristic Callback to Legacy API (Minimal Change)

Add a new creation function that accepts a heuristic callback:

```c
// New typedef
typedef float (*TCOD_heuristic_func_t)(int x, int y, int goal_x, int goal_y, void* user_data);

// New function signature
TCOD_path_t TCOD_path_new_using_function_ex(
    int map_width,
    int map_height,
    TCOD_path_func_t cost_func,
    TCOD_heuristic_func_t heuristic_func,  // NULL = Euclidean default
    void* user_data,
    float diagonalCost
);
```

**Files to Modify:**
- `src/libtcod/path.h` - Add typedef and function declaration
- `src/libtcod/path_c.c` - Add TCOD_Path.heuristic_func field, modify `TCOD_path_set_cells()`

### Phase 2: Built-in Heuristics (Optional Enhancement)

Provide common heuristics:
```c
float TCOD_heuristic_euclidean(int x, int y, int gx, int gy, void* data);
float TCOD_heuristic_manhattan(int x, int y, int gx, int gy, void* data);
float TCOD_heuristic_diagonal(int x, int y, int gx, int gy, void* data);
float TCOD_heuristic_chebyshev(int x, int y, int gx, int gy, void* data);
```

### Phase 3: C++ Wrapper Update

```cpp
// Add to ITCODPathCallback or create new interface
class ITCODHeuristicCallback {
public:
    virtual float getHeuristic(int x, int y, int goalX, int goalY, void* userData) const = 0;
};

// Or use std::function for modern C++
using HeuristicFunc = std::function<float(int, int, int, int)>;
TCODPath(int width, int height,
         const ITCODPathCallback* costCallback,
         HeuristicFunc heuristic = nullptr,  // nullptr = Euclidean
         void* userData = nullptr,
         float diagonalCost = 1.41f);
```

## Implementation Details

### Modifying TCOD_path_set_cells()

Current code at line 402-408:
```c
if (previousCovered == 0) {
    int offset = cx + cy * path->w;
    float remaining = (float)sqrt((cx - path->dx) * (cx - path->dx) + (cy - path->dy) * (cy - path->dy));
    path->grid[offset] = covered;
    path->heuristic[offset] = covered + remaining;
```

Modified:
```c
if (previousCovered == 0) {
    int offset = cx + cy * path->w;
    float remaining;
    if (path->heuristic_func) {
        remaining = path->heuristic_func(cx, cy, path->dx, path->dy, path->user_data);
    } else {
        remaining = (float)sqrt((cx - path->dx) * (cx - path->dx) + (cy - path->dy) * (cy - path->dy));
    }
    path->grid[offset] = covered;
    path->heuristic[offset] = covered + remaining;
```

### Backward Compatibility

- Existing `TCOD_path_new_using_map` and `TCOD_path_new_using_function` remain unchanged
- Default heuristic behavior (Euclidean) preserved when callback is NULL
- No changes to existing public interface

## Unifying A* and Dijkstra

As noted in the issue, setting `heuristic = 0` for all nodes gives Dijkstra's algorithm. However:

1. The current implementations differ significantly:
   - A* uses floats for distances, Dijkstra uses unsigned ints (×100 scaling)
   - Different priority queue implementations
   - Dijkstra designed for computing distances to all cells, A* for single paths

2. **Recommendation**: Keep separate implementations but document the relationship. A unified implementation could be considered for a future major version where breaking changes are acceptable.

## Testing Strategy

1. **Unit tests** for each built-in heuristic function
2. **Comparison tests** verifying custom heuristic produces same results as built-in when using same formula
3. **Optimality tests** using admissible heuristics
4. **Performance benchmarks** comparing callback overhead vs inline heuristic

## References

- [Stanford A* Heuristics Guide](http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html)
- [daancode/a-star](https://github.com/daancode/a-star) - Modern C++ A* with heuristic selection
- [Rikora/A-star](https://github.com/Rikora/A-star) - C++ implementation with custom heuristics
- [A* Search Algorithm - GeeksforGeeks](https://www.geeksforgeeks.org/dsa/a-search-algorithm/)
- [MichaelMackus/simplerl](https://github.com/MichaelMackus/simplerl) - Contributor's reference implementation

## Conclusion

Adding a custom heuristic function is a reasonable, backwards-compatible enhancement. The minimal implementation (Phase 1) adds a single new function and modifies ~20 lines of code. This addresses the immediate need without requiring the ambitious rewrite HexDecimal mentioned.

The modern pathfinder API (`TCOD_Pathfinder`) could also be extended with heuristic support, but that's a larger undertaking given its n-dimensional design.
