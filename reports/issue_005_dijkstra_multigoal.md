# Issue #5: Rewrite Dijkstra Implementation

## Summary

**Issue**: [Rewrite Dijkstra implementation #5](https://github.com/libtcod/libtcod/issues/5)
**Opened**: December 21, 2018 by HexDecimal
**Status**: Open (enhancement)
**Priority**: High value for roguelike development

## Issue Description

> "The current API can only handle one goal point for Dijkstra. This locks out many popular Dijkstra tricks such as auto-exploration and safety maps."

The current `TCOD_dijkstra_compute()` function takes a single root point:
```c
void TCOD_dijkstra_compute(TCOD_Dijkstra* dijkstra, int root_x, int root_y);
```

This limitation prevents implementing "Dijkstra maps" as described in roguelike development literature.

## The Power of Multi-Goal Dijkstra Maps

### Core Concept (from [RogueBasin](https://www.roguebasin.com/index.php/The_Incredible_Power_of_Dijkstra_Maps))

A Dijkstra map represents distance information where each cell contains the number of steps to reach **any** goal. The algorithm:

1. Set all goal cells to 0, all others to infinity
2. For each floor tile: if value > lowest_neighbor + 1, set to lowest_neighbor + 1
3. Repeat until no changes

Unlike single-goal pathfinding, multi-goal Dijkstra maps enable:

| Application | Description | Goals |
|-------------|-------------|-------|
| **Auto-exploration** | Find nearest unexplored area | All undiscovered tiles |
| **Safety maps** | Flee intelligently from threats | Player + allies (inverted) |
| **Resource gathering** | Path to nearest resource | All resource locations |
| **Pack behavior** | Monsters stay near each other | Other monsters |
| **Desire-driven AI** | Balance multiple priorities | Weighted combination of maps |

### Safety Map Algorithm

1. Create Dijkstra map with player as goal
2. Negate all values: `distance = -distance`
3. Rescan with modified rule: if value > lowest_neighbor + 1, update
4. Result: Each cell shows "safest" direction away from player

### Desire-Driven AI

Create separate Dijkstra maps for:
- Player location (approach or flee)
- Food sources (hunger)
- Treasure (greed)
- Allies (pack behavior)
- Sounds (investigation)

Combine: `final_value = w1*map1 + w2*map2 + ...`

Monsters "roll downhill" on combined map.

## Current Implementation Analysis

### Legacy Dijkstra (`path_c.c`)

```c
typedef struct TCOD_Dijkstra {
  int diagonal_cost;
  int width, height, nodes_max;
  TCOD_Map* map;
  TCOD_path_func_t func;
  void* user_data;
  unsigned int* distances;  // Distance grid
  unsigned int* nodes;      // Processing queue
  TCOD_list_t path;
} TCOD_Dijkstra;
```

**Limitations:**
- Single root point per computation
- No multi-goal initialization
- Integer distances (×100 scaling)
- Fixed 2D grid

### Modern Pathfinder (`pathfinder.c`)

The `TCOD_Pathfinder` struct offers n-dimensional support but also lacks multi-goal API:
```c
TCODLIB_CAPI int TCOD_pf_compute(struct TCOD_Pathfinder* path);
```

## Recommended Implementation

### Phase 1: Multi-Goal API for Legacy Dijkstra

Add new functions that accept multiple goals:

```c
/**
 * @brief Initialize a Dijkstra computation with multiple goal points.
 *
 * @param dijkstra The Dijkstra pathfinder to initialize.
 * @param goals Array of goal coordinates [x0, y0, x1, y1, ...].
 * @param goal_count Number of goal points.
 * @param goal_weights Optional weights for each goal (NULL = all weight 0).
 */
void TCOD_dijkstra_compute_multi(
    TCOD_Dijkstra* dijkstra,
    const int* goals,
    int goal_count,
    const int* goal_weights  // Optional: different starting costs
);

/**
 * @brief Initialize Dijkstra with goal cells marked in a mask array.
 *
 * @param dijkstra The Dijkstra pathfinder to initialize.
 * @param goal_mask Boolean array where true = goal cell.
 */
void TCOD_dijkstra_compute_masked(
    TCOD_Dijkstra* dijkstra,
    const bool* goal_mask
);

/**
 * @brief Initialize Dijkstra with weighted goal values from an array.
 *
 * @param dijkstra The Dijkstra pathfinder to initialize.
 * @param initial_distances Starting distance values (0 = goal, MAX = unknown).
 */
void TCOD_dijkstra_compute_from_distances(
    TCOD_Dijkstra* dijkstra,
    const unsigned int* initial_distances
);
```

### Phase 2: Dijkstra Map Operations

Add utilities for common Dijkstra map manipulations:

```c
/**
 * @brief Get the raw distance grid for direct manipulation.
 */
unsigned int* TCOD_dijkstra_get_distance_grid(TCOD_Dijkstra* dijkstra);

/**
 * @brief Invert a Dijkstra map for flee behavior (safety maps).
 *
 * Negates all values and rescans to create a "flee from goals" map.
 */
void TCOD_dijkstra_invert(TCOD_Dijkstra* dijkstra);

/**
 * @brief Combine multiple Dijkstra maps with weights.
 *
 * result[i] = sum(weights[j] * maps[j][i]) for all j
 */
void TCOD_dijkstra_combine(
    TCOD_Dijkstra* result,
    const TCOD_Dijkstra** maps,
    const float* weights,
    int map_count
);

/**
 * @brief Get the direction of steepest descent (toward goals).
 *
 * @param dijkstra The computed Dijkstra map.
 * @param x Current X position.
 * @param y Current Y position.
 * @param out_dx Output: X direction (-1, 0, or 1).
 * @param out_dy Output: Y direction (-1, 0, or 1).
 * @return true if a valid descent direction exists.
 */
bool TCOD_dijkstra_get_descent(
    const TCOD_Dijkstra* dijkstra,
    int x, int y,
    int* out_dx, int* out_dy
);
```

### Phase 3: GoalMap Class (C++ Wrapper)

Following [RogueSharp's GoalMap](https://roguesharp.wordpress.com/tag/pathfinding/) pattern:

```cpp
namespace tcod {

class GoalMap {
public:
    GoalMap(int width, int height, float diagonal_cost = 1.41f);
    GoalMap(const TCODMap& map, float diagonal_cost = 1.41f);

    // Goal management
    void add_goal(int x, int y, float weight = 0.0f);
    void add_goals(std::span<const std::array<int, 2>> goals);
    void clear_goals();

    // Computation
    void compute();

    // Query
    float get_distance(int x, int y) const;
    std::array<int, 2> get_next_step(int x, int y) const;
    std::vector<std::array<int, 2>> get_path(int x, int y) const;

    // Map operations
    void invert();  // For flee behavior
    void combine(const GoalMap& other, float weight);

    // Direct access for advanced use
    std::span<float> distances();
    std::span<const float> distances() const;

private:
    int width_, height_;
    float diagonal_cost_;
    std::vector<float> distance_grid_;
    std::vector<std::pair<std::array<int, 2>, float>> goals_;
};

}  // namespace tcod
```

## Implementation Notes

### Algorithm Modification

Current `TCOD_dijkstra_compute`:
```c
distances[root] = 0;
nodes[index] = root;
// ... flood fill from root
```

Multi-goal version:
```c
// Initialize all goals
for (int i = 0; i < goal_count; i++) {
    int goal = goals[i * 2] + goals[i * 2 + 1] * width;
    distances[goal] = goal_weights ? goal_weights[i] : 0;
    nodes[last_index++] = goal;
}
// ... flood fill from all goals simultaneously
```

### Performance Considerations

- Multi-goal Dijkstra is inherently O(V log V + E) same as single-goal
- Safety map inversion requires second pass
- Map combination is O(V) per map
- Consider caching for frequently-recomputed maps

### Memory Layout

The existing `distances` and `nodes` arrays can handle multi-goal without reallocation. The queue processing naturally handles multiple starting points.

## Use Case Examples

### Auto-Exploration

```c
// Mark all unexplored tiles as goals
for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
        goal_mask[x + y * width] = !player_has_seen[x + y * width];
    }
}
TCOD_dijkstra_compute_masked(dijkstra, goal_mask);

// Player moves toward nearest unexplored tile
int dx, dy;
TCOD_dijkstra_get_descent(dijkstra, player_x, player_y, &dx, &dy);
```

### Monster Fleeing

```c
// Create flee map
TCOD_dijkstra_compute(dijkstra, player_x, player_y);
TCOD_dijkstra_invert(dijkstra);

// Monster moves away from player intelligently
int dx, dy;
TCOD_dijkstra_get_descent(dijkstra, monster_x, monster_y, &dx, &dy);
```

### Desire-Driven AI

```c
// Compute individual desire maps
TCOD_dijkstra_compute(player_map, player_x, player_y);
TCOD_dijkstra_compute_multi(food_map, food_locations, food_count, NULL);
TCOD_dijkstra_compute_multi(treasure_map, treasure_locations, treasure_count, NULL);

// Combine based on monster's priorities
float weights[] = {-0.5f, 0.3f, 0.2f};  // Flee player, seek food, seek treasure
TCOD_Dijkstra* maps[] = {player_map, food_map, treasure_map};
TCOD_dijkstra_combine(result, maps, weights, 3);
```

## Backward Compatibility

- Existing `TCOD_dijkstra_compute(dijkstra, root_x, root_y)` remains unchanged
- New functions are additions, not replacements
- Single-goal is equivalent to multi-goal with count=1

## Testing Strategy

1. **Equivalence tests**: Single-goal multi API matches legacy single-goal
2. **Multi-goal tests**: Verify equidistant goals produce correct gradient
3. **Inversion tests**: Safety map correctly leads away from threats
4. **Combination tests**: Weighted maps produce expected behavior
5. **Edge cases**: Empty goals, unreachable cells, diagonal-only paths

## References

- [RogueBasin: The Incredible Power of Dijkstra Maps](https://www.roguebasin.com/index.php/The_Incredible_Power_of_Dijkstra_Maps)
- [RogueBasin: Dijkstra Maps Visualized](https://www.roguebasin.com/index.php/Dijkstra_Maps_Visualized)
- [Hacker News Discussion](https://news.ycombinator.com/item?id=34410879)
- [RogueSharp: GoalMap](https://roguesharp.wordpress.com/tag/pathfinding/)
- [Stanford: Introduction to A*](http://theory.stanford.edu/~amitp/GameProgramming/AStarComparison.html)

## Conclusion

Multi-goal Dijkstra maps are a foundational technique in roguelike AI. The current single-goal limitation prevents libtcod users from implementing:
- Intelligent fleeing behavior
- Auto-exploration
- Resource-seeking AI
- Pack/swarm behavior
- Desire-driven decision making

The recommended implementation adds new functions without breaking existing API, provides common operations (invert, combine), and enables the "Dijkstra map" patterns described in roguelike development literature.

**Priority**: This is a high-value enhancement that would significantly improve libtcod's utility for roguelike AI development.
