# Issue #71: Refactoring TCOD's Field-of-View Algorithms

## Summary

**Issue**: [Refactoring TCOD's field-of-view algorithms #71](https://github.com/libtcod/libtcod/issues/71)
**Opened**: July 30, 2020 by HexDecimal
**Status**: Open (enhancement)
**Purpose**: Tracking issue for FOV resources and planned refactoring

## Issue Description

HexDecimal created this issue to track resources for a comprehensive FOV refactoring. The stated long-term goals are:

1. **Custom post-process pass** - Allow POV separate from light source, replacing `light_walls` parameter
2. **Replace TCOD_Map** - Create a lower-level type standard for all algorithms
3. **Refactor into separate functions** - Enable unused algorithm removal in static linking, add permissiveness parameter
4. **Performance tracking** - Benchmark all FOV algorithms
5. **Fix reentrancy** - Address issue #47 (non-reentrant implementations)
6. **FOV accumulation** - Allow combining multiple FOV outputs
7. **New algorithms** - Add additional FOV implementations

## Referenced Resources

### Primary Resources from Issue

| Resource | Description | Status |
|----------|-------------|--------|
| [RogueBasin Comparative Study](http://www.roguebasin.com/index.php?title=Comparative_study_of_field_of_view_algorithms_for_2D_grid_based_worlds) | Older benchmarks, notes on FOV quirks | Active reference |
| [Adam Millington's Blog](http://www.adammil.net/blog/v125_Roguelike_Vision_Algorithms.html) | Comprehensive algorithm comparison | Canonical reference |
| [Western University Thesis](https://ir.lib.uwo.ca/cgi/viewcontent.cgi?article=8883&context=etd) | Academic FOV analysis using TCOD | Academic validation |
| [Albert Ford's Shadowcasting](https://www.albertford.com/shadowcasting/) | Symmetric shadowcast implementation | CC0 licensed, already integrated |

### Current Implementation Files

| Algorithm | File | Lines | Source |
|-----------|------|-------|--------|
| Circular Raycasting | `fov_circular_raycasting.c` | 118 | roguebasin.com |
| Diamond Raycasting | `fov_diamond_raycasting.c` | 252 | oocities (los_rays) |
| Recursive Shadowcast | `fov_recursive_shadowcasting.c` | 134 | roguebasin |
| Permissive (0-8) | `fov_permissive2.c` | 330 | roguebasin |
| Restrictive (MRPAS) | `fov_restrictive.c` | 265 | roguebasin (Mingos) |
| Symmetric Shadowcast | `fov_symmetric_shadowcast.c` | 179 | albertford.com |

### Dispatcher Architecture

`fov_c.c` (247 lines) contains:
- `TCOD_Map` management functions
- `TCOD_map_compute_fov()` dispatcher (switch on algorithm enum)
- `TCOD_map_postprocess()` for wall lighting

## Thread Perspectives

### HexDecimal (Maintainer)
- Stopped adding C++ implementations due to porting hassles
- Notes C++ lambdas were "really useful" for flexible FOV implementations
- Prefers implementing in C and wrapping to C++ rather than vice versa

### Andres6936 (Contributor)
- Successfully implemented Diamond Wall and Permissive in their own fork
- Had issues with Ray Casting implementation
- Links to Doryen implementations as reference

## Algorithm Comparison (Modern Research)

### From Adam Milazzo's Analysis

| Property | Basic | Diamond | Shadow | Permissive | Restrictive | Symmetric |
|----------|-------|---------|--------|------------|-------------|-----------|
| Symmetry | Poor | Poor | Poor | Varies* | Good | Perfect |
| Expansive Walls | No | Yes | No | Yes | No | Yes |
| Performance | Fast | Medium | Fast | Slow | Fast | Fast |
| Indoor Accuracy | Good | Good | Good | Good | Good | Excellent |
| Outdoor Accuracy | Poor | Poor | Poor | Varies | Good | Good |

*Permissive symmetry is inversely proportional to permissiveness (worse with <= 4)

### Symmetric Shadowcasting (Albert Ford)

Key advantages:
- **Perfect floor-tile symmetry**: If A sees B, B always sees A
- **Expansive walls**: All visible walls illuminate
- **Bresenham compatibility**: Matches line-of-sight exactly
- **CC0 license**: No restrictions on use

Implementation approach:
- Quadrant-based processing (4 sectors)
- Rational arithmetic (no floating-point artifacts)
- Callback-based design (`is_blocking`, `mark_visible`)
- Non-recursive iterative variant available

This algorithm was added to libtcod in version 1.16 as `FOV_SYMMETRIC_SHADOWCAST`.

## Current Architecture Analysis

### TCOD_Map Structure (`fov_types.h:43-56`)

```c
struct TCOD_MapCell {
  bool transparent;
  bool walkable;
  bool fov;
};

typedef struct TCOD_Map {
  int width;
  int height;
  int nbcells;
  struct TCOD_MapCell* __restrict cells;
} TCOD_Map;
```

### Post-Processing (`fov_c.c:101-132`)

The `light_walls` parameter is handled via post-processing rather than within algorithms:
```c
static void TCOD_map_postprocess_quadrant(TCOD_Map* map, int x0, int y0, int x1, int y1, int dx, int dy)
```

This spreads FOV to adjacent wall tiles after the main algorithm completes.

### Algorithm Dispatcher (`fov_c.c:164-204`)

```c
switch (algo) {
    case FOV_BASIC: return TCOD_map_compute_fov_circular_raycasting(...);
    case FOV_DIAMOND: return TCOD_map_compute_fov_diamond_raycasting(...);
    // ... etc
}
```

## Recommended Refactoring Approach

### Phase 1: Modernize Common Infrastructure

1. **Create `fov_common.h`** with shared utilities:
   ```c
   // Quadrant/octant transformation matrices
   typedef struct TCOD_FOV_Transform {
       int8_t dx, dy;  // Direction multipliers
   } TCOD_FOV_Transform;

   // Common callback types
   typedef bool (*TCOD_fov_transparent_func)(int x, int y, void* userdata);
   typedef void (*TCOD_fov_mark_visible_func)(int x, int y, void* userdata);
   ```

2. **Standardize algorithm interfaces**:
   ```c
   TCOD_Error TCOD_fov_compute_X(
       int origin_x, int origin_y,
       int width, int height,
       int max_radius,
       TCOD_fov_transparent_func is_transparent,
       TCOD_fov_mark_visible_func mark_visible,
       void* userdata
   );
   ```

### Phase 2: Separate Post-Processing

1. **Make `light_walls` a separate pass**:
   ```c
   // Algorithms no longer handle light_walls
   TCOD_fov_compute_symmetric(origin, is_transparent, mark_visible, userdata);

   // Separate wall lighting
   if (light_walls) {
       TCOD_fov_postprocess_walls(map, origin_x, origin_y, radius);
   }
   ```

2. **Enable custom post-processing**:
   ```c
   typedef void (*TCOD_fov_postprocess_func)(TCOD_Map* map, int x, int y, void* userdata);
   TCOD_Error TCOD_fov_postprocess_custom(TCOD_Map* map, TCOD_fov_postprocess_func func, void* userdata);
   ```

### Phase 3: New Lower-Level API

Replace `TCOD_Map` dependency with array-based API:
```c
// Works with any array layout
TCOD_Error TCOD_fov_compute_on_array(
    int origin_x, int origin_y,
    const bool* transparency,  // Input: is cell transparent?
    bool* visibility,          // Output: is cell visible?
    int width, int height,
    size_t stride,            // Bytes between rows
    int max_radius,
    TCOD_fov_algorithm_t algo
);
```

### Phase 4: Reentrancy Fix (Issue #47)

Current non-reentrant issues:
- Some algorithms may use static variables (need audit)
- Post-processing modifies map in-place

Solution:
- Ensure all algorithms are pure functions of their inputs
- Add thread-safety documentation
- Consider `TCOD_fov_compute_to_buffer()` that writes to separate array

### Phase 5: FOV Accumulation

```c
// Combine multiple FOV computations
TCOD_Error TCOD_fov_accumulate(
    TCOD_Map* map,
    int origin_x, int origin_y,
    int max_radius,
    TCOD_fov_algorithm_t algo,
    bool clear_first  // false = OR with existing FOV
);
```

## Testing Strategy

1. **Visual comparison tests** - Render FOV outputs for all algorithms, compare against baseline images
2. **Symmetry tests** - For symmetric algorithms, verify A sees B implies B sees A
3. **Performance benchmarks** - Time each algorithm on standard map sizes
4. **Reentrancy tests** - Parallel FOV computations from multiple threads

## Backward Compatibility

- Existing `TCOD_map_compute_fov()` signature remains unchanged
- New functions added alongside, not replacing
- `light_walls` behavior preserved in compatibility layer

## Performance Considerations

From comparative research:
- **MRPAS (Restrictive)**: "Very efficient, processes as few points as possible"
- **Recursive Shadowcast**: "Especially fast in confined areas such as corridors and small rooms"
- **Permissive**: Generally slower, exact implementations "fairly slow"

Recommendation: Add benchmark suite as part of refactoring to track regressions.

## References

- [RogueBasin: Comparative study of FOV algorithms](http://www.roguebasin.com/index.php?title=Comparative_study_of_field_of_view_algorithms_for_2D_grid_based_worlds)
- [Adam Millington: Roguelike Vision Algorithms](http://www.adammil.net/blog/v125_Roguelike_Vision_Algorithms.html)
- [Albert Ford: Symmetric Shadowcasting](https://www.albertford.com/shadowcasting/)
- [RogueBasin: FOV using recursive shadowcasting](https://www.roguebasin.com/index.php/FOV_using_recursive_shadowcasting)
- [GitHub: MRPAS-FOV-Algorithm](https://github.com/ZombiePointer/MRPAS-FOV-Algorithm)
- [RogueBasin: Field of Vision](https://www.roguebasin.com/index.php/Field_of_Vision)

## Conclusion

The FOV refactoring is an ambitious but valuable project. The recommended approach is incremental:

1. **Immediate value**: Create common infrastructure and standardize interfaces (Phase 1)
2. **Quick win**: Separate post-processing for flexibility (Phase 2)
3. **Breaking changes**: New array-based API can be added in parallel without removing old API (Phase 3)
4. **Polish**: Reentrancy and accumulation can be addressed incrementally

The symmetric shadowcast algorithm (already added) provides a solid modern baseline. The main work is infrastructure modernization rather than new algorithm development.
