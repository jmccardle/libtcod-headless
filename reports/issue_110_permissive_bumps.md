# Issue #110: fov_permissive2 - Bumps Usage

## Summary

**Issue**: [fov_permissive2 - Bumps usage #110](https://github.com/libtcod/libtcod/issues/110)
**Opened**: March 9, 2022 by wgebczyk
**Status**: Resolved (question answered)
**Related**: [Refactoring TCOD's field-of-view algorithms #71](https://github.com/libtcod/libtcod/issues/71)

## Issue Description

wgebczyk was examining the permissive FOV implementation and questioned whether the `ViewBumpContainer` (bumps collection) was actually being used meaningfully, or if it was only serving as a memory allocator.

The key observation:
> "The collection is not iterated or randomly accessed by any `add_*_bump`. The collection is preinitialized with potentially maximum number of bumps which is all possible tile/cell."

## Code Analysis

### Data Structures (`fov_permissive2.c`)

```c
typedef struct ViewBump {
  int x, y;
  struct ViewBump* parent;  // Linked list pointer
} ViewBump;

typedef struct View {
  Line shallow_line;
  Line steep_line;
  ViewBump* shallow_bump;  // Points into ViewBumpContainer.data
  ViewBump* steep_bump;    // Points into ViewBumpContainer.data
} View;

typedef struct ViewBumpContainer {
  int count;                    // Current allocation index
  ViewBump* __restrict data;    // Preallocated array
} ViewBumpContainer;
```

### Memory Flow

1. **Allocation** (`TCOD_map_compute_fov_permissive2`):
   ```c
   ViewBumpContainer bumps = {.data = malloc(map->width * map->height * sizeof(*bumps.data))};
   ```

2. **Reset per quadrant** (`check_quadrant`):
   ```c
   bumps->count = 0;  // Reset for each quadrant
   ```

3. **Bump creation** (`add_shallow_bump` / `add_steep_bump`):
   ```c
   ViewBump* shallow = &bumps->data[bumps->count++];  // Allocate from pool
   shallow->x = x;
   shallow->y = y;
   shallow->parent = view->shallow_bump;              // Link to previous bump
   view->shallow_bump = shallow;                      // Update view's bump list
   ```

4. **Bump traversal** (in same functions):
   ```c
   const ViewBump* current_bump = view->steep_bump;
   while (current_bump) {
       if (ABOVE(&view->shallow_line, current_bump->x, current_bump->y)) {
           view->shallow_line.xi = current_bump->x;
           view->shallow_line.yi = current_bump->y;
       }
       current_bump = current_bump->parent;  // Traverse linked list
   }
   ```

## Resolution

HexDecimal confirmed wgebczyk's analysis:

> "Note that the memory allocated for ViewBumpContainer is pointed to by the non-owning ViewBump* attributes in View structs. This is how check_view is using the ViewBumpContainer even though it's not directly referencing it in its parameters."

The `ViewBumpContainer` serves as:
1. **Arena allocator**: Preallocates maximum possible bumps to avoid per-bump malloc
2. **Lifetime management**: Single allocation/free for all bumps in a quadrant
3. **Indirection layer**: Views reference bumps through pointers, not container access

The container itself is never iterated because bump access is through the linked list (`parent` pointers) stored in `View.shallow_bump` and `View.steep_bump`.

## Architecture Explanation

### Why This Pattern?

The permissive FOV algorithm needs to track "bumps" (obstacles that modify view lines) as it processes tiles. Each view can accumulate multiple bumps that affect its shallow and steep lines.

**Design choices:**
- **Arena allocation**: Avoids malloc overhead per bump during FOV computation
- **Linked list traversal**: Bumps are naturally ordered by creation time (newer bumps point to older)
- **Separation of concerns**: Container manages memory, Views manage algorithmic state

### Comparison to stb_ds

HexDecimal mentioned the container "could be replaced with a stb_ds array like active_views was." However:
- Current implementation is functional and performant
- stb_ds would add dependency for marginal benefit
- Arena pattern is appropriate for temporary algorithm state

## Potential Simplification

wgebczyk suggested bumps could be stack-allocated. Analysis:

**Pros of stack allocation:**
- No heap allocation per FOV call
- Potentially faster in tight loops

**Cons:**
- Stack overflow risk with large maps
- Current heap allocation is O(1) amortized
- Would complicate recursive calls if any

**Recommendation**: Keep current design. The heap allocation is amortized across thousands of visibility checks and is not a performance bottleneck.

## Code Quality Notes

The implementation has some complexity from the refactoring to make it reentrant:
- Previously used static variables
- Now uses explicit parameter passing
- Memory management is explicit and correct

The pattern of using an arena allocator with linked list traversal is common in algorithms that need temporary, interconnected data structures.

## Related to Issue #71

This issue connects to the broader FOV refactoring goals:
- Understanding algorithm internals helps with future refactoring
- The arena allocator pattern could be standardized across FOV algorithms
- Documentation of this pattern would help future contributors

## References

- [Adam Millington: Roguelike Vision Algorithms](http://www.adammil.net/blog/v125_Roguelike_Vision_Algorithms.html) - Referenced in thread
- [RogueBasin: Precise Permissive Field of View](http://www.roguebasin.com/index.php?title=Precise_Permissive_Field_of_View) - Original algorithm source

## Conclusion

The issue was a question about code architecture, not a bug or feature request. The resolution clarified:

1. **ViewBumpContainer** is a memory arena/pool allocator
2. **View structs** hold pointers into this arena
3. **Bump traversal** uses linked lists (via `parent` pointer), not array indexing
4. **Current design is correct** and appropriate for the algorithm's needs

No code changes are recommended. The design is intentional and serves performance goals (avoiding per-bump allocation) while maintaining algorithmic correctness.
