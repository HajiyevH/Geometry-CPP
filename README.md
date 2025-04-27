# Geometry-CPP

This project implements a C++ function to determine if two polylines are closer to each other than a specified distance threshold.

## The Task

Given two polylines, represented as `std::vector<sPoint2D>`, the goal is to write a C++11 function that checks if the minimum distance between any segment of the first polyline and any segment of the second polyline is less than a given threshold.

The distance threshold is fixed at `1.5F`.

## Implementation Approach

The provided code (`distanceCheck.cpp`) includes three approaches:

1.  **Brute Force (`arePolylinesCloserThanThresholdBrute`):** Compares every segment of polyline1 against every segment of polyline2. Simple but potentially slow for large polylines.
2.  **Bounding Box Optimization (`arePolylinesCloserThanThresholdBoundingBox`):** Calculates bounding boxes for each segment pair. Only performs the detailed distance check if the bounding boxes (expanded by the threshold) overlap. This avoids many unnecessary distance calculations.
3.  **Spatial Grid (`arePolylinesCloserThanThreshold`):** This is the main implementation. It divides the space containing both polylines into a grid where each cell size is related to the distance threshold. Segments are placed into the grid cells they overlap. Distance checks are then only performed between segments within the same cell or adjacent cells. This significantly reduces the number of pairs that need to be checked, especially for large datasets where polylines are spatially separated.

## Runtime Complexity Analysis

Let `n` be the number of points in polyline1 and `m` be the number of points in polyline2. This means there are `n-1` segments in polyline1 and `m-1` segments in polyline2.

*   **Brute Force:** The complexity is roughly **O(n * m)** because every segment from polyline1 is compared with every segment from polyline2. The `pointToSegmentDistanceSquared` function takes constant time, O(1).
*   **Bounding Box Optimization:** In the worst case (all bounding boxes overlap significantly), the complexity can still approach **O(n * m)**. However, in practice, especially for spatially sparse polylines, it performs much better by quickly discarding pairs that are far apart. The average case performance depends heavily on the data distribution.
*   **Spatial Grid:**
    *   **Grid Population:** Placing all segments into the grid takes approximately **O(n + m)** time, assuming the number of cells a single segment overlaps is relatively small on average.
    *   **Checking Cells:** Iterating through the grid cells and checking pairs within cells and adjacent cells. Let `k` be the average number of segments per relevant grid cell. The checking phase complexity is roughly proportional to the number of occupied cells multiplied by `k^2`. In the best and average cases, where segments are somewhat evenly distributed, this can be significantly faster than O(n\*m), potentially approaching **O(n + m)** if the density of segments per cell (`k`) is low. However, in the worst case (e.g., all segments fall into a few cells), it could degrade towards **O((n + m)^2)**, although the bounding box check within `checkCellPair` helps this.

The spatial grid approach generally offers the best performance for large polylines, especially when they aren't densely packed in the same small area.

## Potential Improvements

**Parallelization:** The checks for different cells or even segment pairs within the `checkCellPair` function could potentially be parallelized using threads. 
**More Advanced Spatial Structures:** More advanced spatial data structures like R-trees or k-d trees could be considered for better optimization. The solution that i implemented is one of the basic spatial data structures
