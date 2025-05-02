#include "Explorer.h"
#include <stdlib.h>   // for abs()

// 4-way neighbor offsets up, down, left, right
const int Explorer::kDirections[4][2] = {
    { -1,  0 },  // up
    {  1,  0 },  // down
    {  0, -1 },  // left
    {  0,  1 }   // right
};

// Path-planner scratch space allocated statically, not on the stack
Point Explorer::pathQueue[Explorer::kTotalCells];
Point Explorer::pathParent[GRID_ROWS][GRID_COLS];
Point Explorer::pathTemp[Explorer::kTotalCells];
uint8_t Explorer::pathVisitedBits[Explorer::kVisitedBytes];

// Constructor: stores a reference to the grid map
Explorer::Explorer(GridMap& map)
    : gridMap_(map) {}

// Finds the nearest frontier cell from a given start point
// A frontier cell is a free cell adjacent to at least one unknown cell
Point Explorer::findNearestFrontierCell(const Point& startCell) const {
    Point best = startCell;
    int bestDist = GRID_ROWS * GRID_COLS;

    for (int r = 0; r < GRID_ROWS; ++r) {
        for (int c = 0; c < GRID_COLS; ++c) {
            if (gridMap_.getCell(r, c) != FREE) continue;

            // Check if this cell has an unknown neighbor
            bool isFrontier = false;
            for (int i = 0; i < 4; ++i) {
                int nr = r + kDirections[i][0];
                int nc = c + kDirections[i][1];

                if (nr >= 0 && nr < GRID_ROWS && nc >= 0 && nc < GRID_COLS &&
                    gridMap_.getCell(nr, nc) == UNKNOWN) {
                    isFrontier = true;
                    break;
                }
            }
            if (!isFrontier) continue;

            // Manhattan distance to the start cell
            int dist = abs(r - startCell.row) + abs(c - startCell.col);
            if (dist < bestDist) {
                bestDist = dist;
                best = Point{int8_t(r), int8_t(c)};
            }
        }
    }
    return best;
}

// Plans a path from startCell to goalCell using Bfs
PointList Explorer::planPathTo(const Point& startCell, const Point& goalCell) const {
    PointList path;

    // Return early if already at the goal
    if (startCell.row == goalCell.row && startCell.col == goalCell.col) {
        path.push(startCell);
        return path;
    }

    // Clear the visited bitfield before BFS
    clearVisited();

    int head = 0, tail = 0;
    bool found = false;

    // Enqueue the start cell
    pathQueue[tail++] = startCell;
    setVisited(startCell.row, startCell.col);

    // Perform BFS to find the shortest path
    while (head < tail) {
        Point curr = pathQueue[head++];

        // Stop if goal is reached
        if (curr.row == goalCell.row && curr.col == goalCell.col) {
            found = true;
            break;
        }

        // Explore neighbors
        for (int i = 0; i < 4; ++i) {
            int nr = curr.row + kDirections[i][0];
            int nc = curr.col + kDirections[i][1];

            // Valid neighbor and not yet visited
            if (nr >= 0 && nr < GRID_ROWS && nc >= 0 && nc < GRID_COLS &&
                !isVisited(nr, nc) && gridMap_.getCell(nr, nc) == FREE) {
                setVisited(nr, nc);
                pathParent[nr][nc] = curr;
                pathQueue[tail++] = Point{int8_t(nr), int8_t(nc)};
            }
        }
    }

    if (!found) {
        return path;  // No path found, return empty list
    }

    // Reconstruct the path backward from goal to start
    int len = 0;
    Point cur = goalCell;
    while (!(cur.row == startCell.row && cur.col == startCell.col)) {
        pathTemp[len++] = cur;
        cur = pathParent[cur.row][cur.col];
    }
    pathTemp[len++] = startCell;

    // Push the path into the output list in forward order
    for (int i = len - 1; i >= 0; --i) {
        path.push(pathTemp[i]);
    }

    return path;
}
