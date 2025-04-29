// Explorer.cpp
#include "Explorer.h"
#include <stdlib.h>   // for abs()

// 4‐way neighbor offsets
const int Explorer::kDirections[4][2] = {
  { -1,  0 },  // up
  {  1,  0 },  // down
  {  0, -1 },  // left
  {  0,  1 }   // right
};

// path‐planner scratch (in .bss, not stack)
Point    Explorer::pathQueue[Explorer::kTotalCells];
Point    Explorer::pathParent[GRID_ROWS][GRID_COLS];
Point    Explorer::pathTemp[Explorer::kTotalCells];
uint8_t  Explorer::pathVisitedBits[Explorer::kVisitedBytes];


Explorer::Explorer(GridMap& map)
  : gridMap_(map) {}


Point Explorer::findNearestFrontierCell(const Point& startCell) const {
  // We simply scan all cells, look for FREE cells adjacent to UNKNOWN,
  // compute Manhattan distance to start, pick the minimum.
  Point  best     = startCell;
  int    bestDist = GRID_ROWS * GRID_COLS;
  for (int r = 0; r < GRID_ROWS; ++r) {
    for (int c = 0; c < GRID_COLS; ++c) {
      if (gridMap_.getCell(r, c) != FREE) continue;
      // check if any neighbor is UNKNOWN
      bool isFrontier = false;
      for (int i = 0; i < 4; ++i) {
        int nr = r + kDirections[i][0];
        int nc = c + kDirections[i][1];
        if (nr >= 0 && nr < GRID_ROWS && nc >= 0 && nc < GRID_COLS
            && gridMap_.getCell(nr, nc) == UNKNOWN) {
          isFrontier = true;
          break;
        }
      }
      if (!isFrontier) continue;
      int dist = abs(r - startCell.row) + abs(c - startCell.col);
      if (dist < bestDist) {
        bestDist = dist;
        best = Point{int8_t(r), int8_t(c)};
      }
    }
  }
  return best;
}


PointList Explorer::planPathTo(const Point& startCell,
                               const Point& goalCell) const {
  PointList path;
  if (startCell.row == goalCell.row &&
      startCell.col == goalCell.col) {
    path.push(startCell);
    return path;
  }

  // Clear visited bitfield
  clearVisited();

  int head = 0, tail = 0;
  bool found = false;

  // Enqueue start
  pathQueue[tail++] = startCell;
  setVisited(startCell.row, startCell.col);

  // BFS
  while (head < tail) {
    Point curr = pathQueue[head++];
    if (curr.row == goalCell.row && curr.col == goalCell.col) {
      found = true;
      break;
    }
    for (int i = 0; i < 4; ++i) {
      int nr = curr.row + kDirections[i][0];
      int nc = curr.col + kDirections[i][1];
      if (nr >= 0 && nr < GRID_ROWS && nc >= 0 && nc < GRID_COLS
          && !isVisited(nr, nc)
          && gridMap_.getCell(nr, nc) == FREE) {
        setVisited(nr, nc);
        pathParent[nr][nc] = curr;
        pathQueue[tail++] = Point{int8_t(nr), int8_t(nc)};
      }
    }
  }

  if (!found) {
    return path;  // empty
  }

  // Reconstruct backwards into pathTemp
  int len = 0;
  Point cur = goalCell;
  while (!(cur.row == startCell.row && cur.col == startCell.col)) {
    pathTemp[len++] = cur;
    cur = pathParent[cur.row][cur.col];
  }
  pathTemp[len++] = startCell;

  // Push into PointList in forward order
  for (int i = len - 1; i >= 0; --i) {
    path.push(pathTemp[i]);
  }
  return path;
}
