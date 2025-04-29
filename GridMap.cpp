// GridMap.cpp
#include "GridMap.h"
#include <Arduino.h>  // for constrain()

GridMap::GridMap() {
  reset();
}

// Makes all gird locations unknown
void GridMap::reset() {
  for (int r = 0; r < GRID_ROWS; ++r) {
    for (int c = 0; c < GRID_COLS; ++c) {
      grid[r][c] = UNKNOWN;
    }
  }
}

// Converts coords to grid indexes
Point GridMap::worldToGrid(float x, float y) const {
  int c = int(x / CELL_SIZE);
  c = constrain(c, 0, GRID_COLS - 1);
  int r = int(y / CELL_SIZE);
  r = constrain(r, 0, GRID_ROWS - 1);
  return Point{int8_t(r), int8_t(c)};
}

// Converts coords to grid indexes and marks cell as free
void GridMap::markFree(float x, float y) {
  Point p = worldToGrid(x, y);
  grid[p.row][p.col] = FREE;
}

// Converts coords to grid indexes and marks cell as occupied
void GridMap::markOcc(float x, float y) {
  Point p = worldToGrid(x, y);
  grid[p.row][p.col] = OCCUPIED;
}

// Returns wheter a cell is free occupied or unknown
Cell GridMap::getCell(int r, int c) const {
  return grid[r][c];
}
