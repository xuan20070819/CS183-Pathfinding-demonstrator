// Algorithms used: BFS, DFS, Dijkstra, A* (more to be added)
// Pathfinding fundamentals: Graphs and Grids
// Grid cells represent nodes, adjacent cells have edges with weights
// Weight types: time, cost, distance, etc.
// Pathfinding logic: find the path with minimum cost between start and end
// Paint cells black to indicate obstacles


// Preparation

import java.util.ArrayDeque;
// High efficiency
// Double-ended queue, used for BFS
// Add/remove elements at head and tail, cannot store null
import java.util.ArrayList;
// Slower
// Dynamic Array
// Allows null values, auto-resizable
import java.util.PriorityQueue;
// Extract by priority
// Binary heap
// Greedy Algorithm

// Initialization



// Grid Constants
final int GRID_SIZE = 20;
final int CELL_SIZE = 38;
final int GRID_ORIGIN_X = 10;
final int GRID_ORIGIN_Y = 10;
// Grid coordinates

final int GRID_WIDTH = GRID_SIZE * CELL_SIZE;
final int GRID_HEIGHT = GRID_SIZE * CELL_SIZE;
// Total dimensions

// UI Elements
// Buttons
final int BUTTON_SEARCH_X = 20;
final int BUTTON_Y = GRID_ORIGIN_Y + GRID_HEIGHT + 16;
final int BUTTON_WIDTH = 110;
final int BUTTON_HEIGHT = 32;

final int BUTTON_ALGO_X = 145;
final int BUTTON_ALGO_WIDTH = 100;

final int BUTTON_CLEAR_X = 260;
final int BUTTON_CLEAR_WIDTH = 80;

// Speed Slider
final int SLIDER_X = 420;
final int SLIDER_Y = BUTTON_Y + 8;
final int SLIDER_WIDTH = 180;
final int SLIDER_MIN = 5;
final int SLIDER_MAX = 10;
float sliderHandleX;

// Cell State Constants
final int EMPTY = 0;
final int OBSTACLE = 1;
final int START = 2;
final int END = 3;
final int EXPLORED = 4;
final int FRONTIER = 5;
final int PATH = 6;

// Grid Data
int[][] grid = new int[GRID_SIZE][GRID_SIZE];
int startRow = 0, startCol = 0;
int endRow = GRID_SIZE - 1, endCol = GRID_SIZE - 1;
// Start and end points are mutable

// Window total dimensions
final int WINDOW_WIDTH = 800;
final int WINDOW_HEIGHT = GRID_ORIGIN_Y + GRID_HEIGHT + 60;


// Color Definitions
final color COLOR_EMPTY = #FFFFFF;
final color COLOR_OBSTACLE = #3C3C3C;
final color COLOR_START = #4CAF50;
final color COLOR_END = #F44336;
final color COLOR_EXPLORED = #2233ca;
final color COLOR_FRONTIER = #066790;
final color COLOR_PATH = #e17b1c;
final color COLOR_HOVER = #999999;
final color COLOR_GRID_LINE = #BDBDBD;
final color COLOR_BG = #FAFAFA;
final color COLOR_UI_BG = #ECEFF1;
final color COLOR_BUTTON = #607D8B;
final color COLOR_BUTTON_HOVER = #78909C;
final color COLOR_BUTTON_ACTIVE = #FF7043;
final color COLOR_SLIDER_TRACK = #90A4AE;
final color COLOR_SLIDER_THUMB = #37474F;
final color COLOR_TEXT = #263238;

// Mouse Hover
int hoverRow = -1, hoverCol = -1;

// Search State
boolean searchRunning = false;
boolean searchComplete = false;
boolean pathFound = false;
String currentAlgorithm = "BFS";

// Animation speed control
int stepsPerFrame = 5;
int searchStepCount = 0;

// Search Data Structures
ArrayDeque<int[]> bfsQueue;
boolean[][] visited;
int[][] parentR;
int[][] parentC;
ArrayList<int[]> currentFrontier;
ArrayList<int[]> exploredList;
ArrayList<int[]> finalPath;




// Initialization

// Reset search state after each search
void resetSearchState() {
  searchRunning = false;
  searchComplete = false;
  pathFound = false;
  searchStepCount = 0;
  bfsQueue = null;
  dijkstraPQ = null;
  aStarPQ = null;
  aStarGScore = null;
  aStarFScore = null;
  aStarInOpenSet = null;
  currentFrontier = new ArrayList<int[]>();
  exploredList = new ArrayList<int[]>();
  finalPath = new ArrayList<int[]>();
}

// Initialize grid
void initGrid() {
  for (int r = 0; r < GRID_SIZE; r++) {
    for (int c = 0; c < GRID_SIZE; c++) {
      grid[r][c] = EMPTY;
    }
  }
  grid[startRow][startCol] = START;
  grid[endRow][endCol] = END;
  resetSearchState();
}

void setup() {
  size(800, 830);
  frameRate(60);
  textFont(createFont("Arial", 13));
  initGrid();
  sliderHandleX = SLIDER_X + map(stepsPerFrame, SLIDER_MIN, SLIDER_MAX, 0, SLIDER_WIDTH);
}

void initBFS() {
  bfsQueue = new ArrayDeque<int[]>();

  visited = new boolean[GRID_SIZE][GRID_SIZE];
  parentR = new int[GRID_SIZE][GRID_SIZE];
  parentC = new int[GRID_SIZE][GRID_SIZE];
  currentFrontier = new ArrayList<int[]>();
  exploredList = new ArrayList<int[]>();
  finalPath = new ArrayList<int[]>();

  bfsQueue.offer(new int[]{startRow, startCol});
  visited[startRow][startCol] = true;
  currentFrontier.add(new int[]{startRow, startCol});

  searchRunning = true;
  searchComplete = false;
  pathFound = false;
  searchStepCount = 0;
}

boolean stepBFS() {
  if (bfsQueue == null || bfsQueue.isEmpty()) {
    searchComplete = true;
    searchRunning = false;
    return false;
  }

  int[] current = bfsQueue.poll();

  if (current[0] == endRow && current[1] == endCol) {
    pathFound = true;
    searchComplete = true;
    searchRunning = false;
    reconstructPath();
    return false;
  }

  exploredList.add(current);

  int[][] directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

  for (int[] dir : directions) {
    int newR = current[0] + dir[0];
    int newC = current[1] + dir[1];

    if (newR >= 0 && newR < GRID_SIZE && newC >= 0 && newC < GRID_SIZE &&
        !visited[newR][newC] && grid[newR][newC] != OBSTACLE) {

      visited[newR][newC] = true;
      parentR[newR][newC] = current[0];
      parentC[newR][newC] = current[1];
      bfsQueue.offer(new int[]{newR, newC});

      boolean isFrontier = true;
      for (int[] frontier : currentFrontier) {
        if (frontier[0] == newR && frontier[1] == newC) {
          isFrontier = false;
          break;
        }
      }
      if (isFrontier) {
        currentFrontier.add(new int[]{newR, newC});
      }
    }
  }

  searchStepCount++;
  return true;
}

void reconstructPath() {
  if (!pathFound) return;

  int r = endRow;
  int c = endCol;

  while (r != startRow || c != startCol) {
    finalPath.add(0, new int[]{r, c});
    int pr = parentR[r][c];
    int pc = parentC[r][c];
    r = pr;
    c = pc;
  }
}

// Draw Grid
void drawGrid() {
  for (int r = 0; r < GRID_SIZE; r++) {
    for (int c = 0; c < GRID_SIZE; c++) {
      int x = GRID_ORIGIN_X + c * CELL_SIZE;
      int y = GRID_ORIGIN_Y + r * CELL_SIZE;

      color cellColor = COLOR_EMPTY;

      if (grid[r][c] == OBSTACLE) {
        cellColor = COLOR_OBSTACLE;
      } else if (r == startRow && c == startCol) {
        cellColor = COLOR_START;
      } else if (r == endRow && c == endCol) {
        cellColor = COLOR_END;
      } else if (pathFound) {
        for (int[] p : finalPath) {
          if (p[0] == r && p[1] == c) {
            cellColor = COLOR_PATH;
            break;
          }
        }
      }

      if (cellColor == COLOR_EMPTY) {
        for (int[] exp : exploredList) {
          if (exp[0] == r && exp[1] == c) {
            cellColor = COLOR_EXPLORED;
            break;
          }
        }
      }

      if (cellColor == COLOR_EMPTY) {
        for (int[] fr : currentFrontier) {
          if (fr[0] == r && fr[1] == c) {
            cellColor = COLOR_FRONTIER;
            break;
          }
        }
      }

      if (r == hoverRow && c == hoverCol && grid[r][c] == EMPTY) {
        cellColor = COLOR_HOVER;
      }

      fill(cellColor);
      noStroke();
      rect(x, y, CELL_SIZE - 2, CELL_SIZE - 2);

      stroke(COLOR_GRID_LINE);
      noFill();
      rect(x, y, CELL_SIZE - 2, CELL_SIZE - 2);
    }
  }
}


// UI Drawing
void drawUI() {
  fill(COLOR_UI_BG);
  noStroke();
  rect(0, GRID_ORIGIN_Y + GRID_HEIGHT, WINDOW_WIDTH, 60);

  fill(COLOR_BUTTON);
  rect(BUTTON_SEARCH_X, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT);
  fill(COLOR_TEXT);
  textSize(13);
  textAlign(CENTER, CENTER);
  text("Search", BUTTON_SEARCH_X + BUTTON_WIDTH/2, BUTTON_Y + BUTTON_HEIGHT/2);

  fill(COLOR_BUTTON);
  rect(BUTTON_ALGO_X, BUTTON_Y, BUTTON_ALGO_WIDTH, BUTTON_HEIGHT);
  fill(COLOR_TEXT);
  textAlign(CENTER, CENTER);
  text(currentAlgorithm, BUTTON_ALGO_X + BUTTON_ALGO_WIDTH/2, BUTTON_Y + BUTTON_HEIGHT/2);

  fill(COLOR_BUTTON);
  rect(BUTTON_CLEAR_X, BUTTON_Y, BUTTON_CLEAR_WIDTH, BUTTON_HEIGHT);
  fill(COLOR_TEXT);
  text("Clear", BUTTON_CLEAR_X + BUTTON_CLEAR_WIDTH/2, BUTTON_Y + BUTTON_HEIGHT/2);

  fill(COLOR_SLIDER_TRACK);
  rect(SLIDER_X, SLIDER_Y, SLIDER_WIDTH, 8);
  fill(COLOR_SLIDER_THUMB);
  ellipse(sliderHandleX, SLIDER_Y + 4, 16, 16);

  fill(COLOR_TEXT);
  textAlign(LEFT, CENTER);
  text("Speed: " + stepsPerFrame, SLIDER_X, SLIDER_Y - 15);
}

// Main Loop
void draw() {
  background(COLOR_BG);
  drawGrid();
  drawUI();

  if (searchRunning) {
    for (int i = 0; i < stepsPerFrame; i++) {
      if (searchRunning) {
        if (currentAlgorithm.equals("BFS")) {
          stepBFS();
        } else if (currentAlgorithm.equals("A*")) {
          stepAStar();
        }
      }
    }
  }

  if (searchComplete && pathFound) {
    fill(COLOR_TEXT);
    textAlign(CENTER, CENTER);
    text("Path found! Steps: " + searchStepCount, WINDOW_WIDTH/2, GRID_ORIGIN_Y + GRID_HEIGHT + 30);
  } else if (searchComplete) {
    fill(COLOR_TEXT);
    textAlign(CENTER, CENTER);
    text("No path found!", WINDOW_WIDTH/2, GRID_ORIGIN_Y + GRID_HEIGHT + 30);
  }
}



// Interaction
void mousePressed() {
  if (mouseY >= GRID_ORIGIN_Y && mouseY < GRID_ORIGIN_Y + GRID_HEIGHT &&
      mouseX >= GRID_ORIGIN_X && mouseX < GRID_ORIGIN_X + GRID_WIDTH) {

    int row = (mouseY - GRID_ORIGIN_Y) / CELL_SIZE;
    int col = (mouseX - GRID_ORIGIN_X) / CELL_SIZE;

    if (row >= 0 && row < GRID_SIZE && col >= 0 && col < GRID_SIZE) {
      if (row == startRow && col == startCol) {
        grid[row][col] = START;
      } else if (row == endRow && col == endCol) {
        grid[row][col] = END;
      } else if (grid[row][col] != START && grid[row][col] != END) {
        if (grid[row][col] == OBSTACLE) {
          grid[row][col] = EMPTY;
        } else {
          grid[row][col] = OBSTACLE;
        }
      }
    }
  }

  if (mouseX >= BUTTON_SEARCH_X && mouseX <= BUTTON_SEARCH_X + BUTTON_WIDTH &&
      mouseY >= BUTTON_Y && mouseY <= BUTTON_Y + BUTTON_HEIGHT) {
    if (!searchRunning && !searchComplete) {
      if (currentAlgorithm.equals("BFS")) {
        initBFS();
      } else if (currentAlgorithm.equals("A*")) {
        initAStar();
      }
    }
  }

  if (mouseX >= BUTTON_ALGO_X && mouseX <= BUTTON_ALGO_X + BUTTON_ALGO_WIDTH &&
      mouseY >= BUTTON_Y && mouseY <= BUTTON_Y + BUTTON_HEIGHT) {
    if (currentAlgorithm.equals("BFS")) {
      currentAlgorithm = "A*";
    } else if (currentAlgorithm.equals("A*")) {
      currentAlgorithm = "BFS";
    }
  }

  if (mouseX >= BUTTON_CLEAR_X && mouseX <= BUTTON_CLEAR_X + BUTTON_CLEAR_WIDTH &&
      mouseY >= BUTTON_Y && mouseY <= BUTTON_Y + BUTTON_HEIGHT) {
    initGrid();
  }

  if (mouseX >= SLIDER_X && mouseX <= SLIDER_X + SLIDER_WIDTH &&
      mouseY >= SLIDER_Y - 10 && mouseY <= SLIDER_Y + 20) {
    sliderHandleX = constrain(mouseX, SLIDER_X, SLIDER_X + SLIDER_WIDTH);
    float ratio = (sliderHandleX - SLIDER_X) / float(SLIDER_WIDTH);
    stepsPerFrame = int(map(ratio, 0, 1, SLIDER_MIN, SLIDER_MAX));
  }
}

void mouseMoved() {
  if (mouseY >= GRID_ORIGIN_Y && mouseY < GRID_ORIGIN_Y + GRID_HEIGHT &&
      mouseX >= GRID_ORIGIN_X && mouseX < GRID_ORIGIN_X + GRID_WIDTH) {

    hoverRow = (mouseY - GRID_ORIGIN_Y) / CELL_SIZE;
    hoverCol = (mouseX - GRID_ORIGIN_X) / CELL_SIZE;
  } else {
    hoverRow = -1;
    hoverCol = -1;
  }
}

// Dijkstra priority queue (row, col, distance)
PriorityQueue<int[]> dijkstraPQ;
int[][] distance;


// A* Algorithm Module
PriorityQueue<AStarNode> aStarPQ;
int[][] aStarGScore;
int[][] aStarFScore;
boolean[][] aStarInOpenSet;

class AStarNode implements Comparable<AStarNode> {
  int row, col;
  int fScore;

  AStarNode(int row, int col, int fScore) {
    this.row = row;
    this.col = col;
    this.fScore = fScore;
  }

  @Override
  public int compareTo(AStarNode other) {
    return Integer.compare(this.fScore, other.fScore);
  }
}

void initAStar() {
  aStarPQ = new PriorityQueue<AStarNode>();

  aStarGScore = new int[GRID_SIZE][GRID_SIZE];
  aStarFScore = new int[GRID_SIZE][GRID_SIZE];
  aStarInOpenSet = new boolean[GRID_SIZE][GRID_SIZE];

  visited = new boolean[GRID_SIZE][GRID_SIZE];
  parentR = new int[GRID_SIZE][GRID_SIZE];
  parentC = new int[GRID_SIZE][GRID_SIZE];
  currentFrontier = new ArrayList<int[]>();
  exploredList = new ArrayList<int[]>();
  finalPath = new ArrayList<int[]>();

  for (int r = 0; r < GRID_SIZE; r++) {
    for (int c = 0; c < GRID_SIZE; c++) {
      aStarGScore[r][c] = Integer.MAX_VALUE;
      aStarFScore[r][c] = Integer.MAX_VALUE;
    }
  }

  aStarGScore[startRow][startCol] = 0;
  int startHeuristic = heuristic(startRow, startCol);
  aStarFScore[startRow][startCol] = startHeuristic;

  aStarPQ.offer(new AStarNode(startRow, startCol, startHeuristic));
  aStarInOpenSet[startRow][startCol] = true;
  currentFrontier.add(new int[]{startRow, startCol});

  searchRunning = true;
  searchComplete = false;
  pathFound = false;
  searchStepCount = 0;
}

int heuristic(int row, int col) {
  return Math.abs(row - endRow) + Math.abs(col - endCol);
}


boolean stepAStar() {
  if (aStarPQ == null || aStarPQ.isEmpty()) {
    searchComplete = true;
    searchRunning = false;
    return false;
  }

  AStarNode current = aStarPQ.poll();
  int currentRow = current.row;
  int currentCol = current.col;

  boolean isExplored = false;
  for (int[] exp : exploredList) {
    if (exp[0] == currentRow && exp[1] == currentCol) {
      isExplored = true;
      break;
    }
  }
  if (isExplored) {
    return true;
  }

  exploredList.add(new int[]{currentRow, currentCol});
  aStarInOpenSet[currentRow][currentCol] = false;

  if (currentRow == endRow && currentCol == endCol) {
    int r = endRow;
    int c = endCol;
    while (r != startRow || c != startCol) {
      finalPath.add(0, new int[]{r, c});
      int pr = parentR[r][c];
      int pc = parentC[r][c];
      r = pr;
      c = pc;
    }
    pathFound = true;
    searchComplete = true;
    searchRunning = false;
    return false;
  }

  int[][] directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

  for (int[] dir : directions) {
    int newR = currentRow + dir[0];
    int newC = currentCol + dir[1];

    if (newR >= 0 && newR < GRID_SIZE && newC >= 0 && newC < GRID_SIZE &&
        grid[newR][newC] != OBSTACLE) {

      int tentativeGScore = aStarGScore[currentRow][currentCol] + 1;

      if (tentativeGScore < aStarGScore[newR][newC]) {
        parentR[newR][newC] = currentRow;
        parentC[newR][newC] = currentCol;
        aStarGScore[newR][newC] = tentativeGScore;

        int hScore = heuristic(newR, newC);
        int fScore = tentativeGScore + hScore;
        aStarFScore[newR][newC] = fScore;

        if (!aStarInOpenSet[newR][newC]) {
          aStarPQ.offer(new AStarNode(newR, newC, fScore));
          aStarInOpenSet[newR][newC] = true;

          boolean isFrontier = true;
          for (int[] frontier : currentFrontier) {
            if (frontier[0] == newR && frontier[1] == newC) {
              isFrontier = false;
              break;
            }
          }
          if (isFrontier) {
            currentFrontier.add(new int[]{newR, newC});
          }
        }
      }
    }
  }

  searchStepCount++;
  return true;
}
