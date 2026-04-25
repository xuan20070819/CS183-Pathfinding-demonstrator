// ============================================================
// AI Pathfinder Arena - BFS & Dijkstra Visualizer
// 20×20 Grid, Interactive Obstacles, Step-by-Step Animation
// ============================================================

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.PriorityQueue;

// ---------- Grid Constants ----------
final int GRID_SIZE = 20;          // 20x20 grid
final int CELL_SIZE = 38;          // pixel size of each cell (2px reserved for grid lines)
final int GRID_ORIGIN_X = 10;      // top-left corner X of the grid
final int GRID_ORIGIN_Y = 10;      // top-left corner Y of the grid
final int GRID_WIDTH = GRID_SIZE * CELL_SIZE;   // 760px
final int GRID_HEIGHT = GRID_SIZE * CELL_SIZE;  // 760px

// ---------- Cell State Constants ----------
final int EMPTY = 0;
final int OBSTACLE = 1;
final int START = 2;
final int END = 3;
final int EXPLORED = 4;
final int FRONTIER = 5;
final int PATH = 6;

// ---------- Color Definitions ----------
final color COLOR_EMPTY = #FFFFFF;
final color COLOR_OBSTACLE = #3C3C3C;
final color COLOR_START = #4CAF50;
final color COLOR_END = #F44336;
final color COLOR_EXPLORED = #BBDEFB;
final color COLOR_FRONTIER = #FFB74D;
final color COLOR_PATH = #FFEB3B;
final color COLOR_HOVER = #E0E0E0;
final color COLOR_GRID_LINE = #BDBDBD;
final color COLOR_BG = #FAFAFA;
final color COLOR_UI_BG = #ECEFF1;
final color COLOR_BUTTON = #607D8B;
final color COLOR_BUTTON_HOVER = #78909C;
final color COLOR_BUTTON_ACTIVE = #FF7043;
final color COLOR_SLIDER_TRACK = #90A4AE;
final color COLOR_SLIDER_THUMB = #37474F;
final color COLOR_TEXT = #263238;

// ---------- Grid Data ----------
int[][] grid = new int[GRID_SIZE][GRID_SIZE];
int startRow = 0, startCol = 0;                // mutable start point
int endRow = GRID_SIZE - 1, endCol = GRID_SIZE - 1;  // mutable end point

// ---------- Mouse Hover ----------
int hoverRow = -1, hoverCol = -1;

// ---------- Search State ----------
boolean searchRunning = false;
boolean searchComplete = false;
boolean pathFound = false;
String currentAlgorithm = "BFS";   // "BFS" or "Dijkstra"

// BFS queue
ArrayDeque<int[]> bfsQueue;
// Dijkstra priority queue (row, col, distance)
PriorityQueue<int[]> dijkstraPQ;

boolean[][] visited;
int[][] parentR, parentC;
int[][] distance;                  // used for Dijkstra
ArrayList<int[]> currentFrontier;  // list of current frontier nodes
ArrayList<int[]> exploredList;     // list of explored nodes
ArrayList<int[]> finalPath;        // final path

int stepsPerFrame = 5;             // number of steps per frame (5~10)
int searchStepCount = 0;           // step counter executed

// ---------- UI Elements ----------
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

// Window total dimensions
final int WINDOW_WIDTH = 800;
final int WINDOW_HEIGHT = GRID_ORIGIN_Y + GRID_HEIGHT + 60;

// ---------- Initialization ----------
void setup() {
  size(800, 830);
  frameRate(60);
  textFont(createFont("Arial", 13));

  // Initialize grid
  initGrid();

  // Initialize slider position
  sliderHandleX = SLIDER_X + map(stepsPerFrame, SLIDER_MIN, SLIDER_MAX, 0, SLIDER_WIDTH);

  // ---------- 非动画 BFS：在控制台打印路径 ----------
  println("========================================");
  println("  非动画 BFS - 控制台路径输出");
  println("========================================");
  runFullBFSConsole();
  println("========================================");
  println("  准备好进行分步动画搜索。");
  println("  左键放置/移除障碍物 | 右键清除单个障碍物");
  println("  Shift+左键 设置新起点 | Ctrl+左键 设置新终点");
  println("  点击 'Search' 开始动画搜索");
  println("========================================\n");
}

// ---------- Initialize Grid ----------
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

// ---------- Reset Search State (keep obstacles and start/end) ----------
void resetSearchState() {
  searchRunning = false;
  searchComplete = false;
  pathFound = false;
  searchStepCount = 0;
  bfsQueue = null;
  dijkstraPQ = null;
  visited = new boolean[GRID_SIZE][GRID_SIZE];
  parentR = new int[GRID_SIZE][GRID_SIZE];
  parentC = new int[GRID_SIZE][GRID_SIZE];
  distance = new int[GRID_SIZE][GRID_SIZE];
  currentFrontier = new ArrayList<int[]>();
  exploredList = new ArrayList<int[]>();
  finalPath = new ArrayList<int[]>();

  // Clear search markers (keep obstacles, start, end)
  for (int r = 0; r < GRID_SIZE; r++) {
    for (int c = 0; c < GRID_SIZE; c++) {
      if (grid[r][c] == EXPLORED || grid[r][c] == FRONTIER || grid[r][c] == PATH) {
        grid[r][c] = EMPTY;
      }
    }
  }
  grid[startRow][startCol] = START;
  grid[endRow][endCol] = END;

  // Initialize parent arrays
  for (int r = 0; r < GRID_SIZE; r++) {
    for (int c = 0; c < GRID_SIZE; c++) {
      parentR[r][c] = -1;
      parentC[r][c] = -1;
      distance[r][c] = Integer.MAX_VALUE;
    }
  }
}

// ---------- Non-animated BFS: Full Run and Print to Console ----------
void runFullBFSConsole() {
  // Use local queue
  ArrayDeque<int[]> q = new ArrayDeque<int[]>();
  boolean[][] localVisited = new boolean[GRID_SIZE][GRID_SIZE];
  int[][] localParentR = new int[GRID_SIZE][GRID_SIZE];
  int[][] localParentC = new int[GRID_SIZE][GRID_SIZE];

  for (int r = 0; r < GRID_SIZE; r++) {
    for (int c = 0; c < GRID_SIZE; c++) {
      localParentR[r][c] = -1;
      localParentC[r][c] = -1;
    }
  }

  // Check start and end points
  if (grid[startRow][startCol] == OBSTACLE) {
    println("[BFS] 起点被障碍物阻塞，无法搜索！");
    return;
  }
  if (grid[endRow][endCol] == OBSTACLE) {
    println("[BFS] 终点被障碍物阻塞，无法搜索！");
    return;
  }

  localVisited[startRow][startCol] = true;
  q.addLast(new int[]{startRow, startCol});
  int nodesVisited = 0;

  while (!q.isEmpty()) {
    int[] cur = q.removeFirst();
    int r = cur[0], c = cur[1];
    nodesVisited++;

    // Reached the end point
    if (r == endRow && c == endCol) {
      // Backtrack path
      ArrayList<int[]> path = new ArrayList<int[]>();
      int pr = r, pc = c;
      while (pr != -1 && pc != -1) {
        path.add(0, new int[]{pr, pc});
        int tr = localParentR[pr][pc];
        int tc = localParentC[pr][pc];
        pr = tr;
        pc = tc;
      }
      println("[BFS] 找到路径！长度：" + (path.size() - 1) + " 步，访问节点数：" + nodesVisited);
      print("路径: ");
      for (int i = 0; i < path.size(); i++) {
        int[] p = path.get(i);
        print("(" + p[0] + "," + p[1] + ")");
        if (i < path.size() - 1) print(" -> ");
      }
      println();
      return;
    }

    // Neighbors in four directions: up, down, left, right
    int[][] dirs = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    for (int[] d : dirs) {
      int nr = r + d[0], nc = c + d[1];
      if (nr >= 0 && nr < GRID_SIZE && nc >= 0 && nc < GRID_SIZE) {
        if (!localVisited[nr][nc] && grid[nr][nc] != OBSTACLE) {
          localVisited[nr][nc] = true;
          localParentR[nr][nc] = r;
          localParentC[nr][nc] = c;
          q.addLast(new int[]{nr, nc});
        }
      }
    }
  }
  // Queue empty and end not reached
  println("[BFS] 无法找到路径！访问节点数：" + nodesVisited);
  println("  终点被障碍物包围或不可达。");
}

// ---------- Initialize Step-by-Step Search ----------
void initStepSearch() {
  resetSearchState();
  searchRunning = true;
  searchComplete = false;
  pathFound = false;
  searchStepCount = 0;

  // Check start/end points
  if (grid[startRow][startCol] == OBSTACLE || grid[endRow][endCol] == OBSTACLE) {
    println("[动画搜索] 起点或终点被阻塞！");
    searchRunning = false;
    searchComplete = true;
    pathFound = false;
    return;
  }

  visited[startRow][startCol] = true;
  distance[startRow][startCol] = 0;
  parentR[startRow][startCol] = -1;
  parentC[startRow][startCol] = -1;

  if (currentAlgorithm.equals("BFS")) {
    bfsQueue = new ArrayDeque<int[]>();
    bfsQueue.addLast(new int[]{startRow, startCol});
    dijkstraPQ = null;
    println("[动画 BFS] 开始分步搜索...");
  } else {
    dijkstraPQ = new PriorityQueue<int[]>((a, b) -> Integer.compare(a[2], b[2]));
    dijkstraPQ.add(new int[]{startRow, startCol, 0});
    bfsQueue = null;
    println("[动画 Dijkstra] 开始分步搜索...");
  }

  // Mark start as frontier
  grid[startRow][startCol] = FRONTIER;
  currentFrontier.clear();
  currentFrontier.add(new int[]{startRow, startCol});
  exploredList.clear();
  finalPath.clear();
}

// ---------- Execute One Search Step (Process One Node) ----------
// returns true if search is still in progress, false if completed
boolean executeOneStep() {
  if (searchComplete) return false;

  int[] cur = null;
  int curDist = 0;

  // Take next node from queue
  if (currentAlgorithm.equals("BFS")) {
    if (bfsQueue == null || bfsQueue.isEmpty()) {
      // Search completed but no path found
      searchComplete = true;
      pathFound = false;
      println("[动画 BFS] 搜索完成，未找到路径。步数：" + searchStepCount);
      return false;
    }
    cur = bfsQueue.removeFirst();
    curDist = distance[cur[0]][cur[1]];
  } else {
    if (dijkstraPQ == null || dijkstraPQ.isEmpty()) {
      searchComplete = true;
      pathFound = false;
      println("[动画 Dijkstra] 搜索完成，未找到路径。步数：" + searchStepCount);
      return false;
    }
    int[] pqItem = dijkstraPQ.poll();
    cur = new int[]{pqItem[0], pqItem[1]};
    curDist = pqItem[2];
  }

  int r = cur[0], c = cur[1];

  // Remove from frontier and mark as explored
  grid[r][c] = EXPLORED;
  exploredList.add(new int[]{r, c});
  removeFromFrontier(r, c);

  // Check if reached end point
  if (r == endRow && c == endCol) {
    searchComplete = true;
    pathFound = true;
    buildFinalPath();
    println("[动画 " + currentAlgorithm + "] 找到路径！步数：" + searchStepCount +
      "，路径长度：" + (finalPath.size() - 1));
    return false;
  }

  // Expand neighbors
  int[][] dirs = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  for (int[] d : dirs) {
    int nr = r + d[0], nc = c + d[1];
    if (nr >= 0 && nr < GRID_SIZE && nc >= 0 && nc < GRID_SIZE) {
      if (!visited[nr][nc] && grid[nr][nc] != OBSTACLE) {
        visited[nr][nc] = true;
        parentR[nr][nc] = r;
        parentC[nr][nc] = c;
        distance[nr][nc] = curDist + 1;
        grid[nr][nc] = FRONTIER;
        currentFrontier.add(new int[]{nr, nc});

        if (currentAlgorithm.equals("BFS")) {
          bfsQueue.addLast(new int[]{nr, nc});
        } else {
          dijkstraPQ.add(new int[]{nr, nc, distance[nr][nc]});
        }
      }
    }
  }

  searchStepCount++;
  return true;
}

// ---------- Remove Node from Frontier List ----------
void removeFromFrontier(int r, int c) {
  for (int i = currentFrontier.size() - 1; i >= 0; i--) {
    int[] f = currentFrontier.get(i);
    if (f[0] == r && f[1] == c) {
      currentFrontier.remove(i);
      break;
    }
  }
}

// ---------- Build Final Path ----------
void buildFinalPath() {
  finalPath.clear();
  int pr = endRow, pc = endCol;
  while (pr != -1 && pc != -1) {
    finalPath.add(0, new int[]{pr, pc});
    if (pr == startRow && pc == startCol) break;
    int tr = parentR[pr][pc];
    int tc = parentC[pr][pc];
    pr = tr;
    pc = tc;
  }
  // Mark path on grid (do not overwrite start and end)
  for (int i = 1; i < finalPath.size() - 1; i++) {
    int[] p = finalPath.get(i);
    grid[p[0]][p[1]] = PATH;
  }
}

// ---------- Main Loop ----------
void draw() {
  background(COLOR_BG);

  // Calculate mouse hover cell
  updateHoverCell();

  // Step-by-step search: execute stepsPerFrame steps per frame
  if (searchRunning && !searchComplete) {
    int stepsThisFrame = 0;
    while (stepsThisFrame < stepsPerFrame && !searchComplete) {
      boolean continued = executeOneStep();
      if (!continued) break;
      stepsThisFrame++;
    }
  }

  // Draw grid
  drawGrid();

  // Draw UI
  drawUI();

  // Draw mouse hover highlight (still visible during search but no interaction)
  if (hoverRow >= 0 && hoverCol >= 0) {
    pushStyle();
    noFill();
    strokeWeight(3);
    // Change highlight color hint based on modifier key pressed
    if (keyPressed && keyCode == SHIFT) {
      stroke(#4CAF50);  // green hint for setting start
    } else if (keyPressed && keyCode == CONTROL) {
      stroke(#F44336);  // red hint for setting end
    } else {
      stroke(#FF9800);  // orange default
    }
    rect(GRID_ORIGIN_X + hoverCol * CELL_SIZE, GRID_ORIGIN_Y + hoverRow * CELL_SIZE,
      CELL_SIZE, CELL_SIZE);
    popStyle();
  }
}

// ---------- Update Mouse Hover Cell ----------
void updateHoverCell() {
  int mx = mouseX - GRID_ORIGIN_X;
  int my = mouseY - GRID_ORIGIN_Y;
  if (mx >= 0 && mx < GRID_WIDTH && my >= 0 && my < GRID_HEIGHT) {
    hoverCol = mx / CELL_SIZE;
    hoverRow = my / CELL_SIZE;
  } else {
    hoverRow = -1;
    hoverCol = -1;
  }
}

// ---------- Draw Grid ----------
void drawGrid() {
  pushMatrix();
  translate(GRID_ORIGIN_X, GRID_ORIGIN_Y);

  // Draw cells
  for (int r = 0; r < GRID_SIZE; r++) {
    for (int c = 0; c < GRID_SIZE; c++) {
      color fillColor;
      switch (grid[r][c]) {
        case OBSTACLE: fillColor = COLOR_OBSTACLE; break;
        case START:    fillColor = COLOR_START; break;
        case END:      fillColor = COLOR_END; break;
        case EXPLORED: fillColor = COLOR_EXPLORED; break;
        case FRONTIER: fillColor = COLOR_FRONTIER; break;
        case PATH:     fillColor = COLOR_PATH; break;
        default:       fillColor = COLOR_EMPTY; break;
      }

      // Mouse hover highlight (semi-transparent overlay on cell, preserve original color)
      if (r == hoverRow && c == hoverCol) {
        fill(fillColor);
        noStroke();
        rect(c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE, CELL_SIZE);
        pushStyle();
        noStroke();
        fill(255, 255, 255, 60);  // semi-transparent white highlight
        rect(c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE, CELL_SIZE);
        popStyle();
      } else {
        fill(fillColor);
        stroke(COLOR_GRID_LINE);
        strokeWeight(0.5);
        rect(c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE, CELL_SIZE);
      }
    }
  }

  // Start marker "S"
  fill(255);
  textAlign(CENTER, CENTER);
  textSize(14);
  text("S", startCol * CELL_SIZE + CELL_SIZE / 2, startRow * CELL_SIZE + CELL_SIZE / 2);

  // End marker "E"
  fill(255);
  text("E", endCol * CELL_SIZE + CELL_SIZE / 2, endRow * CELL_SIZE + CELL_SIZE / 2);

  popMatrix();
}

// ---------- Draw UI ----------
void drawUI() {
  int uiY = GRID_ORIGIN_Y + GRID_HEIGHT;

  // UI background
  fill(COLOR_UI_BG);
  noStroke();
  rect(0, uiY, width, height - uiY);

  // Divider line
  stroke(#B0BEC5);
  strokeWeight(1);
  line(0, uiY, width, uiY);

  // ---- Search Button ----
  drawButton(BUTTON_SEARCH_X, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT,
    searchRunning ? "Reset" : "Search",
    isMouseOverButton(BUTTON_SEARCH_X, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT));

  // ---- Algorithm Switch Button ----
  drawButton(BUTTON_ALGO_X, BUTTON_Y, BUTTON_ALGO_WIDTH, BUTTON_HEIGHT,
    currentAlgorithm,
    isMouseOverButton(BUTTON_ALGO_X, BUTTON_Y, BUTTON_ALGO_WIDTH, BUTTON_HEIGHT));

  // ---- Clear Button ----
  drawButton(BUTTON_CLEAR_X, BUTTON_Y, BUTTON_CLEAR_WIDTH, BUTTON_HEIGHT,
    "Clear",
    isMouseOverButton(BUTTON_CLEAR_X, BUTTON_Y, BUTTON_CLEAR_WIDTH, BUTTON_HEIGHT));

  // ---- Speed Slider ----
  drawSlider();

  // ---- Statistics ----
  fill(COLOR_TEXT);
  textAlign(LEFT, CENTER);
  textSize(12);
  String stats = "Explored: " + exploredList.size() +
    " | Frontier: " + currentFrontier.size() +
    " | Steps: " + searchStepCount;
  if (pathFound) {
    stats += " | Path len: " + (finalPath.size() - 1);
  }
  text(stats, SLIDER_X + SLIDER_WIDTH + 20, BUTTON_Y + BUTTON_HEIGHT / 2);
}

// ---------- Draw Button ----------
void drawButton(int bx, int by, int bw, int bh, String label, boolean hovered) {
  pushStyle();
  rectMode(CORNER);
  if (hovered) {
    fill(COLOR_BUTTON_HOVER);
  } else {
    fill(COLOR_BUTTON);
  }
  stroke(#455A64);
  strokeWeight(1);
  rect(bx, by, bw, bh, 5);

  fill(255);
  textAlign(CENTER, CENTER);
  textSize(12);
  text(label, bx + bw / 2, by + bh / 2);
  popStyle();
}

// ---------- Check if Mouse is Over Button ----------
boolean isMouseOverButton(int bx, int by, int bw, int bh) {
  return mouseX >= bx && mouseX <= bx + bw && mouseY >= by && mouseY <= by + bh;
}

// ---------- Draw Speed Slider ----------
void drawSlider() {
  pushStyle();
  // Label
  fill(COLOR_TEXT);
  textAlign(LEFT, CENTER);
  textSize(11);
  text("Speed: " + stepsPerFrame + "/f", SLIDER_X - 70, SLIDER_Y + 8);

  // Track
  stroke(COLOR_SLIDER_TRACK);
  strokeWeight(4);
  line(SLIDER_X, SLIDER_Y + 8, SLIDER_X + SLIDER_WIDTH, SLIDER_Y + 8);

  // Tick marks
  for (int i = SLIDER_MIN; i <= SLIDER_MAX; i++) {
    float tx = SLIDER_X + map(i, SLIDER_MIN, SLIDER_MAX, 0, SLIDER_WIDTH);
    stroke(COLOR_SLIDER_TRACK);
    strokeWeight(2);
    line(tx, SLIDER_Y, tx, SLIDER_Y + 16);
    fill(COLOR_TEXT);
    textSize(9);
    textAlign(CENTER, TOP);
    text(str(i), tx, SLIDER_Y + 18);
  }

  // Handle
  fill(COLOR_SLIDER_THUMB);
  noStroke();
  ellipse(sliderHandleX, SLIDER_Y + 8, 16, 16);

  // Handle highlight ring
  if (isMouseOverSlider()) {
    noFill();
    stroke(#FF9800);
    strokeWeight(2);
    ellipse(sliderHandleX, SLIDER_Y + 8, 20, 20);
  }

  popStyle();
}

// ---------- Check if Mouse is Over Slider Handle ----------
boolean isMouseOverSlider() {
  float d = dist(mouseX, mouseY, sliderHandleX, SLIDER_Y + 8);
  return d < 12;
}

// ---------- Mouse Released Event (Handle All Interactions) ----------
void mouseReleased() {
  // Search Button
  if (isMouseOverButton(BUTTON_SEARCH_X, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT)) {
    if (searchRunning) {
      resetSearchState();
      println("[UI] 搜索已重置。");
    } else {
      initStepSearch();
    }
    return;
  }

  // Algorithm switch button
  if (isMouseOverButton(BUTTON_ALGO_X, BUTTON_Y, BUTTON_ALGO_WIDTH, BUTTON_HEIGHT)) {
    if (!searchRunning) {
      if (currentAlgorithm.equals("BFS")) {
        currentAlgorithm = "Dijkstra";
        println("[UI] 切换到 Dijkstra 算法。");
      } else {
        currentAlgorithm = "BFS";
        println("[UI] 切换到 BFS 算法。");
      }
    }
    return;
  }

  // Clear button
  if (isMouseOverButton(BUTTON_CLEAR_X, BUTTON_Y, BUTTON_CLEAR_WIDTH, BUTTON_HEIGHT)) {
    clearAll();
    return;
  }

  // Grid interaction (only process when mouse is inside grid)
  if (hoverRow >= 0 && hoverCol >= 0) {
    // ---- Modify Start: Hold Shift + Left Click ----
    if (mouseButton == LEFT && keyPressed && keyCode == SHIFT) {
      moveStartTo(hoverRow, hoverCol);
      return;
    }
    // ---- Modify End: Hold Ctrl + Left Click ----
    if (mouseButton == LEFT && keyPressed && keyCode == CONTROL) {
      moveEndTo(hoverRow, hoverCol);
      return;
    }
    // ---- Normal Left Click: Place/Remove Obstacle ----
    if (mouseButton == LEFT && !searchRunning) {
      // Cannot modify start and end points
      if ((hoverRow == startRow && hoverCol == startCol) ||
          (hoverRow == endRow && hoverCol == endCol)) {
        return;
      }
      if (grid[hoverRow][hoverCol] == OBSTACLE) {
        grid[hoverRow][hoverCol] = EMPTY;
      } else if (grid[hoverRow][hoverCol] == EMPTY) {
        grid[hoverRow][hoverCol] = OBSTACLE;
      }
      return;
    }
    // ---- Right Click: Clear Single Obstacle ----
    if (mouseButton == RIGHT && !searchRunning) {
      if (grid[hoverRow][hoverCol] == OBSTACLE) {
        grid[hoverRow][hoverCol] = EMPTY;
      }
      return;
    }
  }
}

// ---------- Move Start Point to New Position ----------
void moveStartTo(int newRow, int newCol) {
  if (newRow == endRow && newCol == endCol) {
    println("[起点] 不能与终点重合！");
    return;
  }
  // Clear old start point
  grid[startRow][startCol] = EMPTY;
  // If new position has obstacle, clear it
  if (grid[newRow][newCol] == OBSTACLE) {
    grid[newRow][newCol] = EMPTY;
  }
  startRow = newRow;
  startCol = newCol;
  grid[startRow][startCol] = START;
  println("[起点] 移动到 (" + startRow + ", " + startCol + ")");
  // Moving start/end invalidates search, reset search state
  resetSearchState();
}

// ---------- Move End Point to New Position ----------
void moveEndTo(int newRow, int newCol) {
  if (newRow == startRow && newCol == startCol) {
    println("[终点] 不能与起点重合！");
    return;
  }
  // Clear old end point
  grid[endRow][endCol] = EMPTY;
  // If new position has obstacle, clear it
  if (grid[newRow][newCol] == OBSTACLE) {
    grid[newRow][newCol] = EMPTY;
  }
  endRow = newRow;
  endCol = newCol;
  grid[endRow][endCol] = END;
  println("[终点] 移动到 (" + endRow + ", " + endCol + ")");
  // Moving start/end invalidates search, reset search state
  resetSearchState();
}

// ---------- Clear All Obstacles and Search State ----------
void clearAll() {
  for (int r = 0; r < GRID_SIZE; r++) {
    for (int c = 0; c < GRID_SIZE; c++) {
      if (grid[r][c] == OBSTACLE || grid[r][c] == EXPLORED ||
        grid[r][c] == FRONTIER || grid[r][c] == PATH) {
        grid[r][c] = EMPTY;
      }
    }
  }
  grid[startRow][startCol] = START;
  grid[endRow][endCol] = END;
  resetSearchState();
  println("[UI] 已清除所有障碍物和搜索状态。");
}

// ---------- Mouse Dragged Event (Slider) ----------
void mouseDragged() {
  if (isMouseOverSlider() || (mouseX >= SLIDER_X - 10 && mouseX <= SLIDER_X + SLIDER_WIDTH + 10 &&
    mouseY >= SLIDER_Y - 5 && mouseY <= SLIDER_Y + 25)) {
    // Update slider position
    float nx = constrain(mouseX, SLIDER_X, SLIDER_X + SLIDER_WIDTH);
    sliderHandleX = nx;
    // Calculate steps
    float t = (sliderHandleX - SLIDER_X) / (float) SLIDER_WIDTH;
    stepsPerFrame = round(lerp(SLIDER_MIN, SLIDER_MAX, t));
    stepsPerFrame = constrain(stepsPerFrame, SLIDER_MIN, SLIDER_MAX);
  }
}

// ---------- Keyboard Events ----------
void keyPressed() {
  if (key == 'r' || key == 'R') {
    // Reset search
    resetSearchState();
    println("[键盘] 搜索已重置。");
  } else if (key == ' ') {
    // Space key: start/reset search
    if (searchRunning) {
      resetSearchState();
      println("[键盘] 搜索已重置。");
    } else {
      initStepSearch();
    }
  } else if (key == 'c' || key == 'C') {
    clearAll();
  } else if (key == 'b' || key == 'B') {
    if (!searchRunning) {
      currentAlgorithm = "BFS";
      println("[键盘] 切换到 BFS。");
    }
  } else if (key == 'd' || key == 'D') {
    if (!searchRunning) {
      currentAlgorithm = "Dijkstra";
      println("[键盘] 切换到 Dijkstra。");
    }
  } else if (key == 'p' || key == 'P') {
    // Print non-animated BFS path under current obstacle layout
    println("\n[键盘] 手动触发非动画 BFS 路径打印：");
    runFullBFSConsole();
    println();
  }
}
