import java.util.*;
import java.util.PriorityQueue;

// ----- Grid Settings -----
int gridCols = 40;
int gridRows = 40;
final int CELL_SIZE = 20;
int gridOffsetX, gridOffsetY;

// Grid cell states
final int EMPTY = 0;
final int OBSTACLE = 1;
final int START = 2;
final int GOAL = 3;
final int GRASS = 4;
final int DESERT = 5;
int[][] grid;

// Terrain weight map
final int WEIGHT_NORMAL = 1;
final int WEIGHT_DESERT = 2;
final int WEIGHT_GRASS = 3;

// Multi-agent support: same cell can have multiple starts/goals
ArrayList<MapPoint> agents = new ArrayList<MapPoint>();
ArrayList<MapPoint> goals = new ArrayList<MapPoint>();
int nextAgentId = 1;
int nextGoalId = 1;

// For drag-to-move tool
enum DragMode { NONE, MOVE_AGENT, MOVE_GOAL }
DragMode currentDragMode = DragMode.NONE;
MapPoint draggedPoint = null;

// ----- Algorithm Related -----
enum Algorithm { BFS, DIJKSTRA, ASTAR }
Algorithm currentAlgo = Algorithm.BFS;

int speed = 1;
Slider speedSlider;
boolean running = false;
boolean paused = false;

// Search process data
ArrayList<Node> openList = new ArrayList<Node>();
ArrayList<Node> closedList = new ArrayList<Node>();
ArrayList<Node> finalPath = new ArrayList<Node>();
Node startNode, goalNode;
int visitedCount = 0;
int pathLength = 0;
int cpuCycles = 0;
boolean pathFound = false;
boolean algorithmFinished = false;

// BFS specific
ArrayDeque<Node> bfsQueue;
boolean[][] visited;
// Dijkstra specific
PriorityQueue<Node> dijkstraQueue;
int[][] dist;
// A* specific
PriorityQueue<Node> aStarQueue;

// ----- UI Controls -----
enum Tool { ADD_AGENT, ADD_GOAL, DRAW_OBSTACLE, DRAW_GRASS, DRAW_DESERT, MOVE_POINT }
Tool currentTool = Tool.DRAW_OBSTACLE;

boolean terrainDropdownExpanded = false;
int terrainDropdownY = 0;
int terrainDropdownBtnH = 30;

// Color definitions
final color COLOR_GRASS = #228B22;
final color COLOR_DESERT = #F4A460;
final color COLOR_OBSTACLE = #c4c4c4ff;
final color COLOR_EMPTY = #1E1E32;
final color COLOR_START = #00C8FF;
final color COLOR_GOAL = #FF6432;
final color COLOR_START_MULTI = #4DD0FF;
final color COLOR_GOAL_MULTI = #FF8A5C;
final color COLOR_BTN_HOVER = #FF6464;
final color COLOR_BTN_NORMAL = #503C8C;
final color COLOR_BTN_TEXT_BG = #FFFFC8;
final color COLOR_BTN_STROKE = #C8B4FF;
final color COLOR_PANEL_BG = #19192DDC;
final color COLOR_PANEL_TEXT = #C8C8FF;
final color COLOR_STATS_LABEL = #B4B4DC;
final color COLOR_STATS_VALUE = #DCDCFF;
final color COLOR_HOVER_TEXT = #64C8FF;
final color COLOR_EXPLORED = #C864FF64;
final color COLOR_FRONTIER = #FF963C78;
final color COLOR_SLIDER_BG = #323246;
final color COLOR_SLIDER_FILL = #00C8C8;
final color COLOR_SLIDER_HANDLE = #FFFF64;
final color COLOR_SLIDER_LABEL = #B4B4FF;

int panelX, panelWidth;
ArrayList<UIButton> buttons = new ArrayList<UIButton>();

String noSolutionMsg = "";
int msgStartTime = 0;

Slider gridColsSlider, gridRowsSlider;

// 对比模式
boolean compareMode = false;
HashMap<Algorithm, PathRecord> historyPaths = new HashMap<Algorithm, PathRecord>();

// Popup
String popupMessage = "";
boolean popupVisible = false;
int popupStartTime = 0;

// ----- Inner Classes -----
class MapPoint {
  int x, y, id;
  MapPoint(int x, int y, int id) { this.x = x; this.y = y; this.id = id; }
}

class PathRecord {
  ArrayList<Node> path;
  int visitedCount;
  int pathLength;
  int cpuCycles;
  PathRecord(ArrayList<Node> p, int v, int pl, int cpu) {
    path = new ArrayList<Node>();
    for (Node n : p) path.add(new Node(n.x, n.y));
    visitedCount = v;
    pathLength = pl;
    cpuCycles = cpu;
  }
}

class UIButton {
  int x, y, w, h;
  String label, id;
  boolean hovered = false;
  UIButton(int x, int y, int w, int h, String label, String id) {
    this.x = x; this.y = y; this.w = w; this.h = h;
    this.label = label; this.id = id;
  }
  boolean isOver(int mx, int my) { return mx >= x && mx <= x + w && my >= y && my <= y + h; }
  void draw() {
    hovered = isOver(mouseX, mouseY);
    fill(hovered ? COLOR_BTN_HOVER : COLOR_BTN_NORMAL);
    stroke(COLOR_BTN_STROKE);
    rect(x, y, w, h, 5);
    fill(COLOR_BTN_TEXT_BG);
    textAlign(CENTER, CENTER);
    textSize(12);
    text(label, x + w / 2, y + h / 2);
  }
}

class Node implements Comparable<Node> {
  int x, y, g, h;
  Node parent;
  Node(int x, int y) { this.x = x; this.y = y; g = Integer.MAX_VALUE; h = 0; parent = null; }
  int f() { return g + h; }
  public int compareTo(Node other) { return this.f() - other.f(); }
  public boolean equals(Object o) {
    if (!(o instanceof Node)) return false;
    Node n = (Node) o;
    return this.x == n.x && this.y == n.y;
  }
  public int hashCode() { return x * 31 + y; }
}

class Slider {
  int x, y, w, h;
  float minVal, maxVal, value;
  boolean dragging = false;
  String label;
  Slider(int x, int y, int w, int h, float minVal, float maxVal, float initVal, String label) {
    this.x = x; this.y = y; this.w = w; this.h = h;
    this.minVal = minVal; this.maxVal = maxVal;
    this.value = constrain(initVal, minVal, maxVal);
    this.label = label;
  }
  void draw() {
    fill(COLOR_SLIDER_BG); stroke(100, 100, 150); rect(x, y, w, h, 5);
    float fillW = map(value, minVal, maxVal, 0, w);
    fill(COLOR_SLIDER_FILL); noStroke(); rect(x, y, fillW, h, 5);
    float handleX = x + fillW;
    fill(COLOR_SLIDER_HANDLE); ellipse(handleX, y + h/2, 10, 10);
    fill(COLOR_SLIDER_LABEL); textAlign(LEFT, CENTER); textSize(12); text(label + ": " + (int)value, x, y + h + 8);
  }
  boolean overHandle(int mx, int my) {
    float fillW = map(value, minVal, maxVal, 0, w);
    float handleX = x + fillW;
    return dist(mx, my, handleX, y + h/2) < 10;
  }
  void setFromMouse(int mx) {
    value = map(constrain(mx, x, x + w), x, x + w, minVal, maxVal);
    value = constrain(value, minVal, maxVal);
  }
}

// ----- Setup -----
public void settings() { size(1060, 900); }

public void setup() {
  surface.setTitle("AI Pathfinding Arena - Multi Start/Goal");
  textFont(createFont("Arial", 14));
  updateLayout();
  grid = new int[gridRows][gridCols];
  resetGrid();
  
  // 初始只有一对起点和终点
  agents.add(new MapPoint(5, 5, nextAgentId++));
  goals.add(new MapPoint(gridCols - 6, gridRows - 6, nextGoalId++));
  
  createButtons();
  updateButtonLabels();
}

void updateLayout() {
  gridOffsetX = 20;
  gridOffsetY = 20;
  panelWidth = 180;
  panelX = width - panelWidth - 10;
}

void resetGrid() {
  grid = new int[gridRows][gridCols];
  for (int r = 0; r < gridRows; r++) Arrays.fill(grid[r], EMPTY);
}

// 获取某个格子是否有起点
boolean hasStartAt(int x, int y) {
  for (MapPoint a : agents) {
    if (a.x == x && a.y == y) return true;
  }
  return false;
}

// 获取某个格子是否有终点
boolean hasGoalAt(int x, int y) {
  for (MapPoint g : goals) {
    if (g.x == x && g.y == y) return true;
  }
  return false;
}

// 获取某个格子的所有起点ID列表
ArrayList<Integer> getStartIdsAt(int x, int y) {
  ArrayList<Integer> ids = new ArrayList<Integer>();
  for (MapPoint a : agents) {
    if (a.x == x && a.y == y) ids.add(a.id);
  }
  Collections.sort(ids);
  return ids;
}

// 获取某个格子的所有终点ID列表
ArrayList<Integer> getGoalIdsAt(int x, int y) {
  ArrayList<Integer> ids = new ArrayList<Integer>();
  for (MapPoint g : goals) {
    if (g.x == x && g.y == y) ids.add(g.id);
  }
  Collections.sort(ids);
  return ids;
}

boolean canAnyStartReachAnyGoal() {
  if (agents.isEmpty() || goals.isEmpty()) return false;
  boolean[][] reachable = new boolean[gridRows][gridCols];
  ArrayDeque<int[]> queue = new ArrayDeque<int[]>();
  for (MapPoint a : agents) {
    if (grid[a.y][a.x] != OBSTACLE) {
      reachable[a.y][a.x] = true;
      queue.add(new int[]{a.x, a.y});
    }
  }
  int[] dx = {-1,1,0,0}, dy = {0,0,-1,1};
  while (!queue.isEmpty()) {
    int[] cur = queue.poll();
    for (int i=0; i<4; i++) {
      int nx = cur[0]+dx[i], ny = cur[1]+dy[i];
      if (nx>=0 && nx<gridCols && ny>=0 && ny<gridRows && !reachable[ny][nx] && grid[ny][nx] != OBSTACLE) {
        reachable[ny][nx] = true;
        queue.add(new int[]{nx, ny});
      }
    }
  }
  for (MapPoint g : goals) {
    if (g.x>=0 && g.x<gridCols && g.y>=0 && g.y<gridRows && reachable[g.y][g.x]) return true;
  }
  return false;
}

boolean isStartBlocked() {
  if (agents.isEmpty()) { showNoSolution("ERROR: No start point!"); return true; }
  if (goals.isEmpty()) { showNoSolution("ERROR: No goal point!"); return true; }
  if (!canAnyStartReachAnyGoal()) {
    showNoSolution("NO SOLUTION! No path from any start to any goal!");
    return true;
  }
  return false;
}

void showNoSolution(String msg) { 
  noSolutionMsg = msg; 
  msgStartTime = millis(); 
  println(msg);
  showResultPopup(false, msg, 0, 0, 0);
}

void showResultPopup(boolean success, String message, int visited, int pathLen, int cpu) {
  String title = success ? "✓ Path Found!" : "✗ Search Failed";
  String stats = success ? "Visited: " + visited + " | Path Length: " + pathLen + " | CPU Cycles: " + cpu : message;
  if (compareMode && success && historyPaths.containsKey(currentAlgo)) {
    PathRecord prev = historyPaths.get(currentAlgo);
    stats += "\n\n[Compare] Previous " + currentAlgo + ":\n  Visited: " + prev.visitedCount + " | Path Len: " + prev.pathLength + " | CPU: " + prev.cpuCycles;
  }
  popupMessage = title + "\n" + stats;
  popupVisible = true;
  popupStartTime = millis();
}

void drawPopup() {
  if (!popupVisible) return;
  float alpha = 1.0;
  if (millis() - popupStartTime > 3000) {
    popupVisible = false;
    return;
  }
  if (millis() - popupStartTime > 2500) {
    alpha = map(millis() - popupStartTime, 2500, 3000, 1, 0);
  }
  pushStyle();
  int w = 450, h = 130;
  int cx = width/2, cy = height/2;
  fill(0, 0, 0, 200 * alpha);
  noStroke();
  rect(cx - w/2, cy - h/2, w, h, 15);
  stroke(100, 255, 100, 255 * alpha);
  strokeWeight(2);
  noFill();
  rect(cx - w/2, cy - h/2, w, h, 15);
  fill(255, 255, 200, 255 * alpha);
  textAlign(CENTER, CENTER);
  textSize(14);
  String[] lines = popupMessage.split("\n");
  for (int i=0; i<lines.length; i++) {
    text(lines[i], cx, cy - 30 + i * 25);
  }
  popStyle();
}

void drawNoSolutionMsg() {
  if (msgStartTime > 0 && millis() - msgStartTime < 3000) {
    float alpha = map(millis() - msgStartTime, 0, 3000, 255, 0);
    pushStyle();
    int w = 400, h = 50, cx = width / 2, cy = height / 2 - 150;
    fill(0, 0, 0, alpha * 0.85); noStroke(); rect(cx - w/2, cy - h/2, w, h, 10);
    stroke(255, 0, 0, alpha); strokeWeight(2); noFill(); rect(cx - w/2, cy - h/2, w, h, 10);
    fill(255, 80, 80, alpha); textAlign(CENTER, CENTER); textSize(16); text(noSolutionMsg, cx, cy);
    popStyle();
  }
}

void resizeGrid(int newCols, int newRows) {
  if (newCols == gridCols && newRows == gridRows) return;
  gridCols = constrain(newCols, 10, 40);
  gridRows = constrain(newRows, 10, 40);
  int[][] newGrid = new int[gridRows][gridCols];
  for (int r = 0; r < min(gridRows, grid.length); r++) {
    for (int c = 0; c < min(gridCols, grid[0].length); c++) {
      newGrid[r][c] = grid[r][c];
    }
  }
  for (int r = 0; r < gridRows; r++) {
    for (int c = 0; c < gridCols; c++) {
      if (newGrid[r][c] != OBSTACLE && newGrid[r][c] != GRASS && newGrid[r][c] != DESERT) {
        newGrid[r][c] = EMPTY;
      }
    }
  }
  grid = newGrid;
  
  for (int i = agents.size()-1; i>=0; i--) {
    MapPoint p = agents.get(i);
    if (p.x >= gridCols || p.y >= gridRows) agents.remove(i);
  }
  for (int i = goals.size()-1; i>=0; i--) {
    MapPoint p = goals.get(i);
    if (p.x >= gridCols || p.y >= gridRows) goals.remove(i);
  }
  if (agents.isEmpty() && gridCols > 1 && gridRows > 1) agents.add(new MapPoint(1, 1, nextAgentId++));
  if (goals.isEmpty() && gridCols > 2 && gridRows > 2) goals.add(new MapPoint(gridCols-2, gridRows-2, nextGoalId++));
  
  updateButtonLabels();
  resetSearch();
  clearHistoryPaths();
}

void createButtons() {
  buttons.clear();
  int yBase = 60, btnH = 30, btnW = panelWidth - 20, x = panelX + 10;
  buttons.add(new UIButton(x, yBase, btnW, btnH, "Algo: A*", "ALGO_ASTAR"));
  buttons.add(new UIButton(x, yBase + 40, btnW, btnH, "Next Algo", "ALGO_NEXT"));
  buttons.add(new UIButton(x, yBase + 80, btnW, btnH, "Compare: OFF", "COMPARE_MODE"));
  buttons.add(new UIButton(x, yBase + 130, btnW/2-2, btnH, "-Speed", "SPEED_DOWN"));
  buttons.add(new UIButton(x + btnW/2+2, yBase + 130, btnW/2-2, btnH, "+Speed", "SPEED_UP"));
  speedSlider = new Slider(x, yBase + 165, btnW, 15, 1, 20, speed, "Speed");
  buttons.add(new UIButton(x, yBase + 200, btnW, btnH, "Tool: Agent", "TOOL_AGENT"));
  buttons.add(new UIButton(x, yBase + 240, btnW, btnH, "Tool: Goal", "TOOL_GOAL"));
  buttons.add(new UIButton(x, yBase + 280, btnW, btnH, "Tool: Move", "TOOL_MOVE"));
  terrainDropdownY = yBase + 320;
  buttons.add(new UIButton(x, terrainDropdownY, btnW, btnH, "Tool: Obstacle ▼", "TOOL_OBSTACLE"));
  buttons.add(new UIButton(x, yBase + 460, btnW, btnH, "Start", "RUN_START"));
  buttons.add(new UIButton(x, yBase + 500, btnW, btnH, "Pause", "RUN_PAUSE"));
  buttons.add(new UIButton(x, yBase + 540, btnW, btnH, "Step", "RUN_STEP"));
  buttons.add(new UIButton(x, yBase + 580, btnW, btnH, "Reset", "RUN_RESET"));
  buttons.add(new UIButton(x, yBase + 620, btnW, btnH, "Clear All", "RUN_CLEAR"));
  gridColsSlider = new Slider(x, yBase + 660, btnW, 15, 10, 40, gridCols, "Cols");
  gridRowsSlider = new Slider(x, yBase + 690, btnW, 15, 10, 40, gridRows, "Rows");
}

void updateButtonLabels() {
  for (UIButton b : buttons) {
    if (b.id.equals("ALGO_ASTAR")) b.label = "Algo: " + currentAlgo.toString();
    else if (b.id.equals("TOOL_AGENT")) b.label = "Tool: " + (currentTool == Tool.ADD_AGENT ? "*Agent" : "Agent");
    else if (b.id.equals("TOOL_GOAL")) b.label = "Tool: " + (currentTool == Tool.ADD_GOAL ? "*Goal" : "Goal");
    else if (b.id.equals("TOOL_MOVE")) b.label = "Tool: " + (currentTool == Tool.MOVE_POINT ? "*Move" : "Move");
    else if (b.id.equals("TOOL_OBSTACLE")) {
      String terrainName = "";
      if (currentTool == Tool.DRAW_OBSTACLE) terrainName = "*Obstacle";
      else if (currentTool == Tool.DRAW_GRASS) terrainName = "*Grass";
      else if (currentTool == Tool.DRAW_DESERT) terrainName = "*Desert";
      else terrainName = "Tool: Obstacle";
      b.label = terrainName + (terrainDropdownExpanded ? " ▲" : " ▼");
    }
    else if (b.id.equals("COMPARE_MODE")) b.label = "Compare: " + (compareMode ? "ON" : "OFF");
  }
}

// ----- Drawing -----
public void draw() {
  background(20, 20, 40);
  drawArena();
  drawInstructions();
  drawPanel();
  drawNoSolutionMsg();
  drawPopup();
  if (running && !paused && !algorithmFinished) {
    for (int i = 0; i < speed; i++)
      if (!algorithmStep()) { 
        algorithmFinished = true; 
        running = false;
        if (pathFound) {
          showResultPopup(true, "Path found!", visitedCount, pathLength, cpuCycles);
          if (compareMode) savePathToHistory();
        } else if (!pathFound && algorithmFinished) {
          showResultPopup(false, "No path exists from any start to any goal!", visitedCount, 0, cpuCycles);
        }
        break;
      }
  }
}

void drawArena() {
  pushMatrix();
  translate(gridOffsetX, gridOffsetY);
  
  // 绘制地形
  for (int r = 0; r < gridRows; r++) {
    for (int c = 0; c < gridCols; c++) {
      int x = c * CELL_SIZE, y = r * CELL_SIZE;
      switch (grid[r][c]) {
        case OBSTACLE: fill(COLOR_OBSTACLE); break;
        case GRASS: fill(COLOR_GRASS); break;
        case DESERT: fill(COLOR_DESERT); break;
        default: fill(COLOR_EMPTY);
      }
      stroke(70, 70, 90); rect(x, y, CELL_SIZE, CELL_SIZE);
    }
  }
  
  drawSearchVisuals();
  
  // 绘制多起点（同一格子可能多个）
  for (int r = 0; r < gridRows; r++) {
    for (int c = 0; c < gridCols; c++) {
      ArrayList<Integer> startIds = getStartIdsAt(c, r);
      ArrayList<Integer> goalIds = getGoalIdsAt(c, r);
      
      float cx = c * CELL_SIZE + CELL_SIZE/2;
      float cy = r * CELL_SIZE + CELL_SIZE/2;
      
      // 如果同一格子既有起点又有终点，分区域显示
      if (!startIds.isEmpty() && !goalIds.isEmpty()) {
        // 上半圆显示起点，下半圆显示终点
        fill(COLOR_START);
        arc(cx, cy, CELL_SIZE * 0.8, CELL_SIZE * 0.8, -PI/2, PI/2, PIE);
        fill(COLOR_GOAL);
        arc(cx, cy, CELL_SIZE * 0.8, CELL_SIZE * 0.8, PI/2, 3*PI/2, PIE);
        fill(255);
        textSize(8);
        textAlign(CENTER, CENTER);
        String startStr = "S";
        for (int id : startIds) startStr += id + ",";
        startStr = startStr.substring(0, startStr.length()-1);
        text(startStr, cx, cy - 5);
        String goalStr = "G";
        for (int id : goalIds) goalStr += id + ",";
        goalStr = goalStr.substring(0, goalStr.length()-1);
        text(goalStr, cx, cy + 5);
      }
      // 只有起点
      else if (!startIds.isEmpty()) {
        color startColor = (startIds.size() == 1 && startIds.get(0) == 1) ? COLOR_START : COLOR_START_MULTI;
        fill(startColor);
        ellipse(cx, cy, CELL_SIZE * 0.8, CELL_SIZE * 0.8);
        fill(255);
        textSize(9);
        textAlign(CENTER, CENTER);
        String label = "S";
        for (int id : startIds) label += id;
        text(label, cx, cy);
      }
      // 只有终点
      else if (!goalIds.isEmpty()) {
        color goalColor = (goalIds.size() == 1 && goalIds.get(0) == 1) ? COLOR_GOAL : COLOR_GOAL_MULTI;
        fill(goalColor);
        ellipse(cx, cy, CELL_SIZE * 0.8, CELL_SIZE * 0.8);
        fill(255);
        textSize(9);
        textAlign(CENTER, CENTER);
        String label = "G";
        for (int id : goalIds) label += id;
        text(label, cx, cy);
      }
    }
  }
  
  popMatrix();
}

void drawSearchVisuals() {
  noStroke();
  fill(COLOR_EXPLORED);
  for (Node n : closedList) {
    if (grid[n.y][n.x] != OBSTACLE) {
      rect(n.x * CELL_SIZE + 1, n.y * CELL_SIZE + 1, CELL_SIZE - 2, CELL_SIZE - 2);
    }
  }
  fill(COLOR_FRONTIER);
  for (Node n : openList) {
    if (grid[n.y][n.x] != OBSTACLE) {
      rect(n.x * CELL_SIZE + 1, n.y * CELL_SIZE + 1, CELL_SIZE - 2, CELL_SIZE - 2);
    }
  }
  
  if (compareMode) {
    for (Map.Entry<Algorithm, PathRecord> entry : historyPaths.entrySet()) {
      Algorithm algo = entry.getKey();
      PathRecord rec = entry.getValue();
      if (rec.path == null || rec.path.size() < 2) continue;
      drawPathWithBloom(rec.path, getAlgoColor(algo, 255), 3);
    }
  }
  if (pathFound && finalPath.size() > 1) {
    drawPathWithBloom(finalPath, getAlgoColor(currentAlgo, 255), 4);
  }
}

void drawPathWithBloom(ArrayList<Node> path, color pathColor, int coreWeight) {
  if (path == null || path.size() < 2) return;
  blendMode(ADD);
  stroke(red(pathColor), green(pathColor), blue(pathColor), 20);
  strokeWeight(coreWeight + 16);
  noFill();
  beginShape();
  for (Node n : path) vertex(n.x * CELL_SIZE + CELL_SIZE/2, n.y * CELL_SIZE + CELL_SIZE/2);
  endShape();
  stroke(red(pathColor), green(pathColor), blue(pathColor), 40);
  strokeWeight(coreWeight + 10);
  beginShape();
  for (Node n : path) vertex(n.x * CELL_SIZE + CELL_SIZE/2, n.y * CELL_SIZE + CELL_SIZE/2);
  endShape();
  stroke(red(pathColor), green(pathColor), blue(pathColor), 60);
  strokeWeight(coreWeight + 6);
  beginShape();
  for (Node n : path) vertex(n.x * CELL_SIZE + CELL_SIZE/2, n.y * CELL_SIZE + CELL_SIZE/2);
  endShape();
  blendMode(BLEND);
  stroke(pathColor);
  strokeWeight(coreWeight);
  beginShape();
  for (Node n : path) vertex(n.x * CELL_SIZE + CELL_SIZE/2, n.y * CELL_SIZE + CELL_SIZE/2);
  endShape();
  strokeWeight(1);
}

color getAlgoColor(Algorithm algo, int alpha) {
  if (algo == Algorithm.BFS) return color(#FF6B6B, alpha);
  else if (algo == Algorithm.DIJKSTRA) return color(#00E5FF, alpha);
  else return color(#FFEB3B, alpha);
}

void drawInstructions() {
  int arenaBottom = gridOffsetY + gridRows * CELL_SIZE;
  int y = arenaBottom + 15;
  int xLeft = gridOffsetX;
  fill(150, 150, 200);
  textAlign(LEFT, TOP);
  textSize(12);
  text("Agent: Left-click → add start, Right-click → remove start", xLeft, y);
  text("Goal: Left-click → add goal, Right-click → remove goal", xLeft, y+18);
  text("Move: Drag start/goal to reposition (同一格子可放多个)", xLeft, y+36);
  text("Obstacle/Grass/Desert: Left-click to draw, Right-click to erase", xLeft, y+54);
  fill(COLOR_START_MULTI);
  text("● 多个起点: 浅青色 | ● 多个终点: 浅橙色", xLeft, y+72);
}

void drawPanel() {
  fill(25, 25, 45, 220); noStroke(); rect(panelX, 0, panelWidth, height);
  fill(COLOR_PANEL_TEXT); textAlign(LEFT, TOP); textSize(16); text("Control Panel", panelX+10, 20);
  for (UIButton b : buttons) b.draw();
  
  if (terrainDropdownExpanded) {
    int btnX = panelX + 10;
    int btnW = panelWidth - 20;
    int subBtnH = terrainDropdownBtnH;
    boolean isObstacleHovered = mouseX >= btnX && mouseX <= btnX+btnW && mouseY >= terrainDropdownY+subBtnH && mouseY <= terrainDropdownY+subBtnH*2;
    boolean isGrassHovered = mouseX >= btnX && mouseX <= btnX+btnW && mouseY >= terrainDropdownY+subBtnH*2 && mouseY <= terrainDropdownY+subBtnH*3;
    boolean isDesertHovered = mouseX >= btnX && mouseX <= btnX+btnW && mouseY >= terrainDropdownY+subBtnH*3 && mouseY <= terrainDropdownY+subBtnH*4;
    fill(isObstacleHovered ? COLOR_BTN_HOVER : COLOR_BTN_NORMAL);
    stroke(COLOR_BTN_STROKE); rect(btnX, terrainDropdownY+subBtnH, btnW, subBtnH, 5);
    fill(COLOR_BTN_TEXT_BG); textAlign(CENTER, CENTER); textSize(12); text("Obstacle", btnX+btnW/2, terrainDropdownY+subBtnH+subBtnH/2);
    fill(isGrassHovered ? COLOR_BTN_HOVER : COLOR_BTN_NORMAL);
    rect(btnX, terrainDropdownY+subBtnH*2, btnW, subBtnH, 5);
    fill(COLOR_BTN_TEXT_BG); text("Grass", btnX+btnW/2, terrainDropdownY+subBtnH*2+subBtnH/2);
    fill(isDesertHovered ? COLOR_BTN_HOVER : COLOR_BTN_NORMAL);
    rect(btnX, terrainDropdownY+subBtnH*3, btnW, subBtnH, 5);
    fill(COLOR_BTN_TEXT_BG); text("Desert", btnX+btnW/2, terrainDropdownY+subBtnH*3+subBtnH/2);
  }
  
  speedSlider.draw();
  gridColsSlider.draw();
  gridRowsSlider.draw();
  
  int statsY = gridRowsSlider.y + gridRowsSlider.h + 25;
  int statsX = panelX + 10;
  pushStyle();
  textAlign(LEFT, TOP);
  textSize(12);
  fill(COLOR_STATS_LABEL);
  text("Statistics:", statsX, statsY);
  fill(COLOR_STATS_VALUE);
  text("Visited: " + visitedCount, statsX, statsY+20);
  text("Path len: " + pathLength, statsX, statsY+40);
  text("CPU cycles: " + cpuCycles, statsX, statsY+60);
  text("Starts: " + agents.size() + " | Goals: " + goals.size(), statsX, statsY+80);
  if (compareMode) {
    fill(200, 200, 100);
    text("Compare Mode ON", statsX, statsY+100);
  }
  popStyle();
  
  if (mouseX >= gridOffsetX && mouseX < gridOffsetX+gridCols*CELL_SIZE &&
      mouseY >= gridOffsetY && mouseY < gridOffsetY+gridRows*CELL_SIZE) {
    int mx = (mouseX - gridOffsetX)/CELL_SIZE;
    int my = (mouseY - gridOffsetY)/CELL_SIZE;
    pushStyle();
    fill(COLOR_HOVER_TEXT);
    textAlign(RIGHT, BOTTOM);
    String info = "Grid: ("+mx+", "+my+")";
    ArrayList<Integer> sIds = getStartIdsAt(mx, my);
    ArrayList<Integer> gIds = getGoalIdsAt(mx, my);
    if (!sIds.isEmpty()) info += " | Starts: S" + sIds.toString().replace("[", "").replace("]", "");
    if (!gIds.isEmpty()) info += " | Goals: G" + gIds.toString().replace("[", "").replace("]", "");
    text(info, width-10, height-10);
    popStyle();
  }
}

// ----- Mouse Interaction -----
public void mousePressed() {
  boolean isSearching = (running && !paused && !algorithmFinished);
  boolean isButtonClick = false;
  for (UIButton b : buttons) if (b.isOver(mouseX, mouseY)) { isButtonClick = true; break; }
  
  if (speedSlider.overHandle(mouseX, mouseY)) { speedSlider.dragging = true; return; }
  if (gridColsSlider.overHandle(mouseX, mouseY)) { gridColsSlider.dragging = true; return; }
  if (gridRowsSlider.overHandle(mouseX, mouseY)) { gridRowsSlider.dragging = true; return; }
  
  if (isSearching && !isButtonClick && mouseX < panelX) return;
  
  for (UIButton b : buttons) {
    if (b.isOver(mouseX, mouseY)) { handleButton(b.id); return; }
  }
  
  if (terrainDropdownExpanded) {
    int btnX = panelX+10, btnW = panelWidth-20;
    if (mouseX >= btnX && mouseX <= btnX+btnW && mouseY >= terrainDropdownY+terrainDropdownBtnH && mouseY <= terrainDropdownY+terrainDropdownBtnH*2) {
      currentTool = Tool.DRAW_OBSTACLE; terrainDropdownExpanded = false; updateButtonLabels(); return;
    }
    if (mouseX >= btnX && mouseX <= btnX+btnW && mouseY >= terrainDropdownY+terrainDropdownBtnH*2 && mouseY <= terrainDropdownY+terrainDropdownBtnH*3) {
      currentTool = Tool.DRAW_GRASS; terrainDropdownExpanded = false; updateButtonLabels(); return;
    }
    if (mouseX >= btnX && mouseX <= btnX+btnW && mouseY >= terrainDropdownY+terrainDropdownBtnH*3 && mouseY <= terrainDropdownY+terrainDropdownBtnH*4) {
      currentTool = Tool.DRAW_DESERT; terrainDropdownExpanded = false; updateButtonLabels(); return;
    }
    terrainDropdownExpanded = false; updateButtonLabels();
  }
  
  if (mouseX >= gridOffsetX && mouseX < gridOffsetX+gridCols*CELL_SIZE &&
      mouseY >= gridOffsetY && mouseY < gridOffsetY+gridRows*CELL_SIZE) {
    int cx = (mouseX - gridOffsetX)/CELL_SIZE;
    int cy = (mouseY - gridOffsetY)/CELL_SIZE;
    if (pathFound) return;
    
    if (currentTool == Tool.MOVE_POINT) {
      for (MapPoint a : agents) {
        if (a.x == cx && a.y == cy) {
          currentDragMode = DragMode.MOVE_AGENT;
          draggedPoint = a;
          return;
        }
      }
      for (MapPoint g : goals) {
        if (g.x == cx && g.y == cy) {
          currentDragMode = DragMode.MOVE_GOAL;
          draggedPoint = g;
          return;
        }
      }
      return;
    }
    
    if (currentTool == Tool.ADD_AGENT) {
      if (mouseButton == LEFT) {
        if (grid[cy][cx] != OBSTACLE) {
          agents.add(new MapPoint(cx, cy, nextAgentId++));
        }
      } else if (mouseButton == RIGHT) {
        removeAgentAt(cx, cy);
      }
    }
    else if (currentTool == Tool.ADD_GOAL) {
      if (mouseButton == LEFT) {
        if (grid[cy][cx] != OBSTACLE) {
          goals.add(new MapPoint(cx, cy, nextGoalId++));
        }
      } else if (mouseButton == RIGHT) {
        removeGoalAt(cx, cy);
      }
    }
    else if (currentTool == Tool.DRAW_OBSTACLE) {
      if (mouseButton == LEFT) {
        if (!hasStartAt(cx, cy) && !hasGoalAt(cx, cy)) {
          grid[cy][cx] = OBSTACLE;
        }
      } else if (mouseButton == RIGHT) {
        if (grid[cy][cx] == OBSTACLE) grid[cy][cx] = EMPTY;
      }
    }
    else if (currentTool == Tool.DRAW_GRASS) {
      if (mouseButton == LEFT) {
        if (!hasStartAt(cx, cy) && !hasGoalAt(cx, cy)) {
          grid[cy][cx] = GRASS;
        }
      } else if (mouseButton == RIGHT) {
        if (grid[cy][cx] == GRASS) grid[cy][cx] = EMPTY;
      }
    }
    else if (currentTool == Tool.DRAW_DESERT) {
      if (mouseButton == LEFT) {
        if (!hasStartAt(cx, cy) && !hasGoalAt(cx, cy)) {
          grid[cy][cx] = DESERT;
        }
      } else if (mouseButton == RIGHT) {
        if (grid[cy][cx] == DESERT) grid[cy][cx] = EMPTY;
      }
    }
  }
}

public void mouseDragged() {
  if (speedSlider.dragging) { speedSlider.setFromMouse(mouseX); speed = (int)speedSlider.value; return; }
  if (gridColsSlider.dragging) {
    gridColsSlider.setFromMouse(mouseX);
    int newCols = (int)gridColsSlider.value;
    if (newCols != gridCols) resizeGrid(newCols, gridRows);
    return;
  }
  if (gridRowsSlider.dragging) {
    gridRowsSlider.setFromMouse(mouseX);
    int newRows = (int)gridRowsSlider.value;
    if (newRows != gridRows) resizeGrid(gridCols, newRows);
    return;
  }
  
  if (running && !paused && !algorithmFinished) return;
  if (pathFound) return;
  
  if (currentDragMode != DragMode.NONE && draggedPoint != null) {
    if (mouseX >= gridOffsetX && mouseX < gridOffsetX + gridCols * CELL_SIZE &&
        mouseY >= gridOffsetY && mouseY < gridOffsetY + gridRows * CELL_SIZE) {
      int newX = (mouseX - gridOffsetX) / CELL_SIZE;
      int newY = (mouseY - gridOffsetY) / CELL_SIZE;
      if (grid[newY][newX] != OBSTACLE) {
        draggedPoint.x = newX;
        draggedPoint.y = newY;
        resetSearch();
      }
    }
    return;
  }
  
  if (currentTool == Tool.DRAW_OBSTACLE || currentTool == Tool.DRAW_GRASS || currentTool == Tool.DRAW_DESERT) {
    if (mouseX >= gridOffsetX && mouseX < gridOffsetX + gridCols * CELL_SIZE &&
        mouseY >= gridOffsetY && mouseY < gridOffsetY + gridRows * CELL_SIZE) {
      int cx = (mouseX - gridOffsetX) / CELL_SIZE;
      int cy = (mouseY - gridOffsetY) / CELL_SIZE;
      if (hasStartAt(cx, cy) || hasGoalAt(cx, cy)) return;
      
      int newVal = EMPTY;
      if (currentTool == Tool.DRAW_OBSTACLE) newVal = OBSTACLE;
      else if (currentTool == Tool.DRAW_GRASS) newVal = GRASS;
      else if (currentTool == Tool.DRAW_DESERT) newVal = DESERT;
      
      if (mouseButton == LEFT) {
        grid[cy][cx] = newVal;
      } else if (mouseButton == RIGHT) {
        if (grid[cy][cx] == newVal) grid[cy][cx] = EMPTY;
      }
    }
  }
}

public void mouseReleased() {
  speedSlider.dragging = false;
  gridColsSlider.dragging = false;
  gridRowsSlider.dragging = false;
  currentDragMode = DragMode.NONE;
  draggedPoint = null;
}

void removeAgentAt(int cx, int cy) {
  for (int i = agents.size() - 1; i >= 0; i--) {
    if (agents.get(i).x == cx && agents.get(i).y == cy) {
      agents.remove(i);
      break;
    }
  }
}

void removeGoalAt(int cx, int cy) {
  for (int i = goals.size() - 1; i >= 0; i--) {
    if (goals.get(i).x == cx && goals.get(i).y == cy) {
      goals.remove(i);
      break;
    }
  }
}

void savePathToHistory() {
  if (!pathFound) return;
  historyPaths.put(currentAlgo, new PathRecord(finalPath, visitedCount, pathLength, cpuCycles));
}

void clearHistoryPaths() {
  historyPaths.clear();
}

void handleButton(String id) {
  switch (id) {
    case "COMPARE_MODE":
      compareMode = !compareMode;
      if (!compareMode) clearHistoryPaths();
      updateButtonLabels();
      break;
    case "ALGO_NEXT":
      if (compareMode && pathFound) savePathToHistory();
      if (currentAlgo == Algorithm.BFS) currentAlgo = Algorithm.DIJKSTRA;
      else if (currentAlgo == Algorithm.DIJKSTRA) currentAlgo = Algorithm.ASTAR;
      else currentAlgo = Algorithm.BFS;
      resetSearch();
      updateButtonLabels();
      break;
    case "SPEED_UP": speed = min(speed + 1, 20); speedSlider.value = speed; break;
    case "SPEED_DOWN": speed = max(speed - 1, 1); speedSlider.value = speed; break;
    case "TOOL_AGENT": currentTool = Tool.ADD_AGENT; break;
    case "TOOL_GOAL": currentTool = Tool.ADD_GOAL; break;
    case "TOOL_MOVE": currentTool = Tool.MOVE_POINT; break;
    case "TOOL_OBSTACLE":
      terrainDropdownExpanded = !terrainDropdownExpanded;
      updateButtonLabels();
      break;
    case "RUN_START":
      if (agents.size() > 0 && goals.size() > 0) {
        if (isStartBlocked()) return;
        resetSearch();
        startNode = new Node(agents.get(0).x, agents.get(0).y);
        goalNode = new Node(goals.get(0).x, goals.get(0).y);
        if (currentAlgo == Algorithm.BFS) { initBFS(); running = true; paused = false; algorithmFinished = false; }
        else if (currentAlgo == Algorithm.DIJKSTRA) { initDijkstra(); running = true; paused = false; algorithmFinished = false; }
        else if (currentAlgo == Algorithm.ASTAR) { initAStar(); running = true; paused = false; algorithmFinished = false; }
      } break;
    case "RUN_PAUSE": paused = !paused; break;
    case "RUN_STEP":
      if (!algorithmFinished) {
        if ((bfsQueue == null && dijkstraQueue == null && aStarQueue == null) && agents.size() > 0 && goals.size() > 0) {
          if (isStartBlocked()) return;
          startNode = new Node(agents.get(0).x, agents.get(0).y);
          goalNode = new Node(goals.get(0).x, goals.get(0).y);
          if (currentAlgo == Algorithm.BFS) initBFS();
          else if (currentAlgo == Algorithm.DIJKSTRA) initDijkstra();
          else if (currentAlgo == Algorithm.ASTAR) initAStar();
          algorithmFinished = false;
        }
        algorithmStep(); paused = true; running = false;
      } break;
    case "RUN_RESET":
      resetSearch();
      running = false; paused = false; algorithmFinished = false;
      clearHistoryPaths();
      break;
    case "RUN_CLEAR":
      agents.clear(); goals.clear();
      nextAgentId = 1; nextGoalId = 1;
      for (int r = 0; r < gridRows; r++) {
        for (int c = 0; c < gridCols; c++) {
          if (grid[r][c] != OBSTACLE && grid[r][c] != GRASS && grid[r][c] != DESERT) {
            grid[r][c] = EMPTY;
          }
        }
      }
      resetSearch();
      running = false; paused = false; algorithmFinished = false;
      clearHistoryPaths();
      break;
  }
  updateButtonLabels();
}

void resetSearch() {
  openList.clear(); closedList.clear(); finalPath.clear();
  visitedCount = 0; pathLength = 0; cpuCycles = 0;
  pathFound = false; algorithmFinished = false;
  startNode = null; goalNode = null;
  bfsQueue = null; visited = null; aStarQueue = null;
}

// ========== Algorithms ==========
void initBFS() {
  bfsQueue = new ArrayDeque<Node>(); visited = new boolean[gridRows][gridCols];
  bfsQueue.add(startNode); visited[startNode.y][startNode.x] = true; openList.add(startNode);
  visitedCount = 0; pathLength = 0; cpuCycles = 0; pathFound = false;
}

void initDijkstra() {
  dijkstraQueue = new PriorityQueue<>(); visited = new boolean[gridRows][gridCols]; dist = new int[gridRows][gridCols];
  for (int i = 0; i < gridRows; i++) Arrays.fill(dist[i], Integer.MAX_VALUE);
  startNode.g = 0; dist[startNode.y][startNode.x] = 0; dijkstraQueue.add(startNode); openList.add(startNode);
  visitedCount = 0; pathLength = 0; cpuCycles = 0; pathFound = false;
}

void initAStar() {
  aStarQueue = new PriorityQueue<>(); visited = new boolean[gridRows][gridCols];
  startNode.g = 0; startNode.h = heuristic(startNode, goalNode); aStarQueue.add(startNode); openList.add(startNode);
  visitedCount = 0; cpuCycles = 0; pathFound = false;
}

int heuristic(Node a, Node b) { return Math.abs(a.x - b.x) + Math.abs(a.y - b.y); }

int getTerrainWeight(int cellType) {
  switch (cellType) {
    case GRASS: return WEIGHT_GRASS;
    case DESERT: return WEIGHT_DESERT;
    default: return WEIGHT_NORMAL;
  }
}

boolean algorithmStep() {
  if (currentAlgo == Algorithm.BFS) return stepBFS();
  else if (currentAlgo == Algorithm.DIJKSTRA) return stepDijkstra();
  else if (currentAlgo == Algorithm.ASTAR) return stepAStar();
  else { algorithmFinished = true; return false; }
}

boolean stepBFS() {
  if (bfsQueue == null || bfsQueue.isEmpty()) { algorithmFinished = true; running = false; return false; }
  Node current = bfsQueue.poll();
  openList.remove(current); closedList.add(current); visitedCount++;
  if (current.equals(goalNode)) { pathFound = true; algorithmFinished = true; running = false; reconstructPath(current); return false; }
  int[] dx = { -1, 1, 0, 0 }, dy = { 0, 0, -1, 1 };
  for (int i = 0; i < 4; i++) {
    int nx = current.x + dx[i], ny = current.y + dy[i];
    if (nx >= 0 && nx < gridCols && ny >= 0 && ny < gridRows && !visited[ny][nx] && grid[ny][nx] != OBSTACLE) {
      visited[ny][nx] = true; Node neighbor = new Node(nx, ny); neighbor.parent = current;
      bfsQueue.add(neighbor); openList.add(neighbor);
    }
  }
  cpuCycles++; return true;
}

boolean stepDijkstra() {
  if (dijkstraQueue == null || dijkstraQueue.isEmpty()) { algorithmFinished = true; running = false; return false; }
  Node current = dijkstraQueue.poll();
  if (visited[current.y][current.x]) return true;
  visited[current.y][current.x] = true; openList.remove(current); closedList.add(current); visitedCount++;
  if (current.equals(goalNode)) { pathFound = true; algorithmFinished = true; running = false; reconstructPath(current); return false; }
  int[] dx = { -1, 1, 0, 0 }, dy = { 0, 0, -1, 1 };
  for (int i = 0; i < 4; i++) {
    int nx = current.x + dx[i], ny = current.y + dy[i];
    if (nx >= 0 && nx < gridCols && ny >= 0 && ny < gridRows && !visited[ny][nx] && grid[ny][nx] != OBSTACLE) {
      int newDist = current.g + getTerrainWeight(grid[ny][nx]);
      if (newDist < dist[ny][nx]) {
        dist[ny][nx] = newDist; Node neighbor = new Node(nx, ny); neighbor.g = newDist; neighbor.parent = current;
        dijkstraQueue.add(neighbor); openList.add(neighbor);
      }
    }
  }
  cpuCycles++; return true;
}

boolean stepAStar() {
  if (aStarQueue == null || aStarQueue.isEmpty()) { algorithmFinished = true; running = false; return false; }
  Node current = aStarQueue.poll();
  if (visited[current.y][current.x]) return true;
  visited[current.y][current.x] = true; openList.remove(current); closedList.add(current); visitedCount++;
  if (current.equals(goalNode)) { pathFound = true; algorithmFinished = true; running = false; reconstructPath(current); return false; }
  int[] dx = { -1, 1, 0, 0 }, dy = { 0, 0, -1, 1 };
  for (int i = 0; i < 4; i++) {
    int nx = current.x + dx[i], ny = current.y + dy[i];
    if (nx < 0 || nx >= gridCols || ny < 0 || ny >= gridRows || grid[ny][nx] == OBSTACLE) continue;
    Node neighbor = new Node(nx, ny);
    neighbor.g = current.g + getTerrainWeight(grid[ny][nx]);
    neighbor.h = heuristic(neighbor, goalNode);
    neighbor.parent = current;
    if (!visited[ny][nx]) {
      aStarQueue.add(neighbor);
      openList.add(neighbor);
    }
  }
  cpuCycles++; return true;
}

void reconstructPath(Node goal) {
  finalPath.clear(); Node cur = goal;
  while (cur != null) { finalPath.add(cur); cur = cur.parent; }
  Collections.reverse(finalPath); pathLength = finalPath.size() - 1;
}
