import java.util.*;
import java.util.PriorityQueue;

// ----- Grid Settings -----
int gridCols=23;
int gridRows=23;
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

// Multi-agent support
ArrayList agents;
ArrayList goals;

// For drag-to-move tool - 支持移动起点、终点、所有地形类型
enum DragMode { NONE, MOVE_AGENT, MOVE_GOAL, MOVE_TERRAIN }
DragMode currentDragMode;
Object draggedPoint; // 可以是 MapPoint 或 int[] {x, y, terrainType}

// ----- Algorithm Related -----
enum Algorithm { BFS, DIJKSTRA, ASTAR }
Algorithm currentAlgo;

int speed;
Slider speedSlider;
boolean running;
boolean paused;

// Search process data
ArrayList openList;
ArrayList closedList;
ArrayList finalPath;
Node startNode;
Node goalNode;
int visitedCount;
int pathLength;
int cpuCycles;
boolean pathFound;
boolean algorithmFinished;

// BFS specific
ArrayDeque bfsQueue;
boolean[][] visited;
// Dijkstra specific
PriorityQueue dijkstraQueue;
int[][] dist;
// A* specific
PriorityQueue aStarQueue;

// ----- UI Controls -----
enum Tool { ADD_AGENT, ADD_GOAL, DRAW_OBSTACLE, DRAW_GRASS, DRAW_DESERT, MOVE_POINT }
Tool currentTool;

boolean terrainDropdownExpanded;
int terrainDropdownY;
int terrainDropdownBtnH;

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

int panelX;
int panelWidth;
ArrayList buttons;

String noSolutionMsg;
int msgStartTime;

Slider gridColsSlider;
Slider gridRowsSlider;

// 对比模式
boolean compareMode;
// 对比模式弹窗的临时快照
HashMap<Algorithm, PathRecord> comparePopupRecords;
// 防止结果弹窗重复触发
boolean resultShown = false;
HashMap historyPaths;

// Popup
String popupMessage;
String popupButtonText;
boolean popupVisible;
int popupStartTime;
int popupType;
int popupButtonClickTime;

// ----- Inner Classes -----
class MapPoint {
  int x, y, id;
  MapPoint(int x, int y, int id) {
    this.x = x;
    this.y = y;
    this.id = id;
  }
}

class PathRecord {
  ArrayList path;
  int visitedCount;
  int pathLength;
  int cpuCycles;
  PathRecord(ArrayList p, int v, int pl, int cpu) {
    path = new ArrayList();
    for (int i = 0; i < p.size(); i++) {
      Node n = (Node)p.get(i);
      path.add(new Node(n.x, n.y));
    }
    visitedCount = v;
    pathLength = pl;
    cpuCycles = cpu;
  }
}

class UIButton {
  int x, y, w, h;
  String label;
  String id;
  boolean hovered;
  
  UIButton(int x, int y, int w, int h, String label, String id) {
    this.x = x;
    this.y = y;
    this.w = w;
    this.h = h;
    this.label = label;
    this.id = id;
    this.hovered = false;
  }
  
  boolean isOver(int mx, int my) {
    return mx >= x && mx <= x + w && my >= y && my <= y + h;
  }
  
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

class Node implements Comparable {
  int x, y, g, h;
  Node parent;
  
  Node(int x, int y) {
    this.x = x;
    this.y = y;
    g = Integer.MAX_VALUE;
    h = 0;
    parent = null;
  }
  
  int f() { 
    return g + h; 
  }
  
  public int compareTo(Object o) {
    Node other = (Node) o;
    return this.f() - other.f();
  }
  
  public boolean equals(Object o) {
    if (!(o instanceof Node)) return false;
    Node n = (Node) o;
    return this.x == n.x && this.y == n.y;
  }
  
  public int hashCode() { 
    return x * 31 + y; 
  }
}

class Slider {
  int x, y, w, h;
  float minVal, maxVal, value;
  boolean dragging;
  String label;
  
  Slider(int x, int y, int w, int h, float minVal, float maxVal, float initVal, String label) {
    this.x = x;
    this.y = y;
    this.w = w;
    this.h = h;
    this.minVal = minVal;
    this.maxVal = maxVal;
    this.value = constrain(initVal, minVal, maxVal);
    this.label = label;
    this.dragging = false;
  }
  
  void draw() {
    fill(COLOR_SLIDER_BG);
    stroke(100, 100, 150);
    rect(x, y, w, h, 5);
    float fillW = map(value, minVal, maxVal, 0, w);
    fill(COLOR_SLIDER_FILL);
    noStroke();
    rect(x, y, fillW, h, 5);
    float handleX = x + fillW;
    fill(COLOR_SLIDER_HANDLE);
    ellipse(handleX, y + h/2, 10, 10);
    fill(COLOR_SLIDER_LABEL);
    textAlign(LEFT, CENTER);
    textSize(12);
    text(label + ": " + (int)value, x, y + h + 8);
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
public void settings() { 
  size(1060, 870); 
}

public void setup() {
  surface.setTitle("AI Pathfinding Arena");
  textFont(createFont("Arial", 14));
  
  // 初始化变量
  agents = new ArrayList();
  goals = new ArrayList();
  currentDragMode = DragMode.NONE;
  draggedPoint = null;
  currentAlgo = Algorithm.BFS;
  speed = 1;
  running = false;
  paused = false;
  openList = new ArrayList();
  closedList = new ArrayList();
  finalPath = new ArrayList();
  startNode = null;
  goalNode = null;
  visitedCount = 0;
  pathLength = 0;
  cpuCycles = 0;
  pathFound = false;
  algorithmFinished = false;
  currentTool = Tool.DRAW_OBSTACLE;
  terrainDropdownExpanded = false;
  terrainDropdownY = 0;
  terrainDropdownBtnH = 30;
  buttons = new ArrayList();
  noSolutionMsg = "";
  msgStartTime = 0;
  compareMode = false;
  historyPaths = new HashMap();
  popupMessage = "";
  popupButtonText = "";
  popupVisible = false;
  popupStartTime = 0;
  popupType = 0;
  popupButtonClickTime = 0;
comparePopupRecords = new HashMap<Algorithm, PathRecord>();
resultShown = false;
  
  updateLayout();
  grid = new int[gridRows][gridCols];
  resetGrid();
  
  // 初始只有一对起点和终点
  agents.add(new MapPoint(5, 5, 1));
  goals.add(new MapPoint(gridCols - 6, gridRows - 6, 1));
  
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
  for (int r = 0; r < gridRows; r++) {
    for (int c = 0; c < gridCols; c++) {
      grid[r][c] = EMPTY;
    }
  }
}

void clearAllObstacles() {
  for (int r = 0; r < gridRows; r++) {
    for (int c = 0; c < gridCols; c++) {
      if (grid[r][c] == OBSTACLE) {
        grid[r][c] = EMPTY;
      }
    }
  }
  resetSearch();
}

boolean hasStartAt(int x, int y) {
  for (int i = 0; i < agents.size(); i++) {
    MapPoint a = (MapPoint)agents.get(i);
    if (a.x == x && a.y == y) return true;
  }
  return false;
}

boolean hasGoalAt(int x, int y) {
  for (int i = 0; i < goals.size(); i++) {
    MapPoint g = (MapPoint)goals.get(i);
    if (g.x == x && g.y == y) return true;
  }
  return false;
}

ArrayList getStartIdsAt(int x, int y) {
  ArrayList ids = new ArrayList();
  for (int i = 0; i < agents.size(); i++) {
    MapPoint a = (MapPoint)agents.get(i);
    if (a.x == x && a.y == y) ids.add(a.id);
  }
  Collections.sort(ids);
  return ids;
}

ArrayList getGoalIdsAt(int x, int y) {
  ArrayList ids = new ArrayList();
  for (int i = 0; i < goals.size(); i++) {
    MapPoint g = (MapPoint)goals.get(i);
    if (g.x == x && g.y == y) ids.add(g.id);
  }
  Collections.sort(ids);
  return ids;
}

boolean canAnyStartReachAnyGoal() {
  if (agents.isEmpty() || goals.isEmpty()) return false;
  boolean[][] reachable = new boolean[gridRows][gridCols];
  ArrayDeque queue = new ArrayDeque();
  for (int i = 0; i < agents.size(); i++) {
    MapPoint a = (MapPoint)agents.get(i);
    if (grid[a.y][a.x] != OBSTACLE) {
      reachable[a.y][a.x] = true;
      queue.add(new int[]{a.x, a.y});
    }
  }
  int[] dx = {-1,1,0,0};
  int[] dy = {0,0,-1,1};
  while (!queue.isEmpty()) {
    int[] cur = (int[])queue.poll();
    for (int i=0; i<4; i++) {
      int nx = cur[0] + dx[i];
      int ny = cur[1] + dy[i];
      if (nx>=0 && nx<gridCols && ny>=0 && ny<gridRows && !reachable[ny][nx] && grid[ny][nx] != OBSTACLE) {
        reachable[ny][nx] = true;
        queue.add(new int[]{nx, ny});
      }
    }
  }
  for (int i = 0; i < goals.size(); i++) {
    MapPoint g = (MapPoint)goals.get(i);
    if (g.x>=0 && g.x<gridCols && g.y>=0 && g.y<gridRows && reachable[g.y][g.x]) return true;
  }
  return false;
}

boolean isStartBlocked() {
  if (agents.isEmpty()) { 
    showWarningPopup("Missing Start Point!\nPlease use Agent tool to add a start point.");
    return true; 
  }
  if (goals.isEmpty()) { 
    showWarningPopup("Missing Goal Point!\nPlease use Goal tool to add a goal point.");
    return true; 
  }
  if (!canAnyStartReachAnyGoal()) {
    showNoSolutionWithClearButton("Cannot reach goal!\nObstacles block all paths.\nClear all obstacles?");
    return true;
  }
  return false;
}

void showWarningPopup(String msg) {
  popupMessage = msg;
  popupButtonText = "OK";
  popupVisible = true;
  popupStartTime = millis();
  popupType = 0;
  popupButtonClickTime = 0;
  resultShown = true;
}

void showNoSolutionWithClearButton(String msg) {
  popupMessage = msg;
  popupButtonText = "Clear Obstacles";
  popupVisible = true;
  popupStartTime = millis();
  popupType = 1;
  noSolutionMsg = msg;
  msgStartTime = millis();
  resultShown = true;
}

void showResultPopup(boolean success, String message, int visited, int pathLen, int cpu) {
  if (compareMode) {
    // 对比模式：保存当前结果并弹出宽屏对比弹窗
    if (success) {
      historyPaths.put(currentAlgo, new PathRecord(finalPath, visited, pathLen, cpu));
    }
    comparePopupRecords = new HashMap<Algorithm, PathRecord>(historyPaths);
    popupMessage = "";
    popupButtonText = "OK";
    popupVisible = true;
    popupStartTime = millis();
    popupType = 2;
    popupButtonClickTime = 0;
    resultShown = true;
    return;
  }

  // 普通弹窗（非对比模式）
  String title = success ? "Path Found!" : "Search Failed";
  String stats = success ? "Visited: " + visited + "\nPath Length: " + pathLen + "\nCPU Cycles: " + cpu : message;
  popupMessage = title + "\n" + stats;
  popupButtonText = "OK";
  popupVisible = true;
  popupStartTime = millis();
  popupType = 0;
  popupButtonClickTime = 0;
  resultShown = true;
}

void drawPopup() {
  if (!popupVisible) return;

if (popupType == 2) {
    // 对比模式弹窗：宽屏三列
    int w = 600;
    int h = 200;
    int cx = width/2;
    int cy = height/2;
    
    fill(0, 0, 0, 230);
    noStroke();
    rect(cx - w/2, cy - h/2, w, h, 15);
    stroke(100, 255, 100);
    strokeWeight(2);
    noFill();
    rect(cx - w/2, cy - h/2, w, h, 15);
    
    // 关闭按钮 X
    int closeSize = 20;
    int closeX = cx + w/2 - closeSize - 8;
    int closeY = cy - h/2 + 8;
    boolean hoverClose = (mouseX >= closeX && mouseX <= closeX + closeSize &&
                          mouseY >= closeY && mouseY <= closeY + closeSize);
    stroke(255, 100, 100);
    strokeWeight(2);
    if (hoverClose) fill(255, 0, 0, 100);
    else noFill();
    rect(closeX, closeY, closeSize, closeSize, 4);
    stroke(255);
    line(closeX + 4, closeY + 4, closeX + closeSize - 4, closeY + closeSize - 4);
    line(closeX + closeSize - 4, closeY + 4, closeX + 4, closeY + closeSize - 4);
    
    // 标题
    fill(255, 255, 200);
    textAlign(CENTER, CENTER);
    textSize(14);
    text("Algorithm Comparison", cx, cy - h/2 + 30);
    
    // 三个算法列
    Algorithm[] algos = {Algorithm.BFS, Algorithm.DIJKSTRA, Algorithm.ASTAR};
    String[] algoNames = {"BFS", "Dijkstra", "A*"};
    float colWidth = (w - 40) / 3.0;
    float startX = cx - w/2 + 20;
    
    for (int i = 0; i < 3; i++) {
      float colX = startX + i * colWidth;
      fill(200, 200, 255);
      textSize(13);
      text(algoNames[i], colX + colWidth/2, cy - h/2 + 55);
      
      PathRecord rec = comparePopupRecords.get(algos[i]);
      if (rec != null) {
        fill(255, 255, 150);
        textSize(11);
        text("Visited: " + rec.visitedCount, colX + colWidth/2, cy - h/2 + 80);
        text("Path Len: " + rec.pathLength, colX + colWidth/2, cy - h/2 + 100);
        text("CPU: " + rec.cpuCycles, colX + colWidth/2, cy - h/2 + 120);
      } else {
        fill(150, 150, 150);
        textSize(11);
        text("No data", colX + colWidth/2, cy - h/2 + 80);
      }
    }
    
    // OK 按钮
    int btnW = 60;
    int btnH = 25;
    int btnX = cx - btnW/2;
    int btnY = cy + h/2 - btnH - 15;
    boolean hoverBtn = (mouseX >= btnX && mouseX <= btnX + btnW &&
                        mouseY >= btnY && mouseY <= btnY + btnH);
    fill(hoverBtn ? COLOR_BTN_HOVER : COLOR_BTN_NORMAL);
    stroke(COLOR_BTN_STROKE);
    rect(btnX, btnY, btnW, btnH, 5);
    fill(COLOR_BTN_TEXT_BG);
    textSize(13);
    text("OK", btnX + btnW/2, btnY + btnH/2);
    
    // 点击处理
    if (mousePressed && millis() - popupButtonClickTime > 200) {
      if (hoverClose) {
        popupVisible = false;
        popupButtonClickTime = millis();
      }
      if (hoverBtn) {
        popupVisible = false;
        popupButtonClickTime = millis();
      }
    }
  } else {
  pushStyle();
  int w = 400;
  int h = (popupButtonText.equals("Clear Obstacles") ? 150 : 130);
  int cx = width/2;
  int cy = height/2;
  int btnW = 100;
  int btnH = 28;
  
  // Background
  fill(0, 0, 0, 220);
  noStroke();
  rect(cx - w/2, cy - h/2, w, h, 15);
  stroke(100, 255, 100);
  strokeWeight(2);
  noFill();
  rect(cx - w/2, cy - h/2, w, h, 15);
 // 关闭按钮 (X) 在右上角
  int closeSize = 20;
  int closeX = cx + w/2 - closeSize - 8;
  int closeY = cy - h/2 + 8;
  boolean hoverClose = (mouseX >= closeX && mouseX <= closeX + closeSize &&
                        mouseY >= closeY && mouseY <= closeY + closeSize);

// 绘制 X 按钮
  stroke(255, 100, 100);
  strokeWeight(2);
  if (hoverClose) fill(255, 0, 0, 100);
  else noFill();
  rect(closeX, closeY, closeSize, closeSize, 4);
  stroke(255);
  line(closeX + 4, closeY + 4, closeX + closeSize - 4, closeY + closeSize - 4);
  line(closeX + closeSize - 4, closeY + 4, closeX + 4, closeY + closeSize - 4);
  
  // Text
  fill(255, 255, 200);
  textAlign(CENTER, CENTER);
  textSize(13);
  String[] lines = popupMessage.split("\n");
  int textStartY = cy - h/2 + 35;
  for (int i=0; i<lines.length; i++) {
    text(lines[i], cx, textStartY + i * 22);
  }
  
  // Button
  int btnX = cx - btnW/2;
  int btnY = cy + h/2 - btnH - 10;
  
  boolean hover = (mouseX >= btnX && mouseX <= btnX + btnW && 
                   mouseY >= btnY && mouseY <= btnY + btnH);
  
  fill(hover ? COLOR_BTN_HOVER : COLOR_BTN_NORMAL);
  stroke(COLOR_BTN_STROKE);
  rect(btnX, btnY, btnW, btnH, 8);
  fill(COLOR_BTN_TEXT_BG);
  textSize(13);
  text(popupButtonText, btnX + btnW/2, btnY + btnH/2);
  
 // Handle button click
  if (mousePressed && millis() - popupButtonClickTime > 200) {
    // 先检查关闭按钮
    if (hoverClose) {
      popupVisible = false;
      popupButtonClickTime = millis();
      popStyle();
      return;
    }
    // 再检查原有按钮
    if (hover) {
      popupButtonClickTime = millis();
      if (popupType == 1 && popupButtonText.equals("Clear Obstacles")) {
        clearAllObstacles();
        resetSearch();
        popupVisible = false;
      } else if (popupButtonText.equals("OK")) {
        popupVisible = false;
      }
    }
  }
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
    MapPoint p = (MapPoint)agents.get(i);
    if (p.x >= gridCols || p.y >= gridRows) agents.remove(i);
  }
  for (int i = goals.size()-1; i>=0; i--) {
    MapPoint p = (MapPoint)goals.get(i);
    if (p.x >= gridCols || p.y >= gridRows) goals.remove(i);
  }
  if (agents.isEmpty() && gridCols > 1 && gridRows > 1) {
    agents.add(new MapPoint(1, 1, 1));
  }
  if (goals.isEmpty() && gridCols > 2 && gridRows > 2) {
    goals.add(new MapPoint(gridCols-2, gridRows-2, 1));
  }
  
  updateButtonLabels();
  resetSearch();
  clearHistoryPaths();
}

void createButtons() {
  buttons.clear();
  int yBase = 60;
  int btnH = 30;
  int btnW = panelWidth - 20;
  int x = panelX + 10;
  
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
  buttons.add(new UIButton(x, terrainDropdownY, btnW, btnH, "Tool: Obstacle", "TOOL_OBSTACLE"));
  buttons.add(new UIButton(x, yBase + 460, btnW, btnH, "Start", "RUN_START"));
  buttons.add(new UIButton(x, yBase + 500, btnW, btnH, "Pause", "RUN_PAUSE"));
  buttons.add(new UIButton(x, yBase + 540, btnW, btnH, "Step", "RUN_STEP"));
  buttons.add(new UIButton(x, yBase + 580, btnW, btnH, "Reset", "RUN_RESET"));
  buttons.add(new UIButton(x, yBase + 620, btnW, btnH, "Clear All", "RUN_CLEAR"));
  gridColsSlider = new Slider(x, yBase + 660, btnW, 15, 10, 35, gridCols, "Cols");
  gridRowsSlider = new Slider(x, yBase + 690, btnW, 15, 10, 35, gridRows, "Rows");
}

void updateButtonLabels() {
  for (int i = 0; i < buttons.size(); i++) {
    UIButton b = (UIButton)buttons.get(i);
    if (b.id.equals("ALGO_ASTAR")) {
      b.label = "Algo: " + currentAlgo.toString();
    } else if (b.id.equals("TOOL_AGENT")) {
      b.label = "Tool: " + (currentTool == Tool.ADD_AGENT ? "*Agent" : "Agent");
    } else if (b.id.equals("TOOL_GOAL")) {
      b.label = "Tool: " + (currentTool == Tool.ADD_GOAL ? "*Goal" : "Goal");
    } else if (b.id.equals("TOOL_MOVE")) {
      b.label = "Tool: " + (currentTool == Tool.MOVE_POINT ? "*Move" : "Move");
    } else if (b.id.equals("TOOL_OBSTACLE")) {
      String terrainName = "";
      if (currentTool == Tool.DRAW_OBSTACLE) terrainName = "*Obstacle";
      else if (currentTool == Tool.DRAW_GRASS) terrainName = "*Grass";
      else if (currentTool == Tool.DRAW_DESERT) terrainName = "*Desert";
      else terrainName = "Obstacle";
      b.label = "Tool: " + terrainName;
    } else if (b.id.equals("COMPARE_MODE")) {
      b.label = "Compare: " + (compareMode ? "ON" : "OFF");
    }
  }
}

// ----- Drawing -----
public void draw() {
  background(20, 20, 40);
  drawArena();
  drawInstructions();
  drawPanel();
  drawPopup();
  
  if (running && !paused && !algorithmFinished) {
    for (int i = 0; i < speed; i++) {
       if (!algorithmStep()) {           
      algorithmFinished = true;
      running = false;
      if (!resultShown) {
        if (pathFound) {
          showResultPopup(true, "Path found!", visitedCount, pathLength, cpuCycles);
        } else {
          showResultPopup(false, "No path exists from any start to any goal!", visitedCount, 0, cpuCycles);
        }
      }
      break;
    }
  }
}
}
void drawArena() {
  pushMatrix();
  translate(gridOffsetX, gridOffsetY);
  
  // Draw terrain
  for (int r = 0; r < gridRows; r++) {
    for (int c = 0; c < gridCols; c++) {
      int x = c * CELL_SIZE;
      int y = r * CELL_SIZE;
      switch (grid[r][c]) {
        case OBSTACLE: fill(COLOR_OBSTACLE); break;
        case GRASS: fill(COLOR_GRASS); break;
        case DESERT: fill(COLOR_DESERT); break;
        default: fill(COLOR_EMPTY);
      }
      stroke(70, 70, 90);
      rect(x, y, CELL_SIZE, CELL_SIZE);
    }
  }
  
  drawSearchVisuals();
  
  // Draw starts and goals - 支持同一格多个起点/终点堆叠显示
  // 先按坐标分组
  HashMap startGroups = new HashMap();
  HashMap goalGroups = new HashMap();
  
  for (int i = 0; i < agents.size(); i++) {
    MapPoint a = (MapPoint)agents.get(i);
    String key = a.x + "," + a.y;
    if (!startGroups.containsKey(key)) startGroups.put(key, new ArrayList());
    ((ArrayList)startGroups.get(key)).add(a);
  }
  for (int i = 0; i < goals.size(); i++) {
    MapPoint g = (MapPoint)goals.get(i);
    String key = g.x + "," + g.y;
    if (!goalGroups.containsKey(key)) goalGroups.put(key, new ArrayList());
    ((ArrayList)goalGroups.get(key)).add(g);
  }
  
  // 绘制起点组
  Iterator startIt = startGroups.keySet().iterator();
  while (startIt.hasNext()) {
    String key = (String)startIt.next();
    ArrayList group = (ArrayList)startGroups.get(key);
    String[] parts = key.split(",");
    int cx = int(parts[0]);
    int cy = int(parts[1]);
    float x = cx * CELL_SIZE + CELL_SIZE/2;
    float y = cy * CELL_SIZE + CELL_SIZE/2;
    int count = group.size();
    MapPoint first = (MapPoint)group.get(0);
    color startColor = (count == 1 && first.id == 1) ? COLOR_START : COLOR_START_MULTI;
    fill(startColor);
    ellipse(x, y, CELL_SIZE * 0.8, CELL_SIZE * 0.8);
    fill(255);
    textSize(count > 2 ? 7 : 9);
    textAlign(CENTER, CENTER);
    if (count == 1) {
      text("S" + first.id, x, y);
    } else {
      String label = "S";
      for (int i = 0; i < group.size(); i++) {
        MapPoint p = (MapPoint)group.get(i);
        label += p.id;
      }
      text(label, x, y);
    }
  }
  
  // 绘制终点组
  Iterator goalIt = goalGroups.keySet().iterator();
  while (goalIt.hasNext()) {
    String key = (String)goalIt.next();
    ArrayList group = (ArrayList)goalGroups.get(key);
    String[] parts = key.split(",");
    int cx = int(parts[0]);
    int cy = int(parts[1]);
    float x = cx * CELL_SIZE + CELL_SIZE/2;
    float y = cy * CELL_SIZE + CELL_SIZE/2;
    int count = group.size();
    MapPoint first = (MapPoint)group.get(0);
    color goalColor = (count == 1 && first.id == 1) ? COLOR_GOAL : COLOR_GOAL_MULTI;
    fill(goalColor);
    ellipse(x, y, CELL_SIZE * 0.8, CELL_SIZE * 0.8);
    fill(255);
    textSize(count > 2 ? 7 : 9);
    textAlign(CENTER, CENTER);
    if (count == 1) {
      text("G" + first.id, x, y);
    } else {
      String label = "G";
      for (int i = 0; i < group.size(); i++) {
        MapPoint p = (MapPoint)group.get(i);
        label += p.id;
      }
      text(label, x, y);
    }
  }
  
  popMatrix();
}

void drawSearchVisuals() {
  noStroke();
  fill(COLOR_EXPLORED);
  for (int i = 0; i < closedList.size(); i++) {
    Node n = (Node)closedList.get(i);
    if (grid[n.y][n.x] != OBSTACLE) {
      rect(n.x * CELL_SIZE + 1, n.y * CELL_SIZE + 1, CELL_SIZE - 2, CELL_SIZE - 2);
    }
  }
  fill(COLOR_FRONTIER);
  for (int i = 0; i < openList.size(); i++) {
    Node n = (Node)openList.get(i);
    if (grid[n.y][n.x] != OBSTACLE) {
      rect(n.x * CELL_SIZE + 1, n.y * CELL_SIZE + 1, CELL_SIZE - 2, CELL_SIZE - 2);
    }
  }
  
  if (compareMode) {
    Iterator it = historyPaths.entrySet().iterator();
    while (it.hasNext()) {
      Map.Entry entry = (Map.Entry)it.next();
      Algorithm algo = (Algorithm)entry.getKey();
      PathRecord rec = (PathRecord)entry.getValue();
      if (rec.path == null || rec.path.size() < 2) continue;
      drawPathWithBloom(rec.path, getAlgoColor(algo, 255), 3);
    }
  }
  if (pathFound && finalPath.size() > 1) {
    drawPathWithBloom(finalPath, getAlgoColor(currentAlgo, 255), 4);
  }
}

void drawPathWithBloom(ArrayList path, color pathColor, int coreWeight) {
  if (path == null || path.size() < 2) return;
  blendMode(ADD);
  stroke(red(pathColor), green(pathColor), blue(pathColor), 20);
  strokeWeight(coreWeight + 16);
  noFill();
  beginShape();
  for (int i = 0; i < path.size(); i++) {
    Node n = (Node)path.get(i);
    vertex(n.x * CELL_SIZE + CELL_SIZE/2, n.y * CELL_SIZE + CELL_SIZE/2);
  }
  endShape();
  stroke(red(pathColor), green(pathColor), blue(pathColor), 40);
  strokeWeight(coreWeight + 10);
  beginShape();
  for (int i = 0; i < path.size(); i++) {
    Node n = (Node)path.get(i);
    vertex(n.x * CELL_SIZE + CELL_SIZE/2, n.y * CELL_SIZE + CELL_SIZE/2);
  }
  endShape();
  stroke(red(pathColor), green(pathColor), blue(pathColor), 60);
  strokeWeight(coreWeight + 6);
  beginShape();
  for (int i = 0; i < path.size(); i++) {
    Node n = (Node)path.get(i);
    vertex(n.x * CELL_SIZE + CELL_SIZE/2, n.y * CELL_SIZE + CELL_SIZE/2);
  }
  endShape();
  blendMode(BLEND);
  stroke(pathColor);
  strokeWeight(coreWeight);
  beginShape();
  for (int i = 0; i < path.size(); i++) {
    Node n = (Node)path.get(i);
    vertex(n.x * CELL_SIZE + CELL_SIZE/2, n.y * CELL_SIZE + CELL_SIZE/2);
  }
  endShape();
  strokeWeight(1);
}

color getAlgoColor(Algorithm algo, int alpha) {
  if (algo == Algorithm.BFS) return color(#FF6B6B, alpha);
  else if (algo == Algorithm.DIJKSTRA) return color(#00E5FF, alpha);
  else return color(#FFEB3B, alpha);
}

void drawInstructions() {
int y = height - 80;
  int xLeft = gridOffsetX;
  fill(150, 150, 200);
  textAlign(LEFT, TOP);
  textSize(12);
  text("Agent: Left-click -> add start, Right-click -> remove start", xLeft, y);
  text("Goal: Left-click -> add goal, Right-click -> remove goal", xLeft, y+18);
  text("Move: Drag start/goal/terrain (Obstacle/Grass/Desert) to reposition", xLeft, y+36);
  text("Obstacle/Grass/Desert: Left-click to draw, Right-click to erase", xLeft, y+54);
}

void drawPanel() {
  fill(25, 25, 45, 220);
  noStroke();
  rect(panelX, 0, panelWidth, height);
  fill(COLOR_PANEL_TEXT);
  textAlign(LEFT, TOP);
  textSize(16);
  text("Control Panel", panelX+10, 20);
  
  for (int i = 0; i < buttons.size(); i++) {
    UIButton b = (UIButton)buttons.get(i);
    b.draw();
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
  
  if (mouseX >= gridOffsetX && mouseX < gridOffsetX + gridCols * CELL_SIZE &&
      mouseY >= gridOffsetY && mouseY < gridOffsetY + gridRows * CELL_SIZE) {
    int mx = (mouseX - gridOffsetX) / CELL_SIZE;
    int my = (mouseY - gridOffsetY) / CELL_SIZE;
    pushStyle();
    fill(COLOR_HOVER_TEXT);
    textAlign(RIGHT, BOTTOM);
    String info = "Grid: (" + mx + ", " + my + ")";
    ArrayList sIds = getStartIdsAt(mx, my);
    ArrayList gIds = getGoalIdsAt(mx, my);
    if (!sIds.isEmpty()) {
      info += " | Starts: S";
      for (int i = 0; i < sIds.size(); i++) {
        info += sIds.get(i);
        if (i < sIds.size()-1) info += ",";
      }
    }
    if (!gIds.isEmpty()) {
      info += " | Goals: G";
      for (int i = 0; i < gIds.size(); i++) {
        info += gIds.get(i);
        if (i < gIds.size()-1) info += ",";
      }
    }
    // 显示当前格子的地形
    if (grid[my][mx] == OBSTACLE) info += " | Obstacle";
    else if (grid[my][mx] == GRASS) info += " | Grass";
    else if (grid[my][mx] == DESERT) info += " | Desert";
    text(info, width-10, height-10);
    popStyle();
  }
}

// ----- Mouse Interaction -----
public void mousePressed() {
  boolean isSearching = (running && !paused && !algorithmFinished);
  boolean isButtonClick = false;
  
  for (int i = 0; i < buttons.size(); i++) {
    UIButton b = (UIButton)buttons.get(i);
    if (b.isOver(mouseX, mouseY)) { 
      isButtonClick = true; 
      break; 
    }
  }
  
  if (speedSlider.overHandle(mouseX, mouseY)) { 
    speedSlider.dragging = true; 
    return; 
  }
  if (gridColsSlider.overHandle(mouseX, mouseY)) { 
    gridColsSlider.dragging = true; 
    return; 
  }
  if (gridRowsSlider.overHandle(mouseX, mouseY)) { 
    gridRowsSlider.dragging = true; 
    return; 
  }
  
  if (isSearching && !isButtonClick && mouseX < panelX) return;
  
  for (int i = 0; i < buttons.size(); i++) {
    UIButton b = (UIButton)buttons.get(i);
    if (b.isOver(mouseX, mouseY)) { 
      handleButton(b.id); 
      return; 
    }
  }
  
  if (mouseX >= gridOffsetX && mouseX < gridOffsetX + gridCols * CELL_SIZE &&
      mouseY >= gridOffsetY && mouseY < gridOffsetY + gridRows * CELL_SIZE) {
    int cx = (mouseX - gridOffsetX) / CELL_SIZE;
    int cy = (mouseY - gridOffsetY) / CELL_SIZE;
    if (pathFound) return;
    
    if (currentTool == Tool.MOVE_POINT) {
  // 先从后往前查找同一格子内的 Agent，确保最晚放置的优先被拖拽
  for (int i = agents.size() - 1; i >= 0; i--) {
    MapPoint a = (MapPoint)agents.get(i);
    if (a.x == cx && a.y == cy) {
      currentDragMode = DragMode.MOVE_AGENT;
      draggedPoint = a;
      return;
    }
  }
  // 再从后往前查找同一格子内的 Goal，优先移动最晚放置的
  for (int i = goals.size() - 1; i >= 0; i--) {
    MapPoint g = (MapPoint)goals.get(i);
    if (g.x == cx && g.y == cy) {
      currentDragMode = DragMode.MOVE_GOAL;
      draggedPoint = g;
      return;
    }
  }
      
      if (grid[cy][cx] == OBSTACLE || grid[cy][cx] == GRASS || grid[cy][cx] == DESERT) {
        currentDragMode = DragMode.MOVE_TERRAIN;
        
        draggedPoint = new int[]{cx, cy, grid[cy][cx]};
        return;
      }
      return;
    }
    
    if (currentTool == Tool.ADD_AGENT) {
      if (mouseButton == LEFT) {
         if (agents.size() >= 4) {
      showWarningPopup("Maximum 4 agents allowed!");
      return;
    }
    if (grid[cy][cx] != OBSTACLE) {
      agents.add(new MapPoint(cx, cy, agents.size() + 1)); 
      resetSearch();
    }
      } else if (mouseButton == RIGHT) {
        removeAgentAt(cx, cy);
        resetSearch();
      }
    } else if (currentTool == Tool.ADD_GOAL) {
      if (mouseButton == LEFT) {
        if (goals.size() >= 4) {
      showWarningPopup("Maximum 4 goals allowed!");
      return;
    }
    if (grid[cy][cx] != OBSTACLE) {
      goals.add(new MapPoint(cx, cy, goals.size() + 1));
      resetSearch();
    }
      } else if (mouseButton == RIGHT) {
        removeGoalAt(cx, cy);
        resetSearch();
      }
    } else if (currentTool == Tool.DRAW_OBSTACLE) {
      if (mouseButton == LEFT) {
        if (!hasStartAt(cx, cy) && !hasGoalAt(cx, cy)) {
          grid[cy][cx] = OBSTACLE;
          resetSearch();
        }
      } else if (mouseButton == RIGHT) {
        if (grid[cy][cx] == OBSTACLE) {
          grid[cy][cx] = EMPTY;
          resetSearch();
        }
      }
    } else if (currentTool == Tool.DRAW_GRASS) {
      if (mouseButton == LEFT) {
        if (!hasStartAt(cx, cy) && !hasGoalAt(cx, cy)) {
          grid[cy][cx] = GRASS;
          resetSearch();
        }
      } else if (mouseButton == RIGHT) {
        if (grid[cy][cx] == GRASS) {
          grid[cy][cx] = EMPTY;
          resetSearch();
        }
      }
    } else if (currentTool == Tool.DRAW_DESERT) {
      if (mouseButton == LEFT) {
        if (!hasStartAt(cx, cy) && !hasGoalAt(cx, cy)) {
          grid[cy][cx] = DESERT;
          resetSearch();
        }
      } else if (mouseButton == RIGHT) {
        if (grid[cy][cx] == DESERT) {
          grid[cy][cx] = EMPTY;
          resetSearch();
        }
      }
    }
  }
}

public void mouseDragged() {
  if (speedSlider.dragging) { 
    speedSlider.setFromMouse(mouseX); 
    speed = (int)speedSlider.value; 
    return; 
  }
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
      
      // 边界检查
      if (newX < 0 || newX >= gridCols || newY < 0 || newY >= gridRows) return;
      
      if (currentDragMode == DragMode.MOVE_AGENT) {
        MapPoint p = (MapPoint)draggedPoint;
        // 不能移动到障碍物上，也不能移动到有起点/终点的格子上（除非是移动自己）
        if (grid[newY][newX] == OBSTACLE) return;
        if ((hasStartAt(newX, newY) && !(p.x == newX && p.y == newY)) || 
            (hasGoalAt(newX, newY))) return;
        p.x = newX;
        p.y = newY;
        resetSearch();
      } else if (currentDragMode == DragMode.MOVE_GOAL) {
        MapPoint p = (MapPoint)draggedPoint;
        if (grid[newY][newX] == OBSTACLE) return;
        if ((hasGoalAt(newX, newY) && !(p.x == newX && p.y == newY)) || 
            (hasStartAt(newX, newY))) return;
        p.x = newX;
        p.y = newY;
        resetSearch();
      } else if (currentDragMode == DragMode.MOVE_TERRAIN) {
        int[] oldData = (int[])draggedPoint;
        int oldX = oldData[0];
        int oldY = oldData[1];
        int terrainType = oldData[2];
        
        // 不能移动到有起点或终点的格子上
        if (hasStartAt(newX, newY) || hasGoalAt(newX, newY)) return;
        // 不能移动到已有非空地形的位置（除非是相同位置）
        if ((grid[newY][newX] == OBSTACLE || grid[newY][newX] == GRASS || grid[newY][newX] == DESERT) && 
            !(newX == oldX && newY == oldY)) return;
        
        // 清除原位置的地形
        grid[oldY][oldX] = EMPTY;
        // 在新位置放置地形
        grid[newY][newX] = terrainType;
        // 更新拖拽点为新位置
        draggedPoint = new int[]{newX, newY, terrainType};
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
        resetSearch();
      } else if (mouseButton == RIGHT) {
        if (grid[cy][cx] == newVal) {
          grid[cy][cx] = EMPTY;
          resetSearch();
        }
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
    MapPoint a = (MapPoint)agents.get(i);
    if (a.x == cx && a.y == cy) {
      agents.remove(i);
      reassignAgentIds();
      break;
    }
  }
}

void removeGoalAt(int cx, int cy) {
  for (int i = goals.size() - 1; i >= 0; i--) {
    MapPoint g = (MapPoint)goals.get(i);
    if (g.x == cx && g.y == cy) {
      goals.remove(i);
      reassignGoalIds();
      break;
    }
  }
}
void reassignAgentIds() {
  for (int i = 0; i < agents.size(); i++) {
    MapPoint a = (MapPoint)agents.get(i);
    a.id = i + 1;
  }
}

void reassignGoalIds() {
  for (int i = 0; i < goals.size(); i++) {
    MapPoint g = (MapPoint)goals.get(i);
    g.id = i + 1;
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
  if (id.equals("COMPARE_MODE")) {
    compareMode = !compareMode;
    if (!compareMode) clearHistoryPaths();
    updateButtonLabels();
  } else if (id.equals("ALGO_NEXT")) {
    if (compareMode && pathFound) savePathToHistory();
    if (currentAlgo == Algorithm.BFS) currentAlgo = Algorithm.DIJKSTRA;
    else if (currentAlgo == Algorithm.DIJKSTRA) currentAlgo = Algorithm.ASTAR;
    else currentAlgo = Algorithm.BFS;
    resetSearch();
    updateButtonLabels();
  } else if (id.equals("SPEED_UP")) {
    speed = min(speed + 1, 20);
    speedSlider.value = speed;
  } else if (id.equals("SPEED_DOWN")) {
    speed = max(speed - 1, 1);
    speedSlider.value = speed;
  } else if (id.equals("TOOL_AGENT")) {
    currentTool = Tool.ADD_AGENT;
    updateButtonLabels();
  } else if (id.equals("TOOL_GOAL")) {
    currentTool = Tool.ADD_GOAL;
    updateButtonLabels();
  } else if (id.equals("TOOL_MOVE")) {
    currentTool = Tool.MOVE_POINT;
    updateButtonLabels();
  } else if (id.equals("TOOL_OBSTACLE")) {
    if (currentTool == Tool.DRAW_OBSTACLE) {
      currentTool = Tool.DRAW_GRASS;
    } else if (currentTool == Tool.DRAW_GRASS) {
      currentTool = Tool.DRAW_DESERT;
    } else if (currentTool == Tool.DRAW_DESERT) {
      currentTool = Tool.DRAW_OBSTACLE;
    }
    updateButtonLabels();
  } else if (id.equals("RUN_START")) {
    if (agents.size() > 0 && goals.size() > 0) {
      if (isStartBlocked()) return;
      resetSearch();
      resultShown = false;
      MapPoint firstAgent = (MapPoint)agents.get(0);
      MapPoint firstGoal = (MapPoint)goals.get(0);
      startNode = new Node(firstAgent.x, firstAgent.y);
      goalNode = new Node(firstGoal.x, firstGoal.y);
      if (currentAlgo == Algorithm.BFS) { 
        initBFS(); 
        running = true; 
        paused = false; 
        algorithmFinished = false; 
      } else if (currentAlgo == Algorithm.DIJKSTRA) { 
        initDijkstra(); 
        running = true; 
        paused = false; 
        algorithmFinished = false; 
      } else if (currentAlgo == Algorithm.ASTAR) { 
        initAStar(); 
        running = true; 
        paused = false; 
        algorithmFinished = false; 
      }
    } else {
      if (agents.isEmpty()) {
        showWarningPopup("Missing Start Point!\nPlease use Agent tool to add a start point.");
      } else if (goals.isEmpty()) {
        showWarningPopup("Missing Goal Point!\nPlease use Goal tool to add a goal point.");
      }
    }
  } else if (id.equals("RUN_PAUSE")) {
    paused = !paused;
  } else if (id.equals("RUN_STEP")) {
    if (!algorithmFinished) {
      if ((bfsQueue == null && dijkstraQueue == null && aStarQueue == null) && agents.size() > 0 && goals.size() > 0) {
        if (isStartBlocked()) return;
        MapPoint firstAgent = (MapPoint)agents.get(0);
        MapPoint firstGoal = (MapPoint)goals.get(0);
        startNode = new Node(firstAgent.x, firstAgent.y);
        goalNode = new Node(firstGoal.x, firstGoal.y);
        if (currentAlgo == Algorithm.BFS) initBFS();
        else if (currentAlgo == Algorithm.DIJKSTRA) initDijkstra();
        else if (currentAlgo == Algorithm.ASTAR) initAStar();
        algorithmFinished = false;
        resultShown = false; 
      }
      algorithmStep();
      if (algorithmFinished && !resultShown) { 
        if (pathFound) {
          showResultPopup(true, "Path found!", visitedCount, pathLength, cpuCycles);
        } else {
          showResultPopup(false, "No path exists from any start to any goal!", visitedCount, 0, cpuCycles);
        }
      }
      paused = true;
      running = false;
    }
  } else if (id.equals("RUN_RESET")) {
   for (int r = 0; r < gridRows; r++) {
        for (int c = 0; c < gridCols; c++) {
            if (grid[r][c] == OBSTACLE || grid[r][c] == GRASS || grid[r][c] == DESERT) {
                grid[r][c] = EMPTY;
            }
        }
    }
    resetSearch();
    running = false;
    paused = false;
    algorithmFinished = false;
    clearHistoryPaths();
  } else if (id.equals("RUN_CLEAR")) {
    agents.clear();
    goals.clear();
    for (int r = 0; r < gridRows; r++) {
      for (int c = 0; c < gridCols; c++) {
        grid[r][c] = EMPTY;
      }
    }
    resetSearch();
    running = false;
    paused = false;
    algorithmFinished = false;
    clearHistoryPaths();
  }
  updateButtonLabels();
}

void resetSearch() {
  openList.clear();
  closedList.clear();
  finalPath.clear();
  visitedCount = 0;
  pathLength = 0;
  cpuCycles = 0;
  pathFound = false;
  algorithmFinished = false;
  startNode = null;
  goalNode = null;
  bfsQueue = null;
  visited = null;
  aStarQueue = null;
  dijkstraQueue = null;
  resultShown = false;
}

// ========== Algorithms ==========
void initBFS() {
  bfsQueue = new ArrayDeque();
  visited = new boolean[gridRows][gridCols];
  bfsQueue.add(startNode);
  visited[startNode.y][startNode.x] = true;
  openList.add(startNode);
  visitedCount = 0;
  pathLength = 0;
  cpuCycles = 0;
  pathFound = false;
}

void initDijkstra() {
  dijkstraQueue = new PriorityQueue();
  visited = new boolean[gridRows][gridCols];
  dist = new int[gridRows][gridCols];
  for (int i = 0; i < gridRows; i++) {
    for (int j = 0; j < gridCols; j++) {
      dist[i][j] = Integer.MAX_VALUE;
    }
  }
  startNode.g = 0;
  dist[startNode.y][startNode.x] = 0;
  dijkstraQueue.add(startNode);
  openList.add(startNode);
  visitedCount = 0;
  pathLength = 0;
  cpuCycles = 0;
  pathFound = false;
}

void initAStar() {
  aStarQueue = new PriorityQueue();
  visited = new boolean[gridRows][gridCols];
  startNode.g = 0;
  startNode.h = heuristic(startNode, goalNode);
  aStarQueue.add(startNode);
  openList.add(startNode);
  visitedCount = 0;
  cpuCycles = 0;
  pathFound = false;
}

int heuristic(Node a, Node b) {
  return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
}

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
  else {
    algorithmFinished = true;
    return false;
  }
}

boolean stepBFS() {
  if (bfsQueue == null || bfsQueue.isEmpty()) {
    algorithmFinished = true;
    running = false;
    return false;
  }
  Node current = (Node)bfsQueue.poll();
  openList.remove(current);
  closedList.add(current);
  visitedCount++;
  
  if (current.equals(goalNode)) {
    pathFound = true;
    algorithmFinished = true;
    running = false;
    reconstructPath(current);
    return false;
  }
  
  int[] dx = {-1, 1, 0, 0};
  int[] dy = {0, 0, -1, 1};
  for (int i = 0; i < 4; i++) {
    int nx = current.x + dx[i];
    int ny = current.y + dy[i];
    if (nx >= 0 && nx < gridCols && ny >= 0 && ny < gridRows && !visited[ny][nx] && grid[ny][nx] != OBSTACLE) {
      visited[ny][nx] = true;
      Node neighbor = new Node(nx, ny);
      neighbor.parent = current;
      bfsQueue.add(neighbor);
      openList.add(neighbor);
    }
  }
  cpuCycles++;
  return true;
}

boolean stepDijkstra() {
  if (dijkstraQueue == null || dijkstraQueue.isEmpty()) {
    algorithmFinished = true;
    running = false;
    return false;
  }
  Node current = (Node)dijkstraQueue.poll();
  if (visited[current.y][current.x]) return true;
  visited[current.y][current.x] = true;
  openList.remove(current);
  closedList.add(current);
  visitedCount++;
  
  if (current.equals(goalNode)) {
    pathFound = true;
    algorithmFinished = true;
    running = false;
    reconstructPath(current);
    return false;
  }
  
  int[] dx = {-1, 1, 0, 0};
  int[] dy = {0, 0, -1, 1};
  for (int i = 0; i < 4; i++) {
    int nx = current.x + dx[i];
    int ny = current.y + dy[i];
    if (nx >= 0 && nx < gridCols && ny >= 0 && ny < gridRows && !visited[ny][nx] && grid[ny][nx] != OBSTACLE) {
      int newDist = current.g + getTerrainWeight(grid[ny][nx]);
      if (newDist < dist[ny][nx]) {
        dist[ny][nx] = newDist;
        Node neighbor = new Node(nx, ny);
        neighbor.g = newDist;
        neighbor.parent = current;
        dijkstraQueue.add(neighbor);
        openList.add(neighbor);
      }
    }
  }
  cpuCycles++;
  return true;
}

boolean stepAStar() {
  if (aStarQueue == null || aStarQueue.isEmpty()) {
    algorithmFinished = true;
    running = false;
    return false;
  }
  Node current = (Node)aStarQueue.poll();
  if (visited[current.y][current.x]) return true;
  visited[current.y][current.x] = true;
  openList.remove(current);
  closedList.add(current);
  visitedCount++;
  
  if (current.equals(goalNode)) {
    pathFound = true;
    algorithmFinished = true;
    running = false;
    reconstructPath(current);
    return false;
  }
  
  int[] dx = {-1, 1, 0, 0};
  int[] dy = {0, 0, -1, 1};
  for (int i = 0; i < 4; i++) {
    int nx = current.x + dx[i];
    int ny = current.y + dy[i];
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
  cpuCycles++;
  return true;
}

void reconstructPath(Node goal) {
  finalPath.clear();
  Node cur = goal;
  while (cur != null) {
    finalPath.add(cur);
    cur = cur.parent;
  }
  Collections.reverse(finalPath);
  pathLength = finalPath.size() - 1;
}
