import java.util.*;
import java.util.PriorityQueue;

// ----- Grid Settings (now adjustable) -----
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

// Terrain weight map for Dijkstra and A*
// Weight: cost to traverse each terrain type
// OBSTACLE (1) = impassable (weight = -1)
// GRASS (4) = high cost (weight = 3)
// DESERT (5) = medium cost (weight = 2)
// EMPTY (0) = normal cost (weight = 1)

final int WEIGHT_NORMAL = 1;
final int WEIGHT_DESERT = 2;
final int WEIGHT_GRASS = 3;
final int WEIGHT_IMPASSABLE = -1;
int[][] terrainWeight;

// List of agents (start points)
ArrayList<PVector> agents = new ArrayList<PVector>();
// List of goal points
ArrayList<PVector> goals  = new ArrayList<PVector>();

// ----- Algorithm Related (for UI display) -----
enum Algorithm { BFS, DIJKSTRA, ASTAR }
Algorithm currentAlgo = Algorithm.BFS;

// Speed control
int speed = 1;
Slider speedSlider;
boolean running = false;
boolean paused = false;
boolean stepMode = false;

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

// BFS specific data structures
ArrayDeque<Node> bfsQueue;
boolean[][] visited;
// Dijkstra specific data structures
PriorityQueue<Node> dijkstraQueue;
int[][] dist;
// A* specific
PriorityQueue<Node> aStarQueue;

// ----- UI Controls -----
enum Tool { ADD_AGENT, ADD_GOAL, DRAW_OBSTACLE, DRAW_GRASS, DRAW_DESERT }
Tool currentTool = Tool.DRAW_OBSTACLE;

// Terrain dropdown state
boolean terrainDropdownExpanded = false;
int terrainDropdownY = 0;
int terrainDropdownBtnH = 30;

// Color definitions for terrain types
// OBSTACLE: dark red (impassable wall)
// GRASS: light green (high cost terrain)
// DESERT: sand yellow (medium cost terrain)

//Convert all color settings to global variables（将所有颜色设置转为全局变量）
//Convert to hexadecimal notation for easy modification（转为十六表示便于修改）

final color COLOR_GRASS = #228B22;
final color COLOR_DESERT = #F4A460;
final color COLOR_OBSTACLE = #c4c4c4ff;
final color COLOR_EMPTY = #1E1E32;
final color COLOR_START = #00C8FF;
final color COLOR_GOAL = #FF6432;
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

// 对比模式：历史路径记录
boolean compareMode = false;
// 保存历史路径：key=算法, value=路径节点列表
HashMap<Algorithm, ArrayList<Node>> historyPaths = new HashMap<Algorithm, ArrayList<Node>>();

// ----- Inner Class: UI Button -----
class UIButton {
    int x, y, w, h;
    String label, id;
    boolean hovered = false;

    UIButton(int x, int y, int w, int h, String label, String id) {
        this.x = x; this.y = y; this.w = w; this.h = h;
        this.label = label; this.id = id;
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

// ----- Inner Class: Grid Node -----
class Node implements Comparable<Node> {
    int x, y, g, h;
    Node parent;

    Node(int x, int y) {
        this.x = x; this.y = y;
        g = Integer.MAX_VALUE; h = 0; parent = null;
    }

    int f() { return g + h; }
    public int compareTo(Node other) { return this.f() - other.f(); }
    public boolean equals(Object o) {
        if (!(o instanceof Node)) return false;
        Node n = (Node) o;
        return this.x == n.x && this.y == n.y;
    }
    public int hashCode() { return x * 31 + y; }
}

public void settings() {
    size(1060, 900);
}

public void setup() {
    surface.setTitle("AI Pathfinding Arena");
    textFont(createFont("Arial", 14));

    updateLayout();
    grid = new int[gridRows][gridCols];
    resetGrid();

    agents.add(new PVector(5, 5));
    goals.add(new PVector(gridCols - 6, gridRows - 6));
    updateGridFromAgentsAndGoals();

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

boolean isStartBlocked() {
    if (agents.isEmpty()) return false;
    int[] dx = {-1, 1, 0, 0}, dy = {0, 0, -1, 1};
    for (PVector agent : agents) {
        int sx = (int)agent.x, sy = (int)agent.y;
        if (grid[sy][sx] == OBSTACLE) { showNoSolution("ERROR: Start point is blocked!"); return true; }
        boolean canMove = false;
        for (int i = 0; i < 4; i++) {
            int nx = sx + dx[i], ny = sy + dy[i];
            if (nx >= 0 && nx < gridCols && ny >= 0 && ny < gridRows && grid[ny][nx] != OBSTACLE) {
                canMove = true; break;
            }
        }
        if (!canMove) { showNoSolution("NO SOLUTION! Start is surrounded by obstacles!"); return true; }
    }
    return false;
}

void showNoSolution(String msg) { noSolutionMsg = msg; msgStartTime = millis(); println(msg); }

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

void updateGridFromAgentsAndGoals() {
    for (int r = 0; r < gridRows; r++)
        for (int c = 0; c < gridCols; c++)
            if (grid[r][c] == START || grid[r][c] == GOAL) grid[r][c] = EMPTY;
    for (PVector a : agents) {
        int cx = (int)a.x, cy = (int)a.y;
        if (cx >= 0 && cx < gridCols && cy >= 0 && cy < gridRows && grid[cy][cx] == EMPTY) grid[cy][cx] = START;
    }
    for (PVector g : goals) {
        int cx = (int)g.x, cy = (int)g.y;
        if (cx >= 0 && cx < gridCols && cy >= 0 && cy < gridRows && grid[cy][cx] == EMPTY) grid[cy][cx] = GOAL;
    }
}

void resizeGrid(int newCols, int newRows) {
    if (newCols == gridCols && newRows == gridRows) return;
    gridCols = constrain(newCols, 10, 40);
    gridRows = constrain(newRows, 10, 40);
    grid = new int[gridRows][gridCols];
    resetGrid();
    for (int i = agents.size() - 1; i >= 0; i--) {
        PVector a = agents.get(i);
        if (a.x >= gridCols || a.y >= gridRows) agents.remove(i);
    }
    for (int i = goals.size() - 1; i >= 0; i--) {
        PVector g = goals.get(i);
        if (g.x >= gridCols || g.y >= gridRows) goals.remove(i);
    }
    if (agents.isEmpty() && gridCols > 1 && gridRows > 1) agents.add(new PVector(1, 1));
    if (goals.isEmpty() && gridCols > 2 && gridRows > 2) goals.add(new PVector(gridCols - 2, gridRows - 2));
    updateGridFromAgentsAndGoals();
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

    buttons.add(new UIButton(x, yBase + 130, btnW / 2 - 2, btnH, "-Speed", "SPEED_DOWN"));
    buttons.add(new UIButton(x + btnW / 2 + 2, yBase + 130, btnW / 2 - 2, btnH, "+Speed", "SPEED_UP"));

    speedSlider = new Slider(x, yBase + 165, btnW, 15, 1, 20, speed, "Speed");

    buttons.add(new UIButton(x, yBase + 200, btnW, btnH, "Tool: Agent", "TOOL_AGENT"));
    buttons.add(new UIButton(x, yBase + 240, btnW, btnH, "Tool: Goal", "TOOL_GOAL"));
    terrainDropdownY = yBase + 280;
    buttons.add(new UIButton(x, terrainDropdownY, btnW, btnH, "Tool: Obstacle ▼", "TOOL_OBSTACLE"));

    buttons.add(new UIButton(x, yBase + 420, btnW, btnH, "Start", "RUN_START"));
    buttons.add(new UIButton(x, yBase + 460, btnW, btnH, "Pause", "RUN_PAUSE"));
    buttons.add(new UIButton(x, yBase + 500, btnW, btnH, "Step", "RUN_STEP"));
    buttons.add(new UIButton(x, yBase + 540, btnW, btnH, "Reset", "RUN_RESET"));
    buttons.add(new UIButton(x, yBase + 580, btnW, btnH, "Clear All", "RUN_CLEAR"));

    gridColsSlider = new Slider(x, yBase + 620, btnW, 15, 10, 40, gridCols, "Cols");
    gridRowsSlider = new Slider(x, yBase + 650, btnW, 15, 10, 40, gridRows, "Rows");
}

void updateButtonLabels() {
    for (UIButton b : buttons) {
        if (b.id.equals("ALGO_ASTAR")) b.label = "Algo: " + currentAlgo.toString();
        else if (b.id.equals("TOOL_AGENT")) b.label = "Tool: " + (currentTool == Tool.ADD_AGENT ? "*Agent" : "Agent");
        else if (b.id.equals("TOOL_GOAL")) b.label = "Tool: " + (currentTool == Tool.ADD_GOAL ? "*Goal" : "Goal");
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

public void draw() {
    background(20, 20, 40);
    drawArena();
    drawInstructions();
    drawPanel();
    drawNoSolutionMsg();
    if (running && !paused && !algorithmFinished) {
        for (int i = 0; i < speed; i++)
            if (!algorithmStep()) { algorithmFinished = true; running = false; break; }
    }
}

void drawArena() {
    pushMatrix();
    translate(gridOffsetX, gridOffsetY);
    for (int r = 0; r < gridRows; r++) {
        for (int c = 0; c < gridCols; c++) {
            int x = c * CELL_SIZE, y = r * CELL_SIZE;
            switch (grid[r][c]) {
                case OBSTACLE: fill(COLOR_OBSTACLE); break;
                case GRASS: fill(COLOR_GRASS); break;
                case DESERT: fill(COLOR_DESERT); break;
                case START: fill(COLOR_START); break;
                case GOAL: fill(COLOR_GOAL); break;
                default: fill(COLOR_EMPTY);
            }
            stroke(70, 70, 90); rect(x, y, CELL_SIZE, CELL_SIZE);
        }
    }
    drawSearchVisuals();
    for (int i = 0; i < agents.size(); i++) {
        PVector a = agents.get(i);
        float cx = a.x * CELL_SIZE + CELL_SIZE / 2, cy = a.y * CELL_SIZE + CELL_SIZE / 2;
        fill(COLOR_START); noStroke(); ellipse(cx, cy, CELL_SIZE * 0.8, CELL_SIZE * 0.8);
        fill(255); textAlign(CENTER, CENTER); textSize(10); text(str(i + 1), cx, cy - 1);
    }
    for (int i = 0; i < goals.size(); i++) {
        PVector g = goals.get(i);
        float cx = g.x * CELL_SIZE + CELL_SIZE / 2, cy = g.y * CELL_SIZE + CELL_SIZE / 2;
        fill(COLOR_GOAL); noStroke(); ellipse(cx, cy, CELL_SIZE * 0.8, CELL_SIZE * 0.8);
        fill(255); textAlign(CENTER, CENTER); textSize(8); text("G" + (i + 1), cx, cy);
    }
    popMatrix();
}

void drawSearchVisuals() {
    noStroke();

    // 当前算法的探索区域
    fill(COLOR_EXPLORED);
    for (Node n : closedList) {
        if (grid[n.y][n.x] != OBSTACLE && grid[n.y][n.x] != GRASS && grid[n.y][n.x] != DESERT) {
            rect(n.x * CELL_SIZE + 1, n.y * CELL_SIZE + 1, CELL_SIZE - 2, CELL_SIZE - 2);
        }
    }
    fill(COLOR_FRONTIER);
    for (Node n : openList) {
        if (grid[n.y][n.x] != OBSTACLE && grid[n.y][n.x] != GRASS && grid[n.y][n.x] != DESERT) {
            rect(n.x * CELL_SIZE + 1, n.y * CELL_SIZE + 1, CELL_SIZE - 2, CELL_SIZE - 2);
        }
    }

    // 对比模式：绘制历史路径 (泛光效果)
    if (compareMode) {
        for (Map.Entry<Algorithm, ArrayList<Node>> entry : historyPaths.entrySet()) {
            Algorithm algo = entry.getKey();
            ArrayList<Node> path = entry.getValue();
            if (path == null || path.size() < 2) continue;
            
            int coreWeight;
            if (algo == Algorithm.BFS) coreWeight = 4;
            else if (algo == Algorithm.DIJKSTRA) coreWeight = 3;
            else coreWeight = 2;
            
            drawPathWithBloom(path, getAlgoColor(algo, 255), coreWeight);
        }
    }

    // 当前路径 (最后绘制，使其覆盖历史路径，泛光效果更强)
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
    for (Node n : path) vertex(n.x * CELL_SIZE + CELL_SIZE / 2, n.y * CELL_SIZE + CELL_SIZE / 2);
    endShape();
    
    stroke(red(pathColor), green(pathColor), blue(pathColor), 40);
    strokeWeight(coreWeight + 10);
    beginShape();
    for (Node n : path) vertex(n.x * CELL_SIZE + CELL_SIZE / 2, n.y * CELL_SIZE + CELL_SIZE / 2);
    endShape();
    
    stroke(red(pathColor), green(pathColor), blue(pathColor), 60);
    strokeWeight(coreWeight + 6);
    beginShape();
    for (Node n : path) vertex(n.x * CELL_SIZE + CELL_SIZE / 2, n.y * CELL_SIZE + CELL_SIZE / 2);
    endShape();
    
    stroke(red(pathColor), green(pathColor), blue(pathColor), 100);
    strokeWeight(coreWeight + 3);
    beginShape();
    for (Node n : path) vertex(n.x * CELL_SIZE + CELL_SIZE / 2, n.y * CELL_SIZE + CELL_SIZE / 2);
    endShape();
    
    blendMode(BLEND);
    
    stroke(pathColor);
    strokeWeight(coreWeight);
    noFill();
    beginShape();
    for (Node n : path) vertex(n.x * CELL_SIZE + CELL_SIZE / 2, n.y * CELL_SIZE + CELL_SIZE / 2);
    endShape();
    strokeWeight(1);
}

color getAlgoColor(Algorithm algo, int alpha) {
    if (algo == Algorithm.BFS) return color(#FF6B6B, alpha);        // Modern coral red
    else if (algo == Algorithm.DIJKSTRA) return color(#00E5FF, alpha);  // Vibrant cyan
    else return color(#FFEB3B, alpha);  // Bright yellow (明光黄)
}

void drawInstructions() {
    int arenaBottom = gridOffsetY + gridRows * CELL_SIZE;
    int y = arenaBottom + 15;
    int xLeft = gridOffsetX;
    fill(150, 150, 200);
    textAlign(LEFT, TOP);
    textSize(12);
    text("Agent: Left-click → place start, Right-click → remove start", xLeft, y);
    text("Goal: Left-click → place goal, Right-click → remove goal", xLeft, y + 18);
    text("Obstacle: Left-click → place wall, Right-click → remove wall, drag to draw/erase continuously", xLeft, y + 36);
}

void drawPanel() {
    fill(25, 25, 45, 220); noStroke(); rect(panelX, 0, panelWidth, height);
    fill(COLOR_PANEL_TEXT); textAlign(LEFT, TOP); textSize(16); text("Control Panel", panelX + 10, 20);
    for (UIButton b : buttons) b.draw();

    // Draw terrain dropdown if expanded
    if (terrainDropdownExpanded) {
        int btnX = panelX + 10;
        int btnW = panelWidth - 20;
        int subBtnH = terrainDropdownBtnH;
        boolean isObstacleHovered = mouseX >= btnX && mouseX <= btnX + btnW &&
            mouseY >= terrainDropdownY + subBtnH && mouseY <= terrainDropdownY + subBtnH * 2;
        boolean isGrassHovered = mouseX >= btnX && mouseX <= btnX + btnW &&
            mouseY >= terrainDropdownY + subBtnH * 2 && mouseY <= terrainDropdownY + subBtnH * 3;
        boolean isDesertHovered = mouseX >= btnX && mouseX <= btnX + btnW &&
            mouseY >= terrainDropdownY + subBtnH * 3 && mouseY <= terrainDropdownY + subBtnH * 4;

        // Obstacle sub-item
        fill(isObstacleHovered ? COLOR_BTN_HOVER : COLOR_BTN_NORMAL);
        stroke(COLOR_BTN_STROKE);
        rect(btnX, terrainDropdownY + subBtnH, btnW, subBtnH, 5);
        fill(COLOR_BTN_TEXT_BG);
        textAlign(CENTER, CENTER);
        textSize(12);
        text("Obstacle", btnX + btnW/2, terrainDropdownY + subBtnH + subBtnH/2);

        // Grass sub-item
        fill(isGrassHovered ? COLOR_BTN_HOVER : COLOR_BTN_NORMAL);
        stroke(COLOR_BTN_STROKE);
        rect(btnX, terrainDropdownY + subBtnH * 2, btnW, subBtnH, 5);
        fill(COLOR_BTN_TEXT_BG);
        text("Grass", btnX + btnW/2, terrainDropdownY + subBtnH * 2 + subBtnH/2);

        // Desert sub-item
        fill(isDesertHovered ? COLOR_BTN_HOVER : COLOR_BTN_NORMAL);
        stroke(COLOR_BTN_STROKE);
        rect(btnX, terrainDropdownY + subBtnH * 3, btnW, subBtnH, 5);
        fill(COLOR_BTN_TEXT_BG);
        text("Desert", btnX + btnW/2, terrainDropdownY + subBtnH * 3 + subBtnH/2);
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
    text("Visited: " + visitedCount, statsX, statsY + 20);
    text("Path len: " + pathLength, statsX, statsY + 40);
    text("CPU cycles: " + cpuCycles, statsX, statsY + 60);
    popStyle();

    if (mouseX >= gridOffsetX && mouseX < gridOffsetX + gridCols * CELL_SIZE &&
        mouseY >= gridOffsetY && mouseY < gridOffsetY + gridRows * CELL_SIZE) {
        int mx = (mouseX - gridOffsetX) / CELL_SIZE;
        int my = (mouseY - gridOffsetY) / CELL_SIZE;
        pushStyle();
        fill(COLOR_HOVER_TEXT);
        textAlign(RIGHT, BOTTOM);
        text("Grid: (" + mx + ", " + my + ")", width - 10, height - 10);
        popStyle();
    }
}

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

    // Handle terrain dropdown sub-items
    if (terrainDropdownExpanded) {
        int btnX = panelX + 10;
        int btnW = panelWidth - 20;
        // Obstacle sub-item
        if (mouseX >= btnX && mouseX <= btnX + btnW &&
            mouseY >= terrainDropdownY + terrainDropdownBtnH && mouseY <= terrainDropdownY + terrainDropdownBtnH * 2) {
            currentTool = Tool.DRAW_OBSTACLE;
            terrainDropdownExpanded = false;
            updateButtonLabels();
            return;
        }
        // Grass sub-item
        if (mouseX >= btnX && mouseX <= btnX + btnW &&
            mouseY >= terrainDropdownY + terrainDropdownBtnH * 2 && mouseY <= terrainDropdownY + terrainDropdownBtnH * 3) {
            currentTool = Tool.DRAW_GRASS;
            terrainDropdownExpanded = false;
            updateButtonLabels();
            return;
        }
        // Desert sub-item
        if (mouseX >= btnX && mouseX <= btnX + btnW &&
            mouseY >= terrainDropdownY + terrainDropdownBtnH * 3 && mouseY <= terrainDropdownY + terrainDropdownBtnH * 4) {
            currentTool = Tool.DRAW_DESERT;
            terrainDropdownExpanded = false;
            updateButtonLabels();
            return;
        }
        // Click outside dropdown closes it
        terrainDropdownExpanded = false;
        updateButtonLabels();
    }

    if (mouseX >= gridOffsetX && mouseX < gridOffsetX + gridCols * CELL_SIZE &&
        mouseY >= gridOffsetY && mouseY < gridOffsetY + gridRows * CELL_SIZE) {
        int cx = (mouseX - gridOffsetX) / CELL_SIZE;
        int cy = (mouseY - gridOffsetY) / CELL_SIZE;
        if (pathFound) return;

        switch (currentTool) {
            case ADD_AGENT:
                if (mouseButton == LEFT) {
                    if (grid[cy][cx] == EMPTY) { agents.add(new PVector(cx, cy)); grid[cy][cx] = START; }
                } else if (mouseButton == RIGHT) {
                    if (grid[cy][cx] == START) removeAgentAt(cx, cy);
                }
                break;
            case ADD_GOAL:
                if (mouseButton == LEFT) {
                    if (grid[cy][cx] == EMPTY) { goals.add(new PVector(cx, cy)); grid[cy][cx] = GOAL; }
                } else if (mouseButton == RIGHT) {
                    if (grid[cy][cx] == GOAL) removeGoalAt(cx, cy);
                }
                break;
            case DRAW_OBSTACLE:
                if (mouseButton == LEFT) {
                    if (grid[cy][cx] != START && grid[cy][cx] != GOAL) grid[cy][cx] = OBSTACLE;
                } else if (mouseButton == RIGHT) {
                    if (grid[cy][cx] == OBSTACLE) grid[cy][cx] = EMPTY;
                }
                break;
            case DRAW_GRASS:
                if (mouseButton == LEFT) {
                    if (grid[cy][cx] != START && grid[cy][cx] != GOAL) grid[cy][cx] = GRASS;
                } else if (mouseButton == RIGHT) {
                    if (grid[cy][cx] == GRASS) grid[cy][cx] = EMPTY;
                }
                break;
            case DRAW_DESERT:
                if (mouseButton == LEFT) {
                    if (grid[cy][cx] != START && grid[cy][cx] != GOAL) grid[cy][cx] = DESERT;
                } else if (mouseButton == RIGHT) {
                    if (grid[cy][cx] == DESERT) grid[cy][cx] = EMPTY;
                }
                break;
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

    if (currentTool == Tool.DRAW_OBSTACLE) {
        if (mouseX >= gridOffsetX && mouseX < gridOffsetX + gridCols * CELL_SIZE &&
            mouseY >= gridOffsetY && mouseY < gridOffsetY + gridRows * CELL_SIZE) {
            int cx = (mouseX - gridOffsetX) / CELL_SIZE, cy = (mouseY - gridOffsetY) / CELL_SIZE;
            if (mouseButton == LEFT) { if (grid[cy][cx] != START && grid[cy][cx] != GOAL) grid[cy][cx] = OBSTACLE; }
            else if (mouseButton == RIGHT) { if (grid[cy][cx] == OBSTACLE) grid[cy][cx] = EMPTY; }
        }
    } else if (currentTool == Tool.DRAW_GRASS) {
        if (mouseX >= gridOffsetX && mouseX < gridOffsetX + gridCols * CELL_SIZE &&
            mouseY >= gridOffsetY && mouseY < gridOffsetY + gridRows * CELL_SIZE) {
            int cx = (mouseX - gridOffsetX) / CELL_SIZE, cy = (mouseY - gridOffsetY) / CELL_SIZE;
            if (mouseButton == LEFT) { if (grid[cy][cx] != START && grid[cy][cx] != GOAL) grid[cy][cx] = GRASS; }
            else if (mouseButton == RIGHT) { if (grid[cy][cx] == GRASS) grid[cy][cx] = EMPTY; }
        }
    } else if (currentTool == Tool.DRAW_DESERT) {
        if (mouseX >= gridOffsetX && mouseX < gridOffsetX + gridCols * CELL_SIZE &&
            mouseY >= gridOffsetY && mouseY < gridOffsetY + gridRows * CELL_SIZE) {
            int cx = (mouseX - gridOffsetX) / CELL_SIZE, cy = (mouseY - gridOffsetY) / CELL_SIZE;
            if (mouseButton == LEFT) { if (grid[cy][cx] != START && grid[cy][cx] != GOAL) grid[cy][cx] = DESERT; }
            else if (mouseButton == RIGHT) { if (grid[cy][cx] == DESERT) grid[cy][cx] = EMPTY; }
        }
    }
}

void mouseReleased() {
    speedSlider.dragging = false;
    gridColsSlider.dragging = false;
    gridRowsSlider.dragging = false;
}

void removeAgentAt(int cx, int cy) {
    for (int i = agents.size() - 1; i >= 0; i--)
        if ((int)agents.get(i).x == cx && (int)agents.get(i).y == cy) { agents.remove(i); break; }
    grid[cy][cx] = EMPTY;
}

void removeGoalAt(int cx, int cy) {
    for (int i = goals.size() - 1; i >= 0; i--)
        if ((int)goals.get(i).x == cx && (int)goals.get(i).y == cy) { goals.remove(i); break; }
    grid[cy][cx] = EMPTY;
}

// 保存当前算法路径到历史记录
void savePathToHistory() {
    if (!pathFound) return;
    ArrayList<Node> copy = new ArrayList<Node>();
    for (Node n : finalPath) {
        copy.add(new Node(n.x, n.y));
    }
    historyPaths.put(currentAlgo, copy);
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
            // 在对比模式下，保存当前路径到历史
            if (compareMode) {
                savePathToHistory();
            }
            // 切换算法
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
        case "TOOL_OBSTACLE":
            terrainDropdownExpanded = !terrainDropdownExpanded;
            updateButtonLabels();
            break;
        case "RUN_START":
            if (agents.size() > 0 && goals.size() > 0) {
                if (isStartBlocked()) return;
                resetSearch();
                startNode = new Node((int)agents.get(0).x, (int)agents.get(0).y);
                goalNode  = new Node((int)goals.get(0).x, (int)goals.get(0).y);
                if (currentAlgo == Algorithm.BFS) { initBFS(); running = true; paused = false; algorithmFinished = false; }
                else if (currentAlgo == Algorithm.DIJKSTRA) { initDijkstra(); running = true; paused = false; algorithmFinished = false; }
                else if (currentAlgo == Algorithm.ASTAR) { initAStar(); running = true; paused = false; algorithmFinished = false; }
            } break;
        case "RUN_PAUSE": paused = !paused; break;
        case "RUN_STEP":
            if (!algorithmFinished) {
                if ((bfsQueue == null && dijkstraQueue == null && aStarQueue == null) && agents.size() > 0 && goals.size() > 0) {
                    if (isStartBlocked()) return;
                    startNode = new Node((int)agents.get(0).x, (int)agents.get(0).y);
                    goalNode  = new Node((int)goals.get(0).x, (int)goals.get(0).y);
                    if (currentAlgo == Algorithm.BFS) initBFS();
                    else if (currentAlgo == Algorithm.DIJKSTRA) initDijkstra();
                    else if (currentAlgo == Algorithm.ASTAR) initAStar();
                    else { println("Algorithm not implemented."); return; }
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
            resetGrid(); resetSearch();
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
        case EMPTY: return WEIGHT_NORMAL;
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
        Node neighbor = new Node(nx, ny); neighbor.g = current.g + getTerrainWeight(grid[ny][nx]); neighbor.h = heuristic(neighbor, goalNode); neighbor.parent = current;
        if (!visited[ny][nx]) { aStarQueue.add(neighbor); openList.add(neighbor); }
    }
    cpuCycles++; return true;
}

void reconstructPath(Node goal) {
    finalPath.clear(); Node cur = goal;
    while (cur != null) { finalPath.add(cur); cur = cur.parent; }
    Collections.reverse(finalPath); pathLength = finalPath.size() - 1;
}

// Slider class
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