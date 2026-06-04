import java.util.*;
import java.util.PriorityQueue;

// ----- Grid Settings -----
final int COLS = 40;
final int ROWS = 30;
final int CELL_SIZE = 20;
int gridOffsetX, gridOffsetY;
// Grid cell states
final int EMPTY = 0;
final int OBSTACLE = 1;
final int START = 2;
final int GOAL = 3;
int[][] grid;

// List of agents (start points)
ArrayList<PVector> agents = new ArrayList<PVector>();
// List of goal points
ArrayList<PVector> goals  = new ArrayList<PVector>();

// ----- Algorithm Related (for UI display) -----
enum Algorithm { BFS, DIJKSTRA, ASTAR }
Algorithm currentAlgo = Algorithm.BFS;

// Speed control
int speed = 5;                // 1-20, higher = faster (steps per frame)
boolean running = false;
boolean paused = false;
boolean stepMode = false;     // Single-step mode (used when Step button pressed)

// Search process data (kept for UI statistics)
ArrayList<Node> openList = new ArrayList<Node>();
ArrayList<Node> closedList = new ArrayList<Node>();
ArrayList<Node> finalPath = new ArrayList<Node>();
Node startNode, goalNode;
int visitedCount = 0;
int pathLength = 0;
int cpuCycles = 0;
boolean pathFound = false;
boolean algorithmFinished = false;

// BFS specific data structures (new)
ArrayDeque<Node> bfsQueue;
boolean[][] visited;   // visited cells during BFS
// Dijkstra specific data structures
PriorityQueue<Node> dijkstraQueue;
int[][] dist;          // 记录到每个点的最短距离

// ----- UI Controls -----
enum Tool { SELECT, ADD_AGENT, ADD_GOAL, DRAW_OBSTACLE }
Tool currentTool = Tool.SELECT;

// Button area
int panelX, panelWidth;

// Custom buttons
ArrayList<UIButton> buttons = new ArrayList<UIButton>();

// ----- Inner Class: UI Button -----
class UIButton {
    int x, y, w, h;
    String label;
    String id;
    boolean hovered = false;

    UIButton(int x, int y, int w, int h, String label, String id) {
        this.x = x;
        this.y = y;
        this.w = w;
        this.h = h;
        this.label = label;
        this.id = id;
    }

    boolean isOver(int mx, int my) {
        return mx >= x && mx <= x + w && my >= y && my <= y + h;
    }

    void draw() {
        hovered = isOver(mouseX, mouseY);
        fill(hovered ? 100 : 70);
        stroke(150);
        rect(x, y, w, h, 5);
        fill(255);
        textAlign(CENTER, CENTER);
        textSize(12);
        text(label, x + w / 2, y + h / 2);
    }
}

// ----- Inner Class: Grid Node (minimal, for UI compatibility) -----
class Node implements Comparable<Node> {
    int x, y;
    int g, h;
    Node parent;

    Node(int x, int y) {
        this.x = x;
        this.y = y;
        g = Integer.MAX_VALUE;
        h = 0;
        parent = null;
    }

    int f() { return g + h; }

    public int compareTo(Node other) {
        if (this.f() < other.f()) return -1;
        if (this.f() > other.f()) return 1;
        return 0;
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

public void settings() {
    size(1024, 700);
}

public void setup() {
    surface.setTitle("AI Pathfinding Arena");
    textFont(createFont("Arial", 14));

    int arenaWidth = COLS * CELL_SIZE;
    int arenaHeight = ROWS * CELL_SIZE;
    gridOffsetX = 20;
    gridOffsetY = 20;
    panelX = gridOffsetX + arenaWidth + 20;
    panelWidth = width - panelX - 10;

    // Initialize grid
    grid = new int[ROWS][COLS];
    resetGrid();

    // Place default one start and one goal
    agents.add(new PVector(5, 5));
    goals.add(new PVector(COLS - 6, ROWS - 6));
    updateGridFromAgentsAndGoals();

    // Create UI buttons
    createButtons();
}

// Reset grid (clear obstacles, keep start/goal markers)
void resetGrid() {
    for (int r = 0; r < ROWS; r++) {
        Arrays.fill(grid[r], EMPTY);
    }
}

// Update grid START and GOAL markers based on agents/goals lists
void updateGridFromAgentsAndGoals() {
    // First clear old start/goal markers
    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
            if (grid[r][c] == START || grid[r][c] == GOAL) {
                grid[r][c] = EMPTY;
            }
        }
    }
    // Mark starts
    for (PVector a : agents) {
        int cx = (int) a.x;
        int cy = (int) a.y;
        if (cx >= 0 && cx < COLS && cy >= 0 && cy < ROWS) {
            if (grid[cy][cx] == EMPTY) grid[cy][cx] = START;
        }
    }
    // Mark goals
    for (PVector g : goals) {
        int cx = (int) g.x;
        int cy = (int) g.y;
        if (cx >= 0 && cx < COLS && cy >= 0 && cy < ROWS) {
            if (grid[cy][cx] == EMPTY) grid[cy][cx] = GOAL;
        }
    }
}

// Create control panel buttons
void createButtons() {
    buttons.clear();
    int yBase = 60;
    int btnH = 30;
    int btnW = panelWidth - 20;
    int x = panelX + 10;

    buttons.add(new UIButton(x, yBase, btnW, btnH, "Algo: A*", "ALGO_ASTAR"));
    buttons.add(new UIButton(x, yBase + 40, btnW, btnH, "Next Algo", "ALGO_NEXT"));

    buttons.add(new UIButton(x, yBase + 90, btnW / 2 - 2, btnH, "-Speed", "SPEED_DOWN"));
    buttons.add(new UIButton(x + btnW / 2 + 2, yBase + 90, btnW / 2 - 2, btnH, "+Speed", "SPEED_UP"));

    // Compare button removed, tools shifted up
    buttons.add(new UIButton(x, yBase + 140, btnW, btnH, "Tool: Select", "TOOL_SELECT"));
    buttons.add(new UIButton(x, yBase + 180, btnW, btnH, "Tool: Agent", "TOOL_AGENT"));
    buttons.add(new UIButton(x, yBase + 220, btnW, btnH, "Tool: Goal", "TOOL_GOAL"));
    buttons.add(new UIButton(x, yBase + 260, btnW, btnH, "Tool: Obstacle", "TOOL_OBSTACLE"));

    buttons.add(new UIButton(x, yBase + 320, btnW, btnH, "Start", "RUN_START"));
    buttons.add(new UIButton(x, yBase + 360, btnW, btnH, "Pause", "RUN_PAUSE"));
    buttons.add(new UIButton(x, yBase + 400, btnW, btnH, "Step", "RUN_STEP"));
    buttons.add(new UIButton(x, yBase + 440, btnW, btnH, "Reset", "RUN_RESET"));
    buttons.add(new UIButton(x, yBase + 480, btnW, btnH, "Clear All", "RUN_CLEAR"));
}

// Update button labels according to current state
void updateButtonLabels() {
    for (UIButton b : buttons) {
        if (b.id.equals("ALGO_ASTAR")) {
            b.label = "Algo: " + currentAlgo.toString();
        } else if (b.id.equals("TOOL_SELECT")) {
            b.label = "Tool: " + (currentTool == Tool.SELECT ? "*Select" : "Select");
        } else if (b.id.equals("TOOL_AGENT")) {
            b.label = "Tool: " + (currentTool == Tool.ADD_AGENT ? "*Agent" : "Agent");
        } else if (b.id.equals("TOOL_GOAL")) {
            b.label = "Tool: " + (currentTool == Tool.ADD_GOAL ? "*Goal" : "Goal");
        } else if (b.id.equals("TOOL_OBSTACLE")) {
            b.label = "Tool: " + (currentTool == Tool.DRAW_OBSTACLE ? "*Obstacle" : "Obstacle");
        }
    }
}

// ----- Main Draw Loop -----
public void draw() {
    background(30);

    // Draw grid arena
    drawArena();

    // Draw control panel
    drawPanel();

    // ---------- BFS integration: automatic stepping ----------
    if (running && !paused && !algorithmFinished) {
        for (int i = 0; i < speed; i++) {
            if (!algorithmStep()) {
                algorithmFinished = true;
                running = false;   // ensure we stop running when search ends
                break;
            }
        }
    }
}

// Draw grid and all visual elements (purely UI)
void drawArena() {
    pushMatrix();
    translate(gridOffsetX, gridOffsetY);

    // Grid background
    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
            int x = c * CELL_SIZE;
            int y = r * CELL_SIZE;

            // Fill color
            switch (grid[r][c]) {
                case OBSTACLE:
                    fill(80, 80, 80);
                    break;
                case START:
                    fill(0, 150, 255);
                    break;
                case GOAL:
                    fill(255, 200, 0);
                    break;
                default:
                    fill(50, 50, 50);
            }
            stroke(60);
            rect(x, y, CELL_SIZE, CELL_SIZE);
        }
    }

    // Draw search visualization (open/closed/final path)
    drawSearchVisuals();

    // Draw agent icons (starting points with numbers inside circles)
    for (int i = 0; i < agents.size(); i++) {
        PVector a = agents.get(i);
        float cx = a.x * CELL_SIZE + CELL_SIZE / 2;
        float cy = a.y * CELL_SIZE + CELL_SIZE / 2;
        fill(0, 150, 255);
        noStroke();
        ellipse(cx, cy, CELL_SIZE * 0.8, CELL_SIZE * 0.8);
        fill(255);
        textAlign(CENTER, CENTER);
        textSize(10);
        text(str(i + 1), cx, cy - 1);
    }

    // Draw goal point icons
    for (int i = 0; i < goals.size(); i++) {
        PVector g = goals.get(i);
        float cx = g.x * CELL_SIZE + CELL_SIZE / 2;
        float cy = g.y * CELL_SIZE + CELL_SIZE / 2;
        fill(255, 200, 0);
        noStroke();
        ellipse(cx, cy, CELL_SIZE * 0.8, CELL_SIZE * 0.8);
        fill(0);
        textAlign(CENTER, CENTER);
        textSize(8);
        text("G" + (i + 1), cx, cy);
    }

    popMatrix();
}

// Draw explored nodes, frontier, and final path
void drawSearchVisuals() {
    pushMatrix();
    translate(gridOffsetX, gridOffsetY);
    noStroke();

    // Explored nodes (closed list) with semi-transparent green squares
    fill(0, 200, 0, 80);
    for (Node n : closedList) {
        rect(n.x * CELL_SIZE + 1, n.y * CELL_SIZE + 1, CELL_SIZE - 2, CELL_SIZE - 2);
    }

    // Frontier (open list) with semi-transparent yellow
    fill(255, 255, 0, 100);
    for (Node n : openList) {
        rect(n.x * CELL_SIZE + 1, n.y * CELL_SIZE + 1, CELL_SIZE - 2, CELL_SIZE - 2);
    }

    // Final path with magenta thick line (already passes through cell centers)
    if (pathFound && finalPath.size() > 1) {
        stroke(255, 0, 255);
        strokeWeight(3);
        noFill();
        beginShape();
        for (Node n : finalPath) {
            float cx = n.x * CELL_SIZE + CELL_SIZE / 2;
            float cy = n.y * CELL_SIZE + CELL_SIZE / 2;
            vertex(cx-20, cy-20);
        }
        endShape();
        strokeWeight(1);
    }

    popMatrix();
}

// Draw right control panel
void drawPanel() {
    fill(40, 40, 40, 200);
    noStroke();
    rect(panelX, 0, panelWidth, height);

    fill(255);
    textAlign(LEFT, TOP);
    textSize(16);
    text("Control Panel", panelX + 10, 20);

    for (UIButton b : buttons) {
        b.draw();
    }

    // Display statistics (position adjusted to fit after button removal)
    int statsY = 630;
    textSize(12);
    fill(200);
    text("Statistics:", 880, statsY);
    text("Visited: " + visitedCount,  880, statsY + 20);
    text("Path len: " + pathLength, 885, statsY + 40);
    text("CPU cycles: " + cpuCycles, 890, statsY + 60);
    text("·Select: Right-click a cell → remove obstacle / agent / goal",159,630);
    text("·Agent: Left-click empty cell → place blue start point",140,650);
    text("·Goal: Left-click empty cell → place gold goal poin",136,670);
    text("·Obstacle: Left-click on empty cell or obstacle → toggle obstacle; hold left button and drag → draw walls continuously",315,690);    

    // Display grid coordinates (mouse position)
    if (mouseX >= gridOffsetX && mouseX < gridOffsetX + COLS * CELL_SIZE &&
        mouseY >= gridOffsetY && mouseY < gridOffsetY + ROWS * CELL_SIZE) {
        int mx = (mouseX - gridOffsetX) / CELL_SIZE;
        int my = (mouseY - gridOffsetY) / CELL_SIZE;
        fill(150);
        text("Grid: (" + mx + ", " + my + ")", panelX-40 , statsY );
    }
}

public void mousePressed() {
    // If search is running (and not paused), block grid edits to avoid corruption
    if (running && !paused && !algorithmFinished) {
        return;
    }

    // Check if a control panel button is clicked
    for (UIButton b : buttons) {
        if (b.isOver(mouseX, mouseY)) {
            handleButton(b.id);
            return;
        }
    }

    // Otherwise treat as arena interaction
    if (mouseX >= gridOffsetX && mouseX < gridOffsetX + COLS * CELL_SIZE &&
        mouseY >= gridOffsetY && mouseY < gridOffsetY + ROWS * CELL_SIZE) {
        int cx = (mouseX - gridOffsetX) / CELL_SIZE;
        int cy = (mouseY - gridOffsetY) / CELL_SIZE;

        switch (currentTool) {
            case SELECT:
                if (mouseButton == RIGHT) {
                    if (grid[cy][cx] == OBSTACLE) {
                        grid[cy][cx] = EMPTY;
                    } else if (grid[cy][cx] == START) {
                        removeAgentAt(cx, cy);
                    } else if (grid[cy][cx] == GOAL) {
                        removeGoalAt(cx, cy);
                    }
                }
                break;
            case ADD_AGENT:
                if (grid[cy][cx] == EMPTY) {
                    agents.add(new PVector(cx, cy));
                    grid[cy][cx] = START;
                }
                break;
            case ADD_GOAL:
                if (grid[cy][cx] == EMPTY) {
                    goals.add(new PVector(cx, cy));
                    grid[cy][cx] = GOAL;
                }
                break;
            case DRAW_OBSTACLE:
                if (mouseButton == LEFT) {
                    if (grid[cy][cx] == EMPTY) {
                        grid[cy][cx] = OBSTACLE;
                    } else if (grid[cy][cx] == OBSTACLE) {
                        grid[cy][cx] = EMPTY;
                    }
                }
                break;
        }
    }
}

// Continuously draw obstacles when dragging
public void mouseDragged() {
    if (running && !paused && !algorithmFinished) {
        return;
    }
    if (currentTool == Tool.DRAW_OBSTACLE && mouseButton == LEFT) {
        if (mouseX >= gridOffsetX && mouseX < gridOffsetX + COLS * CELL_SIZE &&
            mouseY >= gridOffsetY && mouseY < gridOffsetY + ROWS * CELL_SIZE) {
            int cx = (mouseX - gridOffsetX) / CELL_SIZE;
            int cy = (mouseY - gridOffsetY) / CELL_SIZE;
            if (grid[cy][cx] == EMPTY) {
                grid[cy][cx] = OBSTACLE;
            }
        }
    }
}

// Helper method: remove agent at specified position
void removeAgentAt(int cx, int cy) {
    for (int i = agents.size() - 1; i >= 0; i--) {
        PVector a = agents.get(i);
        if ((int) a.x == cx && (int) a.y == cy) {
            agents.remove(i);
            break;
        }
    }
    grid[cy][cx] = EMPTY;
}

void removeGoalAt(int cx, int cy) {
    for (int i = goals.size() - 1; i >= 0; i--) {
        PVector g = goals.get(i);
        if ((int) g.x == cx && (int) g.y == cy) {
            goals.remove(i);
            break;
        }
    }
    grid[cy][cx] = EMPTY;
}

// ----- Button Event Handling -----
void handleButton(String id) {
    switch (id) {
        case "ALGO_NEXT":
            // Cycle through algorithms
            if (currentAlgo == Algorithm.BFS) currentAlgo = Algorithm.DIJKSTRA;
            else if (currentAlgo == Algorithm.DIJKSTRA) currentAlgo = Algorithm.ASTAR;
            else currentAlgo = Algorithm.BFS;
            resetSearch();
            break;
        case "SPEED_UP":
            speed = min(speed + 1, 20);
            break;
        case "SPEED_DOWN":
            speed = max(speed - 1, 1);
            break;
        case "TOOL_SELECT":
            currentTool = Tool.SELECT;
            break;
        case "TOOL_AGENT":
            currentTool = Tool.ADD_AGENT;
            break;
        case "TOOL_GOAL":
            currentTool = Tool.ADD_GOAL;
            break;
        case "TOOL_OBSTACLE":
            currentTool = Tool.DRAW_OBSTACLE;
            break;
        case "RUN_START":
            // Start search using first agent as start, first goal as goal
            if (agents.size() > 0 && goals.size() > 0) {
                resetSearch();
                // For now, only BFS is implemented
                if (currentAlgo == Algorithm.BFS) {
                    startNode = new Node((int)agents.get(0).x, (int)agents.get(0).y);
                    goalNode  = new Node((int)goals.get(0).x, (int)goals.get(0).y);
                    initBFS();
                    running = true;
                    paused = false;
                    algorithmFinished = false;
                } else if (currentAlgo == Algorithm.DIJKSTRA) {
    startNode = new Node((int)agents.get(0).x, (int)agents.get(0).y);
    goalNode  = new Node((int)goals.get(0).x, (int)goals.get(0).y);
    initDijkstra();
    running = true;
    paused = false;
    algorithmFinished = false;
}
            }
            break;
        case "RUN_PAUSE":
            paused = !paused;
            if (paused) running = false;
            else if (!algorithmFinished) running = true;
            break;
        case "RUN_STEP":
    if (!algorithmFinished) {
        // 未初始化时自动初始化
        if ((bfsQueue == null && dijkstraQueue == null) && agents.size() > 0 && goals.size() > 0) {
            startNode = new Node((int)agents.get(0).x, (int)agents.get(0).y);
            goalNode  = new Node((int)goals.get(0).x, (int)goals.get(0).y);
            if (currentAlgo == Algorithm.BFS) {
                initBFS();
            } else if (currentAlgo == Algorithm.DIJKSTRA) {
                initDijkstra();
            } else {
                println("Algorithm not implemented.");
                return;
            }
            algorithmFinished = false;
        }
        algorithmStep();
        paused = true;
        running = false;
    }
    break;
        case "RUN_RESET":
            resetSearch();
            running = false;
            paused = false;
            algorithmFinished = false;
            break;
        case "RUN_CLEAR":
            agents.clear();
            goals.clear();
            resetGrid();
            resetSearch();
            running = false;
            paused = false;
            algorithmFinished = false;
            break;
    }
    updateButtonLabels();
}

// Reset search state (refreshes visualization)
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
    visited = null;   // will be re-created in initBFS
}

// ========== BFS Algorithm Implementation ==========

void initBFS() {
    bfsQueue = new ArrayDeque<Node>();
    visited = new boolean[ROWS][COLS];

    // Start node is treated as frontier
    bfsQueue.add(startNode);
    visited[startNode.y][startNode.x] = true;
    openList.add(startNode);   // show frontier

    visitedCount = 0;
    pathLength = 0;
    cpuCycles = 0;
    pathFound = false;
}
// ========== Dijkstra Algorithm Initialization ==========
void initDijkstra() {
    dijkstraQueue = new PriorityQueue<>();
    visited = new boolean[ROWS][COLS];
    dist = new int[ROWS][COLS];

    // 初始化所有距离为无穷大
    for (int i = 0; i < ROWS; i++) {
        Arrays.fill(dist[i], Integer.MAX_VALUE);
    }

    // 起点初始化
    startNode.g = 0;
    dist[startNode.y][startNode.x] = 0;

    dijkstraQueue.add(startNode);
    openList.add(startNode);   // 加入可视化的 frontier 列表

    visitedCount = 0;
    pathLength = 0;
    cpuCycles = 0;
    pathFound = false;
}

// Returns true if search continues, false if finished (path found or no path)
boolean algorithmStep() {
  if (currentAlgo == Algorithm.BFS) {
    return stepBFS();
  } else if (currentAlgo == Algorithm.DIJKSTRA) {
    return stepDijkstra();
  } else {
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

    Node current = bfsQueue.poll();
    // Remove from open list (frontier) and add to closed list (explored)
    openList.remove(current);
    closedList.add(current);
    visitedCount++;   // one more node explored

    // Goal reached?
    if (current.equals(goalNode)) {
        pathFound = true;
        algorithmFinished = true;
        running = false;
        reconstructPath(current);
        return false;
    }

    // Four directions (up, down, left, right)
    int[] dx = { -1, 1,  0, 0 };
    int[] dy = {  0, 0, -1, 1 };

    for (int i = 0; i < 4; i++) {
        int nx = current.x + dx[i];
        int ny = current.y + dy[i];

        if (nx >= 0 && nx < COLS && ny >= 0 && ny < ROWS) {
            if (!visited[ny][nx] && grid[ny][nx] != OBSTACLE) {
                visited[ny][nx] = true;
                Node neighbor = new Node(nx, ny);
                neighbor.parent = current;
                bfsQueue.add(neighbor);
                openList.add(neighbor);   // show frontier
            }
        }
    }

    cpuCycles++;   // simple cycle counter
    return true;
}
boolean stepDijkstra() {
    if (dijkstraQueue == null || dijkstraQueue.isEmpty()) {
        algorithmFinished = true;
        running = false;
        return false;
    }

    Node current = dijkstraQueue.poll();

    // 已访问过，直接跳过
    if (visited[current.y][current.x]) {
        return true;
    }

    visited[current.y][current.x] = true;
    openList.remove(current);
    closedList.add(current);
    visitedCount++;

    // 到达终点
    if (current.equals(goalNode)) {
        pathFound = true;
        algorithmFinished = true;
        running = false;
        reconstructPath(current);
        return false;
    }

    // 四个方向
    int[] dx = { -1, 1,  0, 0 };
    int[] dy = {  0, 0, -1, 1 };

    for (int i = 0; i < 4; i++) {
        int nx = current.x + dx[i];
        int ny = current.y + dy[i];

        if (nx >= 0 && nx < COLS && ny >= 0 && ny < ROWS) {
            if (!visited[ny][nx] && grid[ny][nx] != OBSTACLE) {
                int newDist = current.g + 1;
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
    }

    cpuCycles++;
    return true;
}

// Reconstruct path from current (which is the goal node) back to start
void reconstructPath(Node goal) {
    finalPath.clear();
    Node cur = goal;
    while (cur != null) {
        finalPath.add(cur);
        cur = cur.parent;
    }
    // Reverse to start→goal order (drawSearchVisuals expects forward order)
    Collections.reverse(finalPath);
    pathLength = finalPath.size() - 1;   // number of steps
}

// Main entry point (required by Processing)
public static void main(String[] args) {
    PApplet.main("PathfindingArena2");
}
