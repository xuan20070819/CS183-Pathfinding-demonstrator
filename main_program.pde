import java.util.*;

  // ----- Grid Settings -----
  final int COLS = 40;
  final int ROWS = 30;
  final int CELL_SIZE = 20;
  int gridOffsetX, gridOffsetY;
  // Grid cell states
  final int EMPTY= 0;
  final int OBSTACLE = 1;
  final int START= 2;
  final int GOAL= 3;
  int[][] grid;
  
  // List of agents (start points)
  ArrayList<PVector> agents = new ArrayList<PVector>();
  // List of goal points
  ArrayList<PVector> goals  = new ArrayList<PVector>();
  
  // ----- Algorithm Related -----
  enum Algorithm { BFS, DIJKSTRA, ASTAR }
  Algorithm currentAlgo = Algorithm.ASTAR;
  
  // Speed control: frames per expansion step
  int speed = 5;                // 1-20, higher = faster (steps per frame)
  boolean running = false;
  boolean paused = false;
  boolean stepMode = false;     // Single-step mode
  
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
  
  // Comparison mode (simplified: left/right split screen for two algorithms)
  boolean compareMode = false;
  Algorithm algoLeft = Algorithm.ASTAR;
  Algorithm algoRight = Algorithm.BFS;
  
  // ----- UI Controls -----
  enum Tool { SELECT, ADD_AGENT, ADD_GOAL, DRAW_OBSTACLE }
  Tool currentTool = Tool.SELECT;
  
  // Button area (control panel on the right)
  int panelX, panelWidth;
  
  // Custom buttons (simple self-drawn)
  ArrayList<UIButton> buttons = new ArrayList<UIButton>();
  
  // ----- Inner Class: Grid Node (for pathfinding) -----
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
  
  // ----- Processing Settings -----
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
    goals.add(new PVector(COLS-6, ROWS-6));
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
      int cx = (int)a.x;
      int cy = (int)a.y;
      if (cx >= 0 && cx < COLS && cy >= 0 && cy < ROWS) {
        if (grid[cy][cx] == EMPTY) grid[cy][cx] = START;
      }
    }
    // Mark goals
    for (PVector g : goals) {
      int cx = (int)g.x;
      int cy = (int)g.y;
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
    
    buttons.add(new UIButton(x, yBase, btnW, btnH, "Algo: A*",     "ALGO_ASTAR"));
    buttons.add(new UIButton(x, yBase+40, btnW, btnH, "Next Algo", "ALGO_NEXT"));
    
    buttons.add(new UIButton(x, yBase+90, btnW/2-2, btnH, "-Speed", "SPEED_DOWN"));
    buttons.add(new UIButton(x+btnW/2+2, yBase+90, btnW/2-2, btnH, "+Speed", "SPEED_UP"));
    
    buttons.add(new UIButton(x, yBase+140, btnW, btnH, "Compare: OFF", "TOGGLE_COMPARE"));
    
    buttons.add(new UIButton(x, yBase+190, btnW, btnH, "Tool: Select", "TOOL_SELECT"));
    buttons.add(new UIButton(x, yBase+230, btnW, btnH, "Tool: Agent",  "TOOL_AGENT"));
    buttons.add(new UIButton(x, yBase+270, btnW, btnH, "Tool: Goal",   "TOOL_GOAL"));
    buttons.add(new UIButton(x, yBase+310, btnW, btnH, "Tool: Obstacle","TOOL_OBSTACLE"));
    
    buttons.add(new UIButton(x, yBase+370, btnW, btnH, "Start",   "RUN_START"));
    buttons.add(new UIButton(x, yBase+410, btnW, btnH, "Pause",   "RUN_PAUSE"));
    buttons.add(new UIButton(x, yBase+450, btnW, btnH, "Step",    "RUN_STEP"));
    buttons.add(new UIButton(x, yBase+490, btnW, btnH, "Reset",   "RUN_RESET"));
    buttons.add(new UIButton(x, yBase+530, btnW, btnH, "Clear All","RUN_CLEAR"));
  }
  
  // Update button labels
  void updateButtonLabels() {
    for (UIButton b : buttons) {
      if (b.id.equals("ALGO_ASTAR")) {
        b.label = "Algo: " + currentAlgo.toString();
      } else if (b.id.equals("TOGGLE_COMPARE")) {
        b.label = "Compare: " + (compareMode ? "ON" : "OFF");
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
    
    // Update algorithm animation
    if (running && !paused && !algorithmFinished) {
      for (int i = 0; i < speed; i++) {
        if (!algorithmStep()) {
          algorithmFinished = true;
          break;
        }
      }
    }
  }
  
  // Draw grid and all visual elements
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
    
    // Draw agent icons (starting points may be multiple, with numbers inside circles)
    for (int i = 0; i < agents.size(); i++) {
      PVector a = agents.get(i);
      float cx = a.x * CELL_SIZE + CELL_SIZE/2;
      float cy = a.y * CELL_SIZE + CELL_SIZE/2;
      fill(0, 150, 255);
      noStroke();
      ellipse(cx, cy, CELL_SIZE*0.8, CELL_SIZE*0.8);
      fill(255);
      textAlign(CENTER, CENTER);
      textSize(10);
      text(str(i+1), cx, cy-1);
    }
    
    // Draw goal point icons
    for (int i = 0; i < goals.size(); i++) {
      PVector g = goals.get(i);
      float cx = g.x * CELL_SIZE + CELL_SIZE/2;
      float cy = g.y * CELL_SIZE + CELL_SIZE/2;
      fill(255, 200, 0);
      noStroke();
      ellipse(cx, cy, CELL_SIZE*0.8, CELL_SIZE*0.8);
      fill(0);
      textAlign(CENTER, CENTER);
      textSize(8);
      text("G" + (i+1), cx, cy);
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
      rect(n.x * CELL_SIZE + 1, n.y * CELL_SIZE + 1, CELL_SIZE-2, CELL_SIZE-2);
    }
    
    // Frontier (open list) with semi-transparent yellow
    fill(255, 255, 0, 100);
    for (Node n : openList) {
      rect(n.x * CELL_SIZE + 1, n.y * CELL_SIZE + 1, CELL_SIZE-2, CELL_SIZE-2);
    }
    
    // Final path with magenta thick line
    if (pathFound && finalPath.size() > 1) {
      stroke(255, 0, 255);
      strokeWeight(3);
      noFill();
      beginShape();
      for (Node n : finalPath) {
        float cx = n.x * CELL_SIZE + CELL_SIZE/2;
        float cy = n.y * CELL_SIZE + CELL_SIZE/2;
        vertex(cx, cy);
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
    text("Control Panel", panelX+10, 20);
    
    for (UIButton b : buttons) {
      b.draw();
    }
    
    // Display statistics
    int statsY = 580;
    textSize(12);
    fill(200);
    text("Statistics:", panelX+10, statsY);
    text("Visited: " + visitedCount, panelX+10, statsY+20);
    text("Path len: " + pathLength, panelX+10, statsY+40);
    text("CPU cycles: " + cpuCycles, panelX+10, statsY+60);
    
    // Display grid coordinates (mouse position)
    if (mouseX >= gridOffsetX && mouseX < gridOffsetX + COLS*CELL_SIZE &&
        mouseY >= gridOffsetY && mouseY < gridOffsetY + ROWS*CELL_SIZE) {
      int mx = (mouseX - gridOffsetX) / CELL_SIZE;
      int my = (mouseY - gridOffsetY) / CELL_SIZE;
      fill(150);
      text("Grid: (" + mx + ", " + my + ")", panelX+10, statsY+90);
    }
  }
  
  // ----- Mouse Interaction -----
  @Override
  public void mousePressed() {
    // Check if a control panel button is clicked
    for (UIButton b : buttons) {
      if (b.isOver(mouseX, mouseY)) {
        handleButton(b.id);
        return;
      }
    }
    
    // Otherwise treat as arena interaction
    if (mouseX >= gridOffsetX && mouseX < gridOffsetX + COLS*CELL_SIZE &&
        mouseY >= gridOffsetY && mouseY < gridOffsetY + ROWS*CELL_SIZE) {
      int cx = (mouseX - gridOffsetX) / CELL_SIZE;
      int cy = (mouseY - gridOffsetY) / CELL_SIZE;
      
      switch (currentTool) {
        case SELECT:
          // Can select and drag elements (simplified: currently only supports deleting obstacles with right-click)
          // Here implement right-click to delete obstacle, left-click drag can be added later
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
    if (currentTool == Tool.DRAW_OBSTACLE && mouseButton == LEFT) {
      if (mouseX >= gridOffsetX && mouseX < gridOffsetX + COLS*CELL_SIZE &&
          mouseY >= gridOffsetY && mouseY < gridOffsetY + ROWS*CELL_SIZE) {
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
    for (int i = agents.size()-1; i >= 0; i--) {
      PVector a = agents.get(i);
      if ((int)a.x == cx && (int)a.y == cy) {
        agents.remove(i);
        break;
      }
    }
    grid[cy][cx] = EMPTY;
  }
  
  void removeGoalAt(int cx, int cy) {
    for (int i = goals.size()-1; i >= 0; i--) {
      PVector g = goals.get(i);
      if ((int)g.x == cx && (int)g.y == cy) {
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
      case "TOGGLE_COMPARE":
        compareMode = !compareMode;
        resetSearch();
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
        if (agents.size() > 0 && goals.size() > 0) {
          initSearch();
          running = true;
          paused = false;
          algorithmFinished = false;
        }
        break;
      case "RUN_PAUSE":
        paused = !paused;
        if (paused) running = false;
        else if (!algorithmFinished) running = true;
        break;
      case "RUN_STEP":
        if (!algorithmFinished) {
          if (!running) {
            initSearch();
          }
          algorithmStep();
          algorithmFinished = !hasMoreSteps();
        }
        paused = true;
        running = false;
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
  
  // ----- Pathfinding Algorithm Core -----
  
  // Initialize search (select first agent and first goal as start and end)
  void initSearch() {
    openList.clear();
    closedList.clear();
    finalPath.clear();
    visitedCount = 0;
    pathLength = 0;
    cpuCycles = 0;
    pathFound = false;
    algorithmFinished = false;
    
    if (agents.size() == 0 || goals.size() == 0) return;
    
    PVector startVec = agents.get(0);
    PVector goalVec = goals.get(0);
    startNode = new Node((int)startVec.x, (int)startVec.y);
    goalNode  = new Node((int)goalVec.x, (int)goalVec.y);
    
    startNode.g = 0;
    startNode.h = heuristic(startNode, goalNode);
    openList.add(startNode);
  }
  
  // Manhattan distance heuristic (A*)
  int heuristic(Node a, Node b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
  }
  
  // Execute one expansion step (returns true if there are more steps to execute)
  boolean algorithmStep() {
    if (algorithmFinished || openList.isEmpty()) {
      algorithmFinished = true;
      return false;
    }
    
    cpuCycles++;
    
    // Select current node (different strategy based on algorithm)
    Node current = null;
    if (currentAlgo == Algorithm.BFS) {
      // BFS: queue FIFO, openList in insertion order
      current = openList.remove(0);
    } else {
      // Dijkstra or A*: minimum f = g + h
      // Using priority queue logic, simplified: traverse to find minimum
      int minIndex = 0;
      int minF = openList.get(0).f();
      for (int i = 1; i < openList.size(); i++) {
        int f = openList.get(i).f();
        if (f < minF) {
          minF = f;
          minIndex = i;
        }
      }
      current = openList.remove(minIndex);
    }
    
    // If reached goal
    if (current.x == goalNode.x && current.y == goalNode.y) {
      reconstructPath(current);
      pathFound = true;
      algorithmFinished = true;
      return false;
    }
    
    closedList.add(current);
    visitedCount++;
    
    // Expand neighbors (4 directions)
    int[][] dirs = {{0,1},{0,-1},{1,0},{-1,0}};
    for (int[] d : dirs) {
      int nx = current.x + d[0];
      int ny = current.y + d[1];
      
      if (nx < 0 || nx >= COLS || ny < 0 || ny >= ROWS) continue;
      if (grid[ny][nx] == OBSTACLE) continue;
      
      Node neighbor = new Node(nx, ny);
      if (containsNode(closedList, neighbor)) continue;
      
      int tentativeG = current.g + 1;  // Movement cost is 1
      
      // Check if in openList
      Node existing = getNodeFromList(openList, neighbor);
      if (existing == null) {
        neighbor.g = tentativeG;
        if (currentAlgo == Algorithm.ASTAR) {
          neighbor.h = heuristic(neighbor, goalNode);
        } else if (currentAlgo == Algorithm.DIJKSTRA) {
          neighbor.h = 0;
        } else {
          // BFS: do not compute h, but for queue order, can be ignored
          neighbor.h = 0;
        }
        neighbor.parent = current;
        openList.add(neighbor);
      } else if (tentativeG < existing.g) {
        existing.g = tentativeG;
        existing.parent = current;
        // Reordering (would require re-insert, simplified for now; affects efficiency but functionally correct)
      }
    }
    
    return true;
  }
  
  boolean hasMoreSteps() {
    return !openList.isEmpty() && !algorithmFinished;
  }
  
  // Reconstruct path
  void reconstructPath(Node node) {
    finalPath.clear();
    Node n = node;
    while (n != null) {
      finalPath.add(0, n);
      n = n.parent;
    }
    pathLength = finalPath.size() - 1; // Step count
  }
  
  // Utility functions
  boolean containsNode(List<Node> list, Node node) {
    for (Node n : list) {
      if (n.x == node.x && n.y == node.y) return true;
    }
    return false;
  }
  
  Node getNodeFromList(List<Node> list, Node node) {
    for (Node n : list) {
      if (n.x == node.x && n.y == node.y) return n;
    }
    return null;
  }
  
  // Reset search state
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
  }
  
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
      return mx >= x && mx <= x+w && my >= y && my <= y+h;
    }
    
    void draw() {
      hovered = isOver(mouseX, mouseY);
      fill(hovered ? 100 : 70);
      stroke(150);
      rect(x, y, w, h, 5);
      fill(255);
      textAlign(CENTER, CENTER);
      textSize(12);
      text(label, x + w/2, y + h/2);
    }
  }
  
  // Main entry point (required by Processing)
  public static void main(String[] args) {
    PApplet.main("PathfindingArena");
  }
