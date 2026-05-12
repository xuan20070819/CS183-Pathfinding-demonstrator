
//需要用到的算法(algorithm)：BFS，DFS，Dijkstra，A*....(有更好用的待补充)
//寻路算法的基本准备(fundamental)：图与网格(Graphs and Grids)
//用格子代表节点(nodes)，相邻格子之间有边界(edge)，需要为边界加上对应数据的权重(Weight)
//权重的形式(the presentation of weight)可以是时间，代价，距离(time,cost,distance)等
//寻路的基本逻辑(logic of pathfinding)：找出起点和终点之间代价最小的路径(find the way which costs least)
//涂黑格子以标识障碍(make the block balck to signal the barrier)


//准备(prepare)

  import java.util.ArrayDeque;
//高效(efficiency)
//双端队列(Double-ended queue)，为BFS做准备
//在头部和尾部增加、删减元素，不能存放null
  import java.util.ArrayList;
//较慢(slower)
//动态数组(Dynamic Array)
//允许存放null，能自动扩容
  import java.util.PriorityQueue;
  //按优先级取出
  //二叉堆(Binary heap)
  //贪心算法(Greedy Algorithm)

  //初始化



// ---------- Grid Constants ----------(绘制网格)
final int GRID_SIZE = 20;          // 20x20 grid(网格大小为20x20)
final int CELL_SIZE = 38;          // pixel size of each cell (2px reserved for grid lines)
final int GRID_ORIGIN_X = 10;      // top-left corner X of the grid
final int GRID_ORIGIN_Y = 10;      // top-left corner Y of the grid
//网格的坐标

final int GRID_WIDTH = GRID_SIZE * CELL_SIZE;   // 760px
final int GRID_HEIGHT = GRID_SIZE * CELL_SIZE;  // 760px
//总高度

// ---------- UI Elements ----------(绘制简易UI)
// Buttons
final int BUTTON_SEARCH_X = 20;
final int BUTTON_Y = GRID_ORIGIN_Y + GRID_HEIGHT + 16;
final int BUTTON_WIDTH = 110;
final int BUTTON_HEIGHT = 32;

final int BUTTON_ALGO_X = 145;
final int BUTTON_ALGO_WIDTH = 100;

final int BUTTON_CLEAR_X = 260;
final int BUTTON_CLEAR_WIDTH = 80;

// Speed Slider(滑块相关数据)
final int SLIDER_X = 420;
final int SLIDER_Y = BUTTON_Y + 8;
final int SLIDER_WIDTH = 180;
final int SLIDER_MIN = 5;
final int SLIDER_MAX = 10;
float sliderHandleX;

// ---------- Cell State Constants ----------(网格状态)
final int EMPTY = 0;
final int OBSTACLE = 1;
final int START = 2;
final int END = 3;
final int EXPLORED = 4;
final int FRONTIER = 5;
final int PATH = 6;

// ---------- Grid Data ----------(网格数据)
int[][] grid = new int[GRID_SIZE][GRID_SIZE];
int startRow = 0, startCol = 0;                // mutable start point
int endRow = GRID_SIZE - 1, endCol = GRID_SIZE - 1;  // mutable end point
//起点和终点可变

// Window total dimensions(窗口)
final int WINDOW_WIDTH = 800; // 窗口宽度固定800
final int WINDOW_HEIGHT = GRID_ORIGIN_Y + GRID_HEIGHT + 60; // 留出控制面板


// ---------- Color Definitions ----------
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

// ---------- Mouse Hover ----------(检测鼠标位置)
int hoverRow = -1, hoverCol = -1;

// ---------- Search State ----------(检测搜索状态)
boolean searchRunning = false;
boolean searchComplete = false;
boolean pathFound = false;
String currentAlgorithm = "BFS";   // "BFS" or "Dijkstra"

//控制动画速度
int stepsPerFrame = 5;             // number of steps per frame (5~10)，值越大，动画速度越快
int searchStepCount = 0;           // step counter executed(步数计算)

// ---------- Search Data Structures ----------
ArrayDeque<int[]> bfsQueue;         // BFS queue
boolean[][] visited;                // visited nodes
int[][] parentR;                    // parent row for path reconstruction
int[][] parentC;                    // parent column for path reconstruction
ArrayList<int[]> currentFrontier;   // current frontier nodes
ArrayList<int[]> exploredList;      // explored nodes
ArrayList<int[]> finalPath;         // final path




// ---------- Initialization ----------

//每次查找完重置查找状态
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

//初始化网格
void initGrid() {
  for (int r = 0; r < GRID_SIZE; r++) {
    for (int c = 0; c < GRID_SIZE; c++) {
      grid[r][c] = EMPTY;//清空网格
    }
  }
  grid[startRow][startCol] = START;//重新制造起点
  grid[endRow][endCol] = END;//重新制造终点
  resetSearchState();//利用重置函数重置之前的残留数据，进行下一次遍历
}

void setup() {
  size(800, 830);//画布
  frameRate(60);//动画速度，越大越快
  textFont(createFont("Arial", 13));//字体和字号大小
  initGrid();//初始化网格
  sliderHandleX = SLIDER_X + map(stepsPerFrame, SLIDER_MIN, SLIDER_MAX, 0, SLIDER_WIDTH);
  //控制动画
  //从 main里面复制粘贴的模块
  //交互体验后续等待优化
}

void initBFS() {
  bfsQueue = new ArrayDeque<int[]>();//创建BFS专用的队列

  visited = new boolean[GRID_SIZE][GRID_SIZE];//记录已经走过的节点
  //记录父节点用于路径回溯和寻路
  //BFS最后的结果是能不能到达终点，路径需要用父结点倒推
  //利用父节点可以在遇到障碍时返回上一级，再次进行寻路
  parentR = new int[GRID_SIZE][GRID_SIZE];
  parentC = new int[GRID_SIZE][GRID_SIZE];
  //统一数组大小，防止越界报错
  currentFrontier = new ArrayList<int[]>();//记录最近的边界(现在正在探索的节点)
  exploredList = new ArrayList<int[]>();//已经走过的节点
  finalPath = new ArrayList<int[]>();//最终代价最少的路线

  bfsQueue.offer(new int[]{startRow, startCol});//将起点加入双端队列
  visited[startRow][startCol] = true; //标记起点探索的布尔值为真
  currentFrontier.add(new int[]{startRow, startCol});//将起点加入当前正在探索的节点

  searchRunning = true;//到达起点，开始查找路线
  searchComplete = false;//未到达终点，标记为false
  pathFound = false;
  searchStepCount = 0; //起点步数为0，开始计数
}

boolean stepBFS() {
  if (bfsQueue == null || bfsQueue.isEmpty()) {
    //检测BFS所用队列是否为空
    //如果是，则停止寻路，返回false，停止计算步数
    searchComplete = true;
    searchRunning = false;
    return false;
  }

  int[] current = bfsQueue.poll();//从队列头部获得最近经过的节点

  if (current[0] == endRow && current[1] == endCol) {
    //判定是否到达终点
    pathFound = true;
    searchComplete = true;
    searchRunning = false;
    reconstructPath();
    return false;
  }

  exploredList.add(current); //将终点加入已经走过的节点

  int[][] directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};//上下左右四个方位移动
  //后续可能有待升级迭代功能增加至八个方向移动
  //后续根据A*和D*等一系列算法迭代或添加功能进行微调

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
//---------绘制网格----------
void drawGrid() {
  for (int r = 0; r < GRID_SIZE; r++) {
    for (int c = 0; c < GRID_SIZE; c++) {
      int x = GRID_ORIGIN_X + c * CELL_SIZE;
      int y = GRID_ORIGIN_Y + r * CELL_SIZE;

      color cellColor = COLOR_EMPTY;
//上色标识
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


//------------UI绘制-----------
//坐等可视化界面后续优化
//此处仅为演示算法使用，并非最终效果
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

//--------背景---------
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



//----------交互-----------
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


//A*算法模块
//核心结构
PriorityQueue<AStarNode> aStarPQ;     // A*优先级队列，fScore升序排列
int[][] aStarGScore;                  // 实际代价
int[][] aStarFScore;                  // 总代价
boolean[][] aStarInOpenSet;           // true表示节点在优先级队列中待处理

//A*节点
class AStarNode implements Comparable<AStarNode> {
  int row, col;   // 
  int fScore;     // f值 = g值(实际代价) + h值(启发式预估代价)
  
 
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


//A*初始化
 
void initAStar() {
  aStarPQ = new PriorityQueue<AStarNode>();  // 创建优先级队列

  aStarGScore = new int[GRID_SIZE][GRID_SIZE];
  aStarFScore = new int[GRID_SIZE][GRID_SIZE];
  aStarInOpenSet = new boolean[GRID_SIZE][GRID_SIZE];

  // 回溯父节点和路径
  visited = new boolean[GRID_SIZE][GRID_SIZE];
  parentR = new int[GRID_SIZE][GRID_SIZE];
  parentC = new int[GRID_SIZE][GRID_SIZE];
  currentFrontier = new ArrayList<int[]>();  // 当前待探索的边界节点
  exploredList = new ArrayList<int[]>();     // 已探索完成的节点
  finalPath = new ArrayList<int[]>();        // 最终路径

  // 未设置起点之前默认无穷大
  for (int r = 0; r < GRID_SIZE; r++) {
    for (int c = 0; c < GRID_SIZE; c++) {
      aStarGScore[r][c] = Integer.MAX_VALUE;
      aStarFScore[r][c] = Integer.MAX_VALUE;
    }
  }

  // 初始代价：
  // g(start) = 0（起点到自身无代价）
  // f(start) = h(start)（仅需预估到终点的距离）
  aStarGScore[startRow][startCol] = 0;
  int startHeuristic = heuristic(startRow, startCol);
  aStarFScore[startRow][startCol] = startHeuristic;

  // 添加起点
  aStarPQ.offer(new AStarNode(startRow, startCol, startHeuristic));
  aStarInOpenSet[startRow][startCol] = true;
  currentFrontier.add(new int[]{startRow, startCol});

  // A*搜索状态
  searchRunning = true;   // 搜索开始
  searchComplete = false; // 未完成
  pathFound = false;      // 未找到路径
  searchStepCount = 0;    // 步数重置
}

//计算启发式代价
int heuristic(int row, int col) {
  return Math.abs(row - endRow) + Math.abs(col - endCol);
}


boolean stepAStar() {
  //检查空队列
  if (aStarPQ == null || aStarPQ.isEmpty()) {
    searchComplete = true;
    searchRunning = false;
    return false;
  }

  // 贪心选择
  AStarNode current = aStarPQ.poll();
  int currentRow = current.row;
  int currentCol = current.col;

  //跳过已处理节点
  boolean isExplored = false;
  for (int[] exp : exploredList) {
    if (exp[0] == currentRow && exp[1] == currentCol) {
      isExplored = true;
      break;
    }
  }
  if (isExplored) {
    return true;  // 继续下一节点
  }

  //标记节点为已探索，移出开放集
  exploredList.add(new int[]{currentRow, currentCol});
  aStarInOpenSet[currentRow][currentCol] = false;

  // 确认到达终点，重建路径
  if (currentRow == endRow && currentCol == endCol) {
    int r = endRow;
    int c = endCol;
    // 构建路径
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

  //扩展四个方向相邻节点
  int[][] directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};  // 上、下、左、右

  for (int[] dir : directions) {
    int newR = currentRow + dir[0];
    int newC = currentCol + dir[1];

    // 边界检查和障碍物检查
    if (newR >= 0 && newR < GRID_SIZE && newC >= 0 && newC < GRID_SIZE &&
        grid[newR][newC] != OBSTACLE) {

      // 计算新路径的g值
      int tentativeGScore = aStarGScore[currentRow][currentCol] + 1;

      // 更新g值更小路径
      if (tentativeGScore < aStarGScore[newR][newC]) {
        //更新父节点记录（用于路径回溯）
        parentR[newR][newC] = currentRow;
        parentC[newR][newC] = currentCol;
        
        // 更新g值和f值
        aStarGScore[newR][newC] = tentativeGScore;
        int hScore = heuristic(newR, newC);
        int fScore = tentativeGScore + hScore;
        aStarFScore[newR][newC] = fScore;

        // 如果节点不在开放集中，加入优先级队列
        if (!aStarInOpenSet[newR][newC]) {
          aStarPQ.offer(new AStarNode(newR, newC, fScore));
          aStarInOpenSet[newR][newC] = true;

          // 更新可视化边界列表
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

  searchStepCount++;  // 步数递增
  return true;  // 继续搜索
}
