
//需要用到的算法（algorithm）：BFS，DFS，Dijkstra，A*....（有更好用的待补充）
//寻路算法的基本准备（fundamental）：图与网格（Graphs and Grids）
//用格子代表节点（nodes），相邻格子之间有边界（edge），需要为边界加上对应数据的权重（Weight）
//权重的形式（the presentation of weight）可以是时间，代价，距离(time,cost,distance)等
//寻路的基本逻辑(logic of pathfinding)：找出起点和终点之间代价最小的路径（find the way which costs least）
//涂黑格子以标识障碍（make the block balck to signal the barrier）


//准备（prepare）

  import java.util.ArrayDeque;
//高效（efficiency）
//双端队列（Double-ended queue），为BFS做准备
//在头部和尾部增加、删减元素，不能存放null
  import java.util.ArrayList;
//较慢（slower）
//动态数组（Dynamic Array）
//允许存放null，能自动扩容
  import java.util.PriorityQueue;
  //按优先级取出
  //二叉堆（Binary heap）
  //贪心算法（Greedy Algorithm）

  //初始化



  // ---------- Grid Constants ----------（绘制网格）
final int GRID_SIZE = 20;          // 20x20 grid（网格大小为20x20）
final int CELL_SIZE = 38;          // pixel size of each cell (2px reserved for grid lines)
final int GRID_ORIGIN_X = 10;      // top-left corner X of the grid
final int GRID_ORIGIN_Y = 10;      // top-left corner Y of the grid
//网格的坐标

final int GRID_WIDTH = GRID_SIZE * CELL_SIZE;   // 760px
final int GRID_HEIGHT = GRID_SIZE * CELL_SIZE;  // 760px
//总高度

// ---------- Cell State Constants ----------（网格状态）
final int EMPTY = 0;
final int OBSTACLE = 1;
final int START = 2;
final int END = 3;
final int EXPLORED = 4;
final int FRONTIER = 5;
final int PATH = 6;

// ---------- Grid Data ----------（网格数据）
int[][] grid = new int[GRID_SIZE][GRID_SIZE];
int startRow = 0, startCol = 0;                // mutable start point
int endRow = GRID_SIZE - 1, endCol = GRID_SIZE - 1;  // mutable end point
//起点和终点可变

// Window total dimensions（窗口）
final int WINDOW_WIDTH = 800;（窗口宽度固定800）
final int WINDOW_HEIGHT = GRID_ORIGIN_Y + GRID_HEIGHT + 60;（留出控制面板）


// ---------- Color Definitions ----------
final color COLOR_EMPTY = #FFFFFF;
final color COLOR_OBSTACLE = #3C3C3C;
final color COLOR_START = #4CAF50;
final color COLOR_END = #F44336;
final color COLOR_EXPLORED = #2233cae4;
final color COLOR_FRONTIER = #066790ff;
final color COLOR_PATH = #e17b1cff;
final color COLOR_HOVER = #999999ff;
final color COLOR_GRID_LINE = #BDBDBD;
final color COLOR_BG = #FAFAFA;
final color COLOR_UI_BG = #ECEFF1;
final color COLOR_BUTTON = #607D8B;
final color COLOR_BUTTON_HOVER = #78909C;
final color COLOR_BUTTON_ACTIVE = #FF7043;
final color COLOR_SLIDER_TRACK = #90A4AE;
final color COLOR_SLIDER_THUMB = #37474F;
final color COLOR_TEXT = #263238;

// ---------- Mouse Hover ----------
int hoverRow = -1, hoverCol = -1;

// ---------- Search State ----------（）
boolean searchRunning = false;
boolean searchComplete = false;
boolean pathFound = false;
String currentAlgorithm = "BFS";   // "BFS" or "Dijkstra"

//------BFS------

// BFS queue
ArrayDeque<int[]> bfsQueue;

