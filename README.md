# AirCombat

AirCombat 是一个基于 Open3D 的无人机集群攻防仿真框架，支持多类型无人机、自动追逐行为、碰撞检测和实时3D可视化。

## 功能特性

- **多类型无人机**：
  - 攻击机 (AttackDrone)：红色，高速度，高攻击力
  - 肉盾机 (TankDrone)：红色，高血量，低速度
  - 侦察机 (ScoutDrone)：蓝色，最高速度，低血量
  - 拦截机 (InterceptorDrone)：蓝色，中等速度，中等攻击力

- **智能行为**：
  - 自动追逐最近敌人
  - 碰撞检测和伤害系统
  - 随机生成新无人机

- **实时可视化**：
  - 3D场景显示无人机、路径轨迹、爆炸效果
  - HUD界面显示队伍统计
  - 鼠标右键重置视角

- **模块化设计**：
  - 无人机基类和继承体系
  - 控制模块（移动、碰撞）
  - 感知模块（雷达探测）
  - 可视化模块（Open3D GUI）

## 项目结构

```
AirCombat/
├── main.py                      # 主程序入口
├── config.yaml                  # 配置文件
├── README.md                    # 说明文档
│
├── drones/                      # 无人机模块
│   ├── base_drone.py            # 无人机基类，定义通用属性和方法
│   ├── offensive/               # 进攻方无人机
│   │   ├── attack_drone.py      # 攻击机实现
│   │   └── tank_drone.py        # 肉盾机实现
│   └── defensive/               # 防守方无人机
│       ├── scout_drone.py       # 侦察机实现
│       └── interceptor_drone.py # 拦截机实现
│
├── Controller/                  # 控制模块
│   ├── single_control.py        # 单机控制逻辑（移动、边界检查）
│   └── collision_handler.py     # 碰撞检测和伤害处理
│
 
│
├── Visual/                      # 可视化模块
│   ├── open3d_display.py        # Open3D GUI封装和界面管理
│   └── render_utils.py          # 渲染辅助工具（网格生成等）
│
├── algorithms/                  # 算法模块（预留）
├── Data/                        # 数据存储（预留）
└── utils/                       # 工具模块
    └── logger.py                # 日志工具
```

### 模块说明

- **drones/**: 无人机类层次结构。`base_drone.py` 定义基础属性如位置、健康、轨迹。子类实现特定行为。
- **Controller/**: 控制逻辑。`single_control.py` 处理移动和边界，`collision_handler.py` 检测碰撞并应用伤害。
 
- **Visual/**: 可视化组件。`open3d_display.py` 管理GUI窗口和3D渲染，`render_utils.py` 提供几何体生成。
- **utils/**: 通用工具，如日志记录。

## 安装和运行

先看这个不然运行大概率报错，目前应该只能看到类似小蝌蚪找妈妈，其他都没有hhh

### 1. 安装环境要求

- Python 3.12+
- Open3D 0.19.0+

确保安装 Python 3.12+ （我用的3.12.13但这个无所谓）和 pip。

### 2. 安装依赖

```bash
pip install open3d pyyaml
#或
pip install -r requirements.txt
```

一键安装解君愁，还有问题自己打开requirements一个个对照着装

### 3. 运行仿真

```bash
python main.py
```

如果 Open3D 可视化可用，会打开一个窗口显示无人机实时位置；若加载了
`Data/maps/*.obj` 城市模型会先做体素化，启动会慢几秒。未安装 open3d 或窗口初始化
失败时会自动退化为无界面模式，只输出日志。

### 控制说明

- 鼠标拖拽：旋转视角
- 鼠标滚轮：缩放
- 右键单击：重置视角
- `R`：重置视角（键盘）
- `P`：暂停 / 继续仿真
- 终端按键 `r`：清空所有路径缓存并立即重规划

### 4. 配置调整

编辑 `config.yaml` 文件来调整参数：

- **地图设置**：`map.width`, `map.height`
- **无人机属性**：`drones.attack.speed`, `drones.tank.health` 等
- **仿真参数**：`simulation.fps`, `simulation.duration`

## 开发（TODO

项目采用模块化设计：

- 添加新无人机：继承 `BaseDrone`，实现特定属性
- 扩展控制：在 `Controller/` 添加新逻辑
- 改进可视化：在 `Visual/` 修改渲染
- 添加新算法：在 `algorithms/` 添加新算法
- 扩展数据存储：在 `data/` 添加新数据（地图/扫描数据
- 扩展工具库：在 `utils/` 添加新工具（位置计算数据

## 模块说明

### drones/ - 无人机类型
- **AttackDrone**: 高速攻击机，适合快速打击
- **TankDrone**: 高血量肉盾机，适合吸收伤害
- **ScoutDrone**: 超高速侦察机，适合情报收集
- **InterceptorDrone**: 中等性能拦截机，适合防御

### Controller/ - 控制系统
- **single_control.py**: 处理单个无人机的移动和边界检查
- **swarm_control.py**: 集群任务分配和协同控制
- **collision_handler.py**: 检测和处理无人机间的碰撞

 

### Visual/ - 可视化
- **open3d_display.py**: Open3D 渲染引擎封装
- **render_utils.py**: 几何体创建和渲染辅助

### algorithms/ - 核心算法
- **auction_assign.py**: 任务分配拍卖算法（待实现）
- **cbs_pathplan.py**: 冲突检测和路径规划算法（待实现）

### data/ - 数据管理
存储地图、探测记录和仿真轨迹数据。

## 关于 __pycache__

项目中出现的 `__pycache__/` 目录是 Python 的字节码缓存文件夹：

- **作用**：存储编译后的 `.pyc` 文件，加速模块导入
- **自动生成**：运行 Python 程序时自动创建
- **安全删除**：可以安全删除，会在下次运行时重新生成
- **版本管理**：通常在 `.gitignore` 中忽略，不提交到 Git

## 常见问题 Q&A

### Q: 项目中的 `__init__.py` 文件都是什么？

**A**: `__init__.py` 是 Python 包的标识文件，让 Python 解释器识别目录为可导入的包：

- **包标记**：告诉 Python "这个目录是一个包"
- **导入支持**：支持 `from package import module` 语法
- **初始化**：可以在包导入时执行初始化代码
- **项目作用**：你的项目有 8 个 `__init__.py` 文件，分别标记各个模块目录为包

例如，没有 `drones/__init__.py`，你就无法使用 `from drones.base_drone import BaseDrone` 这样的导入。

## 代码框架详解

### 核心架构设计

AirCombat 采用模块化设计，各模块职责清晰分离：

```
主程序 (main.py) → 配置加载 → 模块初始化 → 仿真循环 → 可视化输出
                      ↓
               无人机创建 ← 控制系统 ← 感知系统 ← 算法模块
                      ↓
                   数据存储 ← 工具库
```

### 详细代码内容

#### 1. 主入口 (`main.py`)
```python
# 主要功能：
# 1. 加载 YAML 配置
# 2. 创建无人机团队（4种机型）
# 3. 初始化雷达和显示系统
# 4. 运行仿真主循环（60秒，30FPS）
# 5. 处理 Open3D 可视化（支持降级到无界面模式）
```

#### 2. 无人机系统 (`drones/`)
- **`base_drone.py`**: 无人机基类，定义通用属性和方法
  - 属性：位置、速度、血量、攻击力、电池、最大速度
  - 方法：位置更新、速度设置、伤害处理、存活检查

- **具体机型**（继承自 BaseDrone）：
  - `AttackDrone`: 高速攻击机 (speed 8.0, max_speed 32.0, 攻击力25, 血量80)
  - `TankDrone`: 高血量肉盾机 (speed 4.8, max_speed 22.0, 攻击力15, 血量150)
  - `ScoutDrone`: 超高速侦察机 (speed 15.0, max_speed 40.0, 攻击力5, 血量60)
  - `InterceptorDrone`: 中等拦截机 (speed 12.0, max_speed 35.0, 攻击力20, 血量90)
  - `EWDrone`: 电子战机 max_speed 32.0；`AntiRadiationDrone`: max_speed 32.0

#### 3. 控制系统 (`Controller/`)
- **`single_control.py`**: 单机运动控制
  - 位置更新（基于速度和时间）
  - 边界检查（限制在地图范围内）

- **`swarm_control.py`**: 集群控制框架
  - 任务分配接口（调用算法模块）
  - 队形控制准备

- **`collision_handler.py`**: 碰撞检测
  - 检测无人机间距离
  - 碰撞时反向速度避免重叠

 
#### 5. 可视化系统 (`Visual/`)
- **`open3d_display.py`**: Open3D 渲染引擎
  - 窗口创建和管理
  - 实时几何体更新
  - 错误处理（支持无界面模式）

- **`render_utils.py`**: 渲染工具
  - 无人机几何体创建（球体）
  - 路径线段生成

#### 6. 算法模块 (`algorithms/`)
- **`auction_assign.py`**: 改进拍卖 + CMA 动态任务分配（参数见 `config.assignment`）

- **`cbs_pathplan.py`**: 基于网格的 CBS
  - 顶点冲突 + 交换（边）冲突，低层 A* 在 `(格子, 时刻)` 上搜索
  - 约束树 + 闭集；超时返回无冲突子集；同格起点/终点自动错开
  - 单机 `a_star_plan_world` / `line_of_sight_world` 供 PathTracker 使用

#### 7. 数据存储 (`data/`)
- **`maps/base_map.json`**: 地图配置
- **`maps/base_map.json`**: 地图配置
- **`simulation/trajectories.json`**: 轨迹数据（空）

#### 8. 工具库 (`utils/`)
- **`geometry.py`**: 几何计算工具
  - 距离计算函数
  - 位置边界限制

- **`logger.py`**: 日志系统
  - 统一日志接口
  - 支持不同级别日志输出

### 配置系统 (`config.yaml`)

YAML 配置文件包含所有可调参数：

```yaml
map:           # 地图设置
simulation:    # 仿真参数
drones:        # 四种机型详细配置
  attack:      # 攻击机参数
  tank:        # 肉盾机参数
  scout:       # 侦察机参数
  interceptor: # 拦截机参数
```

### 运行流程

1. **初始化阶段**：
  - 加载配置
  - 创建4个无人机实例
  - 初始化显示系统

2. **仿真循环**（每帧）：
  - 更新所有无人机位置
  - 可视化渲染
  - 帧率控制（30FPS）

3. **结束处理**：
   - 关闭显示窗口
   - 输出仿真完成日志

### 扩展接口

框架预留了多个扩展点：

- **算法接口**：`algorithms/` 中的占位符可替换为真实算法
- **控制逻辑**：`swarm_control.py` 可添加复杂协同行为
 
- **可视化扩展**：`Visual/` 可添加雷达圈、路径显示
- **数据持久化**：`data/` 中的 JSON 文件可实现日志保存

这个框架提供了完整的无人机仿真基础，可以根据需求逐步添加高级功能。




## 📅 4/26 更新：网格避障 + CBS 多机路径规划

### 一、地图网格系统 (`utils/map_grid.py`)

将城市建筑转换为统一的障碍物网格，供路径规划查询。

**网格参数**：201×201，分辨率 5 米/格，覆盖 [-500, 500] 米范围。

| 方法 | 说明 |
|------|------|
| `world_to_grid(x, y)` | 世界坐标 → 网格索引 `(gx, gy)` |
| `grid_to_world(gx, gy)` | 网格索引 → 中心点世界坐标 |
| `is_occupied(x, y)` | 判断世界坐标是否被建筑占据 |
| `is_grid_occupied(gx, gy)` | 判断网格是否被占据 |
| `get_neighbors(gx, gy)` | 返回上下左右可通行邻居（已过滤边界和障碍） |
| `width, height` | 网格尺寸属性（均为 201） |

**建筑生成** (`utils/map_loader.py`)：支持 `avoid_overlap` 防重叠、`padding` 间距、`max_attempts` 重试。

---

### 二、路径规划算法 (`algorithms/cbs_pathplan.py`)

#### 2.1 单体 A* 规划

```python
def a_star_plan_world(start, goal, map_grid, max_time_ms=50) -> List[Tuple]
```

- 八连通邻居（含对角），对角代价 1，加权 Chebyshev 启发（`_H_WEIGHT=1.5`，改 1.0 即严格 A*），禁止切角穿墙
- 无约束时走轻量格子 A*；同代价时优先扩展更深节点并偏好直线
- 超时保护，返回世界坐标路径（已平滑）

#### 2.2 视线检测

```python
def line_of_sight_world(start, goal, map_grid) -> bool
```

- Bresenham 直线算法，判断 XY 平面是否被建筑阻挡；与 A* 一样检查对角切角

#### 2.3 CBS 多机规划

```python
def cbs_plan_paths(agent_pairs, map_grid, max_agents=6, time_limit_ms=200) -> List[List]
def cbs_plan_schedules(agent_pairs, map_grid, max_agents=6, time_limit_ms=200,
                       tick_seconds=0.5) -> List[tuple]
```

- 输入：`[(start, goal), ...]` 或 `[(drone_obj, goal), ...]`
- 低层 A* 在 `(格子, 时刻)` 上搜索，支持顶点约束与交换（边）约束；时间 = 步数，
  因此必须按计划速度执行才能保持无冲突
- 高层 CBS 带约束树与闭集，超时返回无冲突子集而不是整批放弃
- `cbs_plan_paths` 返回平滑后的几何折线（可视化/旧接口），不含时间戳
- `cbs_plan_schedules` 返回 `(points, times)`：原始网格点（不平滑）+ `times[i] = i * tick_seconds`，
  供 `PathFollower` 直接执行；主仿真按 `tick_seconds = map.path_cell_size / plan_speed` 传入
- t=0 同格起点不视为冲突（同阵营伤害目前不判定）；同格起点/终点会自动错开到附近空格
- 只规划 XY；z 取目标高度（为 0 时取起点高度）
- 超过 `max_agents` 时只规划前 `max_agents` 架，其余返回 `[]`，由调用方退化处理

回归用例见 `tools/cbs_diagnose.py`（不依赖 open3d）。

---

### 三、路径跟踪器 (`algorithms/path_tracker.py`)

管理单机的路径缓存与按需重规划。

```python
class PathTracker:
    def __init__(self, map_grid, config)
    def update(self, current_pos, goal_pos, current_time) -> Optional[Vector3D]
```

**配置项**：

| 参数 | 说明 |
|------|------|
| `replan_distance` | 目标移动超过该距离才重规划 |
| `replan_cooldown` | 重规划最小间隔（秒） |
| `max_plan_time_ms` | 单次 A* 最大耗时 |
| `path_tolerance` | 路径点跳过容差 |
| `debug_log` | 是否输出规划日志 |

**逻辑**：视线可达则直飞 → 否则 A* 规划 → 缓存路径 → 返回下一航点

---

### 四、控制器集成 (`Controller/single_control.py`)

```python
def chase_target(drone, target, map_grid=None)
def chase_point(drone, target_point, map_grid=None)

class PathFollower:
    def set_path(self, points, start_time, plan_speed=None, times=None) -> bool
    def update(self, now) -> bool
```

- 有 `map_grid` 时使用 `PathTracker` 实现按需 A* 避障
- 无地图或规划失败时退化为直线追逐
- A* 失败/短路径时**不再返回无人机自身位置**（那会让 `chase_point` 距离≈0 而不设速度，表现为边界/楼前卡死）；
  低空先爬升到 20 m，且爬升航点会朝目标水平前进 20 m，避免只升降不前进
- `PathFollower` 执行 CBS 路径：优先使用 `cbs_plan_schedules` 给出的 `times`；
  没有 `times` 时退回“累计距离 / `plan_speed`”估算
- 水平速度按时间表给、垂直速度单独给；落后时提速、超前时减速/悬停

---

### 五、主程序集成 (`main.py`)

```python
update_chase_strategy(drones, target_point, assignment_cfg=None, map_grid=None,
                      base_positions=None, sim_time=None, plan_speed=30.0)
```

- 启动时创建 `Open3DDisplay`（有 OBJ 城市地图时先体素化），每帧 `display.update(...)` +
  `app.run_one_tick()`；未装 open3d 或初始化失败自动 headless，窗口关闭时结束仿真
- `run_simulation` 中优先复用可视化的 `display.map_grid`；无界面时由 `config.map` 生成一次
  `MapGrid`（`map.path_cell_size`，默认 5 m），整局复用
- `sim_time` 为仿真时间；CBS 默认每 1 个仿真秒重规划一次，`map_grid=None` 时退化回直接追逐
- 进攻方：目标不可直达时由 PathTracker/A* 绕行
- 防守方：`cbs_plan_schedules` 出带 tick 时间戳的轨迹，交给 `PathFollower` 逐航点推进；
  `tick_seconds = map_grid.cell_size / plan_speed`，走完路径或路径为空时才退回直接追逐；
  CBS 最多同时规划 10 架防守方（`max_agents=10`），其余退化直接追逐
- 防守方目标有 **8 秒锁定**（`LOCK_TTL`）：短暂被 ARD 屏蔽、或拍卖想换目标时，仍先追原目标；
  超过 8 秒或目标死亡后才重新分配，避免“刚靠近就换目标”
- **近距离可见**：进攻方进入任一防守方 120 m 内视为光学/红外可见，不受 ARD 屏蔽影响；
  这样基地附近的漏网目标仍能被拦截
- **被 EW 干扰的防守方**不再全部原地悬停：非拦截机在离基地 350 m 内、目标进入基地 300 m 时，
  做本地自主拦截并锁定目标，且优先选**最接近基地**的威胁；拦截机仍然优先追 EW
- **终端防御**：进攻方进入任一基地 300 m 内时，强制最近的可用防守方转向拦截；
  每个威胁最多保留一个已有防守方，其余防守方可以接新威胁，避免所有防守方都锁在旧目标上
- 防守方编制按 **全部非 ARD 进攻方 × `defender_ratio`** 补充（默认 1.0），
  新机从随机基地外围环形起飞；`defender_interceptor_ratio` 控制新机中拦截机占比（默认 0.65，更耐撞）
- 出生点用 `MapGrid.is_occupied` 检查，环形/网格搜索都避开建筑；基地周围全被占满时退到全图空格
- 进攻方出生点收缩到 **±460~490 m**（可视地面/规划网格 ±500 内），不再出生在边界外；
  扇区方向生成也会夹到 ±480 内，避免“地图外某个高度卡死”；若出生点落在建筑 footprint 内，
  会直接抬到楼顶 + 10 m
- AntiEW 也不再固定刷在基地中心，而是从基地外围空地生成
- 所有存活无人机最低离地 **5 m**（`simulation.min_flight_altitude`），不会完全贴地飞
- **建筑体移动避让**：`move_drone` 每帧检查新位置是否落入建筑 footprint；若高度 < 楼顶 + 5 m，
  阻止这一帧的水平移动，并**直接给 z 增加 20 m/s 的爬升量**（不是只改 vz，避免控制器下一帧又把
  速度设回下降、导致贴墙上下抽）。末尾还会再检查一次当前位置，兜底碰撞分离等外部改位置的情况。
  进攻方（PathTracker/直追）和防守方（CBS/PathFollower）都走同一个 `move_drone`，因此都会生效。
  CBS/PathTracker 仍是 2D 规划，所以可能看到“贴楼爬升再越过”，但不会穿楼或长期卡在墙边；后续可做 2.5D 高度规划减少这种贴墙
- 同一目标全局只保留一架防守方；碰撞后把两机推开到 8 m 之外，并给 0.8 s 免撞窗口 +
  0.8 s 击退速度保持（期间不被 PathFollower/追击逻辑覆盖），避免贴脸反复反向抖动
- 无限模式下基地全毁会在**同一进程/同一窗口**重置一局，不会退出后被外部重新拉起新窗口
- 分配/CBS 策略按 `simulation.strategy_interval_s`（默认 0.12 s）降频；策略耗时会自动把间隔
  拉长到 `3×耗时`，避免大编队掉帧。`PathFollower` 仍然每帧推进，不受降频影响

**生成控制**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_offensive` | 40 | 最大进攻方数量（演示规模；压力测试可调大） |
| `spawn_interval_min` | 0.8 | 最小生成间隔（秒） |
| `spawn_interval_max` | 0.8 | 最大生成间隔（秒） |
| `defender_ratio` | 1.0 | 防守方数量 ≈ 非 ARD 进攻方 × ratio |
| `defender_interceptor_ratio` | 0.65 | 新补充防守方中拦截机占比（越高越耐撞） |
| `battery_drain_rate` | 0.5 | 电池消耗（点/秒）；5.0 时 60 点电池只能飞 12 秒 |

---

### 六、可视化 (`Visual/`)

`Open3DDisplay` 负责窗口、场景、HUD 和每帧更新；`main.py` 在启动时创建，
失败则自动退化为无界面模式。

| 类型 | 颜色/样式 | 对应属性 / 开关 |
|------|----------|----------------|
| 防守方→目标连线 | 黄色虚线 | `_assigned_target` |
| 历史轨迹 | 机型颜色 | `drone.trail` |
| 坠毁/爆炸 | 球体放大 / 重力下落 | `drone.death_timer`、`impact` |
| HUD | 双阵营数量、基地血量、FPS、操作提示、攻防对应指标 | 每帧刷新 |
| 攻防对应行 | `Visible / Assigned / Collisions` | `main` 每帧传入 `metrics`，与算法同一套可见性判断 |

- `P` 暂停/继续（只刷新窗口和特效，不推进仿真）
- `R` 重置视角；`P` 暂停
- CBS / A* 路径只在内部规划使用，窗口不再画紫色/绿色路径，画面保留黄色目标虚线和可选轨迹
- `duration: 0` 时无限运行；基地全毁只在**同一进程/同一窗口内重置一局**，不会退出后由外部重新启动
- 关闭窗口或 Ctrl+C 才结束；结束/异常时控制台会打印 `===== SIMULATION ENDED (...) =====`
- Open3D `update` 抛 Python 异常时会自动关闭显示并切到 headless，不让窗口异常直接带走整个仿真
- OBJ 城市地图由 `utils/obj_voxelizer.py` 体素化后同时用于渲染和 `MapGrid`

**虚线工具**：

```python
def create_dashed_line(p1, p2, color, dash_length=8.0, gap_length=6.0)
```

---

### 七、配置文件 (`config.yaml`)

```yaml
path_planning:
  plan_speed: 30.0          # CBS 路径执行速度（全体防守方统一）
  replan_distance: 1.0
  replan_cooldown: 2.0
  max_plan_time_ms: 30
  path_tolerance: 1.0
  debug_log: false

simulation:
  fps: 30
  duration: 0                 # 0 = 无限运行；Ctrl+C 或关闭窗口结束
  enable_visualization: true  # false 或无 open3d 时自动无界面运行
  enable_trails: false        # 轨迹线每帧重建，长时间运行建议关闭以降低 Open3D 压力
  strategy_interval_s: 0.12  # 分配/CBS 最小间隔；耗时会自动拉长，PathFollower 仍每帧推进
  max_offensive: 40
  spawn_interval_min: 0.8
  spawn_interval_max: 0.8
  defender_ratio: 1.0
  defender_interceptor_ratio: 0.65
  battery_drain_rate: 0.5
  min_flight_altitude: 5.0    # 最低离地高度（米）

map:
  path_cell_size: 5.0       # 规划网格分辨率（米）
  use_obj_model: true       # 有 Open3D 时优先用 Data/maps 的 OBJ 城市模型
  buildings:
    avoid_overlap: true
    padding: 2.5
```

---

### 八、攻防对应量化 (`tools/battle_trace.py`)

无界面运行，直接向控制台输出可解析的记录：

| 行 | 含义 |
|----|------|
| `SUMMARY,t,...` | 每个策略帧：进攻方、可见进攻方、干扰机、防守方、已分配对数、配对最小距离、基地血量 |
| `TRACE,t,...` | 每 0.5 s 跟踪**一架**防守方：位置、速度、目标、目标位置、距离、PathFollower 进度 |
| `COLLISION` / `BASE` / `DEATH` | 跨阵营碰撞、基地掉血、死亡原因（collision / self_destruct / battery） |

```bash
# 60 秒、40 架进攻方、0.8s 生成间隔、固定随机种子
python tools/battle_trace.py --duration 60 --max-offensive 40 --spawn 0.8 --seed 1

# 跟踪指定防守方（名字从运行日志/TRACE 里拿）
python tools/battle_trace.py --focus Scout-123 --duration 30
```

关键指标：

- `coverage_defenders = avg_assigned / avg_defenders`：有对应目标的防守方比例
- `coverage_visible = avg_assigned / avg_visible`：大于 1 说明锁定目标在屏蔽中仍被追击
- `pair_dist_min`：任一“防守方-目标”对的最小距离；进入 8 m 才触发碰撞
- `collisions / base_hits / base_health`：拦截次数、漏网次数、基地存活
- `target_switches`：目标切换次数，用来衡量锁定/重分配稳定性
- `death_causes`：区分被拦截、自毁、电池耗尽

一次 **60 秒**、40 架进攻方、0.8 s 间隔、65% 拦截机的实测（`battle_bench.py`，seed 1~3，
**已修复进攻方卡在边界/楼前的问题**）：

| seed | collisions | base_health | avg_assigned | pair_dist_min | target_switches | 攻/防死亡 | 最终攻/防 |
|------|-----------|-------------|--------------|---------------|-----------------|-----------|-----------|
| 1 | 70 | 130/300 | 10.08 | 8.1 m | 152 | 34 / 30 | 21 / 16 |
| 2 | 34 | 180/300 | 7.40 | 8.0 m | 101 | 32 / 32 | 27 / 20 |
| 3 | 59 | 110/300 | 8.58 | 7.8 m | 123 | 27 / 21 | 28 / 19 |
| **mean** | **54.3** | **140.0/300** | **8.69** | **7.97 m** | **125** | **31.0 / 27.7** | — |

修复进攻方卡死后，当前组合：
- 终端/本地防御半径 **300 m**；
- 碰撞后 **0.8 s 免撞 + 0.8 s 击退保持**；
- 最小碰撞伤害 **50**；
- 防守方编制 **1.0 × 非 ARD 进攻方**，65% 拦截机。

当前 3 seed 均值 **140/300**；之前单次跑到过 173/300，说明机器帧率/时序方差很大。
1.2 编制能到 178/300，但 defender twitch 检测重新出现 1~2 次，因此没有保留；
最小伤害 60、拦截机 0.8 也都没有提高基地，已回退。

修复前卡死进攻方造成的 180/300 是虚高；当前 140/300 是在进攻方正常移动下的真实基线。

> 判断改动看 `battle_bench.py` 的均值和 min/max；单 seed 和机器帧率方差都很大。

---

### 九、工具脚本 (`tools/`)

| 脚本 | 用途 |
|------|------|
| `battle_trace.py` | 单机运动 + 攻防对应量化（SUMMARY/TRACE/COLLISION/DEATH） |
| `battle_bench.py` | 多 seed 聚合基准（基地/碰撞/分配/切换均值） |
| `cbs_perf_test.py` | 测试不同无人机数量下的 CBS 耗时与成功率 |
| `map_grid_check.py` | 检查建筑重叠、网格占据情况 |

---

## 原版功能（保持不变）

- **无人机类型**：攻击机、肉盾机（红方）；侦察机、拦截机（蓝方）
- **控制模块**：移动、边界检查、碰撞检测
- **感知模块**：雷达探测
- **可视化**：Open3D 实时渲染，鼠标拖拽/滚轮/右键重置

---

## 运行方式

```bash
# 可视化仿真
python main.py

# 无头测试
python remark/test.py --duration 10 --offensive-count 4 --defensive-count 4

# 攻防对应量化（跟踪一架防守方 + 汇总）
python tools/battle_trace.py --duration 25 --max-offensive 40 --spawn 0.8 --seed 1

# 多 seed 聚合基准（推荐用均值判断改动是否有效）
python tools/battle_bench.py --seeds 1,2,3 --duration 60

# 网格检查
python tools/map_grid_check.py

# 性能测试
python tools/cbs_perf_test.py --agents 1 2 4 6
```

---

## 配置调整

编辑 `config.yaml` 修改地图尺寸、无人机属性、路径规划参数、生成频率等。