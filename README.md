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
├── sensing/                     # 感知模块
│   └── radar.py                 # 雷达探测系统
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
- **sensing/**: 感知系统。目前包含雷达探测，用于检测附近无人机。
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

如果 Open3D 可视化可用，会打开一个窗口显示无人机实时位置；否则以无界面模式运行，只输出日志。

还没怎么加接口，不太清楚具体怎么处理，后面加。

### 控制说明

- 鼠标拖拽：旋转视角
- 鼠标滚轮：缩放
- 右键点击：重置视角

### 4. 配置调整

编辑 `config.yaml` 文件来调整参数：

- **地图设置**：`map.width`, `map.height`
- **雷达参数**：`radar.range`, `radar.pulse_interval`
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

### sensing/ - 感知系统
- **radar.py**: 模拟雷达探测，检测范围内无人机
- **scout_system.py**: 侦察机专用精确识别系统

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
  - `AttackDrone`: 高速攻击机 (速度7.5, 攻击力25, 血量80)
  - `TankDrone`: 高血量肉盾机 (速度4.5, 攻击力15, 血量150)
  - `ScoutDrone`: 超高速侦察机 (速度15.0, 攻击力5, 血量60)
  - `InterceptorDrone`: 中等拦截机 (速度12.0, 攻击力20, 血量90)

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

#### 4. 感知系统 (`sensing/`)
- **`radar.py`**: 雷达探测器
  - 距离检测（500米范围）
  - 返回无人机信息（位置、类型、血量）

- **`scout_system.py`**: 侦察系统
  - 精确识别无人机状态
  - 分类和状态监控

#### 5. 可视化系统 (`Visual/`)
- **`open3d_display.py`**: Open3D 渲染引擎
  - 窗口创建和管理
  - 实时几何体更新
  - 错误处理（支持无界面模式）

- **`render_utils.py`**: 渲染工具
  - 无人机几何体创建（球体）
  - 路径线段生成

#### 6. 算法模块 (`algorithms/`)
- **`auction_assign.py`**: 任务分配算法框架
  - 简单的循环分配（占位符）
  - 为后续拍卖算法预留接口

- **`cbs_pathplan.py`**: 路径规划算法框架
  - 基础路径生成（占位符）
  - 为 CBS 算法预留接口

#### 7. 数据存储 (`data/`)
- **`maps/base_map.json`**: 地图配置
- **`detection/radar_log.json`**: 探测记录（空）
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
radar:         # 雷达参数
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
   - 初始化雷达和显示系统

2. **仿真循环**（每帧）：
   - 更新所有无人机位置
   - 雷达扫描检测
   - 可视化渲染
   - 帧率控制（30FPS）

3. **结束处理**：
   - 关闭显示窗口
   - 输出仿真完成日志

### 扩展接口

框架预留了多个扩展点：

- **算法接口**：`algorithms/` 中的占位符可替换为真实算法
- **控制逻辑**：`swarm_control.py` 可添加复杂协同行为
- **感知增强**：`sensing/` 可添加延迟、模糊等效果
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

- 八连通邻居（含对角），欧氏距离启发
- 超时保护，返回世界坐标路径（已平滑）

#### 2.2 视线检测

```python
def line_of_sight_world(start, goal, map_grid) -> bool
```

- Bresenham 直线算法，判断 XY 平面是否被建筑阻挡

#### 2.3 CBS 多机规划

```python
def cbs_plan_paths(agent_pairs, map_grid, time_limit_ms=200, max_agents=8) -> List[List]
```

- 输入：`[(start, goal), ...]` 或 `[(drone_obj, goal), ...]`
- 仅规划 XY 二维（Z 不变），点冲突检测，返回无冲突路径集

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
def chase_target(drone, enemies, dt, map_grid=None, tracker=None, current_time=0)
def chase_point(drone, target_pos, dt, map_grid=None, tracker=None, current_time=0)
```

- 有 `map_grid` 时使用 `PathTracker` 实现避障
- 无地图或规划失败时退化为直线追逐

---

### 五、主程序集成 (`main.py`)

```python
update_chase_strategy(drones, target_point, assignment_cfg=None, map_grid=None)
```

- 在仿真循环中传入 `display.map_grid`
- 进攻方：目标不可直达时调用 A* 绕行
- 防守方：调用 CBS 规划集体路径

**生成控制**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_offensive` | 6 | 最大进攻方数量 |
| `spawn_interval_min` | 5.0 | 最小生成间隔（秒） |
| `spawn_interval_max` | 12.0 | 最大生成间隔（秒） |

---

### 六、可视化增强 (`Visual/`)

| 类型 | 颜色/样式 | 对应属性 |
|------|----------|----------|
| 进攻方避障路径 | 绿色 | `drone._avoid_path` |
| 防守方 CBS 路径 | 紫色 | `drone._cbs_path` |
| 防守方→目标连线 | 黄色虚线 | `create_dashed_line()` |
| 历史轨迹 | 红色 | 原有轨迹线 |

**虚线工具**：

```python
def create_dashed_line(p1, p2, color, dash_length=0.3, gap_length=0.2)
```

---

### 七、配置文件 (`config.yaml`)

```yaml
path_planning:
  replan_distance: 5.0
  replan_cooldown: 0.5
  max_plan_time_ms: 50
  path_tolerance: 0.5
  debug_log: false

simulation:
  max_offensive: 6
  spawn_interval_min: 5.0
  spawn_interval_max: 12.0

map:
  buildings:
    avoid_overlap: true
    padding: 2.5
```

---

### 八、工具脚本 (`tools/`)

| 脚本 | 用途 |
|------|------|
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

# 网格检查
python tools/map_grid_check.py

# 性能测试
python tools/cbs_perf_test.py --agents 1 2 4 6
```

---

## 配置调整

编辑 `config.yaml` 修改地图尺寸、无人机属性、路径规划参数、生成频率等。