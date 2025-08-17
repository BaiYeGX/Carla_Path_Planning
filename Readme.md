# CARLA A\* + Dubins Path Planner

> **环境**：Python 3.7.9 · CARLA 0.9.15\
> **文件**：`carla_astar_dubins_planner.py`\
> **功能定位**：在 CARLA 仿真器中实现完整的自动驾驶路径规划流程，包括两阶段 A\* 全局路径搜索、Dubins 曲线平滑优化、车辆跟踪控制，并提供丰富的可视化与数据导出功能。

---

## 目录

- [快速开始](#快速开始)
- [总体架构](#总体架构)
- [命令行参数](#命令行参数)
- [算法与实现细节](#算法与实现细节)
  - [A\* 两阶段全局规划（Bootstrap→Global）](#a-两阶段全局规划bootstrapglobal)
  - [Dubins 平滑与捷径](#dubins-平滑与捷径)
  - [可视化与输出](#可视化与输出)
  - [简易跟踪（Pure Pursuit）](#简易跟踪pure-pursuit)
  - [Spawn/清场/相机与同步](#spawn清场相机与同步)
- [CSV/图片输出规格](#csv图片输出规格)
- [参数调优建议](#参数调优建议)
- [常见问题与排错](#常见问题与排错)
- [扩展点与二次开发建议](#扩展点与二次开发建议)
- [代码地图（函数/类一览）](#代码地图函数类一览)

---

## 快速开始

1. **启动 CARLA Server**（示例：Windows 双击 `CarlaUE4.exe`，或 Linux `./CarlaUE4.sh -quality-level=Epic -carla-server`）。
2. **安装依赖**（建议虚拟环境）：
   ```bash
   pip install numpy matplotlib
   # 可选：更快/更稳的 Dubins 实现
   pip install dubins
   ```
   > 需确保 `PythonAPI/carla/dist/carla-0.9.15-*.egg` 在 `PYTHONPATH`（或本脚本能正常 `import carla`）。
3. **运行示例**：
   ```bash
   Set-Location "E:\autonomous driving\summer-vacation\carla_path_planning"
   $env:PYTHONPATH = "E:\CARLA\PythonAPI\carla\dist\carla-0.9.15-py3.7-win-amd64.egg;E:\CARLA\PythonAPI"
   # 加载指定地图、默认参数运行
   py -3.7 .\carla_astar_dubins_planner.py --town Town04
   
   # 调高最小转弯半径、加快 Dubins 采样、开启更强起步约束
   py -3.7 .\carla_astar_dubins_planner.py --town Town05 \
       --turning_radius 12.0 --dubins_step 0.3 \
       --boot_dist 25.0 --boot_angle_deg 40 --start_cone_deg 40
   ```

---

## 总体架构

**数据流**：

1. 连接/同步 CARLA，清空 NPC → 生成自车、随机选取起止点（过滤规则确保在可行车道且朝向对齐）。
2. **两阶段 A**\*：
   - *Bootstrap*：从车头方向出发，在前方行驶 `boot_dist` 或到达首个路口。
   - *Global*：从 Bootstrap 末点出发到终点的全局搜索。方向惩罚随距离指数衰减，避免起步时把目标点放到车后方。
3. **Dubins 平滑**：对 A\* 路径分段做最短 Dubins 曲线拼接与两阶段采样合法性校验（含“起步锥”、走廊索引、车道类型/横向边距约束）；可选 **shortcut 迭代** 做局部重连优化长度。
4. **可视化/导出**：在 UE Debug 中绘制 A\* 与平滑路径；保存 **CSV**、**顶视图截图**、**白底图**。
5. **简易跟踪**：Pure Pursuit + “前向投影决定油门”的防卡死策略，车辆沿平滑路径行驶，摄像机跟随。

---

## 命令行参数

> 括号内为默认值

- 连接/运行

  - `--host` (`127.0.0.1`)：CARLA 主机。
  - `--port` (`2000`)：CARLA 端口。
  - `--town` (可选)：指定地图，例如 `Town04`。
  - `--life_time` (`0.0`)：UE Debug 绘制的生存时间（秒），0 为永久。
  - `--seed` (None)：随机种子。
  - `--max_drive_seconds` (`180.0`)：跟踪行驶最大时长，防止无限循环。

- A\*（中心线搜索）

  - `--astar_step` (`3.0`)：前向扩展步长（米）。路口内自动缩短（≈×0.6）以居中取弯。
  - `--goal_thresh` (`5.0`)：抵达判定半径（米）。
  - `--allow_uturn` (`True`)：允许在**非路口**尝试掉头（Bootstrap 阶段默认禁用）。
  - `--boot_dist` (`20.0`)：**起步阶段**尝试沿车头方向前行的距离（米）。
  - `--boot_angle_deg` (`45.0`)：Bootstrap 的方向锥约束（相对车头的最大偏航差）。
  - `--turn_penalty_alpha0` (`1.8`)：方向惩罚初值（越大越偏好“少转向”）。
  - `--turn_penalty_decay` (`25.0`)：方向惩罚随“从起点累计距离”指数衰减的尺度（米）。

- Dubins 平滑/捷径

  - `--turning_radius` (`11.5`)：**最小转弯半径**（米）。
  - `--dubins_step` (`0.35`)：Dubins 曲线采样步长（米）。
  - `--lane_margin` (`0.35`)：相对车道中心的横向边距裕量（越小越贴近中心线）。
  - `--start_cone_deg` (`50.0`)：\*\*第一段 Dubins 的“起步锥”\*\*角（度），限制首段朝向。
  - `--shortcut_iters` (`7`)：局部重连（shortcut）迭代次数。
  - `--shortcut_gap_pts` (`20`)：被重连的两端点在序列中的最小间隔（避免过短片段反复抖动）。
  - `--shortcut_improve` (`0.995`)：新段长度需短于旧段的比例阈值（越小越严格）。

---

## 算法与实现细节

### A\* 两阶段全局规划（Bootstrap→Global)

类：`AStarPlanner`（L105）

- **关键思想**：解决“起步时车头与全局最优方向相反”造成的**原地打死/卡死**。
- **邻居生成** `neighbors()`（L130）：
  - 前向扩展：路口内缩步长（≈0.6×）形成居中转弯锚点；
  - 变道：仅在 `lane_change` 允许且**同向 Driving** 时加入；
  - 掉头：只在**非路口**尝试“原地掉头到对向车道再沿 previous 反向行驶”；Bootstrap 阶段禁用；
  - **Bootstrap 方向锥**：当 `bootstrap_mode=True` 时，仅保留与车头夹角 ≤ `boot_angle_deg` 的候选。
- **Bootstrap 阶段** `plan_bootstrap()`（L235）：
  - 从车头方向出发，累计距离达到 `boot_dist` 或遇到**首个路口**即停止；
  - 启发式：到目标的欧氏距离；
  - 方向惩罚：在此阶段降低权重（主要靠方向锥硬约束）。
- **Global 阶段** `plan()`（L334）：
  - 起点设为 Bootstrap 末点（若失败则为原始起点）；
  - **方向惩罚 × 距离衰减**：对“当前→邻居的航向变化”增加代价，该惩罚随“从起点累计行驶距离”按指数**衰减**（`turn_penalty_alpha0`、`turn_penalty_decay` 控制）；
  - 允许（可选）变道与（非路口）掉头；
  - 重建路径 `_reconstruct_path()`（L438）输出 `(x, y, z, yaw, wp)` 序列。

> **效果**：起步阶段强力拉回“面向前进方向”，即便终局需掉头，也会先向前行进若干米，在更合适位置完成掉头/掉头替代绕行，避免一开始就让目标点落在车后方。

### Dubins 平滑与捷径

类：`DubinsSmoother`（L455）

- **最小转弯半径** `R`、**采样步长** `sample_step`、**横向裕量** `lane_margin`、**起步锥** `start_cone_deg`。
- **性能优化**：
  - Waypoint 查询 **20 cm 量化缓存**；
  - 走廊点（\~5 m 抽样）建立**空间哈希索引**，合法性评估只比对邻近 Bin，降复杂度。
- **合法性校验** `_sample_and_check()`（L521）：
  - 两阶段抽检：先稀疏（约 3× 步长）快速否决，再细检（1× 步长）全面确认；
  - 每个采样点需满足：位于 **Driving 车道且同向**、横向偏移在容许范围内、靠近走廊；路口位置**放宽**车道约束；
  - **第一段专属**：施加“起步锥”与“前向投影>0”的额外限制，防止首段朝向反身。
- **段落拼接** `smooth_greedy()`（L633）：
  - 在 A\* 路径上做“跨点”贪心拼接最短 Dubins 段（优先跳更远以减少段数），每段通过上述合法性检查；
  - 支持区分 **前段允许车道集合**（`allowed_lanes_front`）与全局集合，配合 A\* 的 Bootstrap 段数 `boot_segment_count`，进一步巩固起步阶段的**同向约束**。
- **局部重连** `shortcut_iterative()`（L694）：
  - 在已得采样序列上随机挑两点，尝试用**更短** Dubins 段替换原段；
  - 仅当新段**合法**且长度改进超过 `--shortcut_improve` 才替换；重复 `--shortcut_iters` 轮；
  - 典型地显著缩短段数与总长度，并抑制“锯齿”。
- **极端兜底**（主流程）：若首段与车头夹角依然很大（>\~143°），在序列前部插入**本地 U-turn 补丁**，保证起步对齐。

### 可视化与输出

类：`Visualizer`（L738）

- A\* 起步段：浅绿点/线；A\* 全局段：青色点/黄线；起点红点、终点蓝点；
- 顶视图截图 `take_topdown_screenshot()`（L787）：路径包围盒自适应相机位置/焦距；
- “论文风格”渲染 `render_paper_figure_from_csv()`（L841）：白底/网格/黑障碍（非 Driving 区域）/路网线/红色路径/起终点点标。

### 简易跟踪（Pure Pursuit）

函数：`pure_pursuit_control()`（L768）

- 以车辆→目标点方位角误差生成转向；
- **前向投影油门**：用 `cos(误差角)` 决定推进力度，最低油门 `~0.07`，大角误差时也**缓慢前行**纠正，避免“角误差大就刹车=原地卡死”；
- 目标点选择：仅在车辆**前方窗口**内搜索最近目标；若窗口为空，**强制向前跳**固定步长以维持前行。

### Spawn/清场/相机与同步

- 清场 `_clear_npcs()`（L998）与安全析构 `_destroy_actors_safe()`（L988）。
- 选车 `_pick_ego_blueprint()`（L1014）：优先选常见轿车，兜底任意 `vehicle.*`。
- Spawn 点筛选 `_is_good_spawn_tf()`（L1045）：
  - 仅 Driving 车道；
  - 非路口，且前/后若干米仍非路口（避免生在路口中心/边缘）；
  - 初始朝向与车道方向**对齐**（避免逆行）。
- 对齐 `_align_tf_to_lane()`（L1036），过滤 `_filter_spawn_points()`（L1070），生成 `_try_spawn_vehicle()`（L1080）。
- 观众相机 `follow()`（L1208）：从车后上方俯视跟随。
- 同步模式：固定步长仿真；连接/等待 `_wait_for_server()`（L74）。

---

## CSV/图片输出规格

- 运行时会在 **当前工作目录**创建一个以时间戳命名的文件夹（例如 `20250101_123456/`），包含：
  - `path_*.csv`：平滑路径采样点与分段索引；
  - `topdown_*.png`：CARLA 顶拍图（相机自适应路径范围）；
  - `paper_*.png`：白底论文风格渲染图（带网格/障碍/路网/路径/起终点）。
- **CSV 列**：`idx, seg_idx, x, y, z, yaw_deg`。

---

## 参数调优建议

- **起步对齐相关**：
  - `--boot_dist` ↑、`--boot_angle_deg` ↓、`--start_cone_deg` ↓ 可显著降低“起步反向”风险；
  - `--turn_penalty_alpha0` ↑、`--turn_penalty_decay` 适中，可让前段更“顺着车头”搜索，后段恢复自由度。
- **曲率与采样**：
  - `--turning_radius` 由车模/速度决定（常见 9–12 m）；半径越大，越“温和”。
  - `--dubins_step` 越小路径越平滑但采样更多；一般 0.3–0.5 m。
  - `--lane_margin` 过大易“贴边穿模”，过小会过于保守（推荐 \~0.3–0.4 m）。
- **shortcut**：先适中 `--shortcut_gap_pts` 与 `--shortcut_improve`，逐步增加 `--shortcut_iters` 观察收益。

---

## 常见问题与排错

- **连不上 CARLA**：检查 Server 是否运行、端口是否被占用、同步模式是否冲突；脚本提供了快速 `get_server_version()` ping 与更长超时的 `_wait_for_server()`。
- **Spawn 点太少/无效**：地图 `get_spawn_points()` < 2 或过滤过严；脚本会回退到“对齐到所有 Driving 车道的 spawn”。
- **没有路径/Dubins 校验失败**：
  - 放宽 `--lane_margin`、增大 `--turning_radius`、增大 `--boot_dist`；
  - 减小 `--dubins_step` 以更密采样；
  - 检查是否被路口/对向车道限制导致首段非法。
- **起步仍卡死**：
  - 启用更严格的 `--start_cone_deg` 与更长的 `--boot_dist`；
  - 观察日志是否触发了**本地 U-turn 补丁**；
  - 适当减小 Pure Pursuit `kp_steer`，或在低速区加速度限制。

---

## 扩展点与二次开发建议

- **控制器替换**：将 Pure Pursuit 换为 Stanley/Lat+Lon Split/MPC，沿用“前向投影油门”思想。
- **更强合法性**：
  - 引入占据栅格/HD Map 边界作为硬约束；
  - 叠加动态障碍（Traffic Manager 车辆）并做时空检查。
- **规划器增强**：
  - A\* 代价加入**曲率/换道**显式代价，或切换到 Hybrid A\*/State Lattice；
  - Dubins 段落与 Bézier/spline 混合，提升几何连续性（C2）。
- **工具化**：将 CSV/PNG 导出转为统一 `results/` 下的 runID 结构，便于数据管理与回放。

---

## 代码地图（函数/类一览）

> 行号 `L*` 为源码中的大致起始行（便于定位）。

- \*\*class \*\*\`\` (L105)
  - `neighbors()` (L130)：邻居生成（前向扩展、同向变道、非路口掉头、Bootstrap 方向锥）。
  - `plan_bootstrap()` (L235)：起步阶段搜索到 `boot_dist`/首路口。
  - `plan()` (L334)：两阶段 A\*（方向惩罚×距离衰减）。
  - `_reconstruct_path()` (L438)：输出 `(x,y,z,yaw,wp)`。
- \*\*class \*\*\`\` (L455)
  - `_lane_ok()` (L506)：车道类型/横向边距校验。
  - `_sample_and_check()` (L521)：两阶段采样合法性评估（含第一段起步锥）。
  - `smooth_greedy()` (L633)：跨点贪心拼接 Dubins 段。
  - `shortcut_iterative()` (L694)：随机局部重连迭代优化。
- \*\*class \*\*\`\` (L738)
  - `draw_points()`/`draw_polyline()`/`draw_start_end()`：UE Debug 绘制。
- **运行辅助**
  - `_wait_for_server()` (L74)：带重试的 server/map 就绪检查。
  - `pure_pursuit_control()` (L768)：前向投影油门的追踪控制。
  - `take_topdown_screenshot()` (L787)：顶视图相机自适应。
  - `render_paper_figure_from_csv()` (L841)：论文风格渲染。
  - `_clear_npcs()` / `_destroy_actors_safe()`：清理。
  - `_pick_ego_blueprint()` / `_align_tf_to_lane()` / `_is_good_spawn_tf()` / `_filter_spawn_points()` / `_try_spawn_vehicle()`：spawn 相关。
  - `follow()` (L1208)：观众相机跟随。
  - `main()` (L1096)：参数解析、连接、规划、平滑、可视化、导出与跟踪主流程。

---

## 版本记要（本版改进点梳理）

- A\* 唯一键更稳健；Bootstrap 起步 + 方向惩罚衰减，解决“起步反向卡死”。
- Dubins 采样合法性更严格（车道类型 + 横向边距 + 走廊索引 + 路口放宽）；首段“起步锥”。
- 性能：Waypoint 20cm 量化缓存 + 走廊空间哈希。
- 可靠性：spawn `try_spawn` + 全面清理；行驶最大时长保护。
- 可视化与导出：顶拍/白底图；CSV 字段更全；运行摘要打印。

> 有任何问题或需要在你们地图/车模上专项调参，欢迎在 Issue/PR 中给出复现信息（地图名、参数、日志、截图）。

