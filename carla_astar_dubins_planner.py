#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CARLA 0.9.15 | Python 3.8.10
A* (centerline) + Dubins shortcut smoothing + simple path following

更新要点（本版相对上一版的优化）
- ✅ 更稳健：修复 A* 结点唯一性键在少数地图上可能退化的问题；
- ✅ 更安全：Dubins 采样增加“横向偏移 + 车道类型”双重校验；
- ✅ 更贴合仿真：参数调优（默认：A*步长3.0m、最小转弯半径9.0m、Dubins采样0.5m、目标判定半径5.0m）；
- ✅ 更耐用：spawn 使用 try_spawn + 多次重试；所有 Actor/传感器在异常也能清理；
- ✅ 更可控：行驶循环设置最大时长与末端缓行，防止无限循环或冲出终点；
- ✅ 更直观：CSV 字段更全；打印运行摘要；截图相机自适应路径尺寸。

功能:
- 连接 CARLA, 清空 NPC, 仅生成自车(ego)
- 随机在道路中心线上选起点与终点 (spawn points)
- 用 A* 在车道中心线上做全局粗路径 (只前进扩展, 支持路口多分支)
- Dubins 多段跳点平滑 (尽量跳过更多 A* 点), 每段做道路有效性(碰撞)检查与最小转弯半径校验
- 在 UE4 里绘制: 离散点(一种颜色)、连线(第二种颜色)、最终整条平滑路径(第三种颜色), 起终点标注
- 让摄像机跟随自车, 车辆沿最终路径简单跟踪(非CARLA自带规划/自动驾驶)
- 输出 CSV(包含坐标、航向、段索引等) 与 一张叠加CARLA地图的路径图 到 以时间戳命名的文件夹

使用:
  python carla_astar_dubins_planner.py --host 127.0.0.1 --port 2000 --town Town03 

注:
  - 脚本不依赖 GlobalRoutePlanner/BehaviorAgent 等
  - Dubins 优先使用 `dubins` 库；若缺失，使用保守圆弧-直线-圆弧备选生成可行路径
"""

import os
import sys
import math
import csv
import time
import random
import argparse
import datetime
from collections import deque, defaultdict

# —— CARLA PythonAPI ——
# 不在程序里自动查找 .egg；请先在 PowerShell 设置好 PYTHONPATH 后再运行
try:
    import carla  # type: ignore
except ImportError:
    raise RuntimeError('未找到 CARLA Python API。请先在 PowerShell 中设置 PYTHONPATH 指向 CARLA 的 .egg 和 PythonAPI 目录，再运行本脚本。')

# 尝试引入第三方 dubins 库
_HAS_DUBINS = True
try:
    import dubins  # type: ignore
except Exception:
    _HAS_DUBINS = False

### -------------------- 工具函数 -------------------- ###

def to_rad(deg: float) -> float:
    return deg * math.pi / 180.0

def to_deg(rad: float) -> float:
    return rad * 180.0 / math.pi

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def euclid2(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def quant(v: float, q: float):
    return int(round(v / q))

# ---- 等待 CARLA 服务器与地图完全就绪（顶层工具函数） ----
def _wait_for_server(cli, host: str = '127.0.0.1', port: int = 2000, tries: int = 300, pause: float = 1.0, require_map: bool = False):
    """等待 CARLA 服务器就绪；可选等待地图可用(require_map)。更长的等待+更详细日志。"""
    last_exc = None
    print(f"[INFO] Waiting for CARLA at {host}:{port} (require_map={require_map})...", flush=True)
    for i in range(1, tries + 1):
        try:
            # 先校验握手（版本）
            try:
                sv = cli.get_server_version()
                cv = cli.get_client_version()
                if i == 1:
                    print(f"[INFO] Client v{cv} ↔ Server v{sv}", flush=True)
            except Exception as ve:
                last_exc = ve
                time.sleep(pause)
                continue
            w = cli.get_world()
            if require_map:
                _ = w.get_map()
            if i % 5 == 0 or i == 1:
                print(f"[OK] Connected after ~{int(i*pause)}s.", flush=True)
            return w
        except Exception as e:
            last_exc = e
            if i % 10 == 0:
                print(f"[WAIT] ... {int(i*pause)}s elapsed (last: {type(e).__name__})", flush=True)
            time.sleep(pause)
    raise RuntimeError(f"CARLA not ready after ~{int(tries*pause)}s: {last_exc}")

### -------------------- A* 搜索(在中心线上) -------------------- ###

class AStarPlanner:
    def __init__(self, world: 'carla.World', step: float = 3.0, goal_thresh: float = 5.0, key_xy_quant: float = 1.0, allow_uturn: bool = False,
                 boot_dist: float = 20.0, boot_angle_deg: float = 45.0, turn_penalty_alpha0: float = 1.8, turn_penalty_decay: float = 25.0):
        self.world = world
        self.map = world.get_map()
        self.step = step
        self.goal_thresh = goal_thresh
        self.key_xy_quant = key_xy_quant  # 用于 waypoint 唯一键的坐标量化
        self.allow_uturn = allow_uturn
        self.boot_dist = boot_dist
        self.boot_angle_deg = boot_angle_deg
        self.turn_penalty_alpha0 = turn_penalty_alpha0
        self.turn_penalty_decay = turn_penalty_decay

    def heuristic(self, loc_xy, goal_xy):
        return euclid2(loc_xy, goal_xy)

    def is_goal(self, loc_xy, goal_xy):
        return euclid2(loc_xy, goal_xy) <= self.goal_thresh

    def _wp_key(self, wp: 'carla.Waypoint') -> tuple:
        # 使用 (road_id, lane_id, qx, qy) 作为节点键，避免依赖可能不存在的 wp.s
        loc = wp.transform.location
        return (wp.road_id, wp.lane_id, quant(loc.x, self.key_xy_quant), quant(loc.y, self.key_xy_quant))

    def neighbors(self, wp: 'carla.Waypoint', bootstrap_mode: bool = False, psi0: float = None, allow_lane_change: bool = True):
        """返回相邻可扩展节点列表: [(neighbor_wp, move_cost)]
        规则更新：
        - 前向扩展：在路口内使用更短步长（居中取弯），非路口用常规步长；
        - 车道变换：仅在 lane_change 允许且同向时加入；
        - 掉头（U-Turn）：在非路口、当前路段"中段"优先做原地掉头（横跨至对向车道后沿反向 previous），避免先驶入下一条路再掉头。
        - bootstrap_mode：起步模式，限制方向锥和禁止后退
        """
        res = []
        uniq = set()
        # 路口内缩短步长，帮助在路口中部形成转弯锚点
        step_fwd = self.step * (0.6 if getattr(wp, 'is_junction', False) else 1.0)
        step_fwd = max(1.0, step_fwd)

        # 前向扩展
        try:
            nxt = [n for n in wp.next(step_fwd) if n.lane_type == carla.LaneType.Driving]
        except Exception:
            nxt = []
        for n in nxt:
            k = self._wp_key(n)
            if k in uniq:
                continue
            uniq.add(k)
            res.append((n, float(step_fwd)))
        
        # Bootstrap模式下的方向锥约束
        if bootstrap_mode and psi0 is not None:
            # 过滤掉超出方向锥的邻居
            filtered_res = []
            for n_wp, cost in res:
                n_yaw = to_rad(n_wp.transform.rotation.yaw)
                angle_diff = abs((n_yaw - psi0 + math.pi) % (2*math.pi) - math.pi)
                if to_deg(angle_diff) <= self.boot_angle_deg:
                    filtered_res.append((n_wp, cost))
            res = filtered_res

        # 车道变换（仅在允许时，同向 Driving）
        if not (bootstrap_mode and not allow_lane_change):  # Bootstrap模式初期可能禁止变道
            try:
                lc = getattr(wp, 'lane_change', 0)
            except Exception:
                lc = 0
            if lc & carla.LaneChange.Left:
                try:
                    lwp = wp.get_left_lane()
                except Exception:
                    lwp = None
                if lwp and lwp.lane_type == carla.LaneType.Driving and (lwp.lane_id * wp.lane_id) > 0:
                    cost = euclid2((wp.transform.location.x, wp.transform.location.y), (lwp.transform.location.x, lwp.transform.location.y))
                    res.append((lwp, max(1.0, float(cost))))
            if lc & carla.LaneChange.Right:
                try:
                    rwp = wp.get_right_lane()
                except Exception:
                    rwp = None
                if rwp and rwp.lane_type == carla.LaneType.Driving and (rwp.lane_id * wp.lane_id) > 0:
                    cost = euclid2((wp.transform.location.x, wp.transform.location.y), (rwp.transform.location.x, rwp.transform.location.y))
                    res.append((rwp, max(1.0, float(cost))))

        # 掉头（非路口，尽量在本路段中部完成，不先驶入下一条路）
        # Bootstrap模式下禁止掉头
        if self.allow_uturn and not getattr(wp, 'is_junction', False) and not bootstrap_mode:
            # 沿左右方向搜到“对向”车道（lane_id 符号翻转）
            def _find_opposite(cur):
                # 向左找
                l = cur
                for _ in range(6):
                    try:
                        nl = l.get_left_lane()
                    except Exception:
                        nl = None
                    if not nl or nl.lane_type != carla.LaneType.Driving:
                        break
                    l = nl
                    if (l.lane_id * cur.lane_id) < 0:
                        return l
                # 向右找
                r = cur
                for _ in range(6):
                    try:
                        nr = r.get_right_lane()
                    except Exception:
                        nr = None
                    if not nr or nr.lane_type != carla.LaneType.Driving:
                        break
                    r = nr
                    if (r.lane_id * cur.lane_id) < 0:
                        return r
                return None
            opp = _find_opposite(wp)
            if opp is not None:
                # 在对向车道上“反向”推进一个步长，表示完成 U 型转弯后回到反向流
                try:
                    prevs = [p for p in opp.previous(step_fwd) if p.lane_type == carla.LaneType.Driving]
                except Exception:
                    prevs = []
                cand = prevs[0] if prevs else opp
                k = self._wp_key(cand)
                if k not in uniq:
                    uniq.add(k)
                    # 成本稍高，避免无谓 U-turn；但允许在需要时使用
                    res.append((cand, float(step_fwd * 2.2)))
        return res

    def plan_bootstrap(self, start_wp: 'carla.Waypoint', psi0: float, goal_wp: 'carla.Waypoint' = None, max_iters: int = 50000):
        """Bootstrap规划：从起点开始，沿车头方向前进boot_dist距离或到达第一个路口
        Args:
            start_wp: 起始waypoint
            psi0: 车辆初始朝向（弧度）
            goal_wp: 可选的目标waypoint，用于计算启发式
            max_iters: 最大迭代次数
        Returns:
            (boot_path, boot_end_wp, accumulated_dist)
        """
        import heapq
        
        print(f"[BOOT] Starting bootstrap planning with psi0={to_deg(psi0):.1f}°, target_dist={self.boot_dist}m", flush=True)
        
        start_xy = (start_wp.transform.location.x, start_wp.transform.location.y)
        start_key = self._wp_key(start_wp)
        
        open_heap = []  # (f, g, key, waypoint, accumulated_dist)
        if goal_wp:
            goal_xy = (goal_wp.transform.location.x, goal_wp.transform.location.y)
            h0 = self.heuristic(start_xy, goal_xy)
        else:
            h0 = 0.0
            
        heapq.heappush(open_heap, (h0, 0.0, start_key, start_wp, 0.0))
        came = {}
        gscore = {start_key: 0.0}
        dist_score = {start_key: 0.0}  # 累计行驶距离
        visited = set()
        
        iters = 0
        best_node = None
        allow_lane_change = False  # 初始禁止变道
        
        while open_heap and iters < max_iters:
            iters += 1
            f, g, key, wp, acc_dist = heapq.heappop(open_heap)
            
            if key in visited:
                continue
            visited.add(key)
            
            # 检查是否到达路口或累计距离足够
            if wp.is_junction or acc_dist >= self.boot_dist:
                best_node = (key, wp, acc_dist)
                print(f"[BOOT] Reached {'junction' if wp.is_junction else f'{acc_dist:.1f}m'}, stopping bootstrap", flush=True)
                break
                
            # 记录最远的非路口点
            if best_node is None or acc_dist > best_node[2]:
                best_node = (key, wp, acc_dist)
            
            # 获取邻居（Bootstrap模式）
            for nb, move_cost in self.neighbors(wp, bootstrap_mode=True, psi0=psi0, allow_lane_change=allow_lane_change):
                nb_key = self._wp_key(nb)
                new_dist = acc_dist + move_cost
                
                # 方向惩罚（较小，主要靠方向锥约束）
                yaw_cur = to_rad(wp.transform.rotation.yaw)
                yaw_nb = to_rad(nb.transform.rotation.yaw)
                d_yaw = abs((yaw_nb - yaw_cur + math.pi) % (2*math.pi) - math.pi)
                heading_pen = 0.5 * d_yaw  # Bootstrap阶段降低转向惩罚
                
                tentative_g = g + float(move_cost) + heading_pen
                
                if nb_key in gscore and tentative_g >= gscore[nb_key]:
                    continue
                    
                came[nb_key] = (key, wp)
                gscore[nb_key] = tentative_g
                dist_score[nb_key] = new_dist
                
                if goal_wp:
                    nb_xy = (nb.transform.location.x, nb.transform.location.y)
                    h = self.heuristic(nb_xy, goal_xy)
                else:
                    h = 0.0
                    
                fscore = tentative_g + h * 0.5  # 降低启发式权重，更注重可行性
                heapq.heappush(open_heap, (fscore, tentative_g, nb_key, nb, new_dist))
        
        # 如果没找到合适的，尝试放宽约束
        if best_node is None or best_node[2] < self.boot_dist * 0.5:
            print(f"[BOOT] Initial bootstrap failed (dist={best_node[2] if best_node else 0:.1f}m), relaxing constraints...", flush=True)
            # 允许变道重试
            allow_lane_change = True
            # 可以增加角度阈值等其他放宽策略
            # 这里简化处理，返回当前最好的结果
        
        if best_node:
            key, wp, dist = best_node
            boot_path = self._reconstruct_path(came, key, wp)
            print(f"[BOOT] Bootstrap path found: {len(boot_path)} waypoints, {dist:.1f}m", flush=True)
            return boot_path, wp, dist
        else:
            print("[WARN] Bootstrap failed completely, returning start point only", flush=True)
            return [(start_wp.transform.location.x, start_wp.transform.location.y, start_wp.transform.location.z, 
                     start_wp.transform.rotation.yaw, start_wp)], start_wp, 0.0

    def plan(self, start_wp: 'carla.Waypoint', goal_wp: 'carla.Waypoint', max_iters: int = 200000, start_yaw: float = None):
        """两阶段A*规划：Bootstrap起步 + 全局规划"""
        import heapq
        
        # 阶段1：Bootstrap起步规划
        boot_path = []
        boot_end_wp = start_wp
        if start_yaw is not None:
            psi0 = start_yaw
            # 检查起步方向与目标的夹角
            goal_xy = (goal_wp.transform.location.x, goal_wp.transform.location.y)
            start_xy = (start_wp.transform.location.x, start_wp.transform.location.y)
            to_goal_angle = math.atan2(goal_xy[1] - start_xy[1], goal_xy[0] - start_xy[0])
            angle_diff = abs((to_goal_angle - psi0 + math.pi) % (2*math.pi) - math.pi)
            
            print(f"[A*] Start yaw={to_deg(psi0):.1f}°, to_goal={to_deg(to_goal_angle):.1f}°, diff={to_deg(angle_diff):.1f}°", flush=True)
            
            # 执行Bootstrap规划
            boot_path, boot_end_wp, boot_dist = self.plan_bootstrap(start_wp, psi0, goal_wp)
            
            # 如果目标在反向且Bootstrap成功，尝试本地U-Turn
            if angle_diff > to_rad(120) and boot_dist > 5.0:
                print(f"[UTURN] Target is behind ({to_deg(angle_diff):.1f}°), attempting local U-turn...", flush=True)
                # 这里可以实现U-Turn逻辑，暂时跳过
        
        # 阶段2：从Bootstrap终点开始的全局规划
        if len(boot_path) > 1:
            print(f"[A* GLOBAL] Starting global planning from bootstrap endpoint", flush=True)
            actual_start_wp = boot_end_wp
        else:
            print(f"[A* GLOBAL] No bootstrap phase, starting global planning from origin", flush=True)
            actual_start_wp = start_wp
            
        start_xy = (actual_start_wp.transform.location.x, actual_start_wp.transform.location.y)
        goal_xy = (goal_wp.transform.location.x, goal_wp.transform.location.y)
        start_key = self._wp_key(actual_start_wp)
        
        open_heap = []  # (f, g, key, waypoint, travel_dist)
        heapq.heappush(open_heap, (self.heuristic(start_xy, goal_xy), 0.0, start_key, actual_start_wp, 0.0))
        came = {}
        gscore = {start_key: 0.0}
        visited = set()
        
        iters = 0
        best_node = (start_key, actual_start_wp)
        best_h = self.heuristic(start_xy, goal_xy)

        while open_heap and iters < max_iters:
            iters += 1
            f, g, key, wp, travel_dist = heapq.heappop(open_heap)
            if key in visited:
                continue
            visited.add(key)
            loc_xy = (wp.transform.location.x, wp.transform.location.y)
            h = self.heuristic(loc_xy, goal_xy)
            if h < best_h:
                best_h = h
                best_node = (key, wp)
            if self.is_goal(loc_xy, goal_xy):
                goal_key = key
                # 合并Bootstrap路径和全局路径
                global_path = self._reconstruct_path(came, goal_key, wp)
                if len(boot_path) > 1:
                    # 去除重复的连接点
                    combined_path = boot_path[:-1] + global_path
                else:
                    combined_path = global_path
                print(f"[A*] Combined path: bootstrap={len(boot_path)}, global={len(global_path)}, total={len(combined_path)}", flush=True)
                return combined_path

            for nb, move_cost in self.neighbors(wp):
                nb_key = self._wp_key(nb)
                new_travel_dist = travel_dist + move_cost
                
                # —— 方向惩罚随距离衰减 ——
                yaw_cur = to_rad(wp.transform.rotation.yaw)
                yaw_nb  = to_rad(nb.transform.rotation.yaw)
                d_yaw = abs((yaw_nb - yaw_cur + math.pi) % (2*math.pi) - math.pi)
                
                # 计算衰减的方向惩罚系数
                alpha = self.turn_penalty_alpha0 * math.exp(-travel_dist / self.turn_penalty_decay)
                heading_pen = alpha * d_yaw
                
                # 车道变换惩罚（也随距离衰减）
                lane_pen = (0.5 * alpha) if (nb.lane_id != wp.lane_id) else 0.0
                
                tentative_g = g + float(move_cost) + heading_pen + lane_pen
                if nb_key in gscore and tentative_g >= gscore[nb_key]:
                    continue
                came[nb_key] = (key, wp)
                gscore[nb_key] = tentative_g
                fscore = tentative_g + self.heuristic((nb.transform.location.x, nb.transform.location.y), goal_xy)
                heapq.heappush(open_heap, (fscore, tentative_g, nb_key, nb, new_travel_dist))

        # 若未到达, 返回至当前最接近目标的 best 节点路径
        key, wp = best_node
        global_path = self._reconstruct_path(came, key, wp)
        if len(boot_path) > 1:
            combined_path = boot_path[:-1] + global_path
        else:
            combined_path = global_path
        print(f"[A*] Partial path: bootstrap={len(boot_path)}, global={len(global_path)}, total={len(combined_path)}", flush=True)
        return combined_path

    def _reconstruct_path(self, came, last_key, last_wp):
        path = [(last_key, last_wp)]
        cur_key = last_key
        while cur_key in came:
            prev_key, prev_wp = came[cur_key]
            path.append((prev_key, prev_wp))
            cur_key = prev_key
        path.reverse()
        # 转换为 (x,y,z,yaw,wp) 列表
        out = []
        for _, wp in path:
            t = wp.transform
            out.append((t.location.x, t.location.y, t.location.z, t.rotation.yaw, wp))
        return out

### -------------------- Dubins 生成与平滑 -------------------- ###

class DubinsSmoother:
    def __init__(self, world: 'carla.World', turning_radius: float = 9.0, sample_step: float = 0.5, lane_margin: float = 0.35,
                 start_cone_deg: float = 50.0):
        self.world = world
        self.map = world.get_map()
        self.R = turning_radius
        self.sample_step = sample_step
        self.margin = lane_margin
        self.start_cone_deg = start_cone_deg  # 起步锥角度
        # —— 性能优化：缓存与走廊空间索引 ——
        self.wp_cache = {}               # 量化坐标 → (wp, is_ok)
        self.cache_quant = 0.2           # 20cm 量化，近似即可
        self.corridor_bins = None        # {(ix,iy): [(x,y), ...]}
        self.bin_size = 3.0              # 与 near_tol 同量级

    # ---------- 加速：缓存的 waypoint 查询 ----------
    def _wp_lookup(self, x: float, y: float):
        qx = round(x / self.cache_quant) * self.cache_quant
        qy = round(y / self.cache_quant) * self.cache_quant
        key = (qx, qy)
        hit = self.wp_cache.get(key, None)
        if hit is not None:
            return hit
        loc = carla.Location(x=float(qx), y=float(qy), z=0.1)
        wp = self.map.get_waypoint(loc, project_to_road=False)
        self.wp_cache[key] = wp
        return wp

    # ---------- 加速：走廊点空间哈希 ----------
    def set_corridor_index(self, corridor_xy: list, near_tol: float = 3.0):
        self.bin_size = max(near_tol, 1.0)
        bins = {}
        bs = self.bin_size
        for (x, y) in corridor_xy:
            ix = int(math.floor(x / bs))
            iy = int(math.floor(y / bs))
            bins.setdefault((ix, iy), []).append((x, y))
        self.corridor_bins = bins

    def _nearby_corridor_points(self, x: float, y: float):
        if not self.corridor_bins:
            return None
        bs = self.bin_size
        ix = int(math.floor(x / bs))
        iy = int(math.floor(y / bs))
        out = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                out.extend(self.corridor_bins.get((ix+dx, iy+dy), ()))
        return out

    def _lane_ok(self, loc: 'carla.Location') -> bool:
        """严格判定 loc 是否处于可驾驶车道内部：
        - 使用 project_to_road=False（不投影），避免“贴边点被吸附到道路上”的假阳性；
        - 再做横向偏移约束，确保在本车道中心附近而非擦着路肩/花坛；
        """
        wp = self._wp_lookup(loc.x, loc.y)
        if wp is None or wp.lane_type != carla.LaneType.Driving:
            return False
        center = wp.transform.location
        yaw = to_rad(wp.transform.rotation.yaw)
        dx, dy = loc.x - center.x, loc.y - center.y
        lat = -math.sin(yaw) * dx + math.cos(yaw) * dy
        half = 0.5 * (wp.lane_width if hasattr(wp, 'lane_width') else 3.5)
        return abs(lat) <= half - 0.05 + self.margin  # 贴边收缩 5cm

    def _sample_and_check(self, configs, allowed_lanes: set, corridor_xy: list, near_tol: float = 3.0, 
                         first_segment: bool = False, psi0: float = None):
        """逐点采样校验（两阶段，带空间索引）：
        1) 先稀疏抽查（步长≈3×），快速否决；
        2) 再全量细查（步长≈1×）。
        校验项：车道内点、允许车道（路口放宽）、贴合走廊距离。
        first_segment: 是否为第一段（起步段），需要额外的方向锥和前向投影约束
        """
        amap = self.map
        # —— 稀疏抽查 ——
        stride = 3
        for pass_id in (0, 1):
            if pass_id == 1:
                stride = 1
            for idx in range(0, len(configs), stride):
                x, y, theta = configs[idx]
                
                # 第一段的起步锥和前向投影约束
                if first_segment and psi0 is not None:
                    # 方向锥约束
                    angle_diff = abs((theta - psi0 + math.pi) % (2*math.pi) - math.pi)
                    if to_deg(angle_diff) > self.start_cone_deg:
                        return False
                    # 前向投影约束（相对于起点不能向后）
                    if idx == 0:
                        x0, y0 = x, y
                    else:
                        dx = x - x0
                        dy = y - y0
                        forward_proj = math.cos(psi0) * dx + math.sin(psi0) * dy
                        if forward_proj < -0.5:  # 允许小幅后退以适应转弯
                            return False
                
                loc = carla.Location(x=float(x), y=float(y), z=0.1)
                if not self._lane_ok(loc):
                    return False
                wp = self._wp_lookup(x, y)
                if wp is None:
                    return False
                if (not wp.is_junction) and ((wp.road_id, wp.lane_id) not in allowed_lanes):
                    return False
                # 走廊贴合（优先用空间哈希加速）
                tol = near_tol * (1.5 if wp.is_junction else 1.0)
                cand = self._nearby_corridor_points(x, y)
                if cand is None:
                    cand = corridor_xy
                min_d2 = tol*tol + 1.0
                for (ax, ay) in cand:
                    dx = ax - x; dy = ay - y
                    d2 = dx*dx + dy*dy
                    if d2 < min_d2:
                        min_d2 = d2
                        if d2 <= tol*tol:
                            break
                if min_d2 > tol*tol:
                    return False
        return True

    def _dubins_shortest(self, q0, q1):
        if _HAS_DUBINS:
            path = dubins.shortest_path(q0, q1, self.R)
            configs, _ = path.sample_many(self.sample_step)
            return configs, path.path_length()
        return self._fallback_arc_straight_arc(q0, q1)

    def _fallback_arc_straight_arc(self, q0, q1):
        x0, y0, th0 = q0
        x1, y1, th1 = q1
        dir_th = math.atan2(y1 - y0, x1 - x0)
        dth1 = (dir_th - th0 + math.pi) % (2*math.pi) - math.pi
        turn1_dir = 1.0 if dth1 > 0 else -1.0
        arc_len1 = abs(dth1) * self.R
        cx1 = x0 - turn1_dir * self.R * math.sin(th0)
        cy1 = y0 + turn1_dir * self.R * math.cos(th0)
        n1 = max(1, int(max(arc_len1, self.sample_step) / self.sample_step))
        arc1 = []
        for i in range(1, n1+1):
            ang = th0 + turn1_dir * (i/n1) * dth1
            px = cx1 + turn1_dir * self.R * math.sin(ang)
            py = cy1 - turn1_dir * self.R * math.cos(ang)
            arc1.append((px, py, ang))
        sx, sy, sth = (arc1[-1] if arc1 else (x0, y0, th0))
        dx, dy = x1 - sx, y1 - sy
        dist = math.hypot(dx, dy)
        line_len = max(0.0, dist - self.R)
        n2 = max(1, int(max(line_len, self.sample_step) / self.sample_step))
        line = []
        for i in range(1, n2+1):
            lam = i / n2
            px = sx + lam * math.cos(dir_th) * line_len
            py = sy + lam * math.sin(dir_th) * line_len
            line.append((px, py, dir_th))
        lx, ly, lth = (line[-1] if line else (sx, sy, sth))
        dth2 = (th1 - dir_th + math.pi) % (2*math.pi) - math.pi
        turn2_dir = 1.0 if dth2 > 0 else -1.0
        arc_len2 = abs(dth2) * self.R
        cx2 = lx - turn2_dir * self.R * math.sin(dir_th)
        cy2 = ly + turn2_dir * self.R * math.cos(dir_th)
        n3 = max(1, int(max(arc_len2, self.sample_step) / self.sample_step))
        arc2 = []
        for i in range(1, n3+1):
            ang = dir_th + turn2_dir * (i/n3) * dth2
            px = cx2 + turn2_dir * self.R * math.sin(ang)
            py = cy2 - turn2_dir * self.R * math.cos(ang)
            arc2.append((px, py, ang))
        path = []
        path.extend(arc1)
        path.extend(line)
        path.extend(arc2)
        length = arc_len1 + line_len + arc_len2
        return path, length

    def smooth_greedy(self, poses, allowed_lanes: set, corridor_xy: list, 
                     allowed_lanes_front: set = None, boot_segment_count: int = 0, psi0: float = None):
        """Greedy 跨段 + 加速：
        - 走廊建索引；
        - j 回退采用分段递减（每次减少 (j-i)//3，后期再逐一细化），大幅减少尝试次数；
        - 采样校验两阶段（稀疏→细化）；
        - allowed_lanes_front: 起步段的允许车道集合
        - boot_segment_count: 起步段的waypoint数量
        """
        N = len(poses)
        if N < 2:
            return [], []
        # 建立走廊空间索引（near_tol≈3m，与 _sample_and_check 保持一致）
        self.set_corridor_index(corridor_xy, near_tol=3.0)

        samples = []
        seg_ends = []
        i = 0
        while i < N - 1:
            q0 = (poses[i][0], poses[i][1], to_rad(poses[i][3]))
            # 先试更远的 j，失败就快速回退，接近后再细化
            j_far = N - 1
            accepted = None
            j = j_far
            while j > i:
                q1 = (poses[j][0], poses[j][1], to_rad(poses[j][3]))
                configs, _ = self._dubins_shortest(q0, q1)
                if len(configs) >= 2:
                    # 判断是否为第一段（起步段）
                    is_first_segment = (i == 0 and boot_segment_count > 0)
                    if is_first_segment and allowed_lanes_front is not None:
                        # 第一段使用起步走廊和起步锥约束
                        check_result = self._sample_and_check(configs, allowed_lanes_front, corridor_xy, 
                                                             first_segment=True, psi0=psi0)
                    else:
                        # 其他段使用全局走廊
                        check_result = self._sample_and_check(configs, allowed_lanes, corridor_xy)
                    
                    if check_result:
                        accepted = configs
                        break
                step = max(1, (j - i) // 3)
                if j - step <= i + 1:
                    # 最后阶段逐一细化
                    j -= 1
                else:
                    j -= step
            if accepted is None:
                j = i + 1
                q1 = (poses[j][0], poses[j][1], to_rad(poses[j][3]))
                accepted, _ = self._dubins_shortest(q0, q1)
                if not self._sample_and_check(accepted, allowed_lanes, corridor_xy):
                    accepted = [(poses[j][0], poses[j][1], to_rad(poses[j][3]))]
            if samples and accepted:
                samples.extend(accepted[1:])
            else:
                samples.extend(accepted)
            seg_ends.append(len(samples) - 1)
            i = j
        return samples, seg_ends

    def shortcut_iterative(self, samples, allowed_lanes: set, corridor_xy: list, iters: int = 4, min_gap_pts: int = 12, improve_ratio: float = 0.98):
        """在已有 Dubins 采样序列上做局部重连 (shortcut) 迭代，进一步平滑/缩短路径。
        - 使用走廊空间索引与缓存加速校验；
        - 仅当新段满足校验且长度优于旧段 improve_ratio 比例时替换。
        返回 (new_samples, new_seg_ends)
        """
        import random as _rnd
        S = list(samples)
        L = len(S)
        if L < min_gap_pts + 2:
            return S, [L-1]
        # 建立走廊空间索引（若还没建）
        if not self.corridor_bins:
            self.set_corridor_index(corridor_xy, near_tol=3.0)
        for _ in range(max(0, iters)):
            improved = False
            for _try in range(160):
                i = _rnd.randint(0, L - min_gap_pts - 2)
                j = _rnd.randint(i + min_gap_pts, L - 1)
                q0 = (S[i][0], S[i][1], S[i][2])
                q1 = (S[j][0], S[j][1], S[j][2])
                cfgs, _ = self._dubins_shortest(q0, q1)
                if len(cfgs) < 2:
                    continue
                if not self._sample_and_check(cfgs, allowed_lanes, corridor_xy):
                    continue
                # 计算旧段/新段长度
                old_len = 0.0
                for k in range(i, j):
                    old_len += euclid2(S[k][:2], S[k+1][:2])
                new_len = 0.0
                for k in range(len(cfgs)-1):
                    new_len += euclid2(cfgs[k][:2], cfgs[k+1][:2])
                if new_len <= improve_ratio * old_len:
                    S = S[:i] + cfgs + S[j+1:]
                    L = len(S)
                    improved = True
                    break
            if not improved:
                break
        return S, [len(S)-1]

### -------------------- 可视化与保存 -------------------- ###

class Visualizer:
    def __init__(self, world: 'carla.World'):
        self.world = world
        self.dbg = world.debug

    def draw_points(self, points_xyz, color, size=0.09, life_time=0.0, z_offset=0.35):
        for x, y, z in points_xyz:
            loc = carla.Location(x=float(x), y=float(y), z=float(z) + z_offset)
            self.dbg.draw_point(loc, size=size, color=color, life_time=life_time)

    def draw_polyline(self, points_xyz, color, thickness=0.18, life_time=0.0, z_offset=0.28):
        for i in range(len(points_xyz)-1):
            x1, y1, z1 = points_xyz[i]
            x2, y2, z2 = points_xyz[i+1]
            a = carla.Location(x=float(x1), y=float(y1), z=float(z1) + z_offset)
            b = carla.Location(x=float(x2), y=float(y2), z=float(z2) + z_offset)
            self.dbg.draw_line(a, b, thickness=thickness, color=color, life_time=life_time)

    def draw_start_end(self, start_xyz, end_xyz, life_time=0.0):
        sx, sy, sz = start_xyz
        ex, ey, ez = end_xyz
        # 起点：深绿色圆点 + 标签
        self.dbg.draw_point(carla.Location(sx, sy, sz+0.8), size=0.28, color=carla.Color(46, 125, 50), life_time=life_time)
        self.dbg.draw_string(carla.Location(sx, sy, sz+1.5), 'START', draw_shadow=False, color=carla.Color(46, 125, 50), life_time=life_time)
        # 终点：蓝色圆点 + 标签
        self.dbg.draw_point(carla.Location(ex, ey, ez+0.8), size=0.28, color=carla.Color(21, 101, 192), life_time=life_time)
        self.dbg.draw_string(carla.Location(ex, ey, ez+1.5), 'GOAL', draw_shadow=False, color=carla.Color(21, 101, 192), life_time=life_time)

### -------------------- 车辆跟踪(简易 Pure Pursuit) -------------------- ###

def pure_pursuit_control(vehicle: 'carla.Vehicle', target_xy, kp_steer=1.6):
    tf = vehicle.get_transform()
    loc = tf.location
    yaw = to_rad(tf.rotation.yaw)
    px, py = target_xy
    dx, dy = px - loc.x, py - loc.y
    target_heading = math.atan2(dy, dx)
    ang_err = (target_heading - yaw + math.pi) % (2*math.pi) - math.pi
    steer = clamp(kp_steer * ang_err, -1.0, 1.0)
    # 前进偏好：根据前向投影决定推进力度，避免大角误差时“踩刹车卡死”
    forward = max(0.0, math.cos(ang_err))  # -pi..pi → 1 前向, -1 后向
    base = 0.07
    var = 0.18 * (1.0 - min(1.0, abs(ang_err)/1.2))
    throttle = clamp(base + var * forward, 0.08, 0.26)
    brake = 0.0  # 不再因角误差强制刹车，改为小油门慢慢纠正
    return carla.VehicleControl(throttle=throttle, steer=steer, brake=brake, hand_brake=False)

### -------------------- 截图保存 -------------------- ###

def take_topdown_screenshot(world: 'carla.World', path_xyz, out_png_path: str, fov=90):
    if not path_xyz:
        return
    min_x = min(p[0] for p in path_xyz)
    max_x = max(p[0] for p in path_xyz)
    min_y = min(p[1] for p in path_xyz)
    max_y = max(p[1] for p in path_xyz)
    cx = 0.5 * (min_x + max_x)
    cy = 0.5 * (min_y + max_y)
    span = max(max_x - min_x, max_y - min_y)
    height = max(40.0, span)

    bp_lib = world.get_blueprint_library()
    cam_bp = bp_lib.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', '2560')
    cam_bp.set_attribute('image_size_y', '1440')
    cam_bp.set_attribute('fov', str(fov))

    cam_tf = carla.Transform(
        location=carla.Location(x=cx, y=cy, z=height),
        rotation=carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0)
    )
    sensor = world.try_spawn_actor(cam_bp, cam_tf)
    if sensor is None:
        sensor = world.spawn_actor(cam_bp, cam_tf)

    done = {'saved': False}
    def _save(image: 'carla.Image'):
        if not done['saved']:
            image.save_to_disk(out_png_path)
            done['saved'] = True
    sensor.listen(_save)

    for _ in range(15):
        world.tick()
        if done['saved']:
            break
    try:
        sensor.stop()
    except Exception:
        pass
    finally:
        try:
            sensor.destroy()
        except Exception:
            pass

### -------------------- 论文风格(白底+网格+障碍+路网+路径)渲染 -------------------- ###
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MultipleLocator

def render_paper_figure_from_csv(world: 'carla.World', csv_path: str, out_png_path: str,
                                 grid_res: float = 1.0, margin: float = 25.0,
                                 max_cells: int = 300000, info_text: str = None):
    """生成论文风格的 2D 图：白底、网格、黑色障碍（非道路）、路网线、红色路径、绿点起点、蓝点终点。
    - grid_res: 网格栅格间距(米)
    - margin: 在路径包围盒基础上扩张的边界(米)
    - max_cells: 网格最大像素数上限(防止极大范围导致卡顿)
    """
    # 1) 读取 CSV 路径点
    xs, ys = [], []
    import csv as _csv
    with open(csv_path, 'r', newline='') as f:
        reader = _csv.reader(f)
        header = next(reader, None)
        for row in reader:
            try:
                x = float(row[2]); y = float(row[3])
                xs.append(x); ys.append(y)
            except Exception:
                continue
    if len(xs) < 2:
        raise RuntimeError('CSV 中路径点不足，无法生成论文风格图')

    # 2) 计算绘制区域
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    min_x -= margin; max_x += margin; min_y -= margin; max_y += margin

    # 3) 动态控制网格分辨率，防止像素过多
    nx = int(math.ceil((max_x - min_x) / grid_res))
    ny = int(math.ceil((max_y - min_y) / grid_res))
    cells = nx * ny
    if cells > max_cells and cells > 0:
        scale = math.sqrt(cells / max_cells)
        grid_res *= scale * 1.05
        nx = int(math.ceil((max_x - min_x) / grid_res))
        ny = int(math.ceil((max_y - min_y) / grid_res))

    # 4) 构建占据网格：非 Driving 车道视为障碍(黑)
    occ = np.ones((ny, nx), dtype=np.uint8)  # 1=障碍(黑), 0=可行驶(白)
    carla_map = world.get_map()
    for j in range(ny):
        y = min_y + (j + 0.5) * grid_res
        for i in range(nx):
            x = min_x + (i + 0.5) * grid_res
            loc = carla.Location(x=float(x), y=float(y), z=0.1)
            wp = carla_map.get_waypoint(loc, project_to_road=False)
            if wp is not None and wp.lane_type == carla.LaneType.Driving:
                occ[j, i] = 0

    # 5) 开始绘图
    fig, ax = plt.subplots(figsize=(8.5, 8.5), dpi=220)
    ax.set_facecolor('white')

    # 背景：白底 + 黑色障碍
    ax.imshow(occ, cmap='binary', origin='lower',
              extent=[min_x, max_x, min_y, max_y], interpolation='nearest')

    # 路网线：用拓扑粗略描绘（浅灰细线）
    try:
        topo = carla_map.get_topology()
        for wpa, wpb in topo:
            a = wpa.transform.location; b = wpb.transform.location
            if (min_x-5 <= a.x <= max_x+5 and min_y-5 <= a.y <= max_y+5) or \
               (min_x-5 <= b.x <= max_x+5 and min_y-5 <= b.y <= max_y+5):
                ax.plot([a.x, b.x], [a.y, b.y], linewidth=0.5, color='#999999', alpha=0.5, zorder=2)
    except Exception:
        pass

    # 规划路径：红色线（更粗）
    # 路径底光晕（浅红，稍粗）
    ax.plot(xs, ys, color='#ffcdd2', linewidth=4.0, zorder=3, alpha=0.8,
            solid_capstyle='round', solid_joinstyle='round')
    # 规划路径：深红（更粗、圆角）
    ax.plot(xs, ys, color='#c62828', linewidth=2.4, zorder=4, label='Path',
            solid_capstyle='round', solid_joinstyle='round')

    # 起点(绿点更粗)、终点(蓝点更粗)
    ax.plot(xs[0], ys[0], 'o', markersize=8, markerfacecolor='#2e7d32', markeredgecolor='#1b5e20', zorder=5, label='Start')
    ax.plot(xs[-1], ys[-1], 'o', markersize=8, markerfacecolor='#1565c0', markeredgecolor='#0d47a1', zorder=5, label='Goal')

    # 网格
    major = max(5.0, grid_res * 5)
    minor = grid_res
    ax.xaxis.set_major_locator(MultipleLocator(major))
    ax.yaxis.set_major_locator(MultipleLocator(major))
    ax.xaxis.set_minor_locator(MultipleLocator(minor))
    ax.yaxis.set_minor_locator(MultipleLocator(minor))
    ax.grid(which='major', color='#d4d9df', linestyle='-', linewidth=0.7, zorder=1)
    ax.grid(which='minor', color='#eef2f7', linestyle='-', linewidth=0.35, zorder=1)

    # 比例尺（20 m）
    span_x = max_x - min_x
    span_y = max_y - min_y
    scale_len = 20.0
    sx0 = max_x - 0.1*span_x - scale_len
    sx1 = max_x - 0.1*span_x
    sy  = min_y + 0.08*span_y
    ax.plot([sx0, sx1], [sy, sy], color='black', linewidth=2.2, zorder=6)
    ax.text((sx0+sx1)/2, sy + 0.01*span_y, f"{int(scale_len)} m", ha='center', va='bottom', fontsize=8)

    # 北向箭头
    nx = max_x - 0.08*span_x
    ny = min_y + 0.22*span_y
    ax.annotate('N', xy=(nx, ny), xytext=(nx, ny+0.06*span_y),
                arrowprops=dict(arrowstyle='-|>', lw=1.2, color='black'),
                ha='center', va='bottom', fontsize=9, color='black')

    # 信息说明框（右上角）
    # 统计路径长度
    path_len = 0.0
    for i in range(1, len(xs)):
        path_len += math.hypot(xs[i]-xs[i-1], ys[i]-ys[i-1])
    meta_lines = []
    town = getattr(world.get_map(), 'name', 'Unknown')
    meta_lines.append(f"Town: {town}")
    meta_lines.append(f"Path length: {path_len:.1f} m")
    if info_text:
        meta_lines.append(info_text)
    box_text = "\n".join(meta_lines)
    ax.text(max_x - 0.01*span_x, max_y - 0.01*span_y, box_text,
            ha='right', va='top', fontsize=8,
            bbox=dict(boxstyle='round,pad=0.35', fc='white', ec='#444444', alpha=0.9))

    # 图例（小号、半透明边框）
    leg = ax.legend(loc='lower left', fontsize=8, frameon=True)
    leg.get_frame().set_alpha(0.9)
    leg.get_frame().set_edgecolor('#aaaaaa')

    # 轴外观微调
    for s in ax.spines.values():
        s.set_color('#b0b5bb')
        s.set_linewidth(0.6)

    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')

    fig.tight_layout()
    fig.savefig(out_png_path, dpi=220)
    plt.close(fig)


### -------------------- 主流程 -------------------- ###

def _destroy_actors_safe(actors):
    if not actors:
        return
    for a in actors:
        try:
            a.destroy()
        except Exception:
            pass


def _clear_npcs(world: 'carla.World'):
    actors = world.get_actors()
    to_kill = [a for a in actors if a.type_id.startswith('vehicle.') or a.type_id.startswith('walker.')]
    if to_kill:
        try:
            batch = [carla.command.DestroyActor(a.id) for a in to_kill]
            world.get_client().apply_batch_sync(batch, True)
        except Exception:
            for a in to_kill:
                try:
                    a.destroy()
                except Exception:
                    pass

# -------- 选车 & 过滤 spawn 点（避免路口中间/逆向） -------- #

def _pick_ego_blueprint(bp_lib: 'carla.BlueprintLibrary') -> 'carla.ActorBlueprint':
    prefs = [
        'vehicle.tesla.cybertruck',
        '*cyber*',
        '*pickup*',
        'vehicle.ford.f150',
        'vehicle.jeep.*',
        'vehicle.tesla.model3'
    ]
    for pat in prefs:
        try:
            cands = bp_lib.filter(pat)
            if cands:
                return cands[0]
        except Exception:
            continue
    cands = bp_lib.filter('vehicle.*')
    if cands:
        return random.choice(cands)
    raise RuntimeError('No vehicle blueprint found')


def _align_tf_to_lane(tf: 'carla.Transform', wp: 'carla.Waypoint') -> 'carla.Transform':
    # 将朝向对齐到车道前进方向，保持位置在 spawn 点
    new_tf = carla.Transform(tf.location, tf.rotation)
    new_tf.rotation.yaw = wp.transform.rotation.yaw
    new_tf.rotation.pitch = 0.0
    new_tf.rotation.roll = 0.0
    return new_tf


def _is_good_spawn_tf(carla_map: 'carla.Map', tf: 'carla.Transform') -> tuple:
    """返回 (ok, aligned_tf)。过滤条件：
    - 在 Driving 车道;
    - 非路口，且前/后若干米仍非路口（避免生成在路口中心/边缘）；
    - 生成方向与车道方向对齐（不会逆行）。
    """
    wp = carla_map.get_waypoint(tf.location, project_to_road=True, lane_type=carla.LaneType.Driving)
    if wp is None or wp.lane_type != carla.LaneType.Driving:
        return False, tf
    if wp.is_junction:
        return False, tf
    try:
        nxt = wp.next(8.0)
    except Exception:
        nxt = []
    try:
        prv = wp.previous(8.0)
    except Exception:
        prv = []
    if any(n.is_junction for n in nxt) or any(p.is_junction for p in prv):
        return False, tf
    aligned_tf = _align_tf_to_lane(tf, wp)
    return True, aligned_tf


def _filter_spawn_points(world: 'carla.World', spawns: list) -> list:
    carla_map = world.get_map()
    goods = []
    for tf in spawns:
        ok, atf = _is_good_spawn_tf(carla_map, tf)
        if ok:
            goods.append(atf)
    return goods


def _try_spawn_vehicle(world: 'carla.World', bp_lib, spawns: list, max_tries=40):
    ego_bp = _pick_ego_blueprint(bp_lib)
    try:
        if ego_bp.has_attribute('role_name'):
            ego_bp.set_attribute('role_name', 'hero')
    except Exception:
        pass
    random.shuffle(spawns)
    for _ in range(max_tries):
        tf = random.choice(spawns)
        actor = world.try_spawn_actor(ego_bp, tf)
        if actor is not None:
            return actor, tf
    raise RuntimeError('多次尝试也未能生成自车，请检查地图或清理场景后再试')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--town', type=str, default=None, help='如指定则加载地图')
    parser.add_argument('--astar_step', type=float, default=3.0)
    parser.add_argument('--goal_thresh', type=float, default=5.0)
    parser.add_argument('--turning_radius', type=float, default=11.5)
    parser.add_argument('--dubins_step', type=float, default=0.35)
    parser.add_argument('--lane_margin', type=float, default=0.35)
    parser.add_argument('--allow_uturn', action='store_true', default=True, help='A* 允许后退/掉头扩展')
    parser.add_argument('--shortcut_iters', type=int, default=7, help='局部 Dubins 重连迭代次数')
    parser.add_argument('--shortcut_gap_pts', type=int, default=20, help='重连两端的最小点间隔')
    parser.add_argument('--shortcut_improve', type=float, default=0.995, help='新段长度需小于旧段比例才替换')
    parser.add_argument('--life_time', type=float, default=0.0, help='UE 可视化保留时长, 0=永久')
    parser.add_argument('--seed', type=int, default=None)
    parser.add_argument('--max_drive_seconds', type=float, default=180.0)
    # 新增起步相关参数
    parser.add_argument('--boot_dist', type=float, default=20.0, help='起步前行距离(米)')
    parser.add_argument('--boot_angle_deg', type=float, default=45.0, help='起步阶段允许的偏航夹角阈值(度)')
    parser.add_argument('--turn_penalty_alpha0', type=float, default=1.8, help='方向惩罚初值')
    parser.add_argument('--turn_penalty_decay', type=float, default=25.0, help='方向惩罚衰减距离(米)')
    parser.add_argument('--start_cone_deg', type=float, default=50.0, help='Dubins第一段的起步锥角度(度)')
    args = parser.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    # 连接 CARLA
    print(f"[INFO] Creating CARLA client for {args.host}:{args.port}", flush=True)
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)  # 初始超时设为10秒
    print(f"[INFO] Client created, attempting connection...", flush=True)
    
    # 先尝试快速ping测试
    try:
        client.get_server_version()
        print("[OK] CARLA server is responding", flush=True)
    except Exception as e:
        print(f"[ERROR] CARLA server not responding: {e}", flush=True)
        print("[INFO] Please make sure:")
        print("  1. CARLA server (CarlaUE4.exe) is running")
        print("  2. No other client is connected in synchronous mode")
        print("  3. The port 2000 is not blocked by firewall")
        raise RuntimeError("Cannot connect to CARLA server")
    
    # 延长超时并获取世界
    client.set_timeout(60.0)
    world = _wait_for_server(client, args.host, args.port, require_map=False)

    # 加载地图（如指定且当前不同）
    if args.town:
        try:
            cur = world.get_map().name
        except Exception:
            cur = None
        if (cur is None) or (cur != args.town):
            print(f"[INFO] Loading map: {args.town}", flush=True)
            world = client.load_world(args.town)
            world = _wait_for_server(client, args.host, args.port, require_map=True)

    # 切同步模式
    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    # 输出目录
    tm = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = os.path.join(os.getcwd(), tm)
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, f'path_{tm}.csv')
    top_png = os.path.join(out_dir, f'topdown_{tm}.png')
    paper_png = os.path.join(out_dir, f'paper_{tm}.png')

    ego = None
    try:
        # 清空 NPC
        world.tick()
        _clear_npcs(world)
        world.tick()

        # 生成自车与起点/终点
        bp_lib = world.get_blueprint_library()
        spawns_all = world.get_map().get_spawn_points()
        if len(spawns_all) < 2:
            raise RuntimeError('地图 spawn points 不足 2 个')
        spawns = _filter_spawn_points(world, spawns_all)
        print(f"[INFO] Spawn points: total={len(spawns_all)} filtered={len(spawns)}", flush=True)
        if len(spawns) < 2:
            # 若过滤过严，用对齐后的所有可驾驶 spawn
            spawns = []
            carla_map = world.get_map()
            for tf in spawns_all:
                wp = carla_map.get_waypoint(tf.location, project_to_road=True, lane_type=carla.LaneType.Driving)
                if wp is not None:
                    spawns.append(_align_tf_to_lane(tf, wp))
            print(f"[WARN] Filtered spawns too few, fallback to aligned all: {len(spawns)}", flush=True)
        print(f"[INFO] Try spawn ego from {len(spawns)} filtered spawns", flush=True)
        ego, start_tf = _try_spawn_vehicle(world, bp_lib, spawns)
        print(f"[OK] Ego spawned at ({start_tf.location.x:.1f},{start_tf.location.y:.1f}) yaw={start_tf.rotation.yaw:.1f}", flush=True)
        world.tick()

        end_tf = random.choice(spawns)
        tries = 0
        while (end_tf.location.distance(start_tf.location) < 70.0) and tries < 80:
            end_tf = random.choice(spawns)
            tries += 1

        # 视角跟随
        spectator = world.get_spectator()
        def follow():
            vtf = ego.get_transform()
            yaw = to_rad(vtf.rotation.yaw)
            back = carla.Location(-8.0 * math.cos(yaw), -8.0 * math.sin(yaw), 5.2)
            loc = vtf.location + back
            rot = carla.Rotation(pitch=-15.0, yaw=vtf.rotation.yaw, roll=0.0)
            spectator.set_transform(carla.Transform(loc, rot))

        # A*（两阶段规划）
        print('[INFO] Running two-stage A* search ...', flush=True)
        print(f'[INFO] Bootstrap params: dist={args.boot_dist}m, angle={args.boot_angle_deg}°', flush=True)
        print(f'[INFO] Turn penalty: alpha0={args.turn_penalty_alpha0}, decay={args.turn_penalty_decay}m', flush=True)
        
        amap = world.get_map()
        start_wp = amap.get_waypoint(start_tf.location, project_to_road=True, lane_type=carla.LaneType.Driving)
        goal_wp  = amap.get_waypoint(end_tf.location,   project_to_road=True, lane_type=carla.LaneType.Driving)
        
        # 创建A*规划器，传入新参数
        astar = AStarPlanner(world, step=args.astar_step, goal_thresh=args.goal_thresh, allow_uturn=args.allow_uturn,
                           boot_dist=args.boot_dist, boot_angle_deg=args.boot_angle_deg,
                           turn_penalty_alpha0=args.turn_penalty_alpha0, turn_penalty_decay=args.turn_penalty_decay)
        
        # 执行两阶段规划，传入车辆初始朝向
        start_yaw = to_rad(start_tf.rotation.yaw)
        astar_path = astar.plan(start_wp, goal_wp, start_yaw=start_yaw)
        
        if len(astar_path) < 2:
            raise RuntimeError('A* 未找到有效路径')
        print(f"[OK] A* done: {len(astar_path)} waypoints", flush=True)
        
        # 标记起步段waypoint数量（用于Dubins平滑）
        boot_segment_count = 0
        for i, (x, y, z, yaw, wp) in enumerate(astar_path):
            dist = euclid2((x, y), (start_tf.location.x, start_tf.location.y))
            if dist <= args.boot_dist:
                boot_segment_count = i + 1
            else:
                break
        print(f"[INFO] Bootstrap segment: {boot_segment_count} waypoints", flush=True)

        # 可视化 A*（区分起步段和全局段）
        viz = Visualizer(world)
        astar_xyz = [(x, y, z) for (x, y, z, yaw, _) in astar_path]
        
        if boot_segment_count > 0:
            # 起步段用浅绿色
            boot_xyz = astar_xyz[:boot_segment_count]
            global_xyz = astar_xyz[boot_segment_count-1:]  # 包含连接点
            
            # 绘制起步段（浅绿色）
            viz.draw_points(boot_xyz, color=carla.Color(144, 238, 144), size=0.08, life_time=args.life_time)
            viz.draw_polyline(boot_xyz, color=carla.Color(124, 252, 0), thickness=0.10, life_time=args.life_time)
            
            # 绘制全局段（青色）
            viz.draw_points(global_xyz, color=carla.Color(0, 255, 255), size=0.06, life_time=args.life_time)
            viz.draw_polyline(global_xyz, color=carla.Color(255, 255, 0), thickness=0.08, life_time=args.life_time)
        else:
            # 无起步段，全部用青色
            viz.draw_points(astar_xyz, color=carla.Color(0, 255, 255), size=0.06, life_time=args.life_time)
            viz.draw_polyline(astar_xyz, color=carla.Color(255, 255, 0), thickness=0.08, life_time=args.life_time)
        
        viz.draw_start_end(astar_xyz[0], astar_xyz[-1], life_time=args.life_time)
        world.tick()
        follow()

        # Dubins 平滑
        print('[INFO] Dubins smoothing (greedy) ...', flush=True)
        print(f'[INFO] Dubins start cone: {args.start_cone_deg}°', flush=True)
        
        smoother = DubinsSmoother(world, turning_radius=args.turning_radius, sample_step=args.dubins_step, 
                                 lane_margin=args.lane_margin, start_cone_deg=args.start_cone_deg)
        
        # 构建允许车道集合（分离前段和全局）
        allowed_lanes = set((wp.road_id, wp.lane_id) for (_, _, _, _, wp) in astar_path)
        allowed_lanes_front = set()
        if boot_segment_count > 0:
            # 起步段只允许同向车道
            for i in range(min(boot_segment_count, len(astar_path))):
                _, _, _, _, wp = astar_path[i]
                allowed_lanes_front.add((wp.road_id, wp.lane_id))
            print(f"[INFO] Front corridor: {len(allowed_lanes_front)} lanes", flush=True)
        
        # 构建走廊抽样（~5m）
        corridor_xy = []
        last_xy = None
        acc = 0.0
        for (x, y, z, yaw, wp) in astar_path:
            if last_xy is None:
                last_xy = (x, y)
                corridor_xy.append(last_xy)
                continue
            d = euclid2((x, y), last_xy)
            acc += d
            if acc >= 5.0:
                corridor_xy.append((x, y))
                acc = 0.0
                last_xy = (x, y)
        
        # 执行平滑，传入起步段参数
        dubins_samples, seg_ends = smoother.smooth_greedy(
            astar_path, allowed_lanes, corridor_xy,
            allowed_lanes_front=allowed_lanes_front if boot_segment_count > 0 else None,
            boot_segment_count=boot_segment_count,
            psi0=start_yaw
        )
        print(f"[OK] Dubins greedy: {len(dubins_samples)} samples, {len(seg_ends)} segments", flush=True)
        if len(dubins_samples) < 2:
            raise RuntimeError('Dubins 平滑失败')
        if args.shortcut_iters > 0:
            print('[INFO] Dubins local shortcut iterations ...', flush=True)
            dubins_samples, seg_ends2 = smoother.shortcut_iterative(
                dubins_samples, allowed_lanes, corridor_xy,
                iters=args.shortcut_iters, min_gap_pts=args.shortcut_gap_pts, improve_ratio=args.shortcut_improve)
            print(f"[OK] Shortcut done: {len(dubins_samples)} samples", flush=True)
            if seg_ends2:
                seg_ends = seg_ends2

        # —— 兜底方案：起步方向修正补丁（仅在两阶段规划未能解决问题时使用） ——
        if len(dubins_samples) >= 3:
            yaw0 = to_rad(start_tf.rotation.yaw)
            th1 = dubins_samples[1][2]
            mis0 = abs((th1 - yaw0 + math.pi) % (2*math.pi) - math.pi)
            # 仅在角度差异仍然很大时才使用兜底方案（说明两阶段规划未充分解决）
            if mis0 > 2.5:  # > ~143°（提高阈值，因为前面已有两阶段规划）
                print(f'[FALLBACK] Large angle mismatch ({to_deg(mis0):.1f}°) after two-stage planning, attempting fallback patch...', flush=True)
                patched = False
                for j in range(8, min(60, len(dubins_samples)-1), 2):
                    q1 = dubins_samples[j]
                    cfgs, _ = smoother._dubins_shortest(
                        (start_tf.location.x, start_tf.location.y, yaw0),
                        (q1[0], q1[1], q1[2])
                    )
                    if len(cfgs) >= 2 and smoother._sample_and_check(cfgs, allowed_lanes, corridor_xy):
                        dubins_samples = cfgs + dubins_samples[j+1:]
                        seg_ends = [len(cfgs)-1] + [e - (j+1) + len(cfgs) for e in seg_ends if e > j]
                        print('[FALLBACK] Start alignment patch inserted (local U-turn).', flush=True)
                        patched = True
                        break
                if not patched:
                    print('[WARN] Fallback patch also failed; relying on controller to converge.', flush=True)

        # UE 可视化最终路径
        dubins_xyz = []
        for (x, y, th) in dubins_samples:
            wp = amap.get_waypoint(carla.Location(x=float(x), y=float(y), z=0.0), project_to_road=True)
            z = wp.transform.location.z if wp is not None else 0.1
            dubins_xyz.append((x, y, z))
        viz.draw_points(dubins_xyz, color=carla.Color(0, 255, 0), size=0.06, life_time=args.life_time)
        viz.draw_polyline(dubins_xyz, color=carla.Color(183, 28, 28), thickness=0.20, life_time=args.life_time)
        world.tick()
        follow()

        # 保存 CSV
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['idx', 'seg_idx', 'x', 'y', 'z', 'yaw_deg'])
            seg_idx_by_sample = {}
            for s_i, end_idx in enumerate(seg_ends):
                seg_idx_by_sample[end_idx] = s_i
            cur_seg = 0
            for i, (x, y, th) in enumerate(dubins_samples):
                if i in seg_idx_by_sample:
                    cur_seg = seg_idx_by_sample[i]
                wp = amap.get_waypoint(carla.Location(x=float(x), y=float(y), z=0.0), project_to_road=True)
                z = wp.transform.location.z if wp is not None else 0.0
                yaw_deg = (to_deg(th) + 540.0) % 360.0 - 180.0
                writer.writerow([i, cur_seg, f'{x:.3f}', f'{y:.3f}', f'{z:.3f}', f'{yaw_deg:.2f}'])

        # 顶拍截图 + 论文图
        take_topdown_screenshot(world, dubins_xyz, top_png)
        paper_info = f"A*: step={args.astar_step}m | Dubins: R={args.turning_radius}m, step={args.dubins_step}m | Segments={len(seg_ends)} | Points={len(dubins_samples)}"
        render_paper_figure_from_csv(world, csv_path, paper_png, grid_res=1.0, margin=25.0, info_text=paper_info)

        # 跟踪行驶
        print('[INFO] Driving along planned path ...', flush=True)
        target_pts = [(x, y) for (x, y, _) in dubins_xyz]
        tgt_idx = 0
        t0 = time.time()
        while (time.time() - t0) < args.max_drive_seconds and tgt_idx < len(target_pts):
            world.tick()
            follow()
            # 选择前视点（只选“在车前方”的候选，避免追身后点导致卡死）
            vtf = ego.get_transform()
            loc = vtf.location
            yaw = to_rad(vtf.rotation.yaw)
            def _ahead(pt):
                dx = pt[0] - loc.x; dy = pt[1] - loc.y
                # 车辆坐标系前向投影 > 0 视为在前方
                fx = math.cos(yaw)*dx + math.sin(yaw)*dy
                return fx > 0.2
            best_j = None
            best_d = float('inf')
            for j in range(tgt_idx, min(tgt_idx + 25, len(target_pts))):
                if not _ahead(target_pts[j]):
                    continue
                d = math.hypot(target_pts[j][0]-loc.x, target_pts[j][1]-loc.y)
                if d < best_d:
                    best_d = d
                    best_j = j
            if best_j is None:
                # 如果前方窗口没有候选，向前跳一个固定步长，迫使车朝前走
                best_j = min(tgt_idx + 5, len(target_pts)-1)
                best_d = math.hypot(target_pts[best_j][0]-loc.x, target_pts[best_j][1]-loc.y)
            tgt_idx = max(best_j, tgt_idx)
            ctrl = pure_pursuit_control(ego, target_pts[tgt_idx])
            # 末段更慢
            if tgt_idx > len(target_pts) - 10:
                ctrl.throttle = min(ctrl.throttle, 0.12)
            ego.apply_control(ctrl)
            if best_d < 1.5 and tgt_idx < len(target_pts) - 1:
                tgt_idx += 1
        print('[OK] Drive finished.', flush=True)

    finally:
        # 恢复设置与清理
        try:
            world.apply_settings(original_settings)
        except Exception:
            pass
        _destroy_actors_safe([ego])

if __name__ == '__main__':
    main()
