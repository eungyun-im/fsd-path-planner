#!/usr/bin/env python3
import rospy
import numpy as np
import heapq
import sys
import os
import math
import copy

# [수정됨] 1번 적용: 보로노이(Voronoi) 임포트 추가
from scipy.spatial import Delaunay, KDTree, Voronoi
from scipy.interpolate import splprep, splev

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, Int32

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

try:
    import ma_rrt 
except ImportError:
    rospy.logerr("ma_rrt module not found!")
    pass 

# ==========================================
# [KF] 목표점 보정용 칼만 필터
# ==========================================
class TargetKalmanFilter:
    def __init__(self, dt=0.1):
        self.dt = dt
        self.x = np.zeros(4)  # [x, y, vx, vy]
        self.P = np.eye(4)
        self.F = np.eye(4)
        self.F[0, 2] = self.dt
        self.F[1, 3] = self.dt
        self.H = np.zeros((2, 4))
        self.H[0, 0] = 1
        self.H[1, 1] = 1
        self.Q = np.eye(4) * 0.01 
        self.R = np.eye(2) * 5.0  # 실차 진동을 고려하여 필터링 강화
        self.first_run = True

    def predict(self):
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x[0], self.x[1]

    def update(self, meas_x, meas_y):
        if self.first_run:
            self.x[0] = meas_x
            self.x[1] = meas_y
            self.first_run = False
            return meas_x, meas_y

        z = np.array([meas_x, meas_y])
        y = z - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

        self.x = self.x + np.dot(K, y)
        I = np.eye(4)
        self.P = np.dot(I - np.dot(K, self.H), self.P)
        return self.x[0], self.x[1]

    def set_dt(self, dt):
        if dt <= 0.001: dt = 0.001
        if dt > 0.5: dt = 0.5
        self.dt = dt
        self.F[0, 2] = dt
        self.F[1, 3] = dt


class FSDPlannerNode:
    def __init__(self):
        rospy.init_node('fsd_main_planner')

        self.load_parameters()

        self.viz_frame_id = "os_sensor" 
        self.prev_best_path = None 
        self.prev_best_idx = -1    
        self.prev_steering = 0.0
        self.prev_ld = rospy.get_param('~min_ld', 1.0)  # 초기값: min_ld와 동일하게 설정
        self.prev_speed = 0.0
        self.last_time = rospy.Time.now()
        self.lost_cone_counter = 0 # 콘 소실 카운터 추가

        self.current_target_speed = self.p_target_speed
        self.kf = TargetKalmanFilter(dt=0.1) 

        self.sub_cones = rospy.Subscriber('/center_points', Marker, self.callback_cones)
        
        # [수정] 통합 관제용 Publisher
        self.pub_cone_steer = rospy.Publisher('/cone/steer', Float32, queue_size=1)
        self.pub_cone_speed = rospy.Publisher('/cone/speed', Int32, queue_size=1)
        self.pub_cone_conf  = rospy.Publisher('/cone/confidence', Float32, queue_size=1)
        
        self.pub_v_limit = rospy.Publisher('/fsd_planner/v_limit', Float32, queue_size=1)
        self.pub_max_k = rospy.Publisher('/fsd_planner/max_k', Float32, queue_size=1)
        self.pub_path_viz = rospy.Publisher('/fsd_planner/best_path', Marker, queue_size=1)
        self.pub_candidates_viz = rospy.Publisher('/fsd_planner/candidate_paths', MarkerArray, queue_size=1)
        self.pub_delaunay_viz = rospy.Publisher('/fsd_planner/delaunay_viz', Marker, queue_size=1)
        self.pub_valid_cones_viz = rospy.Publisher('/fsd_planner/valid_cones', Marker, queue_size=1)
        self.pub_target_viz = rospy.Publisher('/fsd_planner/target_point', Marker, queue_size=1)

        print("✅ FSD Hybrid Planner Initialized (Controller Integration Mode + Voronoi + Angle Filter + Safe RRT)")

    def load_parameters(self):
        self.p_max_speed = rospy.get_param('~max_speed', 120.0)
        self.p_target_speed = rospy.get_param('~target_speed', 100.0)
        self.p_min_speed = rospy.get_param('~min_speed', 80.0)
        self.p_steer_sensitivity = rospy.get_param('~steer_sensitivity', 0.5)
        self.p_friction_mu = rospy.get_param('~friction_mu', 0.75)
        self.p_plan_dist = rospy.get_param('~plan_distance', 12.0)
        self.p_wheelbase = rospy.get_param('~wheelbase', 0.6) # 차량 축거 추가
        self.p_speed_unit_gain = rospy.get_param('~speed_unit_gain', 3.6)
        self.p_min_gate_width = rospy.get_param('~min_gate_width', 2.8)
        self.p_max_gate_width = rospy.get_param('~max_gate_width', 5.5)
        self.p_max_step_dist = rospy.get_param('~max_step_dist', 5.0)
        self.p_min_step_dist = rospy.get_param('~min_step_dist', 0.2)
        self.p_angle_limit_deg = rospy.get_param('~angle_limit_deg', 80.0)
        self.p_lateral_penalty = rospy.get_param('~lateral_penalty', 2.0)
        self.p_bspline_s = rospy.get_param('~bspline_s', 0.5)
        self.p_num_paths = rospy.get_param('~num_paths', 11)
        self.p_max_offset = rospy.get_param('~max_offset', 1.3)
        self.p_w_collision = rospy.get_param('~w_collision', 150.0)
        self.p_w_curvature = rospy.get_param('~w_curvature', 5.0)
        self.p_max_curvature_penalty_threshold = rospy.get_param('~max_curvature_penalty_threshold', 0.5) 
        self.p_w_smoothness = rospy.get_param('~w_smoothness', 30.0)
        self.p_w_consistency = rospy.get_param('~w_consistency', 30.0)
        self.p_w_obstacle_potential = rospy.get_param('~w_obstacle_potential', 5.0) 
        self.p_w_steer_sign = rospy.get_param('~w_steer_sign', 200.0)
        self.p_lookahead_gain = rospy.get_param('~lookahead_gain', 0.35)  # 현재 미사용 (speed_ratio 방식으로 대체)
        self.p_min_ld = rospy.get_param('~min_ld', 1.0)   # 속도가 min_speed일 때 ld 하한 (1.0m)
        self.p_max_ld = rospy.get_param('~max_ld', 4.0)
        self.p_speed_lookahead = rospy.get_param('~speed_lookahead', 6.0) 
        self.p_curv_penalty_gain = rospy.get_param('~curv_penalty_gain', 2.0)
        self.p_steer_gain = rospy.get_param('~steer_gain', 2.2)
        self.p_alpha_ld = rospy.get_param('~filter_alpha_ld', 0.2)
        self.p_alpha_steer = rospy.get_param('~filter_alpha_steer', 0.07) 
        self.p_max_circumradius = rospy.get_param('~max_circumradius', 6.0)

    def callback_cones(self, msg):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        if dt > 0:
            self.kf.set_dt(dt)
        self.last_time = current_time
        self.viz_frame_id = msg.header.frame_id

        points = np.array([[p.x, p.y] for p in msg.points]).reshape(-1, 2)

        confidence = 0.0

        if len(points) == 0:
            self.handle_planning_failure()
            return

        mask_roi = (points[:, 0] > -1.0) & (points[:, 0] < self.p_plan_dist + 2.0) & (np.abs(points[:, 1]) < 8.0)
        mask_noise = (points[:, 0] > 0.0) & (points[:, 0] < 1.0) & (np.abs(points[:, 1]) < 0.5)
        valid_indices = mask_roi & (~mask_noise)
        cones_xy = points[valid_indices]
        self.publish_valid_cones(cones_xy)

        # Step 1: Base Path
        midpoints = self.get_delaunay_midpoints(cones_xy)
        sorted_midpoints = []
        if len(midpoints) >= 2:
            sorted_midpoints = self.sort_points_greedy_advanced(midpoints)

        candidate_paths = []
        base_smooth_path = None

        # Step 2: Multi-Hypothesis
        if len(sorted_midpoints) >= 2:
            base_smooth_path = self.apply_bspline_smoothing(sorted_midpoints, s_factor=self.p_bspline_s)
            if len(base_smooth_path) >= 2:
                grad = np.gradient(base_smooth_path, axis=0)
                norms = np.linalg.norm(grad, axis=1) + 1e-6
                nx = -grad[:, 1] / norms
                ny = grad[:, 0] / norms
                offsets = np.linspace(-self.p_max_offset, self.p_max_offset, self.p_num_paths)
                for off in offsets:
                    if abs(off) < 0.1:
                        candidate_paths.append(base_smooth_path)
                    else:
                        off_x = base_smooth_path[:, 0] + nx * off
                        off_y = base_smooth_path[:, 1] + ny * off
                        candidate_paths.append(np.column_stack((off_x, off_y)))
        
        # Step 3: RRT
        rrt_triggered = False
        use_rrt = False
        if len(sorted_midpoints) < 2:
            use_rrt = True
        elif len(candidate_paths) == 0:
            use_rrt = True

        if use_rrt:
            # [수정됨] 3번 적용: RRT 장애물 반경(cost)을 0.5에서 1.5로 증가시켜 바짝 붙는 것 원천 차단
            obs_list = [(c[0], c[1], 1.5) for c in cones_xy]
            goal_pt = [self.p_plan_dist, 0]
            if len(sorted_midpoints) > 0:
                goal_pt = sorted_midpoints[-1]

            rrt_guidance = sorted_midpoints if len(sorted_midpoints) > 1 else self.prev_best_path

            rrt = ma_rrt.RRT(
                start=[0, 0, 0], goal=goal_pt, 
                planDistance=self.p_plan_dist,
                obstacleList=obs_list, 
                guidance_points=rrt_guidance, 
                maxIter=200) 

            rrt_path = rrt.Planning()
            if rrt_path is not None and len(rrt_path) > 3:
                smooth_rrt_path = self.apply_bspline_smoothing(rrt_path, s_factor=1.0)
                candidate_paths.append(smooth_rrt_path)
                rrt_triggered = True

        if len(cones_xy) < 3 or len(candidate_paths) == 0:
            self.handle_planning_failure()
            return
        
        self.lost_cone_counter = 0

        best_idx = 0
        min_cost = float('inf')
        obs_tree = KDTree(cones_xy)

        for i, path in enumerate(candidate_paths):
            if len(path) < 5:
                continue
            is_rrt_path = (rrt_triggered and i == len(candidate_paths) - 1)
            cost = self.evaluate_path_cost(path, obs_tree, is_rrt_path)
            if not is_rrt_path and self.prev_best_idx != -1:
                if i == self.prev_best_idx:
                    cost *= 0.5
                elif abs(i - self.prev_best_idx) <= 1:
                    cost *= 0.7
            if cost < min_cost:
                min_cost = cost
                best_idx = i

        final_path = candidate_paths[best_idx]

        if self.prev_best_path is not None:
            path_diff = np.mean(np.linalg.norm(final_path - self.prev_best_path, axis=1))
            alpha = 0.85
            if path_diff > 1.2: 
                alpha = 1.0     
            final_path = alpha * final_path + (1.0 - alpha) * self.prev_best_path

        self.prev_best_path = final_path 
        self.prev_best_idx = -1 if (rrt_triggered and best_idx == len(candidate_paths) - 1) else best_idx

        self.publish_path_viz(final_path)
        self.publish_multiverse_viz(candidate_paths, best_idx)

        confidence = 1.3
        if rrt_triggered:
            confidence = 0.8
        path_len = np.linalg.norm(final_path[-1] - final_path[0])
        if path_len < 3.0: confidence = 0.5

        self.pure_pursuit_drive(final_path, confidence)

    def handle_planning_failure(self):
        self.lost_cone_counter += 1
        if self.lost_cone_counter < 15: 
            rospy.logwarn_throttle(1.0, "Planning failed! Holding previous steering...")
            hold_speed = max(self.p_min_speed, self.prev_speed * 0.95)
            self.publish_control(self.prev_steering, int(hold_speed), 0.2)
        else:
            rospy.logerr_throttle(1.0, "Planning failed for too long! Emergency Stop.")
            stop_steer = self.prev_steering * 0.8
            self.publish_control(stop_steer, 0, 0.0)
            self.prev_steering = stop_steer

    def publish_control(self, steer, speed, conf):
        self.pub_cone_steer.publish(Float32(data=steer))
        self.pub_cone_speed.publish(Int32(data=speed))
        self.pub_cone_conf.publish(Float32(data=conf))

    def publish_path_viz(self, path):
        m = Marker()
        m.header.frame_id = self.viz_frame_id
        m.header.stamp = rospy.Time.now()
        m.ns, m.id = "best_path", 0
        m.type, m.action = Marker.LINE_STRIP, Marker.ADD
        m.lifetime = rospy.Duration(0.1)
        m.scale.x = 0.2
        m.color.a, m.color.r, m.color.g, m.color.b = 1.0, 1.0, 1.0, 0.0 
        for p in path:
            m.points.append(Point(p[0], p[1], 0))
        self.pub_path_viz.publish(m)

    def publish_valid_cones(self, cones):
        m = Marker()
        m.header.frame_id = self.viz_frame_id
        m.header.stamp = rospy.Time.now()
        m.ns, m.id = "valid_cones", 0
        m.type, m.action = Marker.POINTS, Marker.ADD
        m.scale.x, m.scale.y = 0.2, 0.2
        m.color.a, m.color.r, m.color.g, m.color.b = 1.0, 0.0, 1.0, 1.0 
        for p in cones:
            m.points.append(Point(p[0], p[1], 0))
        self.pub_valid_cones_viz.publish(m)

    def sort_points_greedy_advanced(self, points):
        if len(points) == 0:
            return np.array([])

        num_pts = len(points)
        visited = np.zeros(num_pts, dtype=bool)
        current_pos = np.array([0.0, 0.0])
        current_heading = np.array([1.0, 0.0])
        path = [current_pos]
        for _ in range(min(num_pts, 25)):
            unvisited_idx = np.where(~visited)[0]
            if len(unvisited_idx) == 0: break
            
            candidates = points[unvisited_idx]
            vecs = candidates - current_pos
            dists = np.linalg.norm(vecs, axis=1)
            
            mask = (dists <= self.p_max_step_dist) & (dists >= self.p_min_step_dist) & (candidates[:, 0] >= current_pos[0] - 0.5)
            if not np.any(mask): break
            
            valid_indices = unvisited_idx[mask]
            valid_vecs = vecs[mask]
            valid_dists = dists[mask]
            
            unit_vecs = valid_vecs / (valid_dists[:, np.newaxis] + 1e-6)
            cos_thetas = np.dot(unit_vecs, current_heading)
            angles = np.degrees(np.arccos(np.clip(cos_thetas, -1.0, 1.0)))
            
            angle_mask = angles <= self.p_angle_limit_deg
            if not np.any(angle_mask): break
            
            final_idx = valid_indices[angle_mask]
            final_dists = valid_dists[angle_mask]
            final_angles = angles[angle_mask]
            final_pts = points[final_idx]

            scores = final_dists + (final_angles / 2.0) + (np.abs(final_pts[:, 1]) * self.p_lateral_penalty)
            best_local_idx = np.argmin(scores)
            best_global_idx = final_idx[best_local_idx]

            next_pt = points[best_global_idx]
            path.append(next_pt)
            visited[best_global_idx] = True
            current_heading = (next_pt - current_pos) / (np.linalg.norm(next_pt - current_pos) + 1e-6)
            current_pos = next_pt

        return np.array(path)

    def evaluate_path_cost(self, path, obs_tree, is_rrt_path=False):
        total_cost = 0.0
        dists, _ = obs_tree.query(path)
        min_dist = np.min(dists)

        if min_dist < 0.4:
            return 1e6
        total_cost += self.p_w_collision / (min_dist**2 + 1e-3)

        _, lookahead_k, _ = self.calculate_path_profile(path)
        total_cost += self.p_w_curvature * lookahead_k

        diffs = np.diff(path, axis=0)
        headings = np.arctan2(diffs[:, 1], diffs[:, 0])
        heading_changes = np.abs(np.diff(headings))
        heading_changes = np.where(heading_changes > np.pi, 2 * np.pi - heading_changes, heading_changes)
        total_cost += self.p_w_smoothness * np.sum(heading_changes)

        if self.prev_best_path is not None:
            check_indices = np.linspace(0, len(path)-1, 5, dtype=int)
            deviation = 0
            for idx in check_indices:
                p = path[idx]
                dists_sq = np.sum((self.prev_best_path - p)**2, axis=1)
                deviation += np.sqrt(np.min(dists_sq))
            total_cost += self.p_w_consistency * (deviation / len(check_indices))

        dists, _ = obs_tree.query(path)
        total_cost += np.var(dists) * 20.0 

        path_start_yaw = math.atan2(path[1,1] - path[0,1], path[1,0] - path[0,0])
        prev_steer_rad = math.radians(self.prev_steering / self.p_steer_gain)
        yaw_diff = abs(self.pi_2_pi(path_start_yaw - prev_steer_rad))
        total_cost += yaw_diff * 50.0 

        if is_rrt_path:
            total_cost += 100.0
        return total_cost

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def get_delaunay_midpoints(self, cones_xy):
        if len(cones_xy) < 3:
            return np.array([])
        try:
            tri = Delaunay(cones_xy, qhull_options="QJ")
        except:
            return np.array([])

        simplices = tri.simplices
        pts = cones_xy[simplices] 

        a = np.linalg.norm(pts[:, 0] - pts[:, 1], axis=1)
        b = np.linalg.norm(pts[:, 1] - pts[:, 2], axis=1)
        c = np.linalg.norm(pts[:, 2] - pts[:, 0], axis=1)
        area = 0.5 * np.abs(pts[:,0,0]*(pts[:,1,1]-pts[:,2,1]) + 
                            pts[:,1,0]*(pts[:,2,1]-pts[:,0,1]) + 
                            pts[:,2,0]*(pts[:,0,1]-pts[:,1,1]))
        circumradius = (a * b * c) / (4.0 * area + 1e-6)

        edges = np.vstack([simplices[:, [0, 1]], simplices[:, [1, 2]], simplices[:, [2, 0]]])
        edges = np.sort(edges, axis=1)
        unique_edges, inverse_indices = np.unique(edges, axis=0, return_inverse=True)

        edge_min_radius = np.full(len(unique_edges), np.inf)
        np.minimum.at(edge_min_radius, inverse_indices, np.repeat(circumradius, 3))

        p1, p2 = cones_xy[unique_edges[:, 0]], cones_xy[unique_edges[:, 1]]
        dists = np.linalg.norm(p1 - p2, axis=1)
        midpoints_temp = (p1 + p2) / 2.0

        if self.prev_best_path is not None and len(self.prev_best_path) > 5:
            path_tree = KDTree(self.prev_best_path)
            _, closest_indices = path_tree.query(midpoints_temp)
            
            path_grads = np.gradient(self.prev_best_path, axis=0)
            path_tangents = path_grads / (np.linalg.norm(path_grads, axis=1)[:, np.newaxis] + 1e-6)
            flow_vecs = path_tangents[closest_indices]
        else:
            flow_vecs = np.tile(np.array([1.0, 0.0]), (len(midpoints_temp), 1))

        edge_vecs = p2 - p1
        edge_vecs_unit = edge_vecs / (dists[:, np.newaxis] + 1e-6)
        dot_products = np.abs(np.sum(edge_vecs_unit * flow_vecs, axis=1))
        
        # [수정됨] 4번 적용: 선분의 dx(진행방향) 길이 비율 계산
        dx_ratio = np.abs(edge_vecs[:, 0]) / (dists + 1e-6)

        dist_to_car = np.linalg.norm(midpoints_temp, axis=1)
        dynamic_threshold = np.where(dist_to_car < 3.0, 0.85, 0.8)
        
        # [수정됨] 4번 적용: dx_ratio < 0.8 조건 추가 (진행방향과 평행한 선분 걸러내기)
        mask = (dists > self.p_min_gate_width) & (dists < self.p_max_gate_width) & \
               (dot_products < dynamic_threshold) & (edge_min_radius < self.p_max_circumradius) & \
               (dx_ratio < 0.8)

        valid_p1, valid_p2 = p1[mask], p2[mask]
        midpoints = (valid_p1 + valid_p2) / 2.0

        front_mask = midpoints[:, 0] > -0.5
        midpoints = midpoints[front_mask]

        # ==========================================
        # [수정됨] 1번 적용: 보로노이(Voronoi) 다이어그램 안전 공간 추출
        # ==========================================
        voronoi_points = []
        try:
            vor = Voronoi(cones_xy)
            for v in vor.vertices:
                if -0.5 < v[0] < self.p_plan_dist:
                    # 꼭짓점에서 가장 가까운 콘까지의 최소 거리 계산
                    min_dist = np.min(np.linalg.norm(cones_xy - v, axis=1))
                    # 안전 반경 1.0m 이상 확보, 트랙 허용치 내부인 점만 추가
                    if 1.0 < min_dist < (self.p_max_gate_width / 2.0 + 1.0):
                        voronoi_points.append(v)
        except Exception:
            pass

        # 기존 Delaunay 중점과 보로노이 포인트 합치기
        if len(voronoi_points) > 0:
            midpoints = np.vstack((midpoints, np.array(voronoi_points)))

        if self.pub_delaunay_viz.get_num_connections() > 0:
            self.publish_delaunay_viz(list(zip(valid_p1[front_mask], valid_p2[front_mask])), midpoints)

        return midpoints

    def apply_bspline_smoothing(self, waypoints, s_factor=0.5):
        if len(waypoints) < 3:
            return self._resample_path(np.array(waypoints), 30)
            
        dist_diffs = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
        mask = np.concatenate(([True], dist_diffs > 0.05))
        pts = waypoints[mask]
        
        if len(pts) < 3:
            return self._resample_path(pts, 30)
            
        try:
            current_s = s_factor if len(pts) > 5 else 0.1
            tck, u = splprep([pts[:, 0], pts[:, 1]], s=current_s, k=min(3, len(pts) - 1))
            u_new = np.linspace(0, 1, 30)
            return np.column_stack(splev(u_new, tck))
        except Exception:
            return self._resample_path(pts, 30)

    def _resample_path(self, path, num_points):
        if len(path) < 2:
            return np.tile(path[0], (num_points, 1)) if len(path) == 1 else np.zeros((num_points, 2))
        
        old_indices = np.linspace(0, 1, len(path))
        new_indices = np.linspace(0, 1, num_points)
        new_x = np.interp(new_indices, old_indices, path[:, 0])
        new_y = np.interp(new_indices, old_indices, path[:, 1])
        return np.column_stack((new_x, new_y))

    def calculate_path_profile(self, path):
        if len(path) < 5:
            return 0.0, 0.0, self.p_target_speed

        path_len = np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))
        num_samples = int(path_len / 0.5)

        window = 5 
        if num_samples < window + 1:
            return 0.0, 0.0, self.p_target_speed
        
        resampled = self._resample_path(path, num_samples)
        
        smooth_x = np.convolve(resampled[:, 0], np.ones(window)/window, mode='valid')
        smooth_y = np.convolve(resampled[:, 1], np.ones(window)/window, mode='valid')

        if len(smooth_x) < 2:
            return 0.0, 0.0, self.p_target_speed

        dx = np.gradient(smooth_x)
        dy = np.gradient(smooth_y)
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        numerator = np.abs(dx * ddy - dy * ddx)
        denominator = (dx**2 + dy**2)**1.5 + 1e-7
        curvature = numerator / denominator 
        
        current_k = np.mean(curvature[:min(len(curvature), 3)])
        
        lookahead_pts = max(1, int(self.p_speed_lookahead / 0.5))
        lookahead_k = np.percentile(curvature[:min(len(curvature), lookahead_pts)], 90) 

        lookahead_k_clipped = np.clip(lookahead_k, 1e-4, 2.0)
        radius = 1.0 / lookahead_k_clipped
        
        lat_accel_limit = self.p_friction_mu * 9.81 * 4.0 
        v_phys_limit = np.sqrt(lat_accel_limit * radius) * self.p_speed_unit_gain

        v_limit = min(self.p_target_speed, v_phys_limit)
        
        return current_k, lookahead_k, v_limit

    def pure_pursuit_drive(self, path, confidence):
        current_k, lookahead_k, v_limit = self.calculate_path_profile(path)
        self.pub_v_limit.publish(v_limit)
        self.pub_max_k.publish(lookahead_k)

        target_vel = min(self.p_target_speed, v_limit)
        
        # --- 속도 비례 동적 Lookahead Distance ---
        # target_vel(km/h)을 [min_speed, max_speed] 범위로 정규화한 speed_ratio 계산
        # speed_ratio=0 (min_speed) → ld=min_ld(1.0), speed_ratio=1 (max_speed) → ld=max_ld
        speed_ratio = np.clip(
            (target_vel - self.p_min_speed) / (self.p_max_speed - self.p_min_speed + 1e-6),
            0.0, 1.0
        )
        speed_based_ld = self.p_min_ld + (self.p_max_ld - self.p_min_ld) * speed_ratio

        # 조향각이 클수록 ld 소폭 증가 (안정성 확보)
        ld_steer_factor = 1.0 + (abs(self.prev_steering) / 60.0)
        base_ld = np.clip(speed_based_ld * ld_steer_factor, self.p_min_ld, self.p_max_ld)

        curvature_penalty = np.clip(1.0 - (lookahead_k * self.p_curv_penalty_gain), 0.5, 1.0)
        raw_ld = base_ld * curvature_penalty
        dynamic_ld = (self.prev_ld * (1 - self.p_alpha_ld)) + (raw_ld * self.p_alpha_ld)
        self.prev_ld = np.clip(dynamic_ld, self.p_min_ld, self.p_max_ld)

        rospy.logdebug(f"[ld] speed={target_vel:.1f}km/h  ratio={speed_ratio:.2f}  "
                       f"speed_ld={speed_based_ld:.2f}  raw_ld={raw_ld:.2f}  ld={self.prev_ld:.2f}")

        raw_target = None
        dists = np.hypot(path[:, 0], path[:, 1])
        idx = np.where(dists > dynamic_ld)[0]
        if len(idx) > 0:
            raw_target = path[idx[0]]
        else:
            raw_target = path[-1]

        self.kf.predict()
        fx, fy = self.kf.update(raw_target[0], raw_target[1])
        self.publish_target_viz(fx, fy)

        pure_pursuit_term = math.atan2(2.0 * self.p_wheelbase * fy, dynamic_ld**2)
        ff_term = math.atan(self.p_wheelbase * current_k) * np.sign(fy)
        
        steering_rad = pure_pursuit_term + (ff_term * 0.5) 
        steer_deg = math.degrees(steering_rad)

        steer_diff = steer_deg - self.prev_steering
        if abs(steer_diff) > 10.0:
            max_delta = 8.0  
        else:
            max_delta = 4.0  
            
        steer_diff = steer_deg - self.prev_steering
        steer_deg = self.prev_steering + np.clip(steer_diff, -max_delta, max_delta)

        speed_penalty = abs(steer_deg) * (self.p_steer_sensitivity * 0.1)
        
        if abs(steer_deg) > 25.0:
            speed_penalty *= 1.0
            
        final_vel = max(self.p_min_speed, target_vel - speed_penalty)

        max_speed_delta = 3.0 
        speed_diff = final_vel - self.prev_speed
        final_vel = self.prev_speed + np.clip(speed_diff, -max_speed_delta, max_speed_delta)
        self.prev_speed = final_vel
        self.current_target_speed = final_vel

        pub_steer = np.clip(steer_deg, -30, 30)
        
        pub_steer = self.prev_steering * (1 - self.p_alpha_steer) + pub_steer * self.p_alpha_steer
        self.prev_steering = pub_steer

        self.publish_control(pub_steer, int(final_vel), confidence)

    def publish_multiverse_viz(self, candidates, best_idx):
        ma = MarkerArray()
        for i, path in enumerate(candidates):
            m = Marker()
            m.header.frame_id = self.viz_frame_id
            m.ns, m.id = "candidate_paths", i
            m.type, m.action = Marker.LINE_STRIP, Marker.ADD
            m.lifetime = rospy.Duration(0.1)
            for p in path:
                m.points.append(Point(p[0], p[1], 0))
            if i == best_idx: 
                m.scale.x = 0.15 
                m.color.a, m.color.r, m.color.g, m.color.b = 1.0, 0.0, 1.0, 0.0
            else: 
                m.scale.x = 0.05
                m.color.a, m.color.r, m.color.g, m.color.b = 0.5, 0.7, 0.7, 0.7
            ma.markers.append(m)
        self.pub_candidates_viz.publish(ma)

    def publish_delaunay_viz(self, edges, midpoints):
        m_edge = Marker()
        m_edge.header.frame_id = self.viz_frame_id
        m_edge.ns, m_edge.id = "delaunay_edges", 0
        m_edge.type, m_edge.action = Marker.LINE_LIST, Marker.ADD
        m_edge.lifetime = rospy.Duration(0.1)
        m_edge.scale.x = 0.03
        m_edge.color.a, m_edge.color.r, m_edge.color.g = 0.6, 1.0, 1.0
        for p1, p2 in edges:
            m_edge.points.append(Point(p1[0], p1[1], 0))
            m_edge.points.append(Point(p2[0], p2[1], 0))
        m_pts = Marker()
        m_pts.header.frame_id = self.viz_frame_id
        m_pts.ns, m_pts.id = "delaunay_mids", 1
        m_pts.type, m_pts.action = Marker.POINTS, Marker.ADD
        m_pts.lifetime = rospy.Duration(0.1)
        m_pts.scale.x, m_pts.scale.y = 0.2, 0.2
        m_pts.color.a, m_pts.color.r = 1.0, 1.0
        for mp in midpoints:
            m_pts.points.append(Point(mp[0], mp[1], 0))
        self.pub_delaunay_viz.publish(m_edge)
        self.pub_delaunay_viz.publish(m_pts)

    def publish_target_viz(self, x, y):
        m = Marker()
        m.header.frame_id = self.viz_frame_id
        m.ns, m.id = "kf_target", 200
        m.type, m.action = Marker.SPHERE, Marker.ADD
        m.lifetime = rospy.Duration(0.1)
        m.pose.position.x, m.pose.position.y = x, y
        m.scale.x, m.scale.y, m.scale.z = 0.4, 0.4, 0.4
        m.color.a, m.color.r, m.color.g, m.color.b = 1.0, 1.0, 1.0, 0.0
        self.pub_target_viz.publish(m)

if __name__ == '__main__':
    try:
        FSDPlannerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass