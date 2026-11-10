"""
Midline-guided RRT* Planner
Modified for FSD Hybrid Planning Integration
"""

import random
import math
import copy
import numpy as np

class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, planDistance, obstacleList, guidance_points=None, expandDis=0.5, turnAngle=30, maxIter=200):
        # start: [x, y, yaw]
        # goal: [x, y] (approximate target)
        self.start = Node(start[0], start[1], start[2])
        self.end = Node(goal[0], goal[1], 0.0)  # Goal Orientation is not strict

        self.planDistance = planDistance
        self.expandDis = expandDis
        self.turnAngle = math.radians(turnAngle)
        self.maxIter = maxIter
        self.obstacleList = obstacleList
        self.search_radius = expandDis * 2.5 # RRT* Rewire radius

        self.goal_bias_probability = 0.15 # [추가] Goal Bias 확률
        # [핵심] 미드라인 가이던스 포인트 (Delaunay Midpoints)
        self.guidance_points = guidance_points 
        self.rrt_width = 0.8 # 폭을 약간 좁혀서 더 일관된 샘플링 유도

    def Planning(self):
        self.nodeList = [self.start]
        best_path = None
        min_dist_to_goal = float('inf')

        for i in range(self.maxIter):
            # 1. Guided Sampling
            if random.random() < self.goal_bias_probability: # [개선] Goal Bias 적용
                rnd = [self.end.x, self.end.y]
            elif self.guidance_points is not None and len(self.guidance_points) > 1:
                rnd = self.get_random_point_guided()
            else:
                rnd = self.get_random_point_standard()

            # 2. Find Nearest Node
            nind = self.GetNearestListIndex(self.nodeList, rnd)
            nearestNode = self.nodeList[nind]

            # 3. Steer (Kinematic Constraints)
            newNode = self.steerConstrained(rnd, nind)

            # 4. [RRT* Upgrade] Choose Parent & Rewire
            if self.__CollisionCheck(newNode, self.obstacleList):
                near_inds = self.find_near_nodes(newNode)
                newNode = self.choose_parent(newNode, near_inds)
                self.nodeList.append(newNode)
                self.rewire(newNode, near_inds)
                # RRT*는 maxIter까지 계속 돌려 경로를 최적화함

        # 만약 Goal에 도달 못했으면, Goal에 가장 가까운 노드까지의 경로라도 반환
        last_index = self.get_best_last_index()
        if last_index is not None:
            return self.gen_final_course(last_index)

        return None

    def find_near_nodes(self, newNode):
        nnode = len(self.nodeList)
        r = self.search_radius * math.sqrt(math.log(nnode) / nnode) if nnode > 1 else self.search_radius
        dist_list = [(node.x - newNode.x)**2 + (node.y - newNode.y)**2 for node in self.nodeList]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]
        return near_inds

    def choose_parent(self, newNode, near_inds):
        if not near_inds:
            return newNode

        costs = []
        for i in near_inds:
            near_node = self.nodeList[i]
            d = math.hypot(newNode.x - near_node.x, newNode.y - near_node.y)
            # 조향각 제약 확인
            theta = math.atan2(newNode.y - near_node.y, newNode.x - near_node.x)
            if abs(self.pi_2_pi(theta - near_node.yaw)) <= self.turnAngle:
                costs.append(near_node.cost + d)
            else:
                costs.append(float('inf'))

        min_cost = min(costs)
        if min_cost == float('inf'):
            return newNode

        min_ind = near_inds[costs.index(min_cost)]
        newNode.cost = min_cost
        newNode.parent = min_ind
        return newNode

    def rewire(self, newNode, near_inds):
        for i in near_inds:
            near_node = self.nodeList[i]
            d = math.hypot(near_node.x - newNode.x, near_node.y - newNode.y)
            scost = newNode.cost + d

            if near_node.cost > scost:
                # 조향각 제약 확인
                theta = math.atan2(near_node.y - newNode.y, near_node.x - newNode.x)
                if abs(self.pi_2_pi(theta - newNode.yaw)) <= self.turnAngle:
                    if self.__CollisionCheck(near_node, self.obstacleList):
                        near_node.parent = len(self.nodeList) - 1
                        near_node.cost = scost

    def get_random_point_guided(self):
        """
        [핵심 로직] 미드라인 근처에서 샘플링 (Midline-guided)
        """
        # 가이던스 포인트 중 랜덤한 세그먼트 선택
        idx = random.randint(0, len(self.guidance_points) - 2)
        p1 = self.guidance_points[idx]
        p2 = self.guidance_points[idx + 1]

        # 세그먼트 사이의 랜덤 보간 (t: 0~1)
        t = random.random()
        mid_x = p1[0] + (p2[0] - p1[0]) * t
        mid_y = p1[1] + (p2[1] - p1[1]) * t

        # 세그먼트 방향 벡터 계산
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        yaw = math.atan2(dy, dx)

        # 법선 방향(Normal Vector)으로 랜덤 오프셋 적용
        # 트랙폭 내에서만 샘플링하여 불필요한 탐색 방지
        offset = random.uniform(-self.rrt_width, self.rrt_width)
        norm_yaw = yaw + (math.pi / 2.0)

        rnd_x = mid_x + math.cos(norm_yaw) * offset
        rnd_y = mid_y + math.sin(norm_yaw) * offset

        return [rnd_x, rnd_y]

    def get_random_point_standard(self):
        # Fallback: 일반적인 전방 부채꼴 랜덤 샘플링
        rand_r = random.uniform(2.0, self.planDistance)
        rand_theta = random.uniform(-math.pi/2, math.pi/2)

        # 차량 기준 좌표계 (Local Frame)
        rnd_x = self.start.x + rand_r * math.cos(rand_theta)
        rnd_y = self.start.y + rand_r * math.sin(rand_theta)
        return [rnd_x, rnd_y]

    def steerConstrained(self, rnd, nind):
        nearestNode = self.nodeList[nind]
        theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

        # Dynamic constraints (최대 조향각 제한)
        angleChange = self.pi_2_pi(theta - nearestNode.yaw)

        if angleChange > self.turnAngle:
            angleChange = self.turnAngle
        elif angleChange < -self.turnAngle:
            angleChange = -self.turnAngle

        finalYaw = nearestNode.yaw + angleChange

        newNode = Node(nearestNode.x, nearestNode.y, finalYaw)
        newNode.x += self.expandDis * math.cos(finalYaw)
        newNode.y += self.expandDis * math.sin(finalYaw)
        newNode.cost = nearestNode.cost + self.expandDis
        newNode.parent = nind

        return newNode

    def gen_final_course(self, goalind):
        path = [[self.nodeList[goalind].x, self.nodeList[goalind].y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])

        # Path를 main_node 포맷에 맞게 역순 정렬 (Start -> End)
        path = np.array(path[::-1]) 
        return self.path_pruning(path)

    def path_pruning(self, path):
        """
        [업그레이드] 경로 최적화: 직선 연결이 가능한 구간은 노드를 생략하여 부드럽게 만듦
        """
        if len(path) <= 2:
            return path
            
        pruned_path = [path[0]]
        curr_idx = 0
        while curr_idx < len(path) - 1:
            best_next = curr_idx + 1
            for next_idx in range(len(path) - 1, curr_idx + 1, -1):
                # 장애물과의 충돌 없이 직선 연결이 가능한지 확인
                if self.__check_segment_collision(path[curr_idx], path[next_idx]):
                    best_next = next_idx
                    break
            pruned_path.append(path[best_next])
            curr_idx = best_next
            
        return np.array(pruned_path)

    def get_best_last_index(self):
        # Goal에 도달 못했을 경우, Goal에 가장 가까운 노드 찾기
        dists = []
        for node in self.nodeList:
            d = math.hypot(node.x - self.end.x, node.y - self.end.y)
            dists.append(d)

        if not dists: return None
        return dists.index(min(dists))

    def __CollisionCheck(self, node, obstacleList):
        if not obstacleList: return True
        obs_arr = np.array(obstacleList)
        dists_sq = (obs_arr[:, 0] - node.x)**2 + (obs_arr[:, 1] - node.y)**2

        # 벡터화된 충돌 검사: 하나라도 충돌하면 False
        if np.any(dists_sq <= (obs_arr[:, 2] + 0.3)**2): 
            return False
        return True

    def __check_segment_collision(self, p1, p2):
        """두 점 사이의 선분이 장애물과 충돌하는지 검사"""
        obs_arr = np.array(self.obstacleList)
        if len(obs_arr) == 0: return True
        
        # 선분을 5등분하여 중간 지점들 검사
        for t in np.linspace(0, 1, 5):
            x = p1[0] + (p2[0] - p1[0]) * t
            y = p1[1] + (p2[1] - p1[1]) * t
            dists_sq = (obs_arr[:, 0] - x)**2 + (obs_arr[:, 1] - y)**2
            if np.any(dists_sq <= (obs_arr[:, 2] + 0.4)**2):
                return False
        return True

    def GetNearestListIndex(self, nodeList, rnd):
        nodes = np.array([[n.x, n.y] for n in nodeList])
        dists_sq = np.sum((nodes - rnd)**2, axis=1)
        return np.argmin(dists_sq)

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi


class Node():
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.cost = 0.0
        self.parent = None