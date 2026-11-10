#!/usr/bin/env python3
import rospy
import numpy as np
import sys
import os
import matplotlib.pyplot as plt  # 시각화용

# 1. 경로 설정
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

# 라이브러리 불러오기
from fsd_path_planning import PathPlanner, ConeTypes, MissionTypes

def main():
    # 노드 초기화
    rospy.init_node('fsd_path_planner_test')

    # 플래너 생성
    planner = PathPlanner(MissionTypes.trackdrive) 
    print("✅ 플래너 로딩 성공!")

    # --- 가짜 데이터 만들기 ---

    # 1) 차량 위치 [x, y] 
    # 주의: 이제 theta(각도)는 넣지 않습니다! (2개 값만)
    car_pose = np.array([0.0, 0.0]) 

    # 2) 차량 방향 벡터 [dir_x, dir_y]
    # (1, 0)은 x축 방향(오른쪽)을 바라본다는 뜻입니다.
    car_direction = np.array([1.0, 0.0])

    # 3) 콘 위치 설정
    cones_left = np.array([
        [2.0, 1.5], [5.0, 1.8], [8.0, 1.6], [12.0, 2.0]
    ])
    cones_right = np.array([
        [2.0, -1.5], [5.0, -1.8], [8.0, -1.6], [12.0, -2.0]
    ])

    cones_unknown = np.empty((0, 2))
    cones_orange_big = np.empty((0, 2))
    cones_orange_small = np.empty((0, 2))

    # 리스트로 묶기
    cones_by_type = [
        cones_left, 
        cones_right, 
        cones_unknown, 
        cones_orange_big, 
        cones_orange_small
    ]

    print("🔄 경로 계산 중...")

    # --- 경로 계산 실행 ---
    path = planner.calculate_path_in_global_frame(
        cones_by_type, 
        car_pose,       # [x, y]
        car_direction,  # [dir_x, dir_y]
        return_intermediate_results=False
    )
    
    # --- 결과 확인 ---
    if path.shape[0] > 0:
        print(f"🎉 성공! 경로 점 {len(path)}개를 생성했습니다.")
        print(f"첫 5개 좌표:\n{path[:5]}")
        
        # 그래프 그리기
        try:
            plt.figure(figsize=(10,5))
            # 콘 그리기
            plt.plot(cones_left[:,0], cones_left[:,1], 'bo', label='Blue Cone')
            plt.plot(cones_right[:,0], cones_right[:,1], 'yo', label='Yellow Cone')
            # 경로 그리기 (path의 1, 2번 인덱스가 x, y 좌표입니다)
            plt.plot(path[:,1], path[:,2], 'r.-', label='Calculated Path')
            
            # 차량 위치 표시
            plt.arrow(car_pose[0], car_pose[1], car_direction[0], car_direction[1], 
                      head_width=0.5, head_length=0.5, fc='k', ec='k', label='Car')
            
            plt.axis('equal')
            plt.grid(True)
            plt.legend()
            plt.title("FaSTTUBe Path Planning Test")
            plt.show()
        except Exception as e:
            print(f"시각화 에러 (무시 가능): {e}")
    else:
        print("⚠️ 경로 생성 실패 (콘이 너무 적거나 설정 문제)")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass