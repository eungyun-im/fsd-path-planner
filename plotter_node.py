#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec
from std_msgs.msg import Float32, Int32
from visualization_msgs.msg import Marker, MarkerArray
from collections import deque
import numpy as np
import platform

class FSDControlTerminal:
    def __init__(self, window_size=200):
        rospy.init_node('fsd_control_terminal', anonymous=True)

        # 데이터 저장용 큐 (그래프용)
        self.window_size = window_size
        self.time_axis = np.linspace(-window_size, 0, window_size)
        
        self.steer_data = deque([0.0] * window_size, maxlen=window_size)
        self.speed_data = deque([0.0] * window_size, maxlen=window_size)
        self.conf_data = deque([0.0] * window_size, maxlen=window_size)
        self.cte_data = deque([0.0] * window_size, maxlen=window_size)
        
        # 상태 변수
        self.path_data = np.empty((0, 2))

        # Subscriber 설정
        self.sub_steer = rospy.Subscriber('/cone/steer', Float32, self.steer_callback)
        self.sub_speed = rospy.Subscriber('/cone/speed', Int32, self.speed_callback)
        self.sub_conf = rospy.Subscriber('/cone/confidence', Float32, self.conf_callback)
        self.sub_path = rospy.Subscriber('/fsd_planner/best_path', Marker, self.path_callback)

        # ==========================================
        # 한글 폰트 및 화이트 테마 설정
        # ==========================================
        os_name = platform.system()
        if os_name == 'Windows':
            plt.rcParams['font.family'] = 'Malgun Gothic'
        elif os_name == 'Darwin':
            plt.rcParams['font.family'] = 'AppleGothic'
        else:
            plt.rcParams['font.family'] = 'NanumGothic'
        plt.rcParams['axes.unicode_minus'] = False 
        
        # 색상 팔레트
        self.bg_color = '#FFFFFF'
        self.text_main = '#111827'
        self.text_sub = '#6B7280'
        self.grid_color = '#E5E7EB'
        self.color_cone = '#F59E0B'    # CONE 주행 (오렌지/옐로우)
        self.color_lane = '#3B82F6'    # LANE 주행 (블루)
        self.color_speed = '#10B981'   # 속도 그래프 (그린)
        self.color_steer = '#EF4444'   # 조향 그래프 (레드)
        self.color_cte = '#8B5CF6'     # CTE 그래프 (퍼플)

        self.fig = plt.figure(figsize=(14, 7), facecolor=self.bg_color)
        self.fig.canvas.manager.set_window_title('KUUVe Real-time Telemetry')
        
        # 레이아웃 분할: 1행 2열 (좌측: 텍스트 HUD, 우측: 그래프)
        gs = gridspec.GridSpec(1, 2, width_ratios=[1, 1.5], wspace=0.1, left=0.05, right=0.95, top=0.92, bottom=0.08)

        # ------------------------------------------
        # 1. Left Panel: Text HUD (ax_hud)
        # ------------------------------------------
        self.ax_hud = self.fig.add_subplot(gs[0])
        self.ax_hud.set_facecolor(self.bg_color)
        self.ax_hud.axis('off')
        
        # 주행 모드 텍스트 (좌측 최상단)
        self.txt_mode = self.ax_hud.text(0.1, 0.9, '주행 모드 확인 중...', fontsize=24, ha='left', va='center', fontweight='bold')
        
        # 속도 텍스트
        self.ax_hud.text(0.1, 0.65, 'CURRENT SPEED', fontsize=14, ha='left', va='center', color=self.text_sub, fontweight='bold')
        self.txt_speed_val = self.ax_hud.text(0.1, 0.5, '0.0', fontsize=65, ha='left', va='center', color=self.text_main, fontweight='bold')
        self.txt_speed_unit = self.ax_hud.text(0.6, 0.48, 'km/h', fontsize=20, ha='left', va='bottom', color=self.text_sub)
        
        # 조향각 텍스트
        self.ax_hud.text(0.1, 0.25, 'STEERING ANGLE', fontsize=14, ha='left', va='center', color=self.text_sub, fontweight='bold')
        self.txt_steer_val = self.ax_hud.text(0.1, 0.1, '0.0°', fontsize=50, ha='left', va='center', color=self.text_main, fontweight='bold')

        # ------------------------------------------
        # 2. Right Panel: Real-time Graphs (ax_graphs)
        # ------------------------------------------
        gs_right = gridspec.GridSpecFromSubplotSpec(3, 1, subplot_spec=gs[1], hspace=0.3)
        
        # 2-1. 속도 그래프
        self.ax_speed = self.fig.add_subplot(gs_right[0])
        self.setup_graph_axis(self.ax_speed, 'Speed (km/h)')
        self.line_speed, = self.ax_speed.plot(self.time_axis, self.speed_data, color=self.color_speed, lw=2.5)
        self.ax_speed.set_ylim(-5, 130)

        # 2-2. 조향각 그래프
        self.ax_steer = self.fig.add_subplot(gs_right[1])
        self.setup_graph_axis(self.ax_steer, 'Steering Angle (°)')
        self.line_steer, = self.ax_steer.plot(self.time_axis, self.steer_data, color=self.color_steer, lw=2.5)
        self.ax_steer.set_ylim(-45, 45)
        self.ax_steer.axhline(0, color=self.text_sub, linestyle='--', lw=1, alpha=0.5)

        # 2-3. CTE (Cross Track Error) 그래프
        self.ax_cte = self.fig.add_subplot(gs_right[2])
        self.setup_graph_axis(self.ax_cte, 'Cross Track Error (m)')
        self.line_cte, = self.ax_cte.plot(self.time_axis, self.cte_data, color=self.color_cte, lw=2.5)
        self.ax_cte.set_ylim(-3.0, 3.0)
        self.ax_cte.axhline(0, color=self.text_sub, linestyle='--', lw=1, alpha=0.5)

    def setup_graph_axis(self, ax, title):
        ax.set_facecolor(self.bg_color)
        ax.set_title(title, color=self.text_main, fontsize=11, fontweight='bold', loc='left')
        ax.grid(True, linestyle='-', alpha=0.5, color=self.grid_color)
        ax.set_xlim(-self.window_size, 0)
        ax.set_xticklabels([])
        ax.tick_params(colors=self.text_sub, labelsize=9)
        for spine in ax.spines.values():
            spine.set_color(self.grid_color)

    # ================= 콜백 함수들 =================
    def steer_callback(self, msg): self.steer_data.append(msg.data)
    def speed_callback(self, msg): self.speed_data.append(float(msg.data))
    def conf_callback(self, msg): self.conf_data.append(msg.data)
    def path_callback(self, msg): 
        if msg.points:
            self.path_data = np.array([[p.x, p.y] for p in msg.points])

    # ================= 렌더링 루프 =================
    def update_plot(self, frame):
        # 최신 데이터 추출
        c_speed = self.speed_data[-1] if len(self.speed_data) > 0 else 0.0
        c_steer = self.steer_data[-1] if len(self.steer_data) > 0 else 0.0
        c_conf = self.conf_data[-1] if len(self.conf_data) > 0 else 0.0
        
        # 경로 오차(CTE) 계산 및 저장
        cte = self.path_data[0, 1] if len(self.path_data) > 0 else 0.0
        self.cte_data.append(cte)

        # 모드 판별 로직 (신뢰도 기준)
        is_cone_mode = c_conf > 0.5
        
        # 1. 텍스트 HUD 업데이트
        if is_cone_mode:
            self.txt_mode.set_text("▶ CONE 주행 모드")
            self.txt_mode.set_color(self.color_cone)
        else:
            self.txt_mode.set_text("▶ LANE 주행 모드")
            self.txt_mode.set_color(self.color_lane)

        self.txt_speed_val.set_text(f"{c_speed:.1f}")
        self.txt_steer_val.set_text(f"{c_steer:+.1f}°")
        
        # 조향 방향에 따른 텍스트 색상 포인트
        if abs(c_steer) < 1.5:
            self.txt_steer_val.set_color(self.text_main)
        elif c_steer > 0:
            self.txt_steer_val.set_color(self.color_lane) # 좌측
        else:
            self.txt_steer_val.set_color(self.color_cone) # 우측

        # 2. 그래프 데이터 업데이트
        self.line_speed.set_ydata(self.speed_data)
        self.line_steer.set_ydata(self.steer_data)
        self.line_cte.set_ydata(self.cte_data)

        return []

if __name__ == '__main__':
    try:
        plotter = FSDControlTerminal()
        # interval을 낮춰 더 부드러운 실시간 그래프 렌더링
        ani = FuncAnimation(plotter.fig, plotter.update_plot, interval=40, blit=False)
        plt.show() 
    except rospy.ROSInterruptException:
        pass