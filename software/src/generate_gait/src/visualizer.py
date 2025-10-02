# visualization_process.py
import zmq
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import time

class BipedVisualizer3D:
    def __init__(self):
        # ZeroMQ設定
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("tcp://127.0.0.1:5555")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        
        # リアルタイム性を確保するための設定
        # CONFLATEオプション: キューサイズを1に制限（最新のメッセージのみ保持）
        self.socket.setsockopt(zmq.CONFLATE, 1)
        # 受信タイムアウト: 100ms（データが来ない場合を検知）
        self.socket.setsockopt(zmq.RCVTIMEO, 100)
        
        self.current_data = None
        self.last_receive_time = None
        self.setup_plot()
    
    def setup_plot(self):
        """3Dプロット設定"""
        self.fig = plt.figure(figsize=(8, 8))
        
        # 左上: 3D表示
        self.ax1 = self.fig.add_subplot(221, projection='3d')
        self.ax1.set_xlim(-0.3, 0.3)
        self.ax1.set_ylim(-0.3, 0.3)
        self.ax1.set_zlim(-0.6, 0.2)
        self.ax1.set_xlabel('X [m] (Forward)')
        self.ax1.set_ylabel('Y [m] (Left-Right)')
        self.ax1.set_zlabel('Z [m] (Up)')
        self.ax1.set_title('3D Robot Pose')
        self.ax1.view_init(elev=15, azim=-60)
        
        # 脚のライン
        self.line_left, = self.ax1.plot([], [], [], 'b-o', linewidth=3, 
                                         markersize=6, label='Left Leg')
        self.line_right, = self.ax1.plot([], [], [], 'r-o', linewidth=3, 
                                          markersize=6, label='Right Leg')
        self.line_body, = self.ax1.plot([], [], [], 'g-', linewidth=4)
        
        # 床のグリッド
        x = np.linspace(-0.3, 0.3, 5)
        y = np.linspace(-0.3, 0.3, 5)
        X, Y = np.meshgrid(x, y)
        Z = np.zeros_like(X)
        self.ax1.plot_wireframe(X, Y, Z, color='gray', alpha=0.3, linewidth=0.5)
        
        self.ax1.legend()
        
        # 右上: Y-Z平面図（側面図）
        self.ax2 = self.fig.add_subplot(222)
        self.ax2.set_xlim(-0.3, 0.3)
        self.ax2.set_ylim(-0.6, 0.2)
        self.ax2.set_aspect('equal')
        self.ax2.grid(True)
        self.ax2.set_xlabel('Y [m]')
        self.ax2.set_ylabel('Z [m]')
        self.ax2.set_title('Y-Z View (Front View)')
        
        self.line_left_yz, = self.ax2.plot([], [], 'b-o', linewidth=2, 
                                            markersize=5, label='Left Leg')
        self.line_right_yz, = self.ax2.plot([], [], 'r-o', linewidth=2, 
                                             markersize=5, label='Right Leg')
        self.ax2.legend()
        
        # 左下: X-Z平面図（前面図）
        self.ax3 = self.fig.add_subplot(223)
        self.ax3.set_xlim(-0.3, 0.3)
        self.ax3.set_ylim(-0.6, 0.2)
        self.ax3.set_aspect('equal')
        self.ax3.grid(True)
        self.ax3.set_xlabel('X [m]')
        self.ax3.set_ylabel('Z [m]')
        self.ax3.set_title('X-Z View (Side View)')
        
        self.line_left_xz, = self.ax3.plot([], [], 'b-o', linewidth=2, 
                                            markersize=5, label='Left Leg')
        self.line_right_xz, = self.ax3.plot([], [], 'r-o', linewidth=2, 
                                             markersize=5, label='Right Leg')
        self.ax3.legend()
        
        # 右下: X-Y平面図（上面図）
        self.ax4 = self.fig.add_subplot(224)
        self.ax4.set_xlim(-0.3, 0.3)
        self.ax4.set_ylim(-0.3, 0.3)
        self.ax4.set_aspect('equal')
        self.ax4.grid(True)
        self.ax4.set_xlabel('X [m]')
        self.ax4.set_ylabel('Y [m]')
        self.ax4.set_title('X-Y View (Top View)')
        
        self.line_left_xy, = self.ax4.plot([], [], 'b-o', linewidth=2, 
                                            markersize=5, label='Left Leg')
        self.line_right_xy, = self.ax4.plot([], [], 'r-o', linewidth=2, 
                                             markersize=5, label='Right Leg')
        self.ax4.legend()
    
    def update(self, frame):
        """アニメーション更新（描画のみ）"""
        # データ受信: 最新のメッセージのみを取得
        # キューに複数のメッセージがある場合、古いものをスキップ
        latest_data = None
        messages_skipped = 0
        
        try:
            # ループで全てのメッセージを読み込み、最新のものだけを使用
            while True:
                try:
                    data = self.socket.recv_pyobj(zmq.NOBLOCK)
                    if latest_data is not None:
                        messages_skipped += 1
                    latest_data = data
                except zmq.Again:
                    break
            
            if latest_data is not None:
                self.current_data = latest_data
                self.last_receive_time = time.time()
                
                # スキップされたメッセージ数を表示（リアルタイム性の確認）
                if messages_skipped > 0:
                    print(f"Warning: Skipped {messages_skipped} old messages (keeping only latest)")
                
                # 関節角度情報を表示（デバッグ用）
                if 'joint_angles' in latest_data:
                    angles = latest_data['joint_angles']
                    # 左脚: j11~j15, 右脚: j21~j25
                    if hasattr(self, '_last_print_time'):
                        if latest_data['timestamp'] - self._last_print_time > 5.0:
                            print(f"Joint angles - Left: j11={angles.get('j11', 0):.1f}°, j13={angles.get('j13', 0):.1f}°, j14={angles.get('j14', 0):.1f}°, j15={angles.get('j15', 0):.1f}°")
                            print(f"              Right: j21={angles.get('j21', 0):.1f}°, j23={angles.get('j23', 0):.1f}°, j24={angles.get('j24', 0):.1f}°, j25={angles.get('j25', 0):.1f}°")
                            self._last_print_time = latest_data['timestamp']
                    else:
                        self._last_print_time = latest_data['timestamp']
        except zmq.Again:
            # タイムアウト: データが来ない場合
            if self.last_receive_time is not None:
                time_since_last = time.time() - self.last_receive_time
                if time_since_last > 1.0:  # 1秒以上データが来ない場合
                    if not hasattr(self, '_timeout_warned') or not self._timeout_warned:
                        print(f"Warning: No data received for {time_since_last:.1f}s (publisher may have stopped)")
                        self._timeout_warned = True
            pass
        
        if self.current_data is None:
            return (self.line_left, self.line_right, self.line_body,
                    self.line_left_yz, self.line_right_yz,
                    self.line_left_xz, self.line_right_xz,
                    self.line_left_xy, self.line_right_xy)
        
        # 受信した位置データをそのまま使う（運動学計算なし！）
        left_positions = np.array(self.current_data['left_positions'])
        right_positions = np.array(self.current_data['right_positions'])
        
        # 3D描画更新
        self.line_left.set_data(left_positions[:, 0], left_positions[:, 1])
        self.line_left.set_3d_properties(left_positions[:, 2])
        
        self.line_right.set_data(right_positions[:, 0], right_positions[:, 1])
        self.line_right.set_3d_properties(right_positions[:, 2])
        
        # 胴体ライン
        body_x = [left_positions[0, 0], right_positions[0, 0]]
        body_y = [left_positions[0, 1], right_positions[0, 1]]
        body_z = [left_positions[0, 2], right_positions[0, 2]]
        self.line_body.set_data(body_x, body_y)
        self.line_body.set_3d_properties(body_z)
        
        # Y-Z平面図（側面図）
        self.line_left_yz.set_data(left_positions[:, 1], left_positions[:, 2])
        self.line_right_yz.set_data(right_positions[:, 1], right_positions[:, 2])
        
        # X-Z平面図（前面図）
        self.line_left_xz.set_data(left_positions[:, 0], left_positions[:, 2])
        self.line_right_xz.set_data(right_positions[:, 0], right_positions[:, 2])
        
        # X-Y平面図（上面図）
        self.line_left_xy.set_data(left_positions[:, 0], left_positions[:, 1])
        self.line_right_xy.set_data(right_positions[:, 0], right_positions[:, 1])
        
        return (self.line_left, self.line_right, self.line_body,
                self.line_left_yz, self.line_right_yz,
                self.line_left_xz, self.line_right_xz,
                self.line_left_xy, self.line_right_xy)
    
    def run(self):
        """可視化開始"""
        print("可視化プロセス開始")
        print("受信データをそのまま描画（運動学計算なし）")
        print("関節番号: 左脚 j11~j15, 右脚 j21~j25")
        print("\nリアルタイム設定:")
        print("- CONFLATE有効: キューに最新メッセージのみ保持")
        print("- 古いメッセージは自動的にスキップされます")
        print("- パブリッシャー停止時は描画も即座に停止します\n")
        
        # 20Hzで更新（制御の50Hzより低くてOK）
        ani = FuncAnimation(self.fig, self.update, interval=50, blit=False)
        plt.tight_layout()
        plt.show()
        
        self.socket.close()
        self.context.term()

if __name__ == "__main__":
    visualizer = BipedVisualizer3D()
    visualizer.run()