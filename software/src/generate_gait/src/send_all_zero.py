# arm_control.py
import zmq
import numpy as np
import time

class TwoLinkArmController:
    """3次元空間に配置された2リンクアームの制御"""
    
    def __init__(self):
        # ZeroMQ設定
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://127.0.0.1:5555")
        time.sleep(0.1)
        
        # アームパラメータ
        self.link_lengths = {
            'hip_width': 0.2,      # 左右のアーム基部間距離（Y方向）
            'hip_yaw': 0.05,       # ヨー軸のオフセット
            'thigh': 0.2,          # 第1リンク長さ
            'shank': 0.2,          # 第2リンク長さ
            'foot': 0.1            # エンドエフェクタ
        }
        
        # 全ての角度を0に設定
        self.angles = {
            'j11': 0.0,  # yaw
            'j12': 0.0,  # roll
            'j13': 0.0,  # pitch (第1関節)
            'j14': 0.0,  # pitch (第2関節)
            'j15': 0.0   # pitch (エンドエフェクタ)
        }
    
    def forward_kinematics_arm(self, angles, is_left=True):
        """
        順運動学（描画用） - Z-Y-X座標系
        angles: [j11, j12, j13, j14, j15] (degrees)
        returns: 関節位置のリスト [[x,y,z], ...]
        
        座標系:
        - X軸: 前後方向（正が前、負が後）
        - Y軸: 左右方向（正が右、負が左）
        - Z軸: 上下方向（正が上、負が下）
        """
        yaw, roll, pitch1, pitch2, pitch3 = np.radians(angles)
        
        L = self.link_lengths
        side = 1 if is_left else -1  # 左は正、右は負
        
        positions = []
        
        # 0: アーム基部（ヨー軸）
        # ロボット原点（左右脚の付け根の中点）を座標系原点(0,0,0)に配置
        base = np.array([0, side * L['hip_width']/2, 0.0])
        positions.append(base)
        
        # 1: ヨー軸後（j11） - X-Y平面での回転
        yaw_offset = np.array([
            L['hip_yaw'] * np.sin(yaw),
            L['hip_yaw'] * np.cos(yaw) * side,
            0
        ])
        roll_pos = base + yaw_offset
        positions.append(roll_pos)
        
        # 2: ロール軸（j12）
        positions.append(roll_pos.copy())
        
        # 3: 第1リンク後 = 第2関節（j13 -> j14）
        # X-Z平面での回転（ピッチ）
        link1_vec = np.array([
            L['thigh'] * np.sin(pitch1),
            0,
            -L['thigh'] * np.cos(pitch1)
        ])
        
        # ロール回転適用（Y軸周りの回転）
        c_roll, s_roll = np.cos(roll), np.sin(roll)
        roll_matrix = np.array([
            [c_roll, 0, s_roll],
            [0, 1, 0],
            [-s_roll, 0, c_roll]
        ])
        link1_vec = roll_matrix @ link1_vec
        joint2_pos = roll_pos + link1_vec
        positions.append(joint2_pos)
        
        # 4: 第2リンク後 = エンドエフェクタ関節（j14 -> j15）
        total_pitch = pitch1 + pitch2
        link2_vec = np.array([
            L['shank'] * np.sin(total_pitch),
            0,
            -L['shank'] * np.cos(total_pitch)
        ])
        link2_vec = roll_matrix @ link2_vec
        joint3_pos = joint2_pos + link2_vec
        positions.append(joint3_pos)
        
        # 5: エンドエフェクタ先端（j15）
        total_pitch_end = total_pitch + pitch3
        end_vec = np.array([
            L['foot'] * np.sin(total_pitch_end),
            0,
            -L['foot'] * np.cos(total_pitch_end)
        ])
        end_vec = roll_matrix @ end_vec
        end_pos = joint3_pos + end_vec
        positions.append(end_pos)
        
        return positions
    
    def get_arm_angles(self):
        """
        全ての角度を0度で返す
        returns: (left_angles, right_angles) それぞれ5自由度
        """
        angles_list = [
            self.angles['j11'],
            self.angles['j12'],
            self.angles['j13'],
            self.angles['j14'],
            self.angles['j15']
        ]
        
        # 左右同じ角度
        left_angles = angles_list.copy()
        right_angles = angles_list.copy()
        
        return left_angles, right_angles
    
    def run(self):
        """50Hzメインループ"""
        freq = 50
        dt = 1.0 / freq
        t = 0
        
        print("2リンクアーム制御プロセス開始 (50Hz)")
        print("全ての角度: 0度")
        print(f"アームパラメータ:")
        print(f"- 第1リンク: {self.link_lengths['thigh']}m")
        print(f"- 第2リンク: {self.link_lengths['shank']}m")
        print(f"- エンドエフェクタ: {self.link_lengths['foot']}m")
        print(f"座標系: X=前後, Y=左右, Z=上下")
        
        try:
            while True:
                start_time = time.time()
                
                # 1. 角度取得（全て0）
                left_angles, right_angles = self.get_arm_angles()
                
                # 2. 順運動学で各関節の位置を計算
                left_positions = self.forward_kinematics_arm(left_angles, is_left=True)
                right_positions = self.forward_kinematics_arm(right_angles, is_left=False)
                
                # 3. データパッケージ作成
                data = {
                    'timestamp': time.time(),
                    'cycle': t,
                    'joint_angles': left_angles + right_angles,
                    'left_positions': np.array(left_positions).tolist(),
                    'right_positions': np.array(right_positions).tolist()
                }
                
                # 4. ZeroMQで送信
                self.socket.send_pyobj(data)
                
                t += dt
                
                # 周期維持
                elapsed = time.time() - start_time
                sleep_time = max(0, dt - elapsed)
                time.sleep(sleep_time)
                
                # パフォーマンス監視（5秒ごと）
                if abs(t % 5.0) < dt:
                    actual_freq = 1.0 / (elapsed + sleep_time)
                    print(f"[t={t:.1f}s] 制御周期: {actual_freq:.1f} Hz, "
                          f"計算時間: {elapsed*1000:.2f} ms, "
                          f"Left angles: {left_angles}")
                
        except KeyboardInterrupt:
            print("\n制御プロセス終了")
            self.socket.close()
            self.context.term()

if __name__ == "__main__":
    controller = TwoLinkArmController()
    controller.run()
