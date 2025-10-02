# swing_horizontal.py
import zmq
import numpy as np
import time

class ToeTrajectoryController:
    """つま先を前後に動かす制御"""
    
    def __init__(self):
        # ZeroMQ設定
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        
        # リアルタイム性を確保するための設定
        # CONFLATEオプション: キューサイズを1に制限（最新のメッセージのみ保持）
        self.socket.setsockopt(zmq.CONFLATE, 1)
        
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
        
        # 軌跡パラメータ（X軸方向）
        self.x_max = 0.1   # 前方最大位置 [m]
        self.x_min = -0.1  # 後方最大位置 [m]
        self.z_fixed = -0.3  # 固定高さ（座標系でZは下が負）
        self.period = 2.0  # 往復周期 [s]
    
    def inverse_kinematics(self, target_pos, is_left=True):
        """
        2リンクアームの逆運動学
        target_pos: [x, y, z] 足首の目標位置（胴体中心座標系）
        returns: [j1, j2, j3, j4, j5] (degrees)
        
        簡略化のため：
        - j1 (yaw) = 0
        - j2 (roll) = 0
        - X-Z平面内の2リンクIK
        """
        L = self.link_lengths
        side = 1 if is_left else -1
        
        # 目標位置を基部座標系に変換
        base_y = side * L['hip_width'] / 2
        target_x = target_pos[0] # - L['hip_yaw']  # yawオフセット分を引く
        target_y = target_pos[1] #- base_y
        target_z = target_pos[2]
        
        # エンドエフェクタ長さ分を考慮して手首位置を計算
        # wrist_x = rel_x
        # wrist_z = rel_z
        
        # 2リンクIK（X-Z平面）
        L1 = L['thigh']
        L2 = L['shank']
        
        # 距離
        d = np.sqrt(target_x**2 + target_z**2)
        
        # 到達可能性チェック
        if d > (L1 + L2) or d < abs(L1 - L2):
            print(f"Warning: Target unreachable. Distance: {d:.3f}, Max: {L1+L2:.3f}, Min: {abs(L1-L2):.3f}")
            d = np.clip(d, abs(L1 - L2) + 0.001, L1 + L2 - 0.001)
        
        # コサイン定理で膝角度
        cos_knee = (L1**2 + L2**2 - d**2) / (2 * L1 * L2)
        cos_knee = np.clip(cos_knee, -1, 1)
        knee_angle = np.arccos(cos_knee)  # 常に正（0～pi）
        
        # 第2関節の角度（j4）
        # ピッチ角度の定義に合わせる（負で屈曲）
        j4 = -np.degrees(knee_angle - np.pi)  # 伸びた状態を0とする
        
        # 第1関節の角度（j3）
        alpha = np.arctan2(target_x, -target_z)  # 目標方向の角度
        beta = np.arcsin((L2 * np.sin(knee_angle)) / d)  # 三角形の角度
        j3 = np.degrees(alpha - beta)
        
        # その他の角度
        j1 = 0.0  # yaw
        j2 = 0.0  # roll
        
        # j5: つま先を鉛直方向（真下）に保つ
        # j3 + j4 + j5 = 0 となるように設定
        # （全ての回転の合計が0になれば、footは鉛直下向き）
        j5 = -(j3 + j4)  # foot pitch
        
        return [j1, j2, j3, j4, j5]
    
    def forward_kinematics_arm(self, angles, is_left=True):
        """
        順運動学（描画用） - Z-Y-X座標系
        angles: [j1, j2, j3, j4, j5] (degrees)
        returns: 関節位置のリスト [[x,y,z], ...]
        """
        yaw, roll, pitch1, pitch2, pitch3 = np.radians(angles)
        
        L = self.link_lengths
        side = 1 if is_left else -1
        
        positions = []
        
        # 0: アーム基部（ヨー軸）
        base = np.array([0, side * L['hip_width']/2, 0.0])
        positions.append(base)
        
        # 1: ヨー軸後（j1）
        yaw_offset = np.array([
            L['hip_yaw'] * np.sin(yaw),
            L['hip_yaw'] * np.cos(yaw) * side,
            0
        ])
        roll_pos = base + yaw_offset
        positions.append(roll_pos)
        
        # 2: ロール軸（j2）
        positions.append(roll_pos.copy())
        
        # 3: 第1リンク後 = 第2関節（j3 -> j4）
        link1_vec = np.array([
            L['thigh'] * np.sin(pitch1),
            0,
            -L['thigh'] * np.cos(pitch1)
        ])
        
        # ロール回転適用
        c_roll, s_roll = np.cos(roll), np.sin(roll)
        roll_matrix = np.array([
            [c_roll, 0, s_roll],
            [0, 1, 0],
            [-s_roll, 0, c_roll]
        ])
        link1_vec = roll_matrix @ link1_vec
        joint2_pos = roll_pos + link1_vec
        positions.append(joint2_pos)
        
        # 4: 第2リンク後 = エンドエフェクタ関節（j4 -> j5）
        total_pitch = pitch1 + pitch2
        link2_vec = np.array([
            L['shank'] * np.sin(total_pitch),
            0,
            -L['shank'] * np.cos(total_pitch)
        ])
        link2_vec = roll_matrix @ link2_vec
        joint3_pos = joint2_pos + link2_vec
        positions.append(joint3_pos)
        
        # 5: エンドエフェクタ先端（j5）
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
    
    def generate_toe_trajectory(self, t):
        """
        つま先の目標位置を生成（正弦波で前後）
        t: 時刻 [s]
        returns: [x, y, z] 目標位置（左脚）
        """
        # X軸方向に正弦波
        x_center = (self.x_min + self.x_max) / 2
        x_amplitude = (self.x_max - self.x_min) / 2
        x = x_center + x_amplitude * np.sin(2 * np.pi * t / self.period)
        
        # Y, Z は固定
        y = self.link_lengths['hip_width'] / 2
        z = self.z_fixed
        
        return [x, y, z]
    
    def run(self):
        """制御ループ"""

        freq = 50 #[Hz]
        dt = 1.0 / freq
        t = 0
        
        print("つま先軌跡制御プロセス開始 (50Hz) - 前後方向")
        print(f"X軸範囲: {self.x_min:.2f}m ~ {self.x_max:.2f}m (前後方向)")
        print(f"Z軸高さ: {-self.z_fixed:.2f}m (固定)")
        print(f"周期: {self.period}s")
        print(f"アームパラメータ:")
        print(f"- 第1リンク: {self.link_lengths['thigh']}m")
        print(f"- 第2リンク: {self.link_lengths['shank']}m")
        print(f"- エンドエフェクタ: {self.link_lengths['foot']}m")
        print(f"- 最大リーチ: {self.link_lengths['thigh'] + self.link_lengths['shank'] + self.link_lengths['foot']}m")
        print(f"座標系: X=前後, Y=左右, Z=上下（負が下）")
        print(f"\nリアルタイム設定: CONFLATE有効（最新データのみ送信）\n")
        
        try:
            while True:
                start_time = time.time()
                
                # 1. 目標位置生成
                target_pos_left = self.generate_toe_trajectory(t)
                target_pos_right = self.generate_toe_trajectory(t)
                
                # 2. 逆運動学で角度計算
                left_angles = self.inverse_kinematics(target_pos_left, is_left=True)
                right_angles = self.inverse_kinematics(target_pos_right, is_left=False)
                
                # 3. 順運動学で各関節の位置を計算
                left_positions = self.forward_kinematics_arm(left_angles, is_left=True)
                right_positions = self.forward_kinematics_arm(right_angles, is_left=False)
                
                # 4. データパッケージ作成
                data = {
                    'timestamp': time.time(),
                    'cycle': t,
                    'joint_angles': {
                        'j11': left_angles[0],  # left yaw
                        'j12': left_angles[1],  # left roll
                        'j13': left_angles[2],  # left pitch1
                        'j14': left_angles[3],  # left pitch2
                        'j15': left_angles[4],  # left pitch3
                        'j21': right_angles[0], # right yaw
                        'j22': right_angles[1], # right roll
                        'j23': right_angles[2], # right pitch1
                        'j24': right_angles[3], # right pitch2
                        'j25': right_angles[4]  # right pitch3
                    },
                    'left_positions': np.array(left_positions).tolist(),
                    'right_positions': np.array(right_positions).tolist(),
                    'target_left': target_pos_left,
                    'target_right': target_pos_right
                }
                
                # 5. ZeroMQで送信
                self.socket.send_pyobj(data)
                
                t += dt
                
                # パフォーマンス監視（5秒ごと）
                if abs(t % 5.0) < dt:
                    elapsed = time.time() - start_time
                    sleep_time = max(0, dt - elapsed)
                    actual_freq = 1.0 / (elapsed + sleep_time) if (elapsed + sleep_time) > 0 else 0
                    toe_x = target_pos_left[0]  # X位置（前後）
                    total_pitch = left_angles[2] + left_angles[3] + left_angles[4]  # j3 + j4 + j5
                    print(f"[t={t:.1f}s] 制御周期: {actual_freq:.1f} Hz, "
                          f"つま先X位置: {toe_x:.3f}m, "
                          f"関節角度: j3={left_angles[2]:.1f}°, j4={left_angles[3]:.1f}°, j5={left_angles[4]:.1f}°, "
                          f"合計={total_pitch:.1f}°")
                
                # 周期維持
                elapsed = time.time() - start_time
                sleep_time = max(0, dt - elapsed)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print("\n制御プロセス終了")
            self.socket.close()
            self.context.term()

if __name__ == "__main__":
    controller = ToeTrajectoryController()
    controller.run()
