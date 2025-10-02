# control_process.py
import zmq
import numpy as np
import time

class BipedController:
    def __init__(self):
        # ZeroMQ設定
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://127.0.0.1:5555")
        time.sleep(0.1)
        
        # ロボットパラメータ
        self.link_lengths = {
            'hip_width': 0.2,      # 左右の股関節間距離（Y方向）
            'hip_yaw': 0.05,       # ヨー軸のオフセット
            'thigh': 0.2,          # 大腿部長さ (j13-j14)
            'shank': 0.2,          # 下腿部長さ (j14-j15)
            'foot': 0.1            # 足首から足先
        }
        
        # 歩容パラメータ
        self.step_height = 0.1    # 足上げ高さ [m] (Z方向)
        self.step_length = 0.0    # ストライド長さ [m] (X方向)
        self.step_frequency = 0.5  # 歩行周波数 [Hz]
        
    def camber_trajectory(self, phase):
        """
        直線往復軌道生成（X-Z平面）
        phase: 0.0-1.0 (1歩行周期)
        returns: (x, z) 相対座標 (j13基準、股関節から見た足先の目標位置)
        
        座標系:
        - X軸: 前後方向（正が前方）
        - Z軸: 上下方向（負が下方＝重力方向）
        
        つま先は直線上を往復:
        - phase 0.0-0.5: 前方へ移動（接地相）
        - phase 0.5-1.0: 後方へ移動（遊脚相、持ち上げて戻る）
        """
        if phase < 0.5:
            # 接地相: 地面を直線に蹴る (後ろから前へ)
            progress = phase / 0.5  # 0.0 -> 1.0
            x = self.step_length/2 * (2*progress - 1)  # -0.075 -> +0.075
            z = 0  # 地面に接地
        else:
            # 遊脚相: 持ち上げて直線的に戻る (前から後ろへ)
            progress = (phase - 0.5) / 0.5  # 0.0 -> 1.0
            x = self.step_length/2 * (1 - 2*progress)  # +0.075 -> -0.075
            z = self.step_height  # 一定高さで持ち上げ
        
        return x, z
    
    def inverse_kinematics_2link(self, target_x, target_z, l1, l2):
        """
        2リンクの逆運動学 (j13, j14) - X-Z平面
        
        初期姿勢（j13=0, j14=0）: 脚はまっすぐ下に垂直
        - 大腿部: (0, 0) -> (0, -l1)
        - 下腿部: (0, -l1) -> (0, -l1-l2)
        
        target_x, target_z: j13基準の目標足首位置（股関節から見た相対座標）
        l1: 大腿部長さ (thigh)
        l2: 下腿部長さ (shank)
        returns: (j13_angle, j14_angle) [degrees]
        
        j13: 股関節ピッチ (正=前方への回転、負=後方への回転)
        j14: 膝関節ピッチ (正=膝を曲げる方向)
        """
        
        # 目標までの距離
        d = np.sqrt(target_x**2 + target_z**2)
        
        # 到達可能性チェック
        max_reach = l1 + l2 - 0.001
        min_reach = abs(l1 - l2) + 0.001
        
        if d > max_reach:
            # 到達できない場合はスケール
            scale = max_reach / d
            target_x *= scale
            target_z *= scale
            d = max_reach
        elif d < min_reach:
            d = min_reach
            
        # 余弦定理で膝の角度を求める
        cos_j14 = (l1**2 + l2**2 - d**2) / (2 * l1 * l2)
        cos_j14 = np.clip(cos_j14, -1.0, 1.0)
        
        # j14: 膝の屈曲角度（0=伸びきり、正=曲げる）
        j14 = np.arccos(cos_j14)
        
        # j13を求める
        # 目標への角度（垂直下向き=0を基準）
        theta_target = np.arctan2(target_x, -target_z)
        
        # 膝が曲がった時の大腿部の補正角度
        theta_correction = np.arctan2(l2 * np.sin(j14), l1 + l2 * np.cos(j14))
        
        j13 = theta_target - theta_correction
        
        return np.degrees(j13), np.degrees(j14)
    
    def calculate_foot_angle(self, j13_deg, j14_deg):
        """
        足裏が地面と平行を保つためのj15角度計算
        j13_deg: 股関節ピッチ角度 [degrees]
        j14_deg: 膝関節ピッチ角度 [degrees]
        returns: j15_angle [degrees]
        """
        # 足裏を水平に保つ: j13 + j14 + j15 = 0
        j15 = -(j13_deg + j14_deg)
        return j15
    
    def generate_walking_pattern(self, t):
        """
        歩行パターン生成
        t: 時刻 [s]
        returns: (left_angles, right_angles) それぞれ5自由度
        """
        # 歩行周期での位相
        phase = (t * self.step_frequency) % 1.0
        
        # 左右で半周期ずれた位相
        left_phase = phase
        right_phase = (phase + 0.5) % 1.0
        
        # リンク長
        l1 = self.link_lengths['thigh']
        l2 = self.link_lengths['shank']
        
        # --- 左脚 ---
        # 直線往復軌道で目標位置生成（股関節基準の相対座標）
        rel_x, rel_z = self.camber_trajectory(left_phase)
        
        # 初期姿勢では足先は股関節の真下 (0, -(l1+l2+foot))
        # そこからの相対移動
        target_left_x = 0 + rel_x
        target_left_z = -(l1 + l2 + self.link_lengths['foot']) + rel_z
        
        # 2リンク逆運動学
        left_j13, left_j14 = self.inverse_kinematics_2link(
            target_left_x, target_left_z, l1, l2
        )
        
        # 足首角度（足裏を地面と平行に）
        left_j15 = self.calculate_foot_angle(left_j13, left_j14)
        
        left_angles = [
            0,           # j11 (hip_yaw)
            0,           # j12 (hip_roll)
            left_j13,    # j13 (hip_pitch)
            left_j14,    # j14 (knee_pitch)
            left_j15     # j15 (ankle_pitch)
        ]
        
        # --- 右脚 ---
        rel_x, rel_z = self.camber_trajectory(right_phase)
        
        target_right_x = 0 + rel_x
        target_right_z = -(l1 + l2 + self.link_lengths['foot']) + rel_z
        
        right_j13, right_j14 = self.inverse_kinematics_2link(
            target_right_x, target_right_z, l1, l2
        )
        
        right_j15 = self.calculate_foot_angle(right_j13, right_j14)
        
        right_angles = [
            0,            # j11 (hip_yaw)
            0,            # j12 (hip_roll)
            right_j13,    # j13 (hip_pitch)
            right_j14,    # j14 (knee_pitch)
            right_j15     # j15 (ankle_pitch)
        ]
        
        return left_angles, right_angles
    
    def forward_kinematics_leg(self, angles, is_left=True):
        """
        順運動学（描画用）
        angles: [j11, j12, j13, j14, j15] (degrees)
        returns: 関節位置のリスト [[x,y,z], ...]
        
        座標系:
        - X軸: 前後方向（正が前、負が後）
        - Y軸: 左右方向（正が右、負が左）
        - Z軸: 上下方向（正が上、負が下）
        """
        yaw, roll, hip_pitch, knee_pitch, ankle_pitch = np.radians(angles)
        
        L = self.link_lengths
        side = 1 if is_left else -1
        
        positions = []
        
        # 0: 股関節中心
        hip_base = np.array([0, side * L['hip_width']/2, 0.8])
        positions.append(hip_base)
        
        # 1: ヨー軸後
        yaw_offset = np.array([
            L['hip_yaw'] * np.sin(yaw),
            L['hip_yaw'] * np.cos(yaw) * side,
            0
        ])
        hip_roll_pos = hip_base + yaw_offset
        positions.append(hip_roll_pos)
        
        # 2: ロール軸
        current_pos = hip_roll_pos.copy()
        positions.append(current_pos)
        
        # 3: 膝関節（j13のピッチ回転後）
        # 初期状態で下向き: (0, 0, -l1)
        # ピッチ回転: X-Z平面での回転
        thigh_vec = np.array([
            L['thigh'] * np.sin(hip_pitch),
            0,
            -L['thigh'] * np.cos(hip_pitch)
        ])
        
        # ロール回転適用（Y軸周りの回転）
        c_roll, s_roll = np.cos(roll), np.sin(roll)
        roll_matrix = np.array([
            [c_roll, 0, s_roll],
            [0, 1, 0],
            [-s_roll, 0, c_roll]
        ])
        thigh_vec = roll_matrix @ thigh_vec
        knee_pos = current_pos + thigh_vec
        positions.append(knee_pos)
        
        # 4: 足首（j14のピッチ回転後）
        # j14は膝関節での追加回転
        total_pitch = hip_pitch + knee_pitch
        shank_vec = np.array([
            L['shank'] * np.sin(total_pitch),
            0,
            -L['shank'] * np.cos(total_pitch)
        ])
        shank_vec = roll_matrix @ shank_vec
        ankle_pos = knee_pos + shank_vec
        positions.append(ankle_pos)
        
        # 5: 足先（j15のピッチ回転後）
        total_pitch_ankle = total_pitch + ankle_pitch
        foot_vec = np.array([
            L['foot'] * np.sin(total_pitch_ankle),
            0,
            -L['foot'] * np.cos(total_pitch_ankle)
        ])
        foot_vec = roll_matrix @ foot_vec
        foot_pos = ankle_pos + foot_vec
        positions.append(foot_pos)
        
        return positions
    
    def run(self):
        """50Hzメインループ"""
        freq = 50
        dt = 1.0 / freq
        t = 0
        
        print("制御プロセス開始 (50Hz)")
        print("直線往復軌道パターン実行中 (X-Z平面)")
        print(f"- ストライド: {self.step_length}m (X方向)")
        print(f"- 足上げ高さ: {self.step_height}m (Z方向)")
        print(f"- 歩行周波数: {self.step_frequency}Hz")
        print(f"座標系: X=前後, Y=左右, Z=上下")
        
        try:
            while True:
                start_time = time.time()
                
                # 1. 歩行パターン生成
                left_angles, right_angles = self.generate_walking_pattern(t)
                
                # 2. 順運動学
                left_positions = self.forward_kinematics_leg(left_angles, is_left=True)
                right_positions = self.forward_kinematics_leg(right_angles, is_left=False)
                
                # 3. データパッケージ
                data = {
                    'timestamp': time.time(),
                    'cycle': t,
                    'joint_angles': left_angles + right_angles,
                    'left_positions': np.array(left_positions).tolist(),
                    'right_positions': np.array(right_positions).tolist()
                }
                
                # 4. 送信
                self.socket.send_pyobj(data)
                
                t += dt
                
                # 周期維持
                elapsed = time.time() - start_time
                sleep_time = max(0, dt - elapsed)
                time.sleep(sleep_time)
                
                # デバッグ出力
                if abs(t % 2.0) < dt:
                    print(f"[t={t:.1f}s] L: j13={left_angles[2]:+6.1f}° j14={left_angles[3]:+6.1f}° j15={left_angles[4]:+6.1f}° | "
                          f"R: j13={right_angles[2]:+6.1f}° j14={right_angles[3]:+6.1f}° j15={right_angles[4]:+6.1f}°")
                
        except KeyboardInterrupt:
            print("\n制御プロセス終了")
            self.socket.close()
            self.context.term()

if __name__ == "__main__":
    controller = BipedController()
    controller.run()