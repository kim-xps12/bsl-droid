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
            'hip_width': 0.2,      # 左右の股関節間距離（X方向）
            'hip_yaw': 0.05,       # ヨー軸のオフセット
            'thigh': 0.2,          # 大腿部長さ (j13-j14)
            'shank': 0.2,          # 下腿部長さ (j14-j15)
            'foot': 0.1            # 足首から足先
        }
        
        # 歩容パラメータ
        self.step_height = 0.1     # 足上げ高さ [m] (Z方向)
        self.step_length = 0.2     # ストライド長さ [m] (Y方向)
        self.step_frequency = 0.5  # 歩行周波数 [Hz]
        self.ground_clearance = 0.05  # 通常時の地面クリアランス
        
    def camber_trajectory(self, phase):
        """
        カマボコ軌道生成（Z-Y平面）
        phase: 0.0-1.0 (1歩行周期)
        returns: (y, z) 相対座標 (j13基準)
        
        座標系:
        - Y軸: 前後方向（正が前方）
        - Z軸: 上下方向（正が上方、負が下方）
        """
        if phase < 0.5:
            # 接地相: 地面を直線に蹴る (前から後ろへ)
            progress = phase / 0.5  # 0.0 -> 1.0
            y = self.step_length/2 * (1 - 2*progress)  # +0.1 -> -0.1
            z = -self.ground_clearance  # 地面に接地
        else:
            # 遊脚相: 円弧で戻る (後ろから前へ)
            progress = (phase - 0.5) / 0.5  # 0.0 -> 1.0
            angle = np.pi * progress  # 0 -> π
            y = -self.step_length/2 + self.step_length/2 * (1 - np.cos(angle))
            z = self.step_height * np.sin(angle) - self.ground_clearance
        
        return y, z
    
    def inverse_kinematics_2link(self, target_y, target_z, l1, l2):
        """
        2リンクの逆運動学 (j13, j14) - Z-Y平面
        target_y, target_z: j13基準の目標位置
        l1: 大腿部長さ (thigh)
        l2: 下腿部長さ (shank)
        returns: (j13_angle, j14_angle) [degrees]
        
        座標系: Y軸が前方、Z軸が上方（重力方向は-Z）
        j13: 股関節ピッチ (正が前方への回転)
        j14: 膝関節ピッチ (正が伸展方向、つまり膝を曲げると負)
        
        初期姿勢: まっすぐ下に垂直（j13=0, j14=0のとき足首は真下）
        """
        # 目標までの距離
        d = np.sqrt(target_y**2 + target_z**2)
        
        # 到達可能性チェック
        if d > (l1 + l2) - 0.001:
            d = (l1 + l2) - 0.001
        if d < abs(l1 - l2) + 0.001:
            d = abs(l1 - l2) + 0.001
        
        # 余弦定理でj14の角度を求める
        # l1, l2, dの三角形で、j14の外角を求める
        cos_angle = (l1**2 + l2**2 - d**2) / (2 * l1 * l2)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        
        # j14は膝の屈曲角度（内角）
        # 伸びきった状態を0度とすると、曲げると負の角度
        j14_inner = np.arccos(cos_angle)  # 0 to π
        j14 = -(np.pi - j14_inner)  # 膝を曲げると負
        
        # j13の角度を求める
        # 目標方向の角度
        phi = np.arctan2(target_y, -target_z)  # Y-Z平面での角度（下向きが基準）
        
        # 三角形の内角
        psi = np.arctan2(l2 * np.sin(j14_inner), l1 + l2 * np.cos(j14_inner))
        
        j13 = phi - psi
        
        return np.degrees(j13), np.degrees(j14)
    
    def calculate_foot_angle(self, j13_deg, j14_deg, initial_foot_angle=0):
        """
        足裏が地面と平行を保つためのj15角度計算
        j13_deg: 股関節ピッチ角度 [degrees]
        j14_deg: 膝関節ピッチ角度 [degrees]
        initial_foot_angle: 足裏の初期角度 [degrees] (通常0 = 地面と平行)
        returns: j15_angle [degrees]
        """
        # 足裏を地面と平行に保つには、j13+j14+j15=initial_foot_angle
        j15 = initial_foot_angle - (j13_deg + j14_deg)
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
        
        # --- 左脚 ---
        # カマボコ軌道で目標位置生成（Y-Z平面）
        left_y, left_z = self.camber_trajectory(left_phase)
        
        # j13基準の目標位置（股関節から見た足首位置）
        # 初期姿勢: まっすぐ下に伸びている状態から相対移動
        base_z = -(self.link_lengths['thigh'] + self.link_lengths['shank'])
        target_left_y = left_y
        target_left_z = base_z + left_z
        
        # 2リンク逆運動学
        left_j13, left_j14 = self.inverse_kinematics_2link(
            target_left_y, target_left_z,
            self.link_lengths['thigh'],
            self.link_lengths['shank']
        )
        
        # 足首角度（足裏を地面と平行に）
        left_j15 = self.calculate_foot_angle(left_j13, left_j14)
        
        # j11, j12は使用しない（ヨー・ロールは0）
        left_angles = [
            0,           # j11 (hip_yaw)
            0,           # j12 (hip_roll)
            left_j13,    # j13 (hip_pitch)
            left_j14,    # j14 (knee_pitch)
            left_j15     # j15 (ankle_pitch)
        ]
        
        # --- 右脚 ---
        right_y, right_z = self.camber_trajectory(right_phase)
        
        target_right_y = right_y
        target_right_z = base_z + right_z
        
        right_j13, right_j14 = self.inverse_kinematics_2link(
            target_right_y, target_right_z,
            self.link_lengths['thigh'],
            self.link_lengths['shank']
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
        順運動学（描画用） - Z-Y-X座標系
        angles: [j11, j12, j13, j14, j15] (degrees)
        returns: 関節位置のリスト [[x,y,z], ...]
        
        座標系:
        - X軸: 左右方向（正が右、負が左）
        - Y軸: 前後方向（正が前、負が後）
        - Z軸: 上下方向（正が上、負が下）
        """
        yaw, roll, hip_pitch, knee_pitch, ankle_pitch = np.radians(angles)
        
        L = self.link_lengths
        side = 1 if is_left else -1  # 左は正、右は負
        
        positions = []
        
        # 0: 股関節中心（ヨー軸）
        hip_base = np.array([side * L['hip_width']/2, 0, 0.8])
        positions.append(hip_base)
        
        # 1: ヨー軸後（j11） - X-Y平面での回転
        yaw_offset = np.array([
            L['hip_yaw'] * np.cos(yaw) * side,
            L['hip_yaw'] * np.sin(yaw),
            0
        ])
        hip_roll_pos = hip_base + yaw_offset
        positions.append(hip_roll_pos)
        
        # 2: ロール軸（j12）
        positions.append(hip_roll_pos.copy())
        
        # 3: 股関節ピッチ後 = 膝関節（j13 -> j14）
        # Y-Z平面での回転（ピッチ）
        thigh_vec = np.array([
            0,
            L['thigh'] * np.sin(hip_pitch),
            -L['thigh'] * np.cos(hip_pitch)
        ])
        
        # ロール回転適用（X軸周りの回転）
        c_roll, s_roll = np.cos(roll), np.sin(roll)
        roll_matrix = np.array([
            [1, 0, 0],
            [0, c_roll, -s_roll],
            [0, s_roll, c_roll]
        ])
        thigh_vec = roll_matrix @ thigh_vec
        knee_pos = hip_roll_pos + thigh_vec
        positions.append(knee_pos)
        
        # 4: 膝ピッチ後 = 足首（j14 -> j15）
        total_pitch = hip_pitch + knee_pitch
        shank_vec = np.array([
            0,
            L['shank'] * np.sin(total_pitch),
            -L['shank'] * np.cos(total_pitch)
        ])
        shank_vec = roll_matrix @ shank_vec
        ankle_pos = knee_pos + shank_vec
        positions.append(ankle_pos)
        
        # 5: 足首ピッチ後 = 足先（j15）
        total_pitch_ankle = total_pitch + ankle_pitch
        foot_vec = np.array([
            0,
            L['foot'] * np.sin(total_pitch_ankle),
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
        print("カマボコ歩容パターン実行中 (Z-Y平面)")
        print(f"- ストライド: {self.step_length}m (Y方向)")
        print(f"- 足上げ高さ: {self.step_height}m (Z方向)")
        print(f"- 歩行周波数: {self.step_frequency}Hz")
        print(f"座標系: X=左右, Y=前後, Z=上下")
        
        try:
            while True:
                start_time = time.time()
                
                # 1. 歩行パターン生成（逆運動学込み）
                left_angles, right_angles = self.generate_walking_pattern(t)
                
                # 2. 順運動学で各関節の位置を計算
                left_positions = self.forward_kinematics_leg(left_angles, is_left=True)
                right_positions = self.forward_kinematics_leg(right_angles, is_left=False)
                
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
                
                # 5. モータへ角度指令送信（実装時）
                # send_to_motors(left_angles + right_angles)
                
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
                          f"L_j13={left_angles[2]:.1f}° L_j14={left_angles[3]:.1f}°")
                
        except KeyboardInterrupt:
            print("\n制御プロセス終了")
            self.socket.close()
            self.context.term()

if __name__ == "__main__":
    controller = BipedController()
    controller.run()