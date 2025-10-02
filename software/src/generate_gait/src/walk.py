# walk.py
import zmq
import numpy as np
import time

class WalkGaitController:
    """歩行歩容生成：直線で蹴り、円弧で戻る"""
    
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
        
        # 歩容パラメータ
        self.stride_length = 0.2   # ストライド長 [m]（前後の合計移動距離）
        self.ground_clearance = 0.05  # 地面からの持ち上げ高さ [m]
        self.ground_height = -0.35    # 地面の高さ（胴体中心座標系）[m]
        self.swing_height = self.ground_height + self.ground_clearance  # スイング時の最高点
        
        # 前後の位置
        self.x_forward = self.stride_length / 2   # 前方位置
        self.x_backward = -self.stride_length / 2  # 後方位置
        
        # タイミング
        self.period = 2.0  # 1サイクル周期 [s]
        self.stance_ratio = 0.6  # 接地期の割合（蹴る動作）
        self.swing_ratio = 1.0 - self.stance_ratio  # 遊脚期の割合（戻る動作）
        
        print("歩行歩容パラメータ:")
        print(f"- ストライド長: {self.stride_length}m")
        print(f"- 地面高さ: {-self.ground_height:.2f}m (胴体から)")
        print(f"- 持ち上げ高さ: {self.ground_clearance}m")
        print(f"- 接地期割合: {self.stance_ratio*100:.0f}%")
        print(f"- 遊脚期割合: {self.swing_ratio*100:.0f}%")
        print(f"- 左右脚位相差: 180° (逆相)")
    
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
        target_x = target_pos[0]
        target_y = target_pos[1]
        target_z = target_pos[2]
        
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
    
    def generate_walk_trajectory(self, t):
        """
        歩行軌跡を生成
        - 接地期（0 ~ stance_ratio）: 直線で後方に蹴る（x: forward -> backward, z: ground）
        - 遊脚期（stance_ratio ~ 1.0）: 円弧で前方に戻る（円弧軌道）
        
        t: 時刻 [s]
        returns: [x, y, z] 目標位置
        """
        # 周期内の位相（0～1）
        phase = (t % self.period) / self.period
        
        # Y座標は固定
        y = self.link_lengths['hip_width'] / 2
        
        if phase < self.stance_ratio:
            # 接地期：直線で蹴る
            # phase: 0 -> stance_ratio
            # x: forward -> backward
            # z: ground (固定)
            local_phase = phase / self.stance_ratio  # 0～1に正規化
            x = self.x_forward + (self.x_backward - self.x_forward) * local_phase
            z = self.ground_height
            
        else:
            # 遊脚期：円弧で戻る
            # phase: stance_ratio -> 1.0
            # x: backward -> forward
            # z: ground -> swing_height -> ground（楕円/円弧軌道）
            local_phase = (phase - self.stance_ratio) / self.swing_ratio  # 0～1に正規化
            
            # X方向：後方から前方へ線形補間
            x = self.x_backward + (self.x_forward - self.x_backward) * local_phase
            
            # Z方向：円弧（正弦波の上半分）
            # local_phase: 0 -> 1 に対して z: ground -> swing_height -> ground
            z = self.ground_height + self.ground_clearance * np.sin(np.pi * local_phase)
        
        return [x, y, z]
    
    def run(self):
        """制御ループ"""
        freq = 50  # [Hz]
        dt = 1.0 / freq
        t = 0
        
        print("\n歩行歩容制御プロセス開始 (50Hz)")
        print(f"座標系: X=前後, Y=左右, Z=上下（負が下）")
        print(f"周期: {self.period}s")
        print(f"制御周波数: {freq}Hz")
        print(f"\nリアルタイム設定: CONFLATE有効（最新データのみ送信）\n")
        
        try:
            while True:
                start_time = time.time()
                
                # 1. 目標位置生成（左右の脚は逆相：半周期ずらす）
                target_pos_left = self.generate_walk_trajectory(t)
                target_pos_right_temp = self.generate_walk_trajectory(t + self.period / 2)
                # 右脚のY座標を反転
                target_pos_right = [target_pos_right_temp[0], -target_pos_right_temp[1], target_pos_right_temp[2]]
                
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
                    
                    # 左脚の位相
                    phase_left = ((t - dt) % self.period) / self.period
                    gait_phase_left = "接地期（蹴り）" if phase_left < self.stance_ratio else "遊脚期（戻り）"
                    
                    # 右脚の位相
                    phase_right = (((t - dt) + self.period / 2) % self.period) / self.period
                    gait_phase_right = "接地期（蹴り）" if phase_right < self.stance_ratio else "遊脚期（戻り）"
                    
                    toe_x_left = target_pos_left[0]
                    toe_z_left = -target_pos_left[2]  # 正の値で表示
                    toe_x_right = target_pos_right[0]
                    toe_z_right = -target_pos_right[2]
                    
                    print(f"[t={t:.1f}s] 制御周期: {actual_freq:.1f} Hz")
                    print(f"  左脚: {gait_phase_left:12s}, x={toe_x_left:+.3f}m, z={toe_z_left:.3f}m, "
                          f"j3={left_angles[2]:+.1f}°, j4={left_angles[3]:+.1f}°")
                    print(f"  右脚: {gait_phase_right:12s}, x={toe_x_right:+.3f}m, z={toe_z_right:.3f}m, "
                          f"j3={right_angles[2]:+.1f}°, j4={right_angles[3]:+.1f}°")
                
                # 周期維持
                elapsed = time.time() - start_time
                sleep_time = max(0, dt - elapsed)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print("\n制御プロセス終了")
            self.socket.close()
            self.context.term()

if __name__ == "__main__":
    controller = WalkGaitController()
    controller.run()
