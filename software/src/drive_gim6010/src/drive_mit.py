#!/usr/bin/env python3
"""
MIT方式位置制御によるモーター原点復帰プログラム
GIM6010-8モーターをCAN通信経由でMIT制御を用いて原点へゆっくり回転させる
リアルタイム制御版（RecurrentTimer使用）
"""

import can
import struct
import time
import math
import threading
from typing import Tuple

# リカレントタイマークラス - 一定周期で処理を実行（高精度版）
class RecurrentTimer:
    def __init__(self, interval, function, *args, **kwargs):
        self.interval = interval
        self.function = function
        self.args = args
        self.kwargs = kwargs
        self.next_call = 0
        self.is_running = False
        self.thread = None
        self.lock = threading.Lock()
        self.total_jitter = 0
        self.jitter_samples = 0
        self.last_execution_time = 0
        
    def _run(self):
        while self.is_running:
            current_time = time.time()
            
            # 実行時間を計測
            start_exec = time.time()
            
            try:
                self.function(*self.args, **self.kwargs)
            except Exception as e:
                print(f"制御関数でエラーが発生: {e}")
            
            # 実行時間を計測
            execution_time = time.time() - start_exec
            self.last_execution_time = execution_time
            
            # 次回の呼び出し時間を計算
            self.next_call += self.interval
            
            # 次回実行までの待機時間を計算
            sleep_time = max(0, self.next_call - time.time())
            
            # ジッター統計の収集
            if self.jitter_samples > 0:  # 最初の1回は計測しない
                jitter = abs((current_time - self.next_call + self.interval))
                self.total_jitter += jitter
                self.jitter_samples += 1
            else:
                self.jitter_samples = 1
            
            # 大幅な遅延が発生した場合はリセット
            if sleep_time <= 0 and abs(sleep_time) > self.interval * 2:
                print(f"警告: 制御周期に大幅な遅延 ({abs(sleep_time):.4f}秒), 次回呼び出し時間をリセットします")
                self.next_call = time.time() + self.interval
                sleep_time = self.interval
            
            # 処理時間が周期を超えた場合の警告
            if execution_time > self.interval * 0.8:
                print(f"警告: 処理時間 {execution_time:.4f}秒が間隔 {self.interval:.4f}秒に近づいています")
            
            # 待機
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def start(self):
        with self.lock:
            if self.is_running:
                return
            
            self.is_running = True
            self.next_call = time.time()
            
            self.thread = threading.Thread(target=self._run)
            self.thread.daemon = True
            self.thread.start()
            
    def stop(self):
        with self.lock:
            self.is_running = False
            
            # 平均ジッター表示
            if self.jitter_samples > 1:
                avg_jitter = self.total_jitter / (self.jitter_samples - 1)
                print(f"平均ジッター: {avg_jitter*1000:.2f}ms")


class MITMotorController:
    """MIT Control方式によるモーター制御クラス"""
    
    # CAN コマンドID
    CMD_SET_AXIS_STATE = 0x007
    CMD_MIT_CONTROL = 0x008
    CMD_GET_ENCODER_COUNT = 0x00A
    CMD_SET_CONTROLLER_MODE = 0x00B
    CMD_CLEAR_ERRORS = 0x018
    
    # 軸状態
    AXIS_STATE_IDLE = 1
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8
    
    # 制御モード
    CONTROL_MODE_VOLTAGE = 0
    CONTROL_MODE_TORQUE = 1
    CONTROL_MODE_VELOCITY = 2
    CONTROL_MODE_POSITION = 3
    
    # 入力モード
    INPUT_MODE_INACTIVE = 0
    INPUT_MODE_PASSTHROUGH = 1
    INPUT_MODE_VEL_RAMP = 2
    INPUT_MODE_POS_FILTER = 3
    INPUT_MODE_MIX_CHANNELS = 4
    INPUT_MODE_TRAP_TRAJ = 5
    INPUT_MODE_TORQUE_RAMP = 6
    INPUT_MODE_MIRROR = 7
    INPUT_MODE_TUNING = 8
    INPUT_MODE_MIT = 9
    
    # エンコーダ設定（GIM6010-8の標準値）
    ENCODER_CPR = 16384 * 8  # Counts Per Revolution (1回転あたりのカウント数) = 131072
    GEAR_RATIO = 8.0    # 減速比 8:1
    
    # MIT制御の位置範囲
    MIT_POS_MIN = -12.5  # rad
    MIT_POS_MAX = 12.5   # rad
    
    def __init__(self, node_id: int, can_channel: str = 'can0', baudrate: int = 1000000):
        """
        初期化
        
        Args:
            node_id: モーターのCANノードID (0-63)
            can_channel: CANインターフェース名
            baudrate: CAN通信速度 (bps)
        """
        self.node_id = node_id
        self.can_channel = can_channel
        
        # CAN初期化
        try:
            self.bus = can.interface.Bus(
                channel=can_channel,
                bustype='socketcan',
                baudrate=baudrate
            )
            print(f"CAN初期化成功: {can_channel} @ {baudrate} bps")
        except Exception as e:
            print(f"CAN初期化エラー: {e}")
            raise
    
    def clear_errors(self):
        """エラーをクリア"""
        can_id = (self.node_id << 5) | self.CMD_CLEAR_ERRORS
        msg = can.Message(
            arbitration_id=can_id,
            data=[0] * 8,
            is_extended_id=False
        )
        
        try:
            self.bus.send(msg)
            print(f"  エラーをクリアしました")
            time.sleep(0.05)
        except Exception as e:
            print(f"エラークリア失敗: {e}")
            raise
    
    def set_controller_mode(self, control_mode: int, input_mode: int):
        """
        コントローラーモードを設定
        
        Args:
            control_mode: 制御モード (0=電圧, 1=トルク, 2=速度, 3=位置)
            input_mode: 入力モード (9=MIT制御)
        """
        can_id = (self.node_id << 5) | self.CMD_SET_CONTROLLER_MODE
        data = struct.pack('<II', control_mode, input_mode)
        
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=False
        )
        
        try:
            self.bus.send(msg)
            mode_name = ["電圧", "トルク", "速度", "位置"][control_mode] if control_mode < 4 else f"MODE_{control_mode}"
            input_name = "MIT制御" if input_mode == 9 else f"INPUT_{input_mode}"
            print(f"  制御モード: {mode_name}制御 ({control_mode}), 入力モード: {input_name} ({input_mode})")
            time.sleep(0.1)
        except Exception as e:
            print(f"コントローラーモード設定エラー: {e}")
            raise
    
    def set_axis_state(self, state: int):
        """
        軸状態を設定
        
        Args:
            state: 軸状態 (1=IDLE, 8=CLOSED_LOOP_CONTROL)
        """
        can_id = (self.node_id << 5) | self.CMD_SET_AXIS_STATE
        data = struct.pack('<I', state) + b'\x00' * 4
        
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=False
        )
        
        try:
            self.bus.send(msg)
            state_name = "CLOSED_LOOP_CONTROL" if state == 8 else "IDLE" if state == 1 else f"STATE_{state}"
            print(f"  軸状態を{state_name}に設定しました")
            time.sleep(0.1)  # 状態遷移待機
        except Exception as e:
            print(f"軸状態設定エラー: {e}")
            raise
    
    def get_encoder_count(self) -> Tuple[int, int]:
        """
        エンコーダカウントを取得
        
        Returns:
            (shadow_count, count_in_cpr): 多圏カウント、単圏カウント
        """
        # Get_Encoder_Count要求を送信
        can_id = (self.node_id << 5) | self.CMD_GET_ENCODER_COUNT
        msg = can.Message(
            arbitration_id=can_id,
            data=[0] * 8,
            is_extended_id=False
        )
        
        try:
            self.bus.send(msg)
            
            # 応答を待機（正しいCAN IDからの応答を待つ）
            response_can_id = (self.node_id << 5) | self.CMD_GET_ENCODER_COUNT
            timeout = time.time() + 1.0
            
            while time.time() < timeout:
                response = self.bus.recv(timeout=0.1)
                if response and response.arbitration_id == response_can_id:
                    # データ解析 (リトルエンディアン)
                    shadow_count = struct.unpack('<i', response.data[0:4])[0]
                    count_in_cpr = struct.unpack('<i', response.data[4:8])[0]
                    return shadow_count, count_in_cpr
            
            raise TimeoutError("エンコーダカウント取得のタイムアウト")
            
        except Exception as e:
            print(f"エンコーダカウント取得エラー: {e}")
            raise
    
    def encoder_count_to_radians(self, shadow_count: int, count_in_cpr: int) -> float:
        """
        エンコーダカウントを角度(rad)に変換
        
        Args:
            shadow_count: 多圏カウント（これが総カウント数）
            count_in_cpr: 単圏カウント
            
        Returns:
            角度 (radians) - 出力軸側、-π〜+πに正規化
        """
        # Shadow_Countが既に総カウント数
        # 仕様書より: Shadow_Count = encoder.shadow_count (多圏計数)
        total_count = shadow_count
        
        # 回転数に変換 (転子側)
        #rotor_revolutions = total_count / self.ENCODER_CPR
        
        # 出力軸側の回転数に変換（減速比を考慮）
        #output_revolutions = rotor_revolutions / self.GEAR_RATIO
        
        # ラジアンに変換
        #angle_rad = output_revolutions * 2.0 * math.pi
        angle_rad = (shadow_count / 16384*8) * 2 * math.pi

        # 連続角（multi-turn）を返す。以前は安全のために-π〜+πに正規化していたが、
        # そのために起動直後の角度がラップして不正な値になることがある。
        # MIT制御側の許容範囲は MIT_POS_MIN/MIT_POS_MAX でチェックするため、
        # ここでは正規化せず連続角を返す。
        return angle_rad
    
    def pack_mit_command(self, position: float, velocity: float, kp: float, kd: float, torque: float) -> bytes:
        """
        MIT制御コマンドをパックする
        
        Args:
            position: 目標位置 (rad) - 出力軸側
            velocity: 目標速度 (rad/s) - 出力軸側
            kp: 位置ゲイン (Nm/rad)
            kd: 速度ゲイン (Nm/(rad/s))
            torque: フィードフォワードトルク (Nm)
            
        Returns:
            8バイトのCANデータ
        """
        # 位置を16bitに変換 (範囲: -12.5 ~ +12.5 rad)
        pos_int = int((position + 12.5) * 65535.0 / 25.0)
        pos_int = max(0, min(65535, pos_int))
        
        # 速度を12bitに変換 (範囲: -65 ~ +65 rad/s)
        vel_int = int((velocity + 65.0) * 4095.0 / 130.0)
        vel_int = max(0, min(4095, vel_int))
        
        # KPを12bitに変換 (範囲: 0 ~ 500)
        kp_int = int(kp * 4095.0 / 500.0)
        kp_int = max(0, min(4095, kp_int))
        
        # KDを12bitに変換 (範囲: 0 ~ 5)
        kd_int = int(kd * 4095.0 / 5.0)
        kd_int = max(0, min(4095, kd_int))
        
        # トルクを12bitに変換 (範囲: -50 ~ +50 Nm)
        torque_int = int((torque + 50.0) * 4095.0 / 100.0)
        torque_int = max(0, min(4095, torque_int))
        
        # データパッキング
        data = bytearray(8)
        
        # BYTE0-1: 位置 (16bit)
        data[0] = (pos_int >> 8) & 0xFF
        data[1] = pos_int & 0xFF
        
        # BYTE2: 速度上位8bit, BYTE3[7-4]: 速度下位4bit
        data[2] = (vel_int >> 4) & 0xFF
        data[3] = (vel_int & 0x0F) << 4
        
        # BYTE3[3-0]: KP上位4bit, BYTE4: KP下位8bit
        data[3] |= (kp_int >> 8) & 0x0F
        data[4] = kp_int & 0xFF
        
        # BYTE5: KD上位8bit, BYTE6[7-4]: KD下位4bit
        data[5] = (kd_int >> 4) & 0xFF
        data[6] = (kd_int & 0x0F) << 4
        
        # BYTE6[3-0]: トルク上位4bit, BYTE7: トルク下位8bit
        data[6] |= (torque_int >> 8) & 0x0F
        data[7] = torque_int & 0xFF
        
        return bytes(data)
    
    def send_mit_command(self, position: float, velocity: float = 0.0, kp: float = 1.0, kd: float = 0.5, torque: float = 0.0, debug: bool = False):
        """
        MIT制御コマンドを送信
        
        Args:
            position: 目標位置 (rad)
            velocity: 目標速度 (rad/s)
            kp: 位置ゲイン (Nm/rad)
            kd: 速度ゲイン (Nm/(rad/s))
            torque: フィードフォワードトルク (Nm)
            debug: デバッグ出力を有効化
        """
        # 位置を範囲内にクリップ
        position_clipped = max(self.MIT_POS_MIN, min(self.MIT_POS_MAX, position))
        if abs(position - position_clipped) > 0.001:
            if debug:
                print(f"    警告: 位置が範囲外です {position:.3f} rad -> {position_clipped:.3f} rad にクリップ")
        
        data = self.pack_mit_command(position_clipped, velocity, kp, kd, torque)
        can_id = (self.node_id << 5) | self.CMD_MIT_CONTROL
        
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=False
        )
        
        if debug:
            print(f"    CAN ID: 0x{can_id:03X}, Data: {data.hex()}, Pos: {position_clipped:.3f} rad")
        
        try:
            self.bus.send(msg)
        except Exception as e:
            print(f"MIT制御コマンド送信エラー: {e}")
            raise
    
    def rotate_to_zero(self, duration: float = 5.0, control_freq: float = 100.0, 
                      kp: float = 0.85, kd: float = 0.03, enable_motor: bool = True):
        """
        現在位置から原点(0 rad)へゆっくり回転（RecurrentTimer使用）
        
        Args:
            duration: 移動にかける時間 (秒)
            control_freq: 制御周期 (Hz)
            kp: 位置ゲイン (Nm/rad)
            kd: 速度ゲイン (Nm/(rad/s))
            enable_motor: モーターを閉環モードにする
        """
        print("=" * 60)
        print("MIT制御による原点復帰開始（RecurrentTimer版）")
        print("=" * 60)
        
        # 0. エラークリアとモード設定
        print("\n[0/8] 初期化...")
        self.clear_errors()
        self.set_controller_mode(self.CONTROL_MODE_POSITION, self.INPUT_MODE_MIT)
        
        if enable_motor:
            print("\n[1/8] モーター閉環制御モード開始...")
            self.set_axis_state(self.AXIS_STATE_CLOSED_LOOP_CONTROL)
        
        # 1. 現在位置を取得
        print("\n[2/8] 現在位置取得中...")
        shadow_count, count_in_cpr = self.get_encoder_count()

        current_position = self.encoder_count_to_radians(shadow_count, count_in_cpr)
        print(f"  Shadow Count: {shadow_count}")
        print(f"  Count in CPR: {count_in_cpr}")
        print(f"  現在位置: {current_position:.4f} rad ({math.degrees(current_position):.2f} deg)")

        # 2. 目標位置を設定
        print("\n[3/8] 目標位置設定")
        target_position = 0.0
        print(f"  目標位置: {target_position:.4f} rad ({math.degrees(target_position):.2f} deg)")
        
        # 位置が範囲外の場合は警告
        if current_position < self.MIT_POS_MIN or current_position > self.MIT_POS_MAX:
            print(f"  ⚠️  警告: 現在位置がMIT制御範囲外です（範囲: {self.MIT_POS_MIN}〜{self.MIT_POS_MAX} rad）")
            print(f"  ⚠️  位置を{self.MIT_POS_MIN}〜{self.MIT_POS_MAX}の範囲にクリップします")
            current_position = max(self.MIT_POS_MIN, min(self.MIT_POS_MAX, current_position))
        
        # 3. 移動時間を設定
        print("\n[4/8] 移動時間設定")
        print(f"  移動時間: {duration:.2f} 秒")
        
        # 4. 制御周期を設定
        print("\n[5/8] 制御周期設定")
        control_period = 1.0 / control_freq
        print(f"  制御周期: {control_freq:.1f} Hz ({control_period*1000:.2f} ms)")
        
        # 5. 制御ループ用の変数を初期化
        print("\n[6/8] 制御ループ準備")
        self.control_duration = duration
        self.control_start_position = current_position
        self.control_target_position = target_position
        self.control_kp = kp
        self.control_kd = kd
        self.control_last_print_time = time.time()
        print(f"  開始位置: {current_position:.4f} rad ({math.degrees(current_position):.2f} deg)")
        print(f"  目標位置: {target_position:.4f} rad ({math.degrees(target_position):.2f} deg)")
        print(f"  総移動量: {target_position - current_position:.4f} rad ({math.degrees(target_position - current_position):.2f} deg)")
        
        # 6. 制御更新関数を定義
        def control_update():
            elapsed = time.time() - self.control_start_time
            progress = min(1.0, elapsed / self.control_duration)
            
            # 線形補間で目標位置を計算
            position_cmd = self.control_start_position + \
                          (self.control_target_position - self.control_start_position) * progress
            
            # MIT制御コマンド送信
            self.send_mit_command(
                position=position_cmd,
                velocity=0.0,
                kp=self.control_kp,
                kd=self.control_kd,
                torque=0.0,
                debug=False
            )
            
            # 1秒ごとに進捗表示
            current_time = time.time()
            if current_time - self.control_last_print_time >= 1.0:
                progress_pct = progress * 100
                print(f"  進捗: {progress_pct:5.1f}% | "
                      f"位置指令: {position_cmd:7.4f} rad ({math.degrees(position_cmd):7.2f} deg) | "
                      f"経過時間: {elapsed:5.2f}s")
                self.control_last_print_time = current_time
        
        # 7. まず現在位置を送信して安定化
        print(f"\n[7/8] MIT制御実行中（RecurrentTimer使用）...")
        print(f"  制御パラメータ: KP={kp:.1f} Nm/rad, KD={kd:.2f} Nm/(rad/s)")
        # print(f"  初期位置を送信して安定化中...")
        
        # 現在位置を数回送信（モーターを現在位置に保持）
        # for i in range(5):
        #     self.send_mit_command(
        #         position=current_position,
        #         velocity=0.0,
        #         kp=kp,
        #         kd=kd,
        #         torque=0.0,
        #         debug=(i == 0)  # 最初の1回だけデバッグ出力
        #     )
        #     time.sleep(0.01)
        
        print(f"  制御ループ開始...")
        print()
        
        # 制御開始時刻を記録（ここで初めて記録！）
        self.control_start_time = time.time()
        
        control_timer = RecurrentTimer(control_period, control_update)
        control_timer.start()
        
        # 制御が完了するまで待機
        start_time = time.time()
        while time.time() - start_time < duration:
            time.sleep(0.1)
        
        # 制御ループを停止
        control_timer.stop()
        
        # 最終位置を目標位置に固定（数回送信）
        print(f"\n  最終位置を目標位置に固定中...")
        for _ in range(10):
            self.send_mit_command(
                position=target_position,
                velocity=0.0,
                kp=kp,
                kd=kd,
                torque=0.0,
                debug=False
            )
            time.sleep(0.01)
        
        # 8. 完了後の位置確認
        print(f"\n[8/8] 完了後の位置確認...")
        time.sleep(0.1)
        shadow_count, count_in_cpr = self.get_encoder_count()
        final_position = self.encoder_count_to_radians(shadow_count, count_in_cpr)
        
        elapsed_time = time.time() - start_time
        print(f"\n完了!")
        print(f"  実行時間: {elapsed_time:.2f} 秒")
        print(f"  最終位置指令: {target_position:.4f} rad ({math.degrees(target_position):.2f} deg)")
        print(f"  実際の最終位置: {final_position:.4f} rad ({math.degrees(final_position):.2f} deg)")
        print(f"  位置誤差: {abs(final_position - target_position):.4f} rad ({math.degrees(abs(final_position - target_position)):.2f} deg)")
        print("=" * 60)
    
    def close(self):
        """CANバスをクローズ"""
        if hasattr(self, 'bus'):
            self.bus.shutdown()
            print("CAN接続をクローズしました")


def main():
    """メイン関数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='MIT制御によるモーター原点復帰')
    parser.add_argument('--node-id', type=int, default=1, help='モーターのCANノードID (デフォルト: 1)')
    parser.add_argument('--can', type=str, default='can0', help='CANインターフェース名 (デフォルト: can0)')
    parser.add_argument('--duration', type=float, default=5.0, help='移動時間[秒] (デフォルト: 5.0)')
    parser.add_argument('--freq', type=float, default=100.0, help='制御周波数[Hz] (デフォルト: 100.0)')
    parser.add_argument('--kp', type=float, default=0.85, help='位置ゲイン[Nm/rad] (デフォルト: 0.85)')
    parser.add_argument('--kd', type=float, default=0.03, help='速度ゲイン[Nm/(rad/s)] (デフォルト: 0.03)')
    
    args = parser.parse_args()
    
    try:
        # コントローラー初期化
        controller = MITMotorController(
            node_id=args.node_id,
            can_channel=args.can
        )
        
        # 原点復帰実行
        controller.rotate_to_zero(
            duration=args.duration,
            control_freq=args.freq,
            kp=args.kp,
            kd=args.kd
        )
        
    except KeyboardInterrupt:
        print("\n\n中断されました")
    except Exception as e:
        print(f"\nエラー: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'controller' in locals():
            controller.close()


if __name__ == "__main__":
    main()
