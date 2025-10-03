import can
import struct
import time
import threading
import math
import sys
import json
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
from matplotlib.animation import FuncAnimation
import queue

# Zero offsets loader for MIT position correction
def load_zero_offsets(filename="default_count_on_zero.json"):
    try:
        with open(filename, 'r') as f:
            data = json.load(f)
        if isinstance(data, dict):
            # flat mapping node->count
            if 'motors' in data and isinstance(data['motors'], dict):
                out = {}
                for k, v in data['motors'].items():
                    try:
                        out[int(k)] = int(v.get('shadow_count', 0)) if isinstance(v, dict) else int(v)
                    except Exception:
                        continue
                return out
            # flat int values
            if all(isinstance(v, int) for v in data.values()):
                return {int(k): v for k, v in data.items()}
    except FileNotFoundError:
        return {}
    except json.JSONDecodeError:
        print(f"Warning: Invalid JSON in '{filename}'")
        return {}

# load once
_ZERO_OFFSETS = load_zero_offsets()

# ノードID設定
NODE_ID = 15#0x01

# GIM6010-8のCPR（1回転あたりのカウント数）
CPR = 16384*8 #15036*8

# 監視対象のCAN ID
CMD_ID_ENCODER_ESTIMATE = 0x00A
EXPECTED_CAN_ID = (NODE_ID << 5) + CMD_ID_ENCODER_ESTIMATE

# CAN ID生成関数（ノードID << 5 + CMD_ID）
def can_id(cmd_id, node_id):
    return (node_id << 5) + cmd_id

# Heartbeatメッセージのデコード（ファームウェア v0.5.13以降）
def decode_heartbeat(msg):
    if len(msg.data) < 8:
        return None
    axis_error = struct.unpack('<I', msg.data[0:4])[0]
    axis_state = msg.data[4]
    flags = msg.data[5]
    life = msg.data[7]
    return axis_error, axis_state, flags, life

# エンコーダカウント解析関数
def parse_encoder_count(msg):
    if msg.dlc >= 8:  # データ長が8バイト以上あることを確認
        try:
            shadow_count, = struct.unpack('<i', msg.data[:4])
            count_in_cpr, = struct.unpack('<i', msg.data[4:8])
            return shadow_count, count_in_cpr
        except struct.error as e:
            print("Parse Error:", e)
    return None, None

# MIT制御用コマンド生成
def mit_control(node_id, position, velocity, kp, kd, torque):
    pos_int = int((position + 12.5) * 65535 / 25)
    vel_int = int((velocity + 65) * 4095 / 130)
    kp_int = int(kp * 4095 / 500)
    kd_int = int(kd * 4095 / 5)
    torque_int = int((torque + 50) * 4095 / 100)

    data = bytearray(8)
    data[0] = (pos_int >> 8) & 0xFF
    data[1] = pos_int & 0xFF
    data[2] = (vel_int >> 4) & 0xFF
    data[3] = ((vel_int & 0xF) << 4) | ((kp_int >> 8) & 0xF)
    data[4] = kp_int & 0xFF
    data[5] = (kd_int >> 4) & 0xFF
    data[6] = ((kd_int & 0xF) << 4) | ((torque_int >> 8) & 0xF)
    data[7] = torque_int & 0xFF

    return can.Message(arbitration_id=can_id(0x008, node_id), data=data, is_extended_id=False)

# 制御モード設定（MIT制御モード）
def set_mit_mode(node_id, bus):
    control_mode = 3  # 位置制御モード
    input_mode = 9    # MITモード
    data = struct.pack('<II', control_mode, input_mode)
    msg = can.Message(arbitration_id=can_id(0x00B, node_id), data=data, is_extended_id=False)
    try:
        bus.send(msg)
        print("MIT制御モードの設定コマンドを送信しました。")
    except can.CanError as e:
        print("MIT制御モード設定送信エラー:", e)

# IDLE状態に設定する関数（CMD ID 0x007）
def set_idle_state(bus, node_id):
    requested_state = 1  # AXIS_STATE_IDLE
    data = struct.pack('<I', requested_state)
    msg = can.Message(arbitration_id=can_id(0x007, node_id), data=data, is_extended_id=False)
    bus.send(msg)
    print("IDLE状態に設定しました。")

# 閉ループ状態へ移行
def enter_closed_loop(node_id, bus):
    requested_state = 8  # Closed Loop Control
    data = struct.pack('<I', requested_state)
    msg = can.Message(arbitration_id=can_id(0x007, node_id), data=data, is_extended_id=False)
    try:
        bus.send(msg)
        print("閉ループ状態への移行コマンドを送信しました。")
    except can.CanError as e:
        print("閉ループ状態送信エラー:", e)

# 現在のエンコーダ位置（pos_estimateとvel_estimate）を取得する（CMD ID 0x009）
def get_encoder_estimates(bus, node_id):
    req_msg = can.Message(arbitration_id=can_id(0x009, node_id), data=[], is_extended_id=False)
    bus.send(req_msg)
    msg = bus.recv(timeout=1.0)
    if msg is None:
        raise TimeoutError("CAN応答タイムアウト（エンコーダ位置取得）")
    pos, vel = struct.unpack('<ff', msg.data)
    return pos, vel

# 線形補間関数: 開始値から終了値まで指定した時間で線形に変化する値を計算
def linear_interpolate(start_val, end_val, progress):
    """
    progress: 0.0から1.0の間の値、補間の進行度
    """
    return start_val + (end_val - start_val) * progress

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

# グローバル変数で状態を管理
g_running = True
g_current_target = 0.0      # 現在の目標角度（ラジアン）
g_current_position = 0.0    # 現在の実際の角度（ラジアン）
g_kp = 0.85              # 位置ゲイン
g_kd = 0.03              # ダンピング用ゲイン
g_velocity = 0.0            # 目標速度 (rad/s)
g_torque = 0.0              # フィードフォワードトルク

# 補間用のパラメータ
g_target_angles_deg = [0, 120, 0, -120, 0]  # 目標角度リスト（度）
g_target_angles_rad = [math.radians(angle) for angle in g_target_angles_deg]  # ラジアンに変換
g_current_index = 0         # 現在の目標角度インデックス
g_next_index = 1            # 次の目標角度インデックス
g_start_angle = g_target_angles_rad[0]  # 開始角度
g_end_angle = g_target_angles_rad[1]    # 終了角度
g_transition_start_time = 0  # 遷移開始時間
g_transition_duration = 0.1  # 遷移時間（秒）

# 初期化フェーズの状態フラグ
g_initialization_phase = True  # 初期化フェーズかどうかのフラグ
g_initial_transition_duration = 1.5  # 初期位置から最初の目標位置への遷移時間（秒）

# データ保存用のデータ構造
# スレッド間の安全なデータ交換のためのキュー
g_data_queue = queue.Queue(maxsize=1000)

# グラフ描画用のデータストア
g_time_data = deque(maxlen=500)    # 時間データ（最大500点）
g_target_data = deque(maxlen=500)  # 目標角度データ
g_actual_data = deque(maxlen=500)  # 実際の角度データ
g_start_time = time.time()         # 開始時間

# CAN受信スレッド - エンコーダ位置の更新
def can_receiver_thread(bus):
    global g_running, g_current_position
    
    print(f"CAN受信スレッド開始: Expected CAN ID: 0x{EXPECTED_CAN_ID:X}")
    
    try:
        while g_running:
            message = bus.recv(timeout=0.01)  # 短いタイムアウトでポーリング
            if message is None:
                continue
                
            # エンコーダカウントメッセージを解析
            if message.arbitration_id == EXPECTED_CAN_ID:
                shadow_count, count_in_cpr = parse_encoder_count(message)
                if shadow_count is not None:
                    # シャドウカウントから角度（ラジアン）に変換
                    g_current_position = (shadow_count / CPR) * 2 * math.pi
                    # print(f"現在位置: {math.degrees(g_current_position):.2f}度")
    except Exception as e:
        print(f"CAN受信スレッドエラー: {e}")
        if g_running:  # 通常終了時以外のエラーの場合
            g_running = False  # 他のスレッドも停止させる
    
    print("CAN受信スレッド終了")

# 初期位置からの遷移を待機する関数
def wait_for_initial_position_update(timeout=3.0):
    global g_current_position
    
    print("初期位置の取得を待機しています...")
    start_time = time.time()
    
    # 現在位置が更新されるまで待機（タイムアウト付き）
    while time.time() - start_time < timeout:
        time.sleep(0.1)
        if g_current_position != 0.0:  # 初期値から変更があった場合
            print(f"初期位置を検出: {math.degrees(g_current_position):.2f}度")
            return True
    
    print("警告: 初期位置の取得がタイムアウトしました。デフォルト値（0.0）を使用します。")
    return False

# 制御更新関数 - リカレントタイマーから呼ばれる
def control_update(bus):
    global g_current_target, g_start_angle, g_end_angle
    global g_current_index, g_next_index, g_transition_start_time
    global g_data_queue, g_initialization_phase
    
    current_time = time.time()
    elapsed_total = current_time - g_start_time
    
    # 初期化フェーズか通常の制御サイクルかで分岐
    if g_initialization_phase:
        # 初期化フェーズでの補間（現在位置から最初の目標位置へ）
        elapsed = current_time - g_transition_start_time
        progress = min(1.0, elapsed / g_initial_transition_duration)
        
        # 現在の補間値を計算
        g_current_target = linear_interpolate(g_start_angle, g_end_angle, progress)
        
        # 初期化フェーズが完了したら通常サイクルへ移行
        if progress >= 1.0:
            g_initialization_phase = False
            g_current_index = 0
            g_next_index = 1
            g_start_angle = g_target_angles_rad[g_current_index]
            g_end_angle = g_target_angles_rad[g_next_index]
            g_transition_start_time = current_time
            print(f"初期化フェーズが完了しました。通常の制御サイクルを開始します。")
    else:
        # 通常の補間進行度を計算（0.0～1.0の範囲）
        elapsed = current_time - g_transition_start_time
        progress = min(1.0, elapsed / g_transition_duration)
        
        # 現在の補間値を計算
        g_current_target = linear_interpolate(g_start_angle, g_end_angle, progress)
        
        # 進行度が1.0に達したら次の目標に更新
        if progress >= 1.0:
            g_current_index = g_next_index
            g_next_index = (g_next_index + 1) % len(g_target_angles_rad)
            g_start_angle = g_target_angles_rad[g_current_index]
            g_end_angle = g_target_angles_rad[g_next_index]
            g_transition_start_time = current_time
            print(f"次の目標角度へ更新: {math.degrees(g_start_angle):.2f}度 → {math.degrees(g_end_angle):.2f}度")
    
    # データポイントをキューに追加（グラフ描画スレッドに渡すため）
    try:
        # キューが満杯の場合は古いデータを捨てる（非ブロッキング）
        if g_data_queue.full():
            try:
                g_data_queue.get_nowait()
            except queue.Empty:
                pass
        
        # 新しいデータポイントを追加
        g_data_queue.put_nowait((
            elapsed_total,
            math.degrees(g_current_target),
            math.degrees(g_current_position)
        ))
    except queue.Full:
        pass  # キューが満杯ならスキップ（グラフよりも制御を優先）
    
    # MIT制御コマンド送信
    # ここでゼロオフセットがあれば位置指令を補正する
    try:
        stored_zero = _ZERO_OFFSETS.get(NODE_ID, 0)
    except Exception:
        stored_zero = 0

    # g_current_target はラジアン -> 回転数 (turns)
    turns = g_current_target / (2.0 * math.pi)
    offset_turns = 0.0
    if stored_zero:
        offset_turns = stored_zero / float(CPR)

    corrected_turns = turns - offset_turns
    # MIT に渡す position はここではラジアンと仮定して戻す
    corrected_position = corrected_turns * (2.0 * math.pi)

    msg = mit_control(NODE_ID, corrected_position, g_velocity, g_kp, g_kd, g_torque)
    #msg = mit_control(NODE_ID, g_current_target, g_velocity, g_kp, g_kd, g_torque)
    try:
        bus.send(msg)
    except can.CanError as e:
        print(f"CAN送信エラー: {e}")

# グラフ描画スレッド（別スレッドでMatplotlibを実行）
def plotting_thread():
    global g_running, g_data_queue, g_time_data, g_target_data, g_actual_data
    
    print("グラフ描画スレッド開始")
    
    # Matplotlibのグラフ設定
    plt.ion()  # インタラクティブモードを有効化
    fig, ax = plt.subplots(figsize=(10, 6))
    target_line, = ax.plot([], [], 'r-', label='Target', linewidth=2)
    actual_line, = ax.plot([], [], 'b-', label='Sensor', linewidth=2)
    
    ax.set_title('Position Control Demo')
    ax.set_xlabel('time [s]')
    ax.set_ylabel('angle [deg]')
    ax.set_xlim(0, 10)
    ax.set_ylim(-180, 180)
    ax.grid(True)
    ax.legend()
    
    plt.tight_layout()
    
    try:
        while g_running:
            # キューからデータを取得して描画用バッファに追加
            try:
                # データを最大100個まとめて取得（バースト処理）
                for _ in range(100):
                    if g_data_queue.empty():
                        break
                    
                    t, target, actual = g_data_queue.get_nowait()
                    g_time_data.append(t)
                    g_target_data.append(target)
                    g_actual_data.append(actual)
                    g_data_queue.task_done()
            except queue.Empty:
                pass
            
            # データが空の場合はスキップ
            if not g_time_data:
                time.sleep(0.1)
                continue
            
            # リストに変換
            x_data = list(g_time_data)
            target_y_data = list(g_target_data)
            actual_y_data = list(g_actual_data)
            
            # ラインの更新
            target_line.set_data(x_data, target_y_data)
            actual_line.set_data(x_data, actual_y_data)
            
            # X軸の範囲を自動調整
            if x_data:
                xmin = max(0, x_data[-1] - 10)  # 直近10秒分を表示
                xmax = x_data[-1] + 0.5
                ax.set_xlim(xmin, xmax)
                
                # Y軸の範囲も自動調整（データの最小値と最大値に少し余裕を持たせる）
                all_y_data = target_y_data + actual_y_data
                if all_y_data:
                    ymin = min(all_y_data) - 10
                    ymax = max(all_y_data) + 10
                    ax.set_ylim(ymin, ymax)
            
            # グラフを更新（制御ループと比べて低い更新レート）
            try:
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
            except Exception as e:
                print(f"グラフ描画エラー: {e}")
                
            # 一定時間スリープ（制御ループを妨げない程度に）
            time.sleep(0.1)  # 10Hzでグラフ更新
            
    except Exception as e:
        print(f"グラフ描画スレッドエラー: {e}")
    
    print("グラフ描画スレッド終了")
    
    # Matplotlibのクリーンアップ
    try:
        plt.close(fig)
        plt.close('all')
    except:
        pass

# クリーンアップ関数
def cleanup():
    global g_running
    g_running = False
    print("終了処理を開始します...")
    
    # タイマーを停止
    if 'control_timer' in globals() and control_timer:
        control_timer.stop()
    
    # IDLE状態に設定
    try:
        set_idle_state(bus, NODE_ID)
        time.sleep(0.1)
    except Exception as e:
        print(f"終了時のIDLE設定エラー: {e}")
    
    # CANバスをシャットダウン
    try:
        bus.shutdown()
    except Exception as e:
        print(f"CANバスシャットダウンエラー: {e}")
    
    # スレッドの終了を待機
    print("スレッドの終了を待機しています...")
    if 'receiver_thread' in globals() and receiver_thread:
        receiver_thread.join(timeout=1.0)
    if 'plot_thread' in globals() and plot_thread:
        plot_thread.join(timeout=1.0)
    
    print("終了処理が完了しました。")

# メインの制御処理
if __name__ == '__main__':
    try:
        bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
    except Exception as e:
        print("CANバス初期化エラー:", e)
        sys.exit(1)

    # 起動時、まずはIDLE状態に設定
    set_idle_state(bus, NODE_ID)
    time.sleep(0.5)
    
    # MIT制御モード設定と閉ループ状態への移行をCAN経由で実行
    set_mit_mode(NODE_ID, bus)
    time.sleep(0.1)
    enter_closed_loop(NODE_ID, bus)
    time.sleep(0.1)
    
    # CAN受信スレッドの開始
    receiver_thread = threading.Thread(target=can_receiver_thread, args=(bus,), daemon=True)
    receiver_thread.start()
    
    # 現在の位置情報が更新されるまで待機
    wait_for_initial_position_update()
    
    # 初期化フェーズのパラメータ設定
    g_initialization_phase = True
    g_start_angle = g_current_position  # 現在位置を開始位置として設定
    g_end_angle = g_target_angles_rad[0]  # 最初の目標位置
    g_transition_start_time = time.time()
    g_start_time = time.time()
    
    print(f"初期化フェーズを開始します: 現在位置 {math.degrees(g_start_angle):.2f}度 → 目標位置 {math.degrees(g_end_angle):.2f}度")
    print(f"初期化の遷移時間: {g_initial_transition_duration:.1f}秒")
    
    # グラフ描画スレッドの開始
    plot_thread = threading.Thread(target=plotting_thread, daemon=True)
    plot_thread.start()
    
    # 制御更新周期（秒）
    update_interval = 0.01
    
    # 改良したリカレントタイマーを開始
    control_timer = RecurrentTimer(update_interval, control_update, bus)
    control_timer.start()
    
    print(f"制御開始: 更新周期 {update_interval*1000:.1f}ms")
    
    try:
        # メインスレッドは終了シグナルを待機
        while g_running:
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("キーボードによる中断を検出しました。")
    finally:
        cleanup()