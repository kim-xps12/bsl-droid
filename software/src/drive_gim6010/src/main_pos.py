#!/usr/bin/env python3
"""
GIM6010-8モータ制御デモアプリケーション

要件に従って以下の動作を実行します:
0度→90度→0度→-90度→0度

Author: BSL Droid Project
Date: 2025-09-26
"""

import time
import sys
import signal
import logging
from pathlib import Path

# srcディレクトリをPythonパスに追加
src_dir = Path(__file__).parent
sys.path.insert(0, str(src_dir))

from gim6010_controller import GIM6010Controller

# ログ設定
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)


class MotorControlDemo:
    """モータ制御デモクラス"""
    
    def __init__(self, node_id: int = 1, can_interface: str = 'can0'):
        """
        デモの初期化
        
        Args:
            node_id: モータのnode ID
            can_interface: CANインターフェース名
        """
        self.controller = GIM6010Controller(node_id=node_id, can_interface=can_interface)
        self.is_running = False
        
        # シグナルハンドラの設定（Ctrl+Cで安全に終了）
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """シグナルハンドラ（安全な終了処理）"""
        logger.info("Shutdown signal received. Stopping motor safely...")
        self.is_running = False
    
    def run_position_sequence(self, wait_time: float = 3.0, use_mit_control: bool = True):
        """
        位置制御シーケンスを実行: 0度→90度→0度→-90度→0度
        
        Args:
            wait_time: 各位置での待機時間 [秒]
            use_mit_control: MIT制御を使用するかどうか
        """
        # 動作シーケンス定義
        target_positions = [0.0, 90.0, 0.0, -90.0, 0.0]
        
        logger.info("=== GIM6010-8 Motor Position Control Demo ===")
        logger.info(f"Target sequence: {' → '.join(f'{pos}°' for pos in target_positions)}")
        logger.info(f"Wait time per position: {wait_time} seconds")
        logger.info(f"Control method: {'MIT Control' if use_mit_control else 'Standard Position Control'}")
        
        # コントローラーの初期化
        if not self.controller.initialize():
            logger.error("Failed to initialize motor controller")
            return False
        
        self.is_running = True
        
        try:
            logger.info("Starting position sequence...")
            
            for i, target_angle in enumerate(target_positions):
                if not self.is_running:
                    logger.info("Sequence interrupted by user")
                    break
                
                logger.info(f"Step {i+1}/{len(target_positions)}: Moving to {target_angle}°")
                
                # 目標位置にモータを移動
                success = self.controller.send_position_degree(
                    target_angle_deg=target_angle,
                    velocity_ff=0.0,
                    torque_ff=0.0,
                    use_mit_control=use_mit_control
                )
                
                if not success:
                    logger.error(f"Failed to send position command for {target_angle}°")
                    break
                
                # 指定時間待機（到達を待つ）
                logger.info(f"Waiting {wait_time} seconds for motor to reach target position...")
                
                # 待機中も中断信号をチェック
                for _ in range(int(wait_time * 10)):
                    if not self.is_running:
                        break
                    time.sleep(0.1)
                
                # エンコーダ値を取得（可能であれば）
                encoder_data = self.controller.get_encoder_estimates()
                if encoder_data:
                    current_pos_rad, current_vel_rad = encoder_data
                    current_pos_deg = current_pos_rad * 180.0 / 3.14159
                    logger.info(f"Current position: {current_pos_deg:.1f}° ({current_pos_rad:.3f} rad), "
                              f"velocity: {current_vel_rad:.3f} rad/s")
                
                logger.info(f"Step {i+1} completed\n")
            
            if self.is_running:
                logger.info("=== Position sequence completed successfully ===")
            
        except Exception as e:
            logger.error(f"Error during position sequence: {e}")
            return False
        
        finally:
            # 安全にモータを停止
            logger.info("Stopping motor and shutting down...")
            self.controller.shutdown()
        
        return True
    
    def run_continuous_demo(self, cycle_time: float = 10.0):
        """
        連続デモモード（シーケンスを繰り返し実行）
        
        Args:
            cycle_time: 1サイクルの実行時間 [秒]
        """
        logger.info("=== Continuous Demo Mode ===")
        logger.info("Press Ctrl+C to stop the demo")
        
        cycle_count = 1
        
        while self.is_running:
            logger.info(f"\n--- Cycle {cycle_count} ---")
            
            if not self.run_position_sequence(wait_time=cycle_time/5):
                break
            
            cycle_count += 1
            
            if self.is_running:
                logger.info(f"Cycle {cycle_count-1} completed. Starting next cycle in 2 seconds...")
                time.sleep(2)
        
        logger.info("Continuous demo stopped")


def main():
    """メイン関数"""
    # コマンドライン引数の簡易解析
    import argparse
    
    parser = argparse.ArgumentParser(description='GIM6010-8 Motor Control Demo')
    parser.add_argument('--node-id', type=int, default=1, 
                       help='Motor CAN node ID (default: 1)')
    parser.add_argument('--can-interface', type=str, default='can0',
                       help='CAN interface name (default: can0)')
    parser.add_argument('--wait-time', type=float, default=3.0,
                       help='Wait time per position in seconds (default: 3.0)')
    parser.add_argument('--continuous', action='store_true',
                       help='Run in continuous mode (repeat sequence)')
    parser.add_argument('--standard-control', action='store_true',
                       help='Use standard position control instead of MIT control')
    
    args = parser.parse_args()
    
    # デモの実行
    demo = MotorControlDemo(node_id=args.node_id, can_interface=args.can_interface)
    
    try:
        if args.continuous:
            demo.run_continuous_demo(cycle_time=args.wait_time * 5)
        else:
            demo.run_position_sequence(
                wait_time=args.wait_time, 
                use_mit_control=not args.standard_control
            )
    
    except KeyboardInterrupt:
        logger.info("Demo interrupted by user")
    
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    
    logger.info("Demo finished")


if __name__ == "__main__":
    main()
