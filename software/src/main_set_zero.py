#!/usr/bin/env python3
"""
GIM6010-8モータの仮想原点設定スクリプト

現在のモータ位置を仮想原点として設定するためのユーティリティスクリプト。
複数のモータに対して個別または一括で仮想原点を設定できます。

Usage:
    # 単一モータの仮想原点設定
    python main_set_zero.py --node-id 1
    
    # 複数モータの仮想原点設定
    python main_set_zero.py --node-id 1 2 3
    
    # 全モータの仮想原点設定（確認付き）
    python main_set_zero.py --all

Author: BSL Droid Project
Date: 2025-09-26
"""

import argparse
import time
import sys
import math
from typing import List
import logging

from gim6010_controller import GIM6010Controller

# ログ設定
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


def set_virtual_origin_single(node_id: int, can_interface: str = 'can0') -> bool:
    """
    単一モータの仮想原点を設定
    
    Args:
        node_id: モータのnode ID
        can_interface: CANインターフェース名
        
    Returns:
        設定成功時True、失敗時False
    """
    print(f"\n=== Setting Virtual Origin for Motor {node_id} ===")
    
    # コントローラーを初期化
    controller = GIM6010Controller(node_id=node_id, can_interface=can_interface)
    
    try:
        # 初期化
        if not controller.initialize():
            print(f"❌ Failed to initialize controller for motor {node_id}")
            return False
        
        print(f"✅ Controller initialized for motor {node_id}")
        
        # 現在位置を取得して表示
        print("📍 Getting current position...")
        encoder_data = controller.get_encoder_estimates()
        
        if encoder_data is None:
            print(f"❌ Failed to get current position for motor {node_id}")
            return False
        
        current_pos_rad, current_vel_rad = encoder_data
        current_pos_deg = math.degrees(current_pos_rad)
        
        print(f"   Current Position: {current_pos_rad:.3f} rad ({current_pos_deg:.1f}°)")
        print(f"   Current Velocity: {current_vel_rad:.3f} rad/s")
        
        # ユーザー確認
        response = input(f"\n🤔 Set this position as virtual origin for motor {node_id}? [y/N]: ")
        if response.lower() not in ['y', 'yes']:
            print("❌ Operation cancelled by user")
            return False
        
        # 仮想原点を設定
        print("🎯 Setting virtual origin...")
        if controller.set_virtual_origin():
            print(f"✅ Virtual origin set successfully for motor {node_id}")
            
            # 設定後の確認
            time.sleep(0.2)
            new_encoder_data = controller.get_encoder_estimates()
            if new_encoder_data:
                new_pos_rad, _ = new_encoder_data
                new_pos_deg = math.degrees(new_pos_rad)
                print(f"   New Position: {new_pos_rad:.3f} rad ({new_pos_deg:.1f}°)")
            
            return True
        else:
            print(f"❌ Failed to set virtual origin for motor {node_id}")
            return False
            
    except KeyboardInterrupt:
        print(f"\n❌ Operation interrupted by user for motor {node_id}")
        return False
    except Exception as e:
        print(f"❌ Error occurred for motor {node_id}: {e}")
        return False
    finally:
        controller.shutdown()


def set_virtual_origin_multiple(node_ids: List[int], can_interface: str = 'can0') -> bool:
    """
    複数モータの仮想原点を設定
    
    Args:
        node_ids: モータのnode IDリスト
        can_interface: CANインターフェース名
        
    Returns:
        全て成功時True、一つでも失敗時False
    """
    print(f"\n=== Setting Virtual Origin for Multiple Motors {node_ids} ===")
    
    # 全モータの現在位置を先に確認
    controllers = []
    motor_positions = {}
    
    print("📍 Checking current positions of all motors...")
    
    for node_id in node_ids:
        controller = GIM6010Controller(node_id=node_id, can_interface=can_interface)
        try:
            if not controller.initialize():
                print(f"❌ Failed to initialize controller for motor {node_id}")
                continue
                
            encoder_data = controller.get_encoder_estimates()
            if encoder_data is None:
                print(f"❌ Failed to get position for motor {node_id}")
                continue
            
            current_pos_rad, current_vel_rad = encoder_data
            current_pos_deg = math.degrees(current_pos_rad)
            
            motor_positions[node_id] = (current_pos_rad, current_pos_deg)
            controllers.append(controller)
            
            print(f"   Motor {node_id}: {current_pos_rad:.3f} rad ({current_pos_deg:.1f}°)")
            
        except Exception as e:
            print(f"❌ Error checking motor {node_id}: {e}")
    
    if not motor_positions:
        print("❌ No motors available for virtual origin setting")
        return False
    
    # ユーザー確認
    print(f"\n🤔 Set virtual origins for {len(motor_positions)} motors? [y/N]: ")
    response = input()
    if response.lower() not in ['y', 'yes']:
        print("❌ Operation cancelled by user")
        for controller in controllers:
            controller.shutdown()
        return False
    
    # 仮想原点設定を実行
    success_count = 0
    print("\n🎯 Setting virtual origins...")
    
    for controller in controllers:
        try:
            if controller.set_virtual_origin():
                print(f"✅ Motor {controller.node_id}: Virtual origin set successfully")
                success_count += 1
            else:
                print(f"❌ Motor {controller.node_id}: Failed to set virtual origin")
        except Exception as e:
            print(f"❌ Motor {controller.node_id}: Error - {e}")
        finally:
            controller.shutdown()
    
    print(f"\n📊 Results: {success_count}/{len(motor_positions)} motors completed successfully")
    return success_count == len(motor_positions)


def main():
    """メイン関数"""
    parser = argparse.ArgumentParser(
        description='Set virtual origin for GIM6010-8 motors',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --node-id 1                 # Set virtual origin for motor 1
  %(prog)s --node-id 1 2 3             # Set virtual origins for motors 1, 2, 3
  %(prog)s --all                       # Set virtual origins for all motors (1-6)
  %(prog)s --node-id 2 --interface can1  # Use specific CAN interface
        """
    )
    
    parser.add_argument(
        '--node-id', 
        type=int, 
        nargs='+', 
        help='Motor node IDs (1-63)'
    )
    
    parser.add_argument(
        '--all', 
        action='store_true', 
        help='Set virtual origins for all motors (node IDs 1-6)'
    )
    
    parser.add_argument(
        '--interface', 
        default='can0', 
        help='CAN interface name (default: can0)'
    )
    
    parser.add_argument(
        '--verbose', '-v', 
        action='store_true', 
        help='Enable verbose logging'
    )
    
    args = parser.parse_args()
    
    # ログレベル設定
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # 引数チェック
    if args.all and args.node_id:
        print("❌ Error: Cannot use --all and --node-id together")
        sys.exit(1)
    
    if not args.all and not args.node_id:
        print("❌ Error: Must specify either --node-id or --all")
        parser.print_help()
        sys.exit(1)
    
    # node IDsの決定
    if args.all:
        node_ids = list(range(1, 7))  # 1-6
        print("🤖 All motors mode: Will set virtual origins for motors 1-6")
    else:
        node_ids = args.node_id
        # node ID範囲チェック
        for node_id in node_ids:
            if not (1 <= node_id <= 63):
                print(f"❌ Error: Node ID {node_id} out of range (1-63)")
                sys.exit(1)
    
    print(f"🔧 Using CAN interface: {args.interface}")
    print(f"🎯 Target motors: {node_ids}")
    
    try:
        # 実行
        if len(node_ids) == 1:
            success = set_virtual_origin_single(node_ids[0], args.interface)
        else:
            success = set_virtual_origin_multiple(node_ids, args.interface)
        
        if success:
            print("\n🎉 All operations completed successfully!")
            sys.exit(0)
        else:
            print("\n❌ Some operations failed. Please check the logs.")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n❌ Operation interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
