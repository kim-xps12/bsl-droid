import can
import struct
import time

# CAN接続の設定（仕様書に従い socketcan を利用）
bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)

NODE_ID = 1  # デバイス側のnode_idに合わせて変更してください
CMD_ID_ENCODER_ESTIMATE = 0x00A
EXPECTED_CAN_ID = (NODE_ID << 5) + CMD_ID_ENCODER_ESTIMATE

def parse_encoder_count(msg):
    if msg.dlc >= 4:
        # デバッグ用に受信データをhex表示
        data_hex = ' '.join(f'{b:02X}' for b in msg.data)
        print(f"Received CAN ID: 0x{msg.arbitration_id:X}, Data: {data_hex}")
        try:
            shadow_count, = struct.unpack('<i', msg.data[:4])
            count_in_cpr = struct.unpack('<i', msg.data[4:])
            return shadow_count, count_in_cpr
        except struct.error as e:
            print("Parse Error:", e)
    return None

print("Expected CAN ID: 0x{:X}".format(EXPECTED_CAN_ID))

try:
    while True:
        message = bus.recv()
        if message is None:
            continue

        # 完全一致でCAN IDをチェック
        if message.arbitration_id == EXPECTED_CAN_ID:
            shadow_count, count_in_cpr = parse_encoder_count(message)
            #if count_in_cpr is not None:
            print(f"Count: {shadow_count}, out turns: {shadow_count / (16384*8):.2f}, in turns: {shadow_count / (16384):.2f}")
        #time.sleep(0.05)
except KeyboardInterrupt:
    print("Stopped")
finally:
    bus.shutdown()
