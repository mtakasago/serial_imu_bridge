#!/usr/bin/env python3
"""
155Hz版ベース + 気温除去対応
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
import serial
import threading
import time
import csv
import io
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class SerialIMUBridge155Hz(Node):

    def __init__(self):
        super().__init__('serial_imu_bridge_155hz')

        # パラメータ宣言
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)  # 155Hz時の設定
        self.declare_parameter('frame_id', 'imu')

        # パラメータ取得
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value

        # 155Hz時と同じQoS設定
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # パブリッシャー作成
        self.imu_publisher = self.create_publisher(
            Imu,
            '/imu',
            qos_profile
        )

        # デバッグ用パブリッシャー（155Hz時と同じ）
        self.accel_publisher = self.create_publisher(
            Vector3,
            '/imu/accel_raw',
            qos_profile
        )

        self.gyro_publisher = self.create_publisher(
            Vector3,
            '/imu/gyro_raw',
            qos_profile
        )

        # 155Hz時と同じシリアル設定
        try:
            self.serial_conn = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=0.01,
                rtscts=False,
                dsrdtr=False,
                xonxoff=False,
                write_timeout=0,
                inter_byte_timeout=None
            )
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            self.get_logger().info(f'Connected to {self.serial_port} at {self.baud_rate} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            return

        # データ受信スレッド開始
        self.running = True
        self.serial_thread = threading.Thread(target=self.serial_reader)
        self.serial_thread.daemon = True
        self.serial_thread.start()

        # 統計情報
        self.msg_count = 0
        self.error_count = 0
        self.last_stats_time = time.time()

        # 155Hz時と同じ統計表示間隔
        self.stats_timer = self.create_timer(10.0, self.print_stats)

        self.get_logger().info('Serial IMU Bridge 155Hz Base (no temp) started')

    def serial_reader(self):
        """155Hz時と同じシリアルデータ読み取り"""

        while self.running:
            try:
                if self.serial_conn.in_waiting > 0:
                    # 155Hz時と同じエラー処理
                    try:
                        data = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    except UnicodeDecodeError:
                        self.serial_conn.reset_input_buffer()
                        continue

                    # 空行やコメント行をスキップ
                    if not data or data.startswith('#') or len(data) < 10:
                        continue

                    # 気温なしなので6つのカンマをチェック
                    if data.count(',') != 6:  # 7つのフィールドなので6つのカンマ
                        self.error_count += 1
                        continue

                    # CSVデータパース
                    self.parse_and_publish(data)

            except Exception as e:
                self.get_logger().debug(f'Serial read error: {e}')
                time.sleep(0.001)

    def parse_and_publish(self, csv_line):
        """気温なし版のパース処理"""
        try:
            # CSVパース（気温フィールドなし）
            reader = csv.reader([csv_line])
            values = next(reader)

            if len(values) != 7:  # timestamp + 6つのIMU値
                return

            timestamp, gx, gy, gz, ax, ay, az = map(float, values)

            # ROS2メッセージ作成
            current_time = self.get_clock().now().to_msg()

            # IMUメッセージ
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = current_time
            imu_msg.header.frame_id = self.frame_id

            # orientationを無効化
            imu_msg.orientation.x = 0
            imu_msg.orientation.y = 0
            imu_msg.orientation.z = 0
            imu_msg.orientation.w = 0

            # 加速度データ（155Hz時と同じ座標変換）
            imu_msg.linear_acceleration.x = -ax
            imu_msg.linear_acceleration.y = -ay
            imu_msg.linear_acceleration.z = -az

            # 角速度データ
            imu_msg.angular_velocity.x = -gx
            imu_msg.angular_velocity.y = -gy
            imu_msg.angular_velocity.z = -gz

            # 共分散行列（155Hz時と同じ）
            imu_msg.linear_acceleration_covariance = [-1.0] * 9
            imu_msg.angular_velocity_covariance = [-1.0] * 9
            imu_msg.orientation_covariance = [-1.0] * 9

            # パブリッシュ
            self.imu_publisher.publish(imu_msg)

            # デバッグ用分離データ（155Hz時と同じ）
            accel_msg = Vector3()
            accel_msg.x, accel_msg.y, accel_msg.z = -ax, -ay, -az
            self.accel_publisher.publish(accel_msg)

            gyro_msg = Vector3()
            gyro_msg.x, gyro_msg.y, gyro_msg.z = -gx, -gy, gz
            self.gyro_publisher.publish(gyro_msg)

            self.msg_count += 1

        except Exception as e:
            self.get_logger().debug(f'Parse error: {e} for line: {csv_line}')

    def print_stats(self):
        """統計情報表示（155Hz時と同じ）"""
        current_time = time.time()
        time_diff = current_time - self.last_stats_time

        if time_diff > 0:
            freq = self.msg_count / time_diff
            self.get_logger().info(f'Publishing at {freq:.1f} Hz ({self.msg_count} messages, {self.error_count} errors in {time_diff:.1f}s)')

        self.msg_count = 0
        self.error_count = 0
        self.last_stats_time = current_time

    def destroy_node(self):
        """ノード終了処理"""
        self.running = False
        if hasattr(self, 'serial_conn'):
            self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    try:
        node = SerialIMUBridge155Hz()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
