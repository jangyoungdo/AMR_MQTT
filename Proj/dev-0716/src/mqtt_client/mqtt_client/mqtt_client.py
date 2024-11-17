import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
from std_msgs.msg import Float64MultiArray


class MqttClientNode(Node):
    def __init__(self):
        super().__init__('mqtt_client_node')

        # MQTT 브로커 설정
        self.declare_parameter('broker.ip', '192.168.192.155')  # 브로커 IP
        self.declare_parameter('broker.port', 1883)
        self.declare_parameter('broker.topic', 'robot/status')

        broker_ip = self.get_parameter('broker.ip').get_parameter_value().string_value
        broker_port = self.get_parameter('broker.port').get_parameter_value().integer_value
        self.broker_topic = self.get_parameter('broker.topic').get_parameter_value().string_value

        # MQTT 클라이언트 초기화
        self.client = mqtt.Client()
        try:
            self.client.connect(broker_ip, broker_port, keepalive=60)
            self.get_logger().info(f"Connected to MQTT broker at {broker_ip}:{broker_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {e}")
            raise e

        self.client.loop_start()

        # ROS 구독자 생성
        self.sub_robot_status = self.create_subscription(
            Float64MultiArray,
            'robot/status',
            self.callback_robot_status,
            10
        )

    def callback_robot_status(self, msg):
        """ROS 메시지를 수신하고 데이터를 MQTT로 송신합니다."""
        try:
            # 메시지 배열의 개수 확인
            expected_length = 17  # 송신부 데이터 필드 개수
            data_length = len(msg.data)

            # 예상 개수와 일치하지 않으면 경고 로그 출력
            if data_length != expected_length:
                self.get_logger().warning(f"Received data length {data_length}, but expected {expected_length}: {msg.data}")
                return

            # 메시지 데이터를 매핑
            status_data = {
                "x": msg.data[0],                       # Position X
                "y": msg.data[1],                       # Position Y
                "theta": msg.data[2],                   # Orientation (Theta)
                "l_enc": int(msg.data[3]),              # Left encoder
                "r_enc": int(msg.data[4]),              # Right encoder
                "odo_l": msg.data[5],                   # Left wheel odometry
                "odo_r": msg.data[6],                   # Right wheel odometry
                "yaw": msg.data[7],                     # IMU Yaw
                "pitch": msg.data[8],                   # IMU Pitch
                "roll": msg.data[9],                    # IMU Roll
                "orientation_x": msg.data[10],          # IMU Orientation X
                "orientation_y": msg.data[11],          # IMU Orientation Y
                "orientation_z": msg.data[12],          # IMU Orientation Z
                "orientation_w": msg.data[13],          # IMU Orientation W
                "gear_ratio": msg.data[14],             # Gear ratio
                "wheel_separation": msg.data[15],       # Wheel separation
                "wheel_radius": msg.data[16]            # Wheel radius
            }

            # JSON 데이터로 변환
            json_data = json.dumps(status_data)

            # MQTT로 데이터 전송
            self.client.publish(self.broker_topic, json_data)
            self.get_logger().info(f"Published to MQTT: {json_data}")

        except IndexError as e:
            self.get_logger().error(f"Index error in received message: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

    def destroy_node(self):
        """Node 종료 시 MQTT 클라이언트를 정리합니다."""
        self.client.loop_stop()
        self.client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MqttClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
