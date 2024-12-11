import json
import paho.mqtt.client as mqtt
import boto3
from datetime import datetime
import uuid

# 설정 파일 로드
with open("config.json", "r") as config_file:
    config = json.load(config_file)

# AWS S3 설정
s3_config = config["aws"]
s3 = boto3.client(
    's3',
    aws_access_key_id=s3_config["access_key"],
    aws_secret_access_key=s3_config["secret_key"],
    region_name=s3_config["region"]
)
bucket_name = s3_config["bucket_name"]

# MQTT 브로커 설정
mqtt_config = config["mqtt"]
broker_ip = mqtt_config["broker_ip"]
broker_port = mqtt_config["broker_port"]
broker_topic = mqtt_config["broker_topic"]

# MQTT 메시지 수신 콜백 함수
def on_message(client, userdata, message):
    try:
        # JSON 메시지를 디코딩
        payload = message.payload.decode('utf-8')
        data = json.loads(payload)
        
        # 데이터 검증 및 필수 필드 확인
        required_fields = [
            "x", "y", "theta", "l_enc", "r_enc", "odo_l", "odo_r",
            "yaw", "pitch", "roll", "orientation_x", "orientation_y",
            "orientation_z", "orientation_w", "gear_ratio", "wheel_separation", "wheel_radius"
        ]
        for field in required_fields:
            if field not in data:
                print(f"Missing field in JSON: {field}")
                return

        # S3에 저장할 데이터 준비
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        unique_id = str(uuid.uuid4())  # 고유 파일 이름
        s3_key = f"robot_data/{timestamp}_{unique_id}.json"  # S3 저장 경로

        # JSON 데이터를 S3에 업로드
        s3.put_object(
            Bucket=bucket_name,
            Key=s3_key,
            Body=json.dumps(data, indent=4),
            ContentType="application/json"
        )
        print(f"Data uploaded to S3: {s3_key}")

    except json.JSONDecodeError:
        print("Failed to decode JSON message")
    except Exception as e:
        print(f"Error: {e}")

# MQTT 클라이언트 설정
client = mqtt.Client()
client.on_message = on_message

# MQTT 브로커 연결
print(f"Connecting to MQTT broker at {broker_ip}:{broker_port}...")
client.connect(broker_ip, broker_port)

# 토픽 구독
client.subscribe(broker_topic)
print(f"Subscribed to topic '{broker_topic}'")

# 메시지 수신 대기
try:
    client.loop_forever()
except KeyboardInterrupt:
    print("Shutting down...")
