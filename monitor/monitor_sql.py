import pymysql
import json
import paho.mqtt.client as mqtt

# MySQL 데이터베이스 연결 설정
db_connection = pymysql.connect(
    host="localhost",  # 데이터베이스 호스트 주소
    user="root",  # 데이터베이스 사용자 이름
    password="1234",  # 데이터베이스 비밀번호
    database="PJ"  # 사용할 데이터베이스 이름
)

# 새로운 테이블 생성 (한 번만 실행)
create_table_query = """
CREATE TABLE IF NOT EXISTS turn_left (
    x FLOAT,
    y FLOAT,
    theta FLOAT,
    l_enc INT,
    r_enc INT,
    odo_l FLOAT,
    odo_r FLOAT,
    yaw FLOAT,
    pitch FLOAT,
    roll FLOAT,
    orientation_x FLOAT,
    orientation_y FLOAT,
    orientation_z FLOAT,
    orientation_w FLOAT,
    gear_ratio FLOAT,
    wheel_separation FLOAT,
    wheel_radius FLOAT
)
"""
with db_connection.cursor() as cursor:
    cursor.execute(create_table_query)
db_connection.commit()

# MQTT 브로커 설정
broker_ip = "192.168.192.155"  # 로봇의 ZeroTier IP 주소
broker_port = 1883
broker_topic = "robot/status"

# 메시지 수신 콜백 함수
def on_message(client, userdata, message):
    try:
        # JSON 메시지를 디코딩하여 MySQL에 저장
        payload = message.payload.decode('utf-8')
        data = json.loads(payload)

        # 데이터 삽입 쿼리
        insert_query = """
        INSERT INTO turn_left (
            x, y, theta, l_enc, r_enc, odo_l, odo_r, 
            yaw, pitch, roll, orientation_x, orientation_y, 
            orientation_z, orientation_w, gear_ratio, wheel_separation, wheel_radius
        ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
        """
        
        # 데이터 매핑
        values = (
            data.get("x", 0.0),
            data.get("y", 0.0),
            data.get("theta", 0.0),
            int(data.get("l_enc", 0)),
            int(data.get("r_enc", 0)),
            data.get("odo_l", 0.0),
            data.get("odo_r", 0.0),
            data.get("yaw", 0.0),
            data.get("pitch", 0.0),
            data.get("roll", 0.0),
            data.get("orientation_x", 0.0),
            data.get("orientation_y", 0.0),
            data.get("orientation_z", 0.0),
            data.get("orientation_w", 0.0),
            data.get("gear_ratio", 0.0),
            data.get("wheel_separation", 0.0),
            data.get("wheel_radius", 0.0)
        )

        with db_connection.cursor() as cursor:
            cursor.execute(insert_query, values)
        db_connection.commit()
        print("Data inserted into MySQL:", values)

    except json.JSONDecodeError:
        print("Failed to decode JSON message")
    except pymysql.MySQLError as e:
        print("MySQL error:", e)
    except Exception as e:
        print("Unexpected error:", e)

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
finally:
    # 연결 종료 시 MySQL 커넥션 닫기
    db_connection.close()
