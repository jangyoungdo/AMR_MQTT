# FMS(Robot Fleet Management System)
![image](https://github.com/user-attachments/assets/9d6677ed-ebeb-46bd-a90d-0d30259a2ef8)

## 네트워크 세팅
로봇과 관제 PC가 서로 다른 공간에 있음을 가정하기에 가상 네트워크 활용하기로 함

## ZeroTier를 활용
 - 네트워크 생성
 - 실험용 Robot 역할 노트북과 관제 노트북 연결
 - 각 장치에 할당된 IP 확인

```
curl -s https://install.zerotier.com | sudo bash
sudo zerotier-cli join <네트워크 ID>
```

# MQTT
## 개요
- 경량 메시지 프로토콜로 IoT 기기 간 데이터 전송에 최적화
- 제한된 네트워크 대역폭과 전력 소비에 효율적

## 주요 특징
### 발행-구독 모델 (Pub/Sub)
- 발행자(Publisher)와 구독자(Subscriber)가 브로커를 통해 데이터 전송
- IoT 환경에서 다수 장치의 효율적인 통신 지원

### 브로커 (Broker)
- 발행자와 구독자 간 중개 역할
- 대표 브로커: Mosquitto, EMQX, HiveMQ

### QoS (Quality of Service)
- QoS 0: 한 번만 전송, 전달 보장 없음
- QoS 1: 적어도 한 번 전달, 중복 가능
- QoS 2: 정확히 한 번 전달, 중복 방지

### 저전력, 저용량 프로토콜
- TCP 기반, 패킷 크기 작아 제한된 대역폭에 적합

### 유지 연결 (Keep Alive)
- 연결 상태 확인을 위해 주기적 PINGREQ 전송, 연결 손실 시 자동 재연결


# MQTT 세팅
### Ubuntu 환경에 세팅
#### 실험용 노트북 ubuntu 20.04에서는 구동
#### 로봇 24.04에서는 구동되는지 궁금
```
 sudo apt update
 sudo apt install mosquitto mosquitto-clients
```
### 실행
```
sudo systemctl start mosquitto
sudo systemctl enable mosquitto  # 부팅 시 자동 실행
```
### MQTT Client 패키지 생성

```
cd ~/Proj/dev-0716/src
ros2 pkg create mqtt_client --build-type ament_python --dependencies rclpy std_msgs
```

### 패키지 실행
```
cd ~/Proj/dev-0716
colcon build --packages-select mqtt_client
source install/setup.bash
ros2 launch mqtt_client mqtt_client_launch.py

```
# 문제
## 문제 1: ROS2와 Mosquitto 컨테이너 간 네트워크 연결 문제
1. 문제 정의
   - ROS2 컨테이너와 새로 생성한 Mosquitto 컨테이너가 서로 다른 Docker 네트워크에 포함되어 있어 통신이 이루어지지 않음.
   - ROS2 노드에서 MQTT 브로커에 연결하려 할 때 connection refused 오류 발생.
  
 2. 문제 원인
    - Docker 컨테이너는 기본적으로 독립된 네트워크를 사용 (bridge 네트워크).
    - 동일 네트워크에 속하지 않은 컨테이너 간에는 통신이 불가능.
   
 3. 해결과정
    1) 사용자 정의 네트워크 생성: ROS2 컨테이너와 Mosquitto 컨테이너를 연결할 네트워크 생성
       ```
       docker network create mqtt_ros2_network
       ```
    2) ROS2 컨테이너 네트워크 연결: 기존 ROS2 컨테이너를 새 네트워크에 추가
       ```
       docker network connect mqtt_ros2_network ros2
       ```
    3) Mosquitto 컨테이너 네트워크 설정: Mosquitto 컨테이너를 생성 시 mqtt_ros2_network에 연결
       ```
       docker run --name mosquitto --network mqtt_ros2_network -p 1883:1883 eclipse-mosquitto
       ```
    4) 네트워크 연결 상태 확인: 두 컨테이너가 동일한 네트워크에 있는지 점검
       ```
       docker network inspect mqtt_ros2_network
       ```

## 문제 2: ROS2 스크립트에서 Mosquitto 브로커 참조 문제
1. 문제 정의
   - ROS2 컨테이너에서 실행 중인 Python 스크립트가 Mosquitto 컨테이너에 연결할 때 IP 주소를 직접 참조하려다 실패.
   - 컨테이너의 동적 IP 변경으로 인해 브로커 연결 오류 발생.
  
 2. 문제 원인
    - 컨테이너 이름 대신 동적 IP를 사용해 Mosquitto 브로커를 참조.
    - 사용자 정의 네트워크에서는 컨테이너 이름을 DNS처럼 사용할 수 있지만, 이를 적용하지 않음.
   
 3. 해결과정
    1) 컨테이너 이름으로 브로커 참조: ROS2 스크립트에서 Mosquitto 브로커를 컨테이너 이름으로 참조
       ```
       mqtt_client.connect("mosquitto", 1883, 60)
       ```
    2) 네트워크에서 컨테이너 이름 확인: 사용자 정의 네트워크에서는 mosquitto라는 이름으로 Mosquitto 컨테이너를 참조 가능
       ```
       docker network inspect mqtt_ros2_network
       ```

## 문제 3: Mosquitto 브로커 포트 바인딩 및 외부 통신 문제
1. 문제 정의
   - Mosquitto 브로커 컨테이너가 내부 네트워크에서만 작동하고, 외부에서 접근하려고 할 때 연결 실패.
   - 외부 관제 시스템 또는 MQTT 클라이언트에서 Mosquitto 컨테이너의 브로커에 접근하지 못함.
  
 2. 문제 원인
    - Mosquitto 컨테이너 실행 시 외부 포트를 Docker 호스트에 바인딩하지 않았거나, 방화벽이 포트를 차단.
   
 3. 해결과정
    1) 포트 바인딩 설정: Mosquitto 컨테이너 실행 시 외부 포트를 Docker 호스트에 바인딩
       ```
       docker run --name mosquitto --network mqtt_ros2_network -p 1883:1883 eclipse-mosquitto
       ```
    2) 방화벽 규칙 추가: Mosquitto 브로커의 1883 포트를 허용
       ```
       sudo ufw allow 1883
       sudo ufw reload
       ```
    3) 외부 접근 테스트: 외부 클라이언트에서 Mosquitto 브로커에 연결
       ```
       mosquitto_pub -h <Docker_Host_IP> -p 1883 -t "test/topic" -m "Hello External"
       mosquitto_sub -h <Docker_Host_IP> -p 1883 -t "test/topic"
       ```
       
## 문제 4: bind_address 설정으로 인한 Mosquitto 실행 오류
1. 문제 정의
   - Mosquitto 설정 파일에서 bind_address 0.0.0.0을 설정했을 때 컨테이너가 실행되지 않거나 다음과 같은 오류 발생
     ```
     Error: Address not available
     ```
  
 2. 문제 원인
    - Docker 환경에서 bind_address 설정은 컨테이너 내부 네트워크 인터페이스와 충돌할 가능성이 높음.
   
 3. 해결과정
    1) bind_address 설정 제거: Mosquitto 설정 파일에서 bind_address 항목을 제거 또는 주석 처리
       ```
       # bind_address 0.0.0.0  # 제거
       listener 1883
       allow_anonymous true
       ```
    2) 네트워크에서 컨테이너 이름 확인: 사용자 정의 네트워크에서는 mosquitto라는 이름으로 Mosquitto 컨테이너를 참조 가능
       - 별도의 bind_address 설정 없이 기본값으로 유지.

# 테스트
1. ROS2에서 Mosquitto 연결 테스트: ROS2 컨테이너에서 Python 스크립트를 실행하여 MQTT 메시지 발행
```
mqtt_client.publish("test/topic", "Hello from ROS2", qos=0)
```
2. Mosquitto 브로커에서 메시지 수신 확인: 로컬 및 외부 클라이언트에서 mosquitto_sub 명령어로 메시지 수신 테스트
```
mosquitto_sub -h localhost -t "test/topic"
```
3. 외부 관제 시스템 연결 테스트: 외부 네트워크에서 Mosquitto 브로커 접근 가능 여부 확인
```
mosquitto_pub -h <외부_IP> -p 1883 -t "test/topic" -m "Hello External"
```

# 파일 구조
```
AMR_Proj/
├── .git/                 
├── com_proj/              
├── foxy/               
├── humble/              
├── Proj/               
│   ├── dev/              
│   ├── dev-0716/          
│   ├── dev-last/          
│   ├── driver/            
│   ├── etc/                
│   └── src/                # ROS2 소스 코드
│       ├── agent_ws/      
│       ├── articubot_one/  # 특정 로봇 관련 설정
│       ├── mqtt_client/    # MQTT 클라이언트 패키지
│       │   ├── config/     # MQTT 설정 파일
│       │   │   └── mqtt_params.yaml # MQTT 브로커 IP, 포트 설정
│       │   ├── launch/     # ROS2 Launch 파일
│       │   │   └── mqtt_client_launch.py # MQTT 클라이언트 실행 런치 파일
│       │   ├── mqtt_client/ # MQTT 클라이언트 코드
│       │   │   ├── __init__.py  # 패키지 초기화 파일
│       │   │   └── mqtt_client.py # MQTT 클라이언트 주요 코드
│       │   ├── resource/   # 패키지 리소스 파일
│       │   │   └── mqtt_client  # 패키지 인식용 빈 파일
│       │   ├── test/     
│       │   ├── package.xml 
│       │   ├── setup.cfg  
│       │   └── setup.py   
│       ├── omo_r1mini_bringup/ 
│       ├── omo_r1mini_cartographer/
│       ├── omo_r1mini_description/  
│       ├── omo_r1mini_navigation2/  
│       ├── stella_ahrs/   # Stella AHRS 센서 드라이버
│       ├── ydlidar_ros2_driver/ # YDLidar ROS2 드라이버
│       └── mosquitto.conf # Mosquitto 브로커 설정 파일
├── .gitignore            
└── README.md             


```
