# Python 3.12 Slim 이미지 사용
FROM python:3.12-slim

# 컨테이너 작업 디렉토리 설정
WORKDIR /app

# Python 의존성 설치
RUN apt-get update && apt-get install -y gcc && apt-get clean

# 필요한 Python 라이브러리 설치
COPY app/requirements.txt /app/requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

# 코드 및 설정 파일 복사
COPY app /app

# 컨테이너 실행 시 Python 스크립트 실행
CMD ["python3", "mqtt_to_s3.py"]
