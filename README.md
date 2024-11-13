# 작품명: Atmega128 2개를 이용한 스마트 도어락과 IoT 시스템 [마이크로프로세서2 기말고사 텀 프로젝트]

***작품의 동작 내용은 영상을 참고***
(유튜브 링크 : https://youtu.be/Z3WCU18oMyQ)

Atmega128 2개를 활용하여 도어락과 스마트 홈 제어 기능을 하게 만들어서 스마트 홈을 구현한 프로젝트

## 배경

IoT 기술을 가정에 접목하여 각종 센서와 액츄에이터들을 활용, 모델 하우스에 구현하여 마이크로프로세서2에서 학습한 내용들에 대해 이해도를 높히기 위해 기획함

## 개발목표

- 각종 센서(조도 센서, 온/습도 센서, 수위 센서, 가스 센서, 인체 감지 센서, 초음파 센서등)들을 활용하여 집의 데이터를 수집한다
- 서보모터를 활용하여 문과 창문의 자동 개폐를 구현한다
- 가습기, 에어컨, 멀티탭 등의 가전들을 릴레이 모듈을 이용해 제어한다
- 비밀번호와 Rfid 카드로 열리는 도어락을 구현하고 사용자 정보들을 EEPROM에 저장하여 Rfid 카드 태그 시 그 설정을 불러온다
- 센서로 수집한 데이터들을 이용하여 모델 하우스를 제어한다
- 비밀번호 변경, Rfid 정보 초기화 등의 기능을 구현한다
- Atmega128의 위치독을 사용하여 올바른 작동이 가능하게 한다
  
## 전체 흐름도

### 도어락 흐름도
![도어락 흐름도](https://github.com/user-attachments/assets/eca38503-9179-4801-8206-e4cc7ae823ed)

### 제어부 흐름도
![제어부 흐름도](https://github.com/user-attachments/assets/073a3634-ddd3-48bc-ad6d-1f4c42ba70f1)

## Uart 데이터 패킷 구조
![데이터 패킷](https://github.com/user-attachments/assets/9cfdd1aa-5fc0-4941-acab-6f847fc951f8)

## 회로도

### 도어락 회로도
![도어락 회로도](https://github.com/user-attachments/assets/ece91a11-e34d-447f-a80c-1111ed658291)


### 제어부 회로도
![제어부 회로도 1](https://github.com/user-attachments/assets/832fc948-5dd8-47a6-b7ee-9e728564179a)
![제어부 회로도 2](https://github.com/user-attachments/assets/6c017a06-4fd7-4e65-bff9-c2e50f12ec17)

## 활용 기술
- Uart, I2C, SPI 통신
- 8 bit timer/counter, 16bit timer/counter
- WatchDog Timer
- Interrupt
- etc

## 개발환경
![개발환경](https://github.com/user-attachments/assets/658e5c49-5000-4e5a-887a-8bd084c46f9a)

## 팀원

| ProFile | Role | Part | Tech Stack |
|:--------:|:--------:|:--------:|:--------:|
| ![KakaoTalk_20241113_230554223](https://github.com/user-attachments/assets/986e1819-2d0d-4715-97ce-590ea6495421) <br> [강송구](https://github.com/Throwball99) |   팀원  |   HW, SW |   Arduino, Fusion 360 |
