# 🔐 ATmega128 기반 스마트 도어락 & IoT 하우스 제어 시스템

2개의 ATmega128 MCU를 UART로 연결하여, 도어락 시스템과 스마트 하우스 제어 기능을 분리·통합한 프로젝트입니다.  
비밀번호/RFID 기반 출입, EEPROM 사용자 설정 저장, 센서 기반 환경 제어, 릴레이를 통한 가전 제어,  
PWM 제어를 통한 서보모터 자동창문 제어 등 다양한 임베디드 제어 기술을 활용하여  
실제 모델하우스와 유사한 구조를 구현했습니다.

---

## 📽️ 시연 영상

[🔗 유튜브 시연 영상 보러가기](https://youtu.be/8ZCu6Y8iaiA)

---

## 📌 핵심 기술 요약

- 🔁 **MCU 통신**: 2개의 ATmega128 UART 연결 (도어락 <-> 스마트홈 제어)
- 📷 **센서 활용**: 조도, 가스, 수위, 초음파, 온습도 등 6종 이상
- 🛡 **출입 제어**: EEPROM 기반 비밀번호, RFID 카드 등록/삭제 기능
- 🔌 **릴레이 제어**: 가전제품 제어 (에어컨, 가습기 등)
- 🔄 **PWM 제어**: 서보모터로 문/창문 자동 제어
- ⚠️ **Watchdog Timer**: MCU 비정상 상태 복구
- ⏱ **인터럽트 처리**: 버튼, 타이머 이벤트 등 안정적 시나리오 처리

---

## 🏗️ 프로젝트 구조

```bash
📁 document         - 개발 보고서 및 발표 자료
📁 mainCode
 ├── controlUnit.c - 스마트홈 제어부 코드
 ├── doorLock.c    - 도어락 처리 코드
 ├── RFIDControl.c/.h, UARTControl.c/.h, TWI.h - 각종 드라이버 모듈
📁 testCode         - 센서별 테스트 코드 모음
```

---

## 🔁 MCU 역할 분담 및 통신 구조

### 🧠 MCU 역할

- **도어락 MCU**
  - 사용자 입력, 비밀번호, RFID 처리
  - EEPROM 기반 사용자 설정 관리
  - 제어부 MCU와 UART 통신으로 명령 송신

- **제어부 MCU**
  - 센서 정보 수집 (조도, 가스, 수위, 온습도 등)
  - 액츄에이터 제어 (서보모터, 릴레이)
  - 환경 자동 제어 시나리오 실행

### 📡 UART 통신 프로토콜

![데이터 패킷](https://github.com/user-attachments/assets/9cfdd1aa-5fc0-4941-acab-6f847fc951f8)

---

## 🖼️ 시스템 구성도

```
[도어락 MCU]
  ├── 버튼 입력
  ├── RFID 인식
  └── EEPROM 저장
        │
       UART
        ↓
[제어부 MCU]
  ├── 센서 정보 수집
  ├── 릴레이/모터 제어
  └── 사용자 설정 자동 실행
```

## 🔎 흐름도

- **도어락 흐름도**  
  ![도어락 흐름도](https://github.com/user-attachments/assets/eca38503-9179-4801-8206-e4cc7ae823ed)

- **제어부 회로도**  
  ![제어부 흐름도](https://github.com/user-attachments/assets/073a3634-ddd3-48bc-ad6d-1f4c42ba70f1)

---

---

## 🧾 회로도

- **도어락 회로도**  
  ![도어락 회로도](https://github.com/user-attachments/assets/ece91a11-e34d-447f-a80c-1111ed658291)

- **제어부 회로도**  
  ![제어부 회로도 1](https://github.com/user-attachments/assets/832fc948-5dd8-47a6-b7ee-9e728564179a)
  ![제어부 회로도 2](https://github.com/user-attachments/assets/6c017a06-4fd7-4e65-bff9-c2e50f12ec17)


---

## ⚙️ 개발 환경

![개발환경](https://github.com/user-attachments/assets/658e5c49-5000-4e5a-887a-8bd084c46f9a)


| 항목     | 내용            |
|----------|-----------------|
| MCU      | ATmega128       |
| 언어     | C               |
| 개발툴   | Atmel Studio 7  |
| 디버깅   | JTAGICE3         |
| 컴파일러 | avr-gcc         |

---

## 👤 팀원

| 프로필 | 역할  | 담당 부분 | 기술 스택 |
|--------|-------|----------|-----------|
| ![강송구](https://github.com/user-attachments/assets/986e1819-2d0d-4715-97ce-590ea6495421) <br> [강송구](https://github.com/StrongThrow) | 팀장  | HW & FW 설계/개발 | ATmega128, UART, EEPROM, PWM, 센서제어 등 |

---

## 📌 프로젝트 핵심 요약

> 이 프로젝트는 단순한 센서제어를 넘어,  
> **2개의 MCU가 통신하며 각자 역할을 분담하는 구조**,  
> **센서 수집 → 판단 → 액츄에이터 제어**의 루프를 포함하는  
> **실제 임베디드 제어 흐름을 완성도 있게 구현**한 결과물입니다.
