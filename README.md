# Scout2Map - STM32F103C8T6 주행 제어 펌웨어

UGV 하부 주행 제어 MCU용 베어메탈 펌웨어다.
HAL과 RTOS를 사용하지 않고 CMSIS 레지스터 레벨로 직접 구현한다.

## 현재 단계

76RPM 엔코더 모터가 장착되어 폐루프(closed-loop) 속도 제어가 활성화된 상태다.
`inc/board_config.h`의 `ENCODER_AVAILABLE`이 1로 설정되어 있다.

개루프 경로는 삭제하지 않고 폴백(fallback) 경로로 유지한다. 엔코더 배선이
단선되거나 신호가 유실되면 런타임에 자동으로 개루프로 전환되므로, 야전에서
로봇이 완전히 정지하는 대신 성능이 저하된 상태로 임무를 지속할 수 있다.

## 툴체인 구성

```
sudo apt install gcc-arm-none-eabi openocd
```

CMSIS 헤더는 저장소에 포함하지 않는다. 최초 1회 아래와 같이 내려받는다.

```
git clone --depth 1 https://github.com/STMicroelectronics/cmsis_device_f1
git clone --depth 1 https://github.com/ARM-software/CMSIS_5

mkdir -p cmsis/Device/ST/STM32F1xx cmsis/Include
cp -r cmsis_device_f1/Include cmsis/Device/ST/STM32F1xx/
cp -r CMSIS_5/CMSIS/Core/Include/* cmsis/Include/
```

## 빌드 및 업로드

```
make
make flash
```

ST-LINK V2 클론은 최신 STM32CubeProgrammer에서 거부되는 경우가 있으므로
OpenOCD 경로를 기본으로 사용한다.

## 디렉토리 구조

```
scout2map-fw-drive/
├── Makefile              # 크로스 빌드, 소스 자동 탐색
├── ld/                   # 링커 스크립트
├── config/
│   └── board_config.h    # 핀맵, 클럭, 튜닝 상수 (모든 계층이 참조)
├── app/
│   ├── main.c            # 협조적 스케줄러
│   ├── board_io.c/.h     # 제어 계층 <-> 드라이버 결선 어댑터
├── lib/
│   ├── hal/              # MCU 주변장치
│   │   ├── clock.c/.h
│   │   ├── systick.c/.h
│   │   └── startup_stm32f103.c
│   ├── drivers/          # 외부 부품
│   │   ├── motor.c/.h    # BTS7960
│   │   └── encoder.c/.h  # TIM1 / TIM4 쿼드러처
│   └── control/          # 하드웨어 비의존 로직
│       └── drive.c/.h    # 기구학, PID, 오도메트리
└── test/                 # 호스트 PC 단위 테스트
    ├── Makefile
    └── test_drive.c
```

헤더는 대응하는 `.c` 파일과 같은 디렉토리에 배치한다. 모듈을 이동할 때
파일 하나만 옮기면 되고, 어떤 헤더가 어느 모듈 소속인지 즉시 드러난다.

`Makefile`은 `app/*.c`와 `lib/*/*.c`를 와일드카드로 자동 탐색한다.
새 모듈을 추가할 때 해당 디렉토리에 파일을 넣기만 하면 되며
Makefile을 수정할 필요가 없다.

### 계층 의존 규칙

```
app  ->  lib/control,  lib/drivers,  lib/hal
lib/drivers  ->  lib/hal
lib/control  ->  (없음)
config  ->  모든 계층이 참조 가능
```

**`lib/control`은 어떤 하드웨어 헤더도 참조하지 않는다.** 모터, 엔코더,
IMU, 시스템 클럭 접근은 전부 `drive_io_t` 함수 테이블을 통해 주입받는다.
결선은 `app/board_io.c`가 전담한다.

이 구조의 실익은 두 가지다. 첫째로 모터 드라이버나 엔코더 방식을 교체할 때
`board_io.c`만 수정하면 되고 제어 로직은 변경되지 않는다. 둘째로 제어
계층 전체를 호스트 PC에서 컴파일하고 실행할 수 있어, 보드 없이 기구학과
PID를 검증할 수 있다.

## 호스트 단위 테스트

크로스 툴체인이나 실물 보드 없이 제어 계층을 검증한다.

```
make test
```

기대 출력은 다음과 같다.

```
=== Scout2Map drive control, host tests ===

straight command produces forward duty     PASS
stale command stops both wheels            PASS
   settled speed=0.2000 duty=760 counts/loop=31
PID converges on the target speed          PASS
request above the gearbox limit is clamped PASS
in place rotation drives the wheels apart  PASS
   odom after 2s at 0.2 m/s: x=0.386 y=0.001 th=0.002
straight run accumulates x only            PASS
straight run leaves heading unchanged      PASS

ALL TESTS PASSED (0 failures)
```

테스트 하네스는 1차 지연 플랜트로 모터를 모사하며, RPi5가 `cmd_vel`을
연속 발행하는 상황을 재현하기 위해 매 루프마다 명령을 갱신한다.
명령 타임아웃 검증에는 갱신을 생략하는 별도 경로를 사용한다.

PID 게인을 수정한 뒤에는 실물에 올리기 전에 이 테스트를 먼저 통과시킨다.

## 만능기판 커넥터 배치

```
Distance Sensor(2D120X):
                    SIG
5V      GND         PA4                 : ADC12_IN4

L Motor Driver(BTS7960) + Encoder:
MD_+    MD_-    EN      L_PWM   R_PWM
5V      GND     PB0     PA0     PA1     : TIM2_CH1 / TIM2_CH2
E_+     E_-     E_A     E_B
3.3V    GND     PA8     PA9             : TIM1 encoder mode

R Motor Driver(BTS7960) + Encoder:
MD_+    MD_-    EN      L_PWM   R_PWM
5V      GND     PB1     PA6     PA7     : TIM3_CH1 / TIM3_CH2
E_+     E_-     E_A     E_B
3.3V    GND     PB6     PB7             : TIM4 encoder mode

IMU(BNO055):
                    SDA     SCL
3.3V    GND         PB11    PB10        : I2C2
```

### 엔코더 핀을 PB0~PB3에 배치할 수 없는 이유

하드웨어 쿼드러처 디코딩은 타이머의 CH1 + CH2 쌍에서만 동작한다.
F103C8T6에서 가용한 조합은 다음이 전부다.

| 타이머 | CH1 | CH2 | 용도 |
|---|---|---|---|
| TIM1 | PA8 | PA9 | 좌측 엔코더 |
| TIM2 | PA0 | PA1 | 좌측 PWM (점유) |
| TIM3 | PA6 | PA7 | 우측 PWM (점유) |
| TIM4 | PB6 | PB7 | 우측 엔코더 |

PB0/PB1은 TIM3의 CH3/CH4이며 엔코더 모드를 지원하지 않는다.
PB2는 타이머 채널이 아니고 BOOT1 핀이므로 리셋 시 Low를 유지해야 한다.
따라서 PB0/PB1은 EN(비상 정지) 용도로 유지하고, 엔코더는 위 표에 따라 배치한다.

EXTI 인터럽트를 이용한 소프트웨어 디코딩도 가능하나, 76RPM 주행 시
초당 8천 회 이상의 인터럽트가 발생하여 200Hz 제어 루프를 잠식하므로 채택하지 않는다.

### IMU 핀 변경 사항

BNO055를 기존 PB6/PB7(I2C1)에서 **PB10/PB11(I2C2)** 로 이설했다.
우측 휠 엔코더용으로 TIM4(PB6/PB7)를 확보하기 위함이다.
이 배치는 AFIO 리맵 설정이 불필요하므로 초기화 코드가 단순해진다.

## 엔코더 배선

| 선 색 | 역할 | 연결 |
|---|---|---|
| 빨강 | 모터 + | BTS7960 M+ |
| 흰색 | 모터 − | BTS7960 M− |
| 파랑 | 엔코더 VCC | E_+ |
| 검정 | 엔코더 GND | E_− |
| 노랑 | A상 (11 PPR) | E_A |
| 초록 | B상 | E_B |

엔코더 전원 범위는 3.3~5V이나 **3.3V 공급을 권장한다.** 출력 레벨이 3.3V로
정렬되어 STM32 입력에 가장 안전하다. 해당 핀들은 5V tolerant이므로 5V도
동작하지만 불필요한 여유는 두지 않는다.

빨강과 흰색을 바꿔 연결하면 회전 방향이 반전된다. 좌우 모터는 서로
마주보게 장착되므로 한쪽은 반대로 결선해야 정방향이 일치한다.
코드에서 `ENC_L_INVERT` / `ENC_R_INVERT`로 처리해도 무방하다.

## 구동계 제원

JGB37-520 12V 데이터시트 기준값이다.

| 항목 | 값 |
|---|---|
| 감속비 | 131 : 1 |
| 무부하 회전수 | 76 RPM |
| 정격 회전수 | 58 RPM |
| 정격 토크 | 15.0 kg·cm |
| 최대 토크 | 24.0 kg·cm |
| 무부하 전류 | 120 mA 이하 |
| 정격 전류 | 1 A 이하 |
| 스톨 전류 | 2.3 A |

데이터시트의 감속비 계열은 6.3 / 10 / 19 / 30 / 56 / 90 / 131 / 168 /
270 / 506 / 810 이며 **150은 존재하지 않는다.** 76RPM은 131:1 항목의
무부하 회전수에 정확히 대응한다.

설계 최고 속도는 **정격 58RPM 기준**으로 산정한다.

```
58 / 60 x 0.2073 m = 0.200 m/s
```

무부하 76RPM으로 산정하면 0.262 m/s가 나오나, 이는 실제로 도달할 수 없는
값이므로 제어기가 영구적으로 포화 상태에 놓인다.

전류 설계는 모터 4개 기준 스톨 9.2 A이며, 3S 6000mAh 팩에서 약 1.7C에
해당하여 25~35C 등급으로 충분하다.

## 엔코더 분해능 실측 (필수 선행 작업)

`GEAR_RATIO` 131은 데이터시트값이며 **반드시 실측으로 검증한다.**
이 값이 틀리면 속도 제어와 오도메트리가 모두 비례 오차를 갖는다.

감속비를 150으로 잘못 가정할 경우의 영향은 다음과 같다.

```
5764 / 6600 = 0.873
실제 1m 주행 -> 87cm로 보고
```

SLAM 지도가 12.7% 압축되며, 복도 왕복 시 루프 클로저 불일치로 즉시 드러난다.

`ENC_PPR` 11 역시 미검증 항목이다. JGB37 계열은 11 PPR과 13 PPR이
혼재하여 유통되므로 상품 페이지에서 반드시 확인한다.
13인 경우 `COUNTS_PER_WHEEL_REV`는 6812가 된다.

측정 절차는 다음과 같다.

1. 차체를 들어올려 바퀴를 공중에 띄운다.
2. `encoder_reset()`을 호출하거나 보드를 리셋한다.
3. 바퀴를 손으로 정확히 10회전시킨다. 시작점 표시를 해두면 정확도가 올라간다.
4. `encoder_get_total()` 값을 10으로 나눈다.
5. 결과를 `COUNTS_PER_WHEEL_REV`에 직접 대입하거나, 역산하여 `GEAR_RATIO`를 수정한다.

좌우를 각각 측정하여 값이 크게 다르면 배선 또는 감속비 불일치를 의심한다.

## 제어 구조

```
목표 속도 (v, w)
   -> 차동 구동 역기구학 -> 좌/우 목표 휠 속도
   -> 피드포워드 (PID_FF) + 속도 PID
   -> 듀티 permille -> 슬루 제한 -> PWM
```

피드포워드가 출력의 대부분을 담당하고 PID는 잔차만 보정한다.
이 구조는 게인을 작게 유지할 수 있어 정정 특성이 안정적이다.

튜닝은 `PID_KP`부터 시작한다. `PID_KI`는 정상상태 오차가 남을 때만 추가하고,
`PID_KD`는 감속기가 있는 구동계에서는 대개 0으로 둔다.
미분항을 사용할 경우 `PID_KD`와 함께 `PID_USE_D`를 1로 설정한다.
전처리기는 부동소수점 비교를 수행할 수 없으므로 별도 스위치가 필요하다.

## 속도 및 각속도 제한

| 항목 | 상수 | 값 | 근거 |
|---|---|---|---|
| 최고 병진 속도 | `MAX_WHEEL_SPEED_MPS` | 0.20 m/s | 정격 58RPM |
| 최대 각속도 | `MAX_ANGULAR_RATE` | 0.80 rad/s | 스캔 품질 |
| 기구적 각속도 한계 | `MAX_ANGULAR_RATE_MECH` | 1.67 rad/s | 참고값 |

제자리 회전 시 기구적 한계는 다음과 같다.

```
w = 2 x v / track = 2 x 0.20 / 0.24 = 1.67 rad/s (약 96 deg/s)
```

RPLiDAR C1의 1회전 주기는 100ms이므로, 96 deg/s로 회전하면 단일 스캔이
약 9.6도 왜곡된다. 스캔 매칭 품질이 저하되므로 명령 각속도는 기구적
한계보다 낮은 0.80 rad/s(약 46 deg/s)로 제한한다.

매핑 정확도보다 기동성이 중요한 상황에서는 이 값을 상향할 수 있으나,
`MAX_ANGULAR_RATE_MECH`를 초과하는 설정은 의미가 없다.

## 안전 설계

| 항목 | 설정값 | 비고 |
|---|---|---|
| 비상 정지 경로 | EN 핀 Low | PWM 레지스터가 아닌 EN 핀으로 처리 |
| 슛스루 차단 | 상시 | RPWM/LPWM 중 한쪽은 항상 0 |
| 방향 전환 데드타임 | 2ms | 하프브리지 관통 방지 |
| 듀티 슬루 제한 | 루프당 2% | 기어박스 및 전원 새그 보호 |
| 명령 타임아웃 | 300ms | RPi5 무응답 시 자동 정지 |
| 스톨 가드 | 고듀티 3초 | 모터 소손 방지 |
| 엔코더 헬스체크 | 무응답 1초 | 개루프로 자동 전환 |
| 적분 와인드업 방지 | 포화 시 클램프 | `PID_I_LIMIT` |
| IWDG | 약 400ms | 펌웨어 행(hang) 방지 |

부팅 시퀀스는 `motor_init()`이 EN을 Low로 내리고 PWM을 0으로 설정한 뒤에야
`motor_enable()`이 호출되도록 구성했다. 리셋 직후 바퀴가 돌발 구동되는
상황을 방지한다.

## 하드웨어 주의사항

- 보드의 VBUS 패턴을 반드시 절단한다. 전원은 UBEC 5V 레일에서 공급한다.
- 모터 GND와 로직 GND는 단일 스타 접지점에서 만나도록 배선한다.
- BTS7960의 IS 핀(R_IS/L_IS)은 과전압 위험으로 미연결 상태를 유지한다.
- 각 IC 근처에 0.1uF 디커플링 캐패시터를 배치한다.
- 거리센서 전원부에 10~100uF 벌크 캐패시터를 추가한다.
- 엔코더 신호선은 모터 전원선과 분리하여 포설한다. 나란히 묶으면
  브러시 노이즈가 유도되어 위상 오검출이 발생한다.

## 검증 절차

1. **클럭 확인**: LED가 정확히 1초 주기로 점멸하는지 확인한다.
   주기가 어긋나면 크리스탈이 8MHz가 아니거나 PLL 설정이 잘못된 것이다.
   이 단계가 통과되지 않으면 USB 열거(enumeration)는 시도하지 않는다.
2. **PWM 확인**: PA0/PA1에서 20kHz 신호가 출력되는지 측정한다.
3. **엔코더 방향 확인**: 바퀴를 손으로 정방향 회전시켰을 때
   `encoder_get_total()`이 증가하는지 확인한다. 감소하면 해당 축의
   `ENC_x_INVERT`를 반전시킨다.
4. **엔코더 분해능 실측**: 위 절차에 따라 `COUNTS_PER_WHEEL_REV`를 확정한다.
   데이터시트 기준 계산값은 11 x 4 x 131 = 5764 이다.
5. **모터 방향 확인**: 바퀴를 띄운 상태로 확인한다. 방향이 반대인 경우
   배선을 변경하지 말고 `motor.c`의 `pwm_write()` 내부 두 줄을 교체한다.
6. **PID 튜닝**: 목표 속도를 계단 입력으로 주고 오버슈트와 정정 시간을 확인한다.

## 향후 작업 순서

| 순서 | 모듈 | 비고 |
|---|---|---|
| 1 | USB CDC | 명령 입력 경로 확보 |
| 2 | 바이너리 프로토콜 | 64바이트 프레임, CRC16 |
| 3 | BNO055 드라이버 | 논블로킹 상태머신, 버스 복구 루틴 포함 |
| 4 | ADC 거리센서 | 룩업테이블 및 중앙값 필터 |
| 5 | 슬립 검출 | 엔코더 오도메트리와 IMU 요레이트 비교 |

STM32는 센서 융합을 수행하지 않는다. 원시값에 타임스탬프만 부여하여
전송하고, `sensor_msgs/Imu` 및 `nav_msgs/Odometry` 변환과 TF 처리는
상위 SBC(RPi5)에서 담당한다. 펌웨어 내부의 오도메트리 적분은 슬립 검출과
디버깅 편의를 위한 보조 수단이며, SLAM의 입력으로 사용하는 값은 아니다.
