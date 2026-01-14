# 🤖 Digital Twin-based Service Robot Operating System
### 디지털 트윈 기반 서비스 로봇 운영 시스템 (Team F-1)

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=flat&logo=ros)
![Isaac Sim](https://img.shields.io/badge/Simulator-NVIDIA_Isaac_Sim-green?style=flat&logo=nvidia)
![YOLOv8](https://img.shields.io/badge/AI-YOLOv8__OBB-red?style=flat&logo=ultralytics)
![Python](https://img.shields.io/badge/Language-Python_3.10-yellow?style=flat&logo=python)

> **"실제 공정에서 발생하는 문제를 디지털 환경에서 먼저 정확하게 재현하고 해결하여 현실에 적용한다."**

<br>

## 1. 프로젝트 개요 (Project Overview)

### 📌 배경 및 목적
산업 현장(자동차 제조 라인, 물류 센터 등)의 컨베이어 벨트 위에서 물체가 이동 중 미세하게 틀어지거나 정렬 불량이 발생할 경우, 로봇이 정확하게 잡지 못해 공정 중단이나 사고가 발생합니다.

본 프로젝트는 **NVIDIA Isaac Sim**을 활용한 디지털 트윈 환경에서 **YOLOv8-OBB(Oriented Bounding Box)** 기술을 적용하여 물체의 위치와 **회전 각도**를 실시간으로 인식하고, **UR10 로봇**이 이를 자동으로 보정(Pick & Place)하는 시스템을 구축했습니다.

### 🎯 기대 효과
* **공정 오류 해결:** 컨베이어 위 박스 충돌, 전복, 위치 틀어짐으로 인한 자동화 설비 오류 해결
* **작업자 안전:** 불규칙한 물체 정렬을 위해 작업자가 개입하다 발생하는 끼임 사고 예방
* **생산성 향상:** 공정 병목 현상 해결 및 불량률 감소

<br>

## 2. 팀원 및 역할 (Team Members)

| 이름 | 역할 | 담당 업무 상세 |
|:---:|:---:|:---|
| **김정욱** | 팀장 | • Isaac Sim 환경 구성 (Conveyor, Robot, Sensors) <br> • UR10 로봇 움직임(Manipulation) 구현 <br> • 프로젝트 총괄 및 환경 통합 |
| **이효원** | 팀원 | • YOLO-OBB 모델 학습 및 데이터 처리 <br> • ROS2 연동 (Topic Publish/Subscribe) <br> • 2D-3D 좌표 변환 알고리즘 구현 |
| **김다빈** | 팀원 | • 데이터셋 수집 및 레이블링 (Roboflow) <br> • 프로젝트 문서화 및 PPT 제작 <br> • 시나리오 기획 및 아이디어 구체화 |

<br>

## 3. 기술 스택 (Tech Stack)

### 💻 Environment
* **OS:** Ubuntu 22.04 LTS
* **GPU:** NVIDIA GeForce RTX Series (RTX 5080/3080 Laptop GPU)
* **Simulator:** NVIDIA Isaac Sim (Replicator 활용)

### 🛠️ Tools & Libraries
* **Middleware:** ROS 2 Humble
* **AI Model:** YOLOv8n-obb (Oriented Bounding Box)
* **Vision:** OpenCV, Realsense SDK (Simulated: RGB + Depth)
* **Labeling:** Roboflow (Polygon Labeling)

<br>

## 4. 시스템 구조 (System Architecture)

시스템은 **Input → Perception → Estimation → Control**의 흐름으로 동작합니다.

### 4.1. Perception (인식)
* **Sensor:** Isaac Sim 내 Realsense D455 카메라 (RGB + Depth)
* **Detection:** YOLOv8-obb 모델이 컨베이어 위 물체를 감지하여 중심점($x_c, y_c$), 크기, **회전 각도($\theta$)** 를 추출합니다.

### 4.2. 3D Pose Estimation (좌표 변환)
2D 이미지 좌표를 로봇이 이해할 수 있는 3D 월드 좌표로 변환하기 위해 카메라의 Intrinsic Parameter와 Depth 값을 사용합니다.

$$
x = \frac{(x_c - c_x) \times z}{f_x}
$$

$$
y = \frac{(y_c - c_y) \times z}{f_y}
$$

$$
z = \text{depth}
$$

* ($f_x, f_y$): Focal Length (초점 거리)
* ($c_x, c_y$): Principal Point (주점)
* **Result:** 계산된 3D 좌표와 회전 정보는 ROS 2 Topic으로 발행(Publish)됩니다.

### 4.3. Robot Control (제어)
* **판단:** 물체의 회전 각도가 설정된 **Threshold(임계값)** 이상일 경우 "보정 필요"로 판단합니다.
* **동작:** UR10 로봇이 해당 좌표로 이동하여 물체를 집어(Pick) 올바른 자세로 교정 후 다시 배치(Place)합니다.

<br>

## 5. 프로젝트 결과 (Results)

### 📊 AI 모델 학습 결과
* **데이터셋:** 총 1,016장 (Train 931 / Test 74)
* **Augmentation:** Flip, Brightness 적용
* **성능 지표:**
    * **mAP50:** 0.9950 (99.5%)
    * **mAP50-95:** 0.90 이상
    * **Confidence:** 평균 80% 이상

### 🤖 시뮬레이션 성능
* **각도 오차:** 평균 1~2도 내외 (허용 오차 범위 내 진입)
* **위치 오차:** 3D 공간 상에서 평균 5도(°) 이내의 오차
* **성과:** $2D \rightarrow 3D$ Pose 변환 성공 및 로봇 제어를 통한 정렬 자동화 구현

<br>

## 6. 한계점 및 향후 발전 방향 (Limitations & Future Work)

### Limitations
1.  **각도 인식 범위 (Yaw Limit):** 현재 모델은 0~90도 사이의 각도만 판단 가능하며, 대칭 물체의 방향성(180도 등) 구분 불가.
2.  **단일 객체 한정:** 특정 부품 1종에 대해서만 학습되어, 다양한 부품이 혼재된 라인 처리 불가.

### Future Work
1.  **Vector 기반 학습:** 모델 레이어에 벡터 값을 추가하여 90도 이상의 회전 각도도 판단할 수 있도록 개선.
2.  **Multi-Class 확장:** 다양한 부품을 학습시켜 하나의 컨베이어 벨트에서 여러 종류의 불량을 선별 및 보정하도록 고도화.
3.  **로봇 제어 정밀화:** 단순히 위치로 이동하는 것을 넘어, 로봇 그리퍼가 물체의 형상과 각도에 맞춰 회전하며 잡는 정밀 제어 구현.

---
**Developers:** Team F-1 (Kim Jeong-wook, Lee Hyo-won, Kim Da-bin)
