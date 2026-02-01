# 🤖 Isaac Sim & YOLOv8-OBB based 3D Pose Correction System

<img width="1404" height="947" alt="image" src="https://github.com/user-attachments/assets/22f419b0-5c83-4b87-8556-04bc781fbfb8" />


<br>

## 🗂️ 목차

1. [Project Overview](#-project-overview)
2. [Team & Roles](#-team--roles)
3. [System Architecture](#-system-architecture)
4. [Tech Stack](#-tech-stack)
5. [Key Features & Logic](#-key-features--logic)
6. [Run Instructions](#-run-instructions)
7. [Project Results](#-project-results)
8. [Demo Video](#-demo-video)

<br>

---

## 🔍 Project Overview
스마트 팩토리 공정에서 컨베이어 벨트 위의 부품이 미세하게 틀어지거나(Orientation Error) 뒤집혀 발생하는 병목 현상을 해결하기 위한 **Digital Twin 기반 로봇 제어 시스템**입니다. 

실제 환경을 **NVIDIA Isaac Sim**으로 완벽하게 구현하고, **YOLOv8-OBB**를 통해 객체의 회전 각도까지 정밀하게 인식하여 로봇이 자동으로 정렬(Pick & Place)하는 자동화 프로세스를 구축했습니다.

<br>

## 👥 Team & Roles

| Name | Role | Responsibility |
|:---:|:---:|:---|
| **Kim Jung-wook** | Team Leader <br> Robotics Engineer | - **Isaac Sim Environment Setup:** 실제 공장 환경(조명, 컨베이어, 로봇) Digital Twin 구축 <br> - **Robot Manipulation:** ROS2 기반 제어 노드 작성 및 Pick & Place 모션 플래닝 <br> - **System Integration:** Vision 데이터와 Robot Control 간 통신 최적화 |
| **Lee Hyo-won** | AI & System Engineer | - **YOLO Training:** Custom Dataset을 활용한 YOLOv8-OBB 모델 학습 및 튜닝 <br> - **ROS Integration:** AI 추론 결과(B-Box, Angle)를 ROS2 토픽으로 발행(Publish) |
| **Kim Da-bin** | Data Engineer & PM | - **Data Pipeline:** Roboflow 활용 학습 데이터셋 구축 및 레이블링(Labeling) <br> - **Documentation:** 산출물 관리, 발표 자료 및 시연 시나리오 기획 |

<br>

## 🛠 System Architecture

<img width="2034" height="570" alt="image" src="https://github.com/user-attachments/assets/d89a93c0-da07-4d6d-b926-2c1c53a708e0" />


이 시스템은 **Perception(인지) → Decision(판단) → Control(제어)**의 유기적인 데이터 파이프라인으로 구성됩니다.

1.  **Vision Node (`obb_node.py`):** RGB-D 카메라 데이터를 받아 YOLO 추론 및 3D 좌표 변환 수행. 노이즈 제거 후 타겟 좌표 발행.
2.  **Control Node (`move_joint.py`):** 타겟 좌표를 수신하여 역기구학(IK) 기반이 아닌, 관절(Joint) 단위의 정밀 시퀀스 제어 수행.
3.  **Simulation (Isaac Sim):** 실제 물리 엔진이 적용된 환경에서 로봇과 그리퍼가 상호작용.

<img width="1268" height="1045" alt="image" src="https://github.com/user-attachments/assets/dc735a11-7782-4bf6-9cf9-f7a859be7c35" />


<br>

## 💻 Tech Stack

| Category | Technology |
| :---: | :--- |
| **Simulation** | ![IsaacSim](https://img.shields.io/badge/NVIDIA-Isaac_Sim-76B900?style=flat-square&logo=nvidia) ![Omniverse](https://img.shields.io/badge/NVIDIA-Omniverse-76B900?style=flat-square&logo=nvidia) |
| **OS / Middleware** | ![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?style=flat-square&logo=ubuntu) ![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?style=flat-square&logo=ros) |
| **AI / Vision** | ![YOLOv8](https://img.shields.io/badge/YOLO-v8_OBB-00FFFF?style=flat-square) ![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?style=flat-square&logo=opencv) ![PyTorch](https://img.shields.io/badge/PyTorch-EE4C2C?style=flat-square&logo=pytorch) |
| **Hardware** | Doosan M0609, Intel RealSense D455 |
| **Language** | ![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=flat-square&logo=python) |

<br>

## 🚀 Key Features & Logic

### 1. Robust Detection with Debounce Logic
단순히 객체를 인식하는 것을 넘어, 현장의 조명이나 노이즈로 인한 오작동을 방지하기 위해 **Debounce 알고리즘**을 적용했습니다.
* **Logic:** `defect_need` (기본값 5프레임) 이상 연속으로 불량이 감지될 때만 로봇에게 신호를 보냅니다. 반대로 `ok_need` 프레임 이상 정상이 유지되어야 상태를 해제합니다.
* **Benefit:** 센서 데이터가 순간적으로 튀어서 로봇이 오작동하는 문제를 원천 차단했습니다.

### 2. Motion Sequencing State Machine
로봇의 움직임을 단일 명령이 아닌 **4단계 상태 머신(State Machine)**으로 정교하게 제어합니다.
* **Step 1 Approach:** 타겟 좌표의 상단(`approach_pose`)으로 안전하게 진입
* **Step 2 Pick:** 계산된 좌표로 하강하여 그리퍼 작동 (Visualizing Gripper Close)
* **Step 3 Retreat:** 물체를 파지한 채 안전 높이로 상승
* **Step 4 Return:** 홈 포지션 복귀

### 3. Hybrid Pose Correction (Hint Gain)
비전 센서의 계측 오차를 보정하기 위해 **Base Pose + Vision Offset** 방식을 사용했습니다.
* 미리 정의된 `pick_base` 좌표에 비전 센서가 감지한 편차(Delta)에 가중치(`hint_gain`)를 적용하여 최종 목표 좌표를 생성합니다. 이를 통해 완전한 Blind Control보다 유연하고, Full Vision Control보다 안정적인 파지가 가능합니다.

<br>

## ▶ Run Instructions

본 프로젝트는 ROS2 패키지로 구성되어 있으며, 주요 노드는 파라미터를 통해 튜닝 가능합니다.


### 1. Vision Node 실행 (YOLO 모델 경로 및 민감도 설정)
ros2 run yolo_obb_3d obb_node_fin --ros-args \
    -p model_path:="/path/to/best.pt" \
    -p defect_need:=5 \
    -p minangle_deg:=10.0

### 2. Control Node 실행 (동작 속도 및 홈 포지션 설정)
ros2 run my_examples move_joint_fin --ros-args \
    -p approach_sec:=1.5 \
    -p pick_sec:=1.0 \
    -p hint_gain:=0.8

<br>

## 📊 Project Results

  * [cite_start]**Detection Accuracy:** mAP50-95 기준 **90% 이상** 달성 [cite: 140]

  * [cite_start]**Pose Estimation Error:** 평균 오차 **5도 내외**로 정밀 보정 성공 [cite: 382]

  * **Impact:** 불량 부품의 자동 재정렬을 통해 공정 병목 현상 해소 및 생산 효율 증대 기대

<img width="2054" height="1059" alt="image" src="https://github.com/user-attachments/assets/13ff309c-a427-4bb5-affb-d1f57412b034" />

<img width="1936" height="941" alt="image" src="https://github.com/user-attachments/assets/2a96205d-1071-4fd6-8c0b-5bb80f382293" />


<br>

## 🎥 Demo Video

https://youtu.be/bfyb3jnT2ic
