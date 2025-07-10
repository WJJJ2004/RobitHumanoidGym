# RobitHumanoidGym

Isaac Lab 기반의 **Robit 휴머노이드 강화학습 프로젝트**입니다.  
본 레포는 로컬에서 관리되는 Robit Humanoid 모델(USD)과 커스텀 환경을 활용하여, 보행 정책 학습을 수행한 실험 결과 및 설정을 정리한 레포입니다.

## 개발 환경

- **OS**: Ubuntu 22.04
- **Isaac Sim**: 4.5.0
- **Isaac Lab 설치 위치**: Conda 가상환경 (python 3.10)
- **로봇 모델 경로**: `/usd_ws/robit/robit_humanoid`
- **학습 프레임워크**: [Isaac Lab](https://github.com/NVIDIA-Omniverse/IsaacLab) (BSD-3-Clause)
- **강화학습 알고리즘**: PPO (RSL-RL 및 SKRL 설정 제공)

##  주요 기능

- Robit 휴머노이드 모델에 맞춘 **평지 / 복합지형 환경** 제공
- **Feet contact / slide / air time** 기반 보상 함수 설계
- **RSL-RL 및 SKRL 기반 PPO 설정** 샘플 포함
- Sim-to-Real 고려한 센서 기반 관측 및 보상 설계 구조

---

##  주의사항

- `usd_ws/robit/robit_humanoid/` 디렉토리는 **로컬 USD 모델을 포함**하고 있습니다
- 이 프로젝트는 학습 및 연구용으로만 활용됩니다.
- 기반 프레임워크인 **Isaac Lab**은 BSD-3-Clause 라이선스를 따릅니다.

---

## 실행 방법

Isaac Sim 환경이 사전에 실행되어 있어야 합니다.  




