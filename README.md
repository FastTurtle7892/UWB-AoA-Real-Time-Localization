# AoA-Based UWB RTLS

> **Single Anchor + Single Tag 실시간 위치 추적 시스템**  
> DS-TWR 기반 거리(Ranging)와 AoA(Angle of Arrival) 기반 각도(Angle Estimation)를 융합해 **저복잡도·실시간 RTLS**를 구현

---

## 1. 개요
**단일 앵커(Anchor)**와 **단일 태그(Tag)** 만으로 실시간 위치 추적(Real-Time Locating System, RTLS)을 구현하는 것을 목표로 합니다.  
기존 삼각측량 방식이 최소 3개 이상의 앵커를 요구하는 것과 달리, **AoA(Angle of Arrival)**를 도입해 **앵커 수를 줄이고 시스템 구조를 단순화**했습니다.

---

## 2. 핵심 기술 및 구현 방식
- **거리 측정 (Ranging)**  
  - DS-TWR(Double-Sided Two-Way Ranging) 사용 → 태그와 앵커 간 거리(R) 정밀 계산
- **각도 측정 (Angle Estimation)**  
  - AoA 기반 → UWB 안테나 배열의 위상차(Phase Difference) 분석으로 태그 도래 각도(θ) 추정
- **위치 추정**  
  - 측정된 **거리 R**과 **각도 θ**를 실시간 융합하여 2D 평면 좌표 계산
- **표준** : IEEE 802.15.4z HRP UWB
- **안테나** : UWB 지향성 안테나 (XR-170)

---

## 3. 주요 성과 및 분석
### ✅ 성과
- **단일 앵커 + 태그 구성**으로 DS-TWR과 AoA 융합 실시간 RTLS 구현 성공
- 실험 환경에서 **실시간 위치 추적 데이터 수집 및 시각화** 완료

### ⚠️ 문제점 및 해결
- **Y축 오차**  
  - Y축 이동 시 앵커 기준 각도 변화폭이 작아 X좌표 분산이 집중됨  
  - → **칼만 필터(Kalman Filter)** 등 가중치 기반 필터 적용을 통한 정확도 향상 가능
- **실시간 시각화 문제**  
  - MATLAB UART 통신 시 지연 및 데이터 누락 → **버퍼 주기적 초기화**로 해결

### 🚀 개선 방향
- 이동 방향 기반 **보정 로직** 추가로 안정적 추적
- 앵커 배치 최적화 및 보조 신호 활용으로 Y축 오차 감소
- 실시간 필터링(EKF, UKF) 적용으로 잡음 및 지터 완화

---

## 4. 개발 및 참고
- **표준**: [IEEE 802.15.4z HRP UWB](https://standards.ieee.org/)
- **안테나**: XR-170 UWB Directional Antenna
- **개발 환경**: C (Embedded), MATLAB (시각화)

---

## 5. 데모영상
- https://www.youtube.com/watch?v=SgOs7Dkw7NQ

## 6. 한국 전자파 학회 제 4회 대학생 창의설계 경진대회 동상 수상

