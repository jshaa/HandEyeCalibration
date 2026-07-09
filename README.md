# Hand-Eye Calibration (C#)

## 📖 프로젝트 개요
이 프로젝트는 스마트팩토리 및 머신비전 환경에서 로봇의 End Effector(TCP)와 비전 카메라 간의 관계 행렬(Transformation Matrix)을 도출하는 **Hand-Eye Calibration** C# 라이브러리입니다.
OpenCV(`OpenCvSharp`)와 `MathNet.Numerics`를 활용하여 선형 해 도출부터 12-DoF 비선형 재투영 최적화까지 지원하며, 상용 로봇 제어기마다 다른 오일러 각도(Euler Angle) 규칙을 자동으로 탐색하고 보정하는 기능을 포함합니다.

## ✨ 주요 기능
* **다양한 캘리브레이션 모드 지원:** Eye-to-Hand 및 Eye-in-Hand 환경 모두 지원
* **고급 최적화 알고리즘:** 선형 캘리브레이션(Tsai, Park, Horaud, Andreff) 외에도 Nelder-Mead 알고리즘과 Huber Robust Loss를 적용한 12-DoF 동시 재투영 비선형 최적화(Nonlinear Reprojection Optimization) 수행
* **체커보드 모호성 자동 해결:** PnP 결과에서 흔히 발생하는 체커보드 180도 방향 뒤집힘 현상을 로봇의 상대 운동(Relative Motion) 비용 함수를 통해 전역적으로 탐색 및 보정
* **로봇 좌표계 자동 매핑:** 제조사별로 상이한 오일러 시퀀스(예: Intrinsic ZYZ, Extrinsic XYZ 등)와 행렬 해석 방식(Default, ReverseOrder, Passive 등)을 자동 비교하여 최적의 규칙 적용
* **이상치(Outlier) 제거:** Leave-One-Out (LOO) 교차 검증을 통해 오차율을 높이는 비정상 포즈(Pose) 데이터를 반복적으로 찾아내어 제거

## 🏗️ 프로젝트 구조 및 핵심 클래스
* `CalibrationRuntime.cs`: HandEyeParams, MathUtils, Optimizer, Calibration, BoardDetector 등 모든 서비스의 수명 주기와 의존성을 관리하는 Composition Root 역할의 클래스입니다.
* `HandEyeCalibration.cs`: 캘리브레이션의 메인 클래스로, 누적된 로봇 및 카메라 포즈 데이터를 바탕으로 OpenCV 함수를 호출하여 초기 4x4 동차 변환 행렬을 도출하고 검증 지표(RMS 등)를 계산합니다.
* `CalibrationOptimizer.cs`: 초기 해 도출 이후, 재투영 오차를 최소화하기 위해 Camera-to-Base와 Target-to-Gripper 행렬을 동시에 최적화하는 12-DoF 비선형 최적화를 담당합니다.
* `BoardDetector.cs`: 입력 영상에서 체커보드 코너를 검출(SubPix 적용)하고, `SolvePnP`를 통해 카메라 기준 타겟의 6-DoF 포즈(`CameraTargetPose`)를 추정하는 비전 서비스입니다.
* `MathUtils.cs`: 4x4 동차 변환, 3x3 회전 행렬, 쿼터니언(Quaternion), 로드리게스(Rodrigues) 회전 벡터 간의 변환과 수학적 유효성 검증을 처리하는 유틸리티입니다.
* `HandEyeParams.cs`: 캘리브레이션 모드, 최적화 방식, 허용 오차 등 프로젝트 전반에 사용되는 설정값(Parameters)을 관리합니다.

## 🛠️ 기술 스택
* **Language:** C#
* **Libraries:** OpenCvSharp4 (비전 처리 및 PnP), MathNet.Numerics (비선형 최적화 및 수학 연산), SharpDX (UI 표기)
