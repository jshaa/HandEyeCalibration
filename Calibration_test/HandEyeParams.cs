using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Calibration_test
{
    public class HandEyeParams
    {
        /// <summary> 로봇 제어기 설정과 일치하는 오일러 각 회전 순서 (기본값: ZYZ Intrinsic) </summary>
        public EEulerSequence RobotEulerSequence { get; set; } = EEulerSequence.Intrinsic_ZYZ;

        /// <summary>
        /// 로봇 제어기/상용 SW의 Euler 구현이 blackbox일 때 적용할 회전 행렬 해석 방식입니다.
        /// DefaultActive는 기존 코드와 동일합니다.
        /// </summary>
        public EulerMatrixConvention RobotEulerMatrixConvention { get; set; } = EulerMatrixConvention.DefaultActive;

        /// <summary>
        /// CSV의 Rx/Ry/Rz 열을 Euler sequence의 1/2/3번째 각도로 매핑하는 방식입니다.
        /// ForceLogic/로봇 컨트롤러가 각도 열 이름을 다르게 쓰는 경우를 검증하기 위한 설정입니다.
        /// </summary>
        public RobotAngleInputOrder RobotAngleInput { get; set; } = RobotAngleInputOrder.RxRyRz;

        /// <summary>
        /// 기존 final16 계열 코드 호환용 별칭입니다. 내부적으로 RobotAngleInput과 동일합니다.
        /// </summary>
        public RobotAngleInputOrder RobotAngleInputOrder
        {
            get { return RobotAngleInput; }
            set { RobotAngleInput = value; }
        }

        /// <summary>
        /// 선택한 Euler Sequence 안에서 CSV 각도 열 매핑과 행렬 해석 방식을 자동 비교해 최선 후보를 선택합니다.
        /// </summary>
        public bool AutoResolveRobotAngleDefinition { get; set; } = true;

        /// <summary>
        /// 선택한 Euler Sequence 안에서 행렬 해석 방식(Default/Reverse/Passive/Sign)을 자동 비교해 최선 후보를 선택합니다.
        /// 기존 호환용이며 AutoResolveRobotAngleDefinition과 함께 사용됩니다.
        /// </summary>
        public bool AutoResolveRobotEulerMatrixConvention { get; set; } = true;

        /// <summary>
        /// Euler 행렬 해석 자동 선택 시, 기존 방식 대비 최소 개선율입니다. 0.02 = 2%.
        /// </summary>
        public double MinEulerConventionImprovementRatio { get; set; } = 0.02;

        /// <summary> 로봇 제어기의 각도 단위 (기본값: Degree) </summary>
        public EAngleUnit RobotAngleUnit { get; set; } = EAngleUnit.Degree;

        /// <summary> 설비의 카메라 장착 물리 구조 (EyeToHand / EyeInHand) </summary>
        public HandEyeMode mode { get; set; } = HandEyeMode.EyeToHand;

        /// <summary>
        /// Eye-to-Hand에서 CSV 로봇 pose가 어떤 동차변환을 의미하는지 정의합니다.
        /// 일반 로봇 TCP pose는 GripperToBase이지만, 외부 SW/ForceLogic 내보내기 형식은 BaseToGripper일 수 있습니다.
        /// </summary>
        public RobotPoseTransformDirection EyeToHandRobotPoseInputDirection { get; set; } = RobotPoseTransformDirection.GripperToBase;

        /// <summary>
        /// Eye-to-Hand에서 Robot pose 방향(GripperToBase/BaseToGripper)을 3D consistency 기준으로 자동 비교합니다.
        /// </summary>
        public bool AutoResolveEyeToHandRobotPoseInputDirection { get; set; } = true;

        /// <summary>
        /// 선택한 Euler sequence 내부 진단 후에도 오차가 크면 전체 Euler sequence/method/pose 방향 sweep으로 fallback합니다.
        /// </summary>
        public bool AutoResolveFullConventionWhenHighError { get; set; } = true;

        /// <summary>
        /// Full convention fallback에서 Right/Left handedness도 함께 비교합니다.
        /// </summary>
        public bool AutoResolveHandednessWhenHighError { get; set; } = true;

        /// <summary>
        /// Full convention fallback 결과를 채택하기 위한 최소 RMS 개선율입니다. 0.05 = 5%.
        /// </summary>
        public double MinFullConventionImprovementRatio { get; set; } = 0.05;

        /// <summary>
        /// Full convention fallback에서 로그로 출력할 상위 후보 개수입니다.
        /// </summary>
        public int FullConventionFallbackTopCount { get; set; } = 12;

        /// <summary> OpenCV 내부 선형 해 도출 알고리즘 (TSAI, PARK, ANDREFF 등) </summary>
        public HandEyeCalibrationMethod CalibrationMethod { get; set; } = HandEyeCalibrationMethod.TSAI;

        /// <summary> 연산 안정성을 위한 mm 단위 변환 스케일 계수 </summary>
        private double _scale = 0.001;

        /// <summary> 기존 코드 호환용: Calibration Scale Factor </summary>
        public double scale
        {
            get { return _scale; }
            set { _scale = value; }
        }

        /// <summary> 권장 표기: Calibration Scale Factor </summary>
        public double Scale
        {
            get { return _scale; }
            set { _scale = value; }
        }

        /// <summary> 포스로직 Result Scale Factor와 동일한 의미입니다. </summary>
        public double ResultScaleFactor
        {
            get { return _scale == 0.0 ? double.NaN : 1.0 / _scale; }
        }

        /// <summary> 카메라 내부 파라미터 (Lens Distortion 및 Focal Length 정보) </summary>
        public Intrinsic CameraIntrinsic { get; set; }

        /// <summary> 선형 해 도출 후, 2D 픽셀 오차 기반의 12-DoF 비선형 최적화(Nonlinear Reprojection) 적용 여부 </summary>
        public OptimizationMethod Optimization { get; set; } = OptimizationMethod.Nonlinear_Reprojection;

        /// <summary>
        /// 기존 코드 호환용 별칭입니다. 내부적으로 Optimization과 동일합니다.
        /// </summary>
        public OptimizationMethod OptimizationMethod
        {
            get { return Optimization; }
            set { Optimization = value; }
        }

        /// <summary> 기존 코드 호환용: Nonlinear 계열 최적화 사용 여부입니다. </summary>
        public bool UseNonlinearOptimization
        {
            get { return ShouldRunNonlinearOptimization; }
            set { Optimization = value ? OptimizationMethod.Nonlinear_Reprojection : OptimizationMethod.Linear; }
        }

        /// <summary> Nonlinear 또는 Nonlinear_Reprojection 선택 시 true입니다. </summary>
        public bool ShouldRunNonlinearOptimization
        {
            get
            {
                return Optimization == OptimizationMethod.Nonlinear ||
                       Optimization == OptimizationMethod.Nonlinear_Reprojection;
            }
        }

        /// <summary> 3D 일관성(mm RMS) 기반 6-DoF 비선형 최적화 실행 여부입니다. </summary>
        public bool ShouldRunNonlinearConsistency
        {
            get { return Optimization == OptimizationMethod.Nonlinear; }
        }

        /// <summary> 2D 재투영(px RMS) 정보를 계산하는 모드입니다. </summary>
        public bool ShouldEvaluateReprojection
        {
            get
            {
                return Optimization == OptimizationMethod.Linear_Reprojection ||
                       Optimization == OptimizationMethod.Nonlinear_Reprojection;
            }
        }

        /// <summary> 12-DoF 비선형 재투영 최적화 실행 여부입니다. </summary>
        public bool ShouldRunNonlinearReprojection
        {
            get { return Optimization == OptimizationMethod.Nonlinear_Reprojection; }
        }

        public void ValidateOptimizerScale()
        {
            if (OptimizerTranslationScale <= 0.0 ||
                OptimizerTranslationRestore <= 0.0 ||
                double.IsNaN(OptimizerTranslationScale) ||
                double.IsNaN(OptimizerTranslationRestore) ||
                double.IsInfinity(OptimizerTranslationScale) ||
                double.IsInfinity(OptimizerTranslationRestore))
            {
                throw new ArgumentException(
                    $"Optimizer translation scale이 올바르지 않습니다. " +
                    $"Scale={OptimizerTranslationScale}, Restore={OptimizerTranslationRestore}");
            }

            double product = OptimizerTranslationScale * OptimizerTranslationRestore;
            if (Math.Abs(product - 1.0) > 1e-9)
            {
                throw new ArgumentException(
                    $"OptimizerTranslationScale과 OptimizerTranslationRestore는 서로 역수 관계여야 합니다. " +
                    $"Scale={OptimizerTranslationScale}, Restore={OptimizerTranslationRestore}, Product={product}");
            }
        }

        public static double[,] CreateIdentity4x4()
        {
            double[,] m = new double[4, 4];

            m[0, 0] = 1;
            m[1, 1] = 1;
            m[2, 2] = 1;
            m[3, 3] = 1;

            return m;
        }

        /// <summary> 최적화 후 도출된 타겟(보드)과 그리퍼 간의 4x4 고정 변환 행렬 </summary>
        public double[,] LastTarget2Gripper { get; set; }

        /// <summary> 비선형 최적화 후의 최종 2D 재투영 오차 (Pixel RMS) </summary>
        public double LastNonlinearReprojectionRmsPx { get; set; } = double.NaN;

        /// <summary> 마지막 비선형 최적화 결과가 실제로 채택되었는지 여부입니다. </summary>
        public bool LastNonlinearAccepted { get; set; } = false;

        /// <summary> Linear_Reprojection 모드에서 계산한 선형 결과의 2D 재투영 RMS(px)입니다. </summary>
        public double LastLinearReprojectionRmsPx { get; set; } = double.NaN;

        /// <summary> Nonlinear 모드에서 계산한 3D 일관성 최적화 RMS(mm)입니다. </summary>
        public double LastNonlinearConsistencyRmsMm { get; set; } = double.NaN;

        // --- UI 제어용 프로퍼티 및 오프셋 행렬 ---
        public EndEffectorHandedness Handedness { get; set; } = EndEffectorHandedness.Right;
        public CoordianteConvert CoordConvertMode { get; set; } = CoordianteConvert.withoutOffset;

        /// <summary> Camera Offset 4x4 행렬 (초기값: 단위행렬) </summary>
        public double[,] CameraOffsetMatrix { get; set; } = CreateIdentity4x4();

        /// <summary> Base Offset 4x4 행렬 (초기값: 단위행렬) </summary>
        public double[,] BaseOffsetMatrix { get; set; } = CreateIdentity4x4();

        public double OptimalSolutionAccuracy { get; set; } = 1e-7;

        private double _maxAcceptReprojectionRmsPx = 3.0;

        /// <summary> 기존 코드 호환용: Nonlinear Reprojection 채택 가능한 최대 RMS(px)입니다. </summary>
        public double maxAcceptReprojectionRmsPx
        {
            get { return _maxAcceptReprojectionRmsPx; }
            set { _maxAcceptReprojectionRmsPx = value; }
        }

        /// <summary> 권장 표기: Nonlinear Reprojection 채택 가능한 최대 RMS(px)입니다. </summary>
        public double MaxAcceptReprojectionRmsPx
        {
            get { return _maxAcceptReprojectionRmsPx; }
            set { _maxAcceptReprojectionRmsPx = value; }
        }

        public void ResetRuntimeResults()
        {
            LastTarget2Gripper = null;
            LastNonlinearReprojectionRmsPx = double.NaN;
            LastNonlinearAccepted = false;
            LastLinearReprojectionRmsPx = double.NaN;
            LastNonlinearConsistencyRmsMm = double.NaN;
        }

        /// <summary> Nonlinear optimizer 내부에서 mm 단위 translation을 축소하는 계수입니다. </summary>
        public double OptimizerTranslationScale { get; set; } = 0.001;   // mm -> scaled

        /// <summary> Nonlinear optimizer 결과 translation을 다시 mm 단위로 복원하는 계수입니다. </summary>
        public double OptimizerTranslationRestore { get; set; } = 1000.0; // scaled -> mm


        /// <summary> Camera2Base와 Target2Gripper를 동시에 푸는 전용 12-DoF 재투영 최적화를 사용합니다. </summary>
        public bool UseDedicatedSimultaneousReprojection12DoF { get; set; } = true;

        /// <summary>
        /// 전용 12-DoF 최적화에서 생성할 최대 seed 후보 개수입니다.
        /// 후보는 한 번씩 평가한 뒤, 성능이 좋은 일부만 실제 최적화합니다.
        /// </summary>
        public int Simultaneous12DofMaxGeneratedSeedCount { get; set; } = 96;

        /// <summary>
        /// 전용 12-DoF 최적화에서 실제 Pattern/Nelder-Mead를 수행할 최대 seed 개수입니다.
        /// 생성 seed를 모두 최적화하면 시간이 급격히 증가하므로 상위 후보만 사용합니다.
        /// </summary>
        public int Simultaneous12DofMaxOptimizedSeedCount { get; set; } = 12;

        /// <summary>
        /// 기존 코드 호환용입니다. 실제로는 Simultaneous12DofMaxOptimizedSeedCount와 동일하게 동작합니다.
        /// </summary>
        public int Simultaneous12DofMaxSeedCount
        {
            get { return Simultaneous12DofMaxOptimizedSeedCount; }
            set { Simultaneous12DofMaxOptimizedSeedCount = value; }
        }

        /// <summary> 전용 12-DoF pattern search 최대 반복 횟수입니다. </summary>
        public int Simultaneous12DofMaxPatternIterations { get; set; } = 800;

        /// <summary> 전용 12-DoF pattern search의 초기 회전 step(rad)입니다. </summary>
        public double Simultaneous12DofInitialRotationStepRad { get; set; } = 0.15;

        /// <summary> 전용 12-DoF pattern search의 초기 translation step입니다. translation은 OptimizerTranslationScale이 적용된 단위입니다. 0.05는 약 50mm입니다. </summary>
        public double Simultaneous12DofInitialTranslationStepScaled { get; set; } = 0.20;

        /// <summary> 전용 12-DoF pattern search 종료 step입니다. </summary>
        public double Simultaneous12DofMinStep { get; set; } = 1e-7;

        /// <summary> 허용 RMS(px)를 넘더라도 이전보다 개선된 12-DoF 결과를 채택할지 여부입니다. 진단 단계에서는 true가 유용합니다. </summary>
        public bool AcceptImprovedReprojectionEvenAboveLimit { get; set; } = true;

        /// <summary> 전용 12-DoF 결과 채택에 필요한 최소 개선율입니다. 0.001은 0.1% 이상 개선을 의미합니다. </summary>
        public double MinSimultaneous12DofImprovementRatio { get; set; } = 0.001;

        /// <summary>전용 12-DoF 재투영 cost에 Huber robust loss를 적용합니다. outlier pose가 초기 탐색을 끌고 가는 것을 줄입니다.</summary>
        public bool Simultaneous12DofUseRobustLoss { get; set; } = true;

        /// <summary>Huber loss 기준 픽셀 오차입니다. 이 값 이하에서는 L2, 초과하면 선형 페널티로 완화합니다.</summary>
        public double Simultaneous12DofHuberDeltaPx { get; set; } = 5.0;

        /// <summary> OpenCV Hand-Eye 및 LOO subset 계산에 필요한 최소 pose 개수입니다. </summary>
        public int MinPoseCount { get; set; } = 10;

        /// <summary> PnP 단일 pose 채택 가능한 최대 reprojection RMS(px)입니다. 초과 시 해당 이미지 pose는 실패 처리합니다. </summary>
        public double MaxAcceptPnPPoseReprojectionRmsPx { get; set; } = 2.0;

        /// <summary> Nonlinear_Reprojection 시작 전 허용하는 초기 reprojection RMS(px)입니다. 이보다 크면 pose pair/좌표계가 먼저 틀렸다고 보고 최적화를 생략합니다. </summary>
        public double MaxStartReprojectionRmsForNonlinearPx { get; set; } = 30.0;

        /// <summary>
        /// 3D consistency refinement 결과를 채택할 때 허용하는 최대 2D 재투영 RMS(px)입니다.
        /// final24 이후 Form1 저장/로그 호환용 프로퍼티입니다.
        /// </summary>
        public double Simultaneous12Dof3DRefineMaxReprojectionRmsPx { get; set; } = 2.0;


        /// <summary>
        /// 대칭 체커보드의 180도 방향 모호성을 Normal/RotZ180 후보 중에서 전역적으로 판정합니다.
        /// PnP reprojection RMS가 낮아도 corner[0]이 pose마다 반대 코너가 되는 문제를 방지합니다.
        /// </summary>
        public bool EnableCheckerboardDirectionDisambiguation { get; set; } = true;

        /// <summary>
        /// Checkerboard 방향 판정에서 사용할 pose pair의 최소 상대 회전각(deg)입니다.
        /// 너무 작은 상대 회전은 방향 판정에 도움이 적어 제외합니다.
        /// </summary>
        public double CheckerboardDirectionMinRelativeRotationDeg { get; set; } = 2.0;

        /// <summary>
        /// Checkerboard 방향 판정 cost가 기존 Normal 상태보다 최소 이 비율 이상 개선되어야 방향 보정을 채택합니다.
        /// </summary>
        public double MinCheckerboardDirectionCostImprovementRatio { get; set; } = 0.05;

        /// <summary>
        /// 방향 판정 후 3D validation RMS가 이 비율 이상 나빠지면 방향 보정을 채택하지 않습니다.
        /// </summary>
        public double MaxCheckerboardDirectionValidationWorseningRatio { get; set; } = 0.10;

        /// <summary> 체커보드가 대칭 패턴이라 FindChessboardCorners의 시작 코너가 뒤집히는 경우를 자동 보정할지 여부입니다. </summary>
        public bool EnableCheckerboardFrameAutoResolve { get; set; } = true;

        /// <summary> 체커보드 frame 후보 자동 선택의 최대 반복 횟수입니다. </summary>
        public int CheckerboardFrameResolveMaxPasses { get; set; } = 4;

        /// <summary> 체커보드 frame 후보 변경을 채택하기 위한 최소 RMS 개선 비율입니다. </summary>
        public double MinCheckerboardFrameImprovementRatio { get; set; } = 0.01;

        /// <summary> LOO를 1회만 하지 않고, 의미 있는 개선이 계속될 때 여러 pose를 순차 제외할지 여부입니다. 기본값은 false입니다. 정상 데이터라면 자동 제외보다 원인 분석이 우선입니다. </summary>
        public bool EnableIterativeOutlierRejection { get; set; } = false;

        /// <summary> 자동 LOO에서 최대로 제외할 pose 개수입니다. 너무 크게 잡으면 잘못된 convention을 outlier 제거로 가릴 수 있습니다. </summary>
        public int MaxAutoExcludedPoseCount { get; set; } = 3;

        /// <summary> 다음 LOO 제거를 채택하기 위한 최소 RMS 개선 비율입니다. 예: 0.08은 8% 이상 개선일 때만 채택합니다. </summary>
        public double MinOutlierImprovementRatio { get; set; } = 0.08;

        /// <summary> 한쪽에만 명시 PoseIndex가 있을 때, 한 칸 shift 후보가 압도적으로 좋으면 자동으로 pair를 선택할지 여부입니다. </summary>
        public bool EnableAutoPairShiftSelection { get; set; } = true;

        /// <summary> AutoPairShiftSelection 채택에 필요한 최소 RMS 개선 비율입니다. 기본 0.70은 70% 이상 RMS가 줄어야 자동 채택합니다. </summary>
        public double AutoPairShiftMinImprovementRatio { get; set; } = 0.70;

        /// <summary> 이 값보다 최종 3D RMS(mm)가 크면 결과를 정상 캘리브레이션으로 보지 않고 경고합니다. </summary>
        public double MaxRecommendedRmsErrorMm { get; set; } = 5.0;

        /// <summary> 이 값보다 최종 Max Error(mm)가 크면 결과를 정상 캘리브레이션으로 보지 않고 경고합니다. </summary>
        public double MaxRecommendedMaxErrorMm { get; set; } = 10.0;

        public void ValidateBasic()
        {
            if (MinPoseCount < 3)
                throw new ArgumentException($"MinPoseCount가 너무 작습니다. MinPoseCount={MinPoseCount}");

            if (Scale <= 0.0 || double.IsNaN(Scale) || double.IsInfinity(Scale))
                throw new ArgumentException($"Scale factor가 올바르지 않습니다. Scale={Scale}");

            if (MaxAcceptReprojectionRmsPx <= 0.0 ||
                double.IsNaN(MaxAcceptReprojectionRmsPx) ||
                double.IsInfinity(MaxAcceptReprojectionRmsPx))
            {
                throw new ArgumentException($"MaxAcceptReprojectionRmsPx가 올바르지 않습니다. MaxAcceptReprojectionRmsPx={MaxAcceptReprojectionRmsPx}");
            }

            if (OptimalSolutionAccuracy <= 0.0 ||
                double.IsNaN(OptimalSolutionAccuracy) ||
                double.IsInfinity(OptimalSolutionAccuracy))
            {
                throw new ArgumentException($"OptimalSolutionAccuracy가 올바르지 않습니다. OptimalSolutionAccuracy={OptimalSolutionAccuracy}");
            }

            if (MaxAcceptPnPPoseReprojectionRmsPx <= 0.0 ||
                double.IsNaN(MaxAcceptPnPPoseReprojectionRmsPx) ||
                double.IsInfinity(MaxAcceptPnPPoseReprojectionRmsPx))
            {
                throw new ArgumentException($"MaxAcceptPnPPoseReprojectionRmsPx가 올바르지 않습니다. MaxAcceptPnPPoseReprojectionRmsPx={MaxAcceptPnPPoseReprojectionRmsPx}");
            }

            if (MaxStartReprojectionRmsForNonlinearPx <= MaxAcceptReprojectionRmsPx ||
                double.IsNaN(MaxStartReprojectionRmsForNonlinearPx) ||
                double.IsInfinity(MaxStartReprojectionRmsForNonlinearPx))
            {
                throw new ArgumentException(
                    $"MaxStartReprojectionRmsForNonlinearPx는 MaxAcceptReprojectionRmsPx보다 커야 합니다. " +
                    $"StartLimit={MaxStartReprojectionRmsForNonlinearPx}, AcceptLimit={MaxAcceptReprojectionRmsPx}");
            }

            if (CheckerboardDirectionMinRelativeRotationDeg < 0.0 ||
                double.IsNaN(CheckerboardDirectionMinRelativeRotationDeg) ||
                double.IsInfinity(CheckerboardDirectionMinRelativeRotationDeg))
            {
                throw new ArgumentException($"CheckerboardDirectionMinRelativeRotationDeg가 올바르지 않습니다. CheckerboardDirectionMinRelativeRotationDeg={CheckerboardDirectionMinRelativeRotationDeg}");
            }

            if (MinCheckerboardDirectionCostImprovementRatio < 0.0 ||
                double.IsNaN(MinCheckerboardDirectionCostImprovementRatio) ||
                double.IsInfinity(MinCheckerboardDirectionCostImprovementRatio))
            {
                throw new ArgumentException($"MinCheckerboardDirectionCostImprovementRatio가 올바르지 않습니다. MinCheckerboardDirectionCostImprovementRatio={MinCheckerboardDirectionCostImprovementRatio}");
            }

            if (MaxCheckerboardDirectionValidationWorseningRatio < 0.0 ||
                double.IsNaN(MaxCheckerboardDirectionValidationWorseningRatio) ||
                double.IsInfinity(MaxCheckerboardDirectionValidationWorseningRatio))
            {
                throw new ArgumentException($"MaxCheckerboardDirectionValidationWorseningRatio가 올바르지 않습니다. MaxCheckerboardDirectionValidationWorseningRatio={MaxCheckerboardDirectionValidationWorseningRatio}");
            }

            if (Simultaneous12Dof3DRefineMaxReprojectionRmsPx <= 0.0 ||
                double.IsNaN(Simultaneous12Dof3DRefineMaxReprojectionRmsPx) ||
                double.IsInfinity(Simultaneous12Dof3DRefineMaxReprojectionRmsPx))
            {
                throw new ArgumentException($"Simultaneous12Dof3DRefineMaxReprojectionRmsPx가 올바르지 않습니다. Simultaneous12Dof3DRefineMaxReprojectionRmsPx={Simultaneous12Dof3DRefineMaxReprojectionRmsPx}");
            }

            if (CheckerboardFrameResolveMaxPasses < 0)
                throw new ArgumentException($"CheckerboardFrameResolveMaxPasses가 올바르지 않습니다. CheckerboardFrameResolveMaxPasses={CheckerboardFrameResolveMaxPasses}");

            if (MinCheckerboardFrameImprovementRatio < 0.0 ||
                double.IsNaN(MinCheckerboardFrameImprovementRatio) ||
                double.IsInfinity(MinCheckerboardFrameImprovementRatio))
            {
                throw new ArgumentException($"MinCheckerboardFrameImprovementRatio가 올바르지 않습니다. MinCheckerboardFrameImprovementRatio={MinCheckerboardFrameImprovementRatio}");
            }

            if (MaxAutoExcludedPoseCount < 0)
                throw new ArgumentException($"MaxAutoExcludedPoseCount가 올바르지 않습니다. MaxAutoExcludedPoseCount={MaxAutoExcludedPoseCount}");

            if (MinOutlierImprovementRatio < 0.0 || MinOutlierImprovementRatio >= 1.0 ||
                double.IsNaN(MinOutlierImprovementRatio) ||
                double.IsInfinity(MinOutlierImprovementRatio))
            {
                throw new ArgumentException($"MinOutlierImprovementRatio가 올바르지 않습니다. MinOutlierImprovementRatio={MinOutlierImprovementRatio}");
            }

            if (AutoPairShiftMinImprovementRatio < 0.0 || AutoPairShiftMinImprovementRatio >= 1.0 ||
                double.IsNaN(AutoPairShiftMinImprovementRatio) ||
                double.IsInfinity(AutoPairShiftMinImprovementRatio))
            {
                throw new ArgumentException($"AutoPairShiftMinImprovementRatio가 올바르지 않습니다. AutoPairShiftMinImprovementRatio={AutoPairShiftMinImprovementRatio}");
            }

            if (MaxRecommendedRmsErrorMm <= 0.0 ||
                double.IsNaN(MaxRecommendedRmsErrorMm) ||
                double.IsInfinity(MaxRecommendedRmsErrorMm))
            {
                throw new ArgumentException($"MaxRecommendedRmsErrorMm가 올바르지 않습니다. MaxRecommendedRmsErrorMm={MaxRecommendedRmsErrorMm}");
            }

            if (MaxRecommendedMaxErrorMm <= 0.0 ||
                double.IsNaN(MaxRecommendedMaxErrorMm) ||
                double.IsInfinity(MaxRecommendedMaxErrorMm))
            {
                throw new ArgumentException($"MaxRecommendedMaxErrorMm가 올바르지 않습니다. MaxRecommendedMaxErrorMm={MaxRecommendedMaxErrorMm}");
            }

            if (MinFullConventionImprovementRatio < 0.0 || MinFullConventionImprovementRatio >= 1.0 ||
                double.IsNaN(MinFullConventionImprovementRatio) ||
                double.IsInfinity(MinFullConventionImprovementRatio))
            {
                throw new ArgumentException($"MinFullConventionImprovementRatio가 올바르지 않습니다. MinFullConventionImprovementRatio={MinFullConventionImprovementRatio}");
            }

            if (FullConventionFallbackTopCount <= 0)
                throw new ArgumentException($"FullConventionFallbackTopCount가 올바르지 않습니다. FullConventionFallbackTopCount={FullConventionFallbackTopCount}");


            if (Simultaneous12DofMaxGeneratedSeedCount <= 0)
                throw new ArgumentException($"Simultaneous12DofMaxGeneratedSeedCount가 올바르지 않습니다. Simultaneous12DofMaxGeneratedSeedCount={Simultaneous12DofMaxGeneratedSeedCount}");

            if (Simultaneous12DofMaxOptimizedSeedCount <= 0)
                throw new ArgumentException($"Simultaneous12DofMaxOptimizedSeedCount가 올바르지 않습니다. Simultaneous12DofMaxOptimizedSeedCount={Simultaneous12DofMaxOptimizedSeedCount}");

            if (Simultaneous12DofMaxOptimizedSeedCount > Simultaneous12DofMaxGeneratedSeedCount)
                throw new ArgumentException(
                    $"Simultaneous12DofMaxOptimizedSeedCount는 Simultaneous12DofMaxGeneratedSeedCount보다 클 수 없습니다. " +
                    $"Generated={Simultaneous12DofMaxGeneratedSeedCount}, Optimized={Simultaneous12DofMaxOptimizedSeedCount}");

            if (Simultaneous12DofMaxPatternIterations <= 0)
                throw new ArgumentException($"Simultaneous12DofMaxPatternIterations가 올바르지 않습니다. Simultaneous12DofMaxPatternIterations={Simultaneous12DofMaxPatternIterations}");

            if (Simultaneous12DofInitialRotationStepRad <= 0.0 ||
                double.IsNaN(Simultaneous12DofInitialRotationStepRad) ||
                double.IsInfinity(Simultaneous12DofInitialRotationStepRad))
            {
                throw new ArgumentException($"Simultaneous12DofInitialRotationStepRad가 올바르지 않습니다. Simultaneous12DofInitialRotationStepRad={Simultaneous12DofInitialRotationStepRad}");
            }

            if (Simultaneous12DofInitialTranslationStepScaled <= 0.0 ||
                double.IsNaN(Simultaneous12DofInitialTranslationStepScaled) ||
                double.IsInfinity(Simultaneous12DofInitialTranslationStepScaled))
            {
                throw new ArgumentException($"Simultaneous12DofInitialTranslationStepScaled가 올바르지 않습니다. Simultaneous12DofInitialTranslationStepScaled={Simultaneous12DofInitialTranslationStepScaled}");
            }

            if (Simultaneous12DofMinStep <= 0.0 ||
                double.IsNaN(Simultaneous12DofMinStep) ||
                double.IsInfinity(Simultaneous12DofMinStep))
            {
                throw new ArgumentException($"Simultaneous12DofMinStep가 올바르지 않습니다. Simultaneous12DofMinStep={Simultaneous12DofMinStep}");
            }

            if (MinSimultaneous12DofImprovementRatio < 0.0 || MinSimultaneous12DofImprovementRatio >= 1.0 ||
                double.IsNaN(MinSimultaneous12DofImprovementRatio) ||
                double.IsInfinity(MinSimultaneous12DofImprovementRatio))
            {
                throw new ArgumentException($"MinSimultaneous12DofImprovementRatio가 올바르지 않습니다. MinSimultaneous12DofImprovementRatio={MinSimultaneous12DofImprovementRatio}");
            }

            if (Simultaneous12DofHuberDeltaPx <= 0.0 ||
                double.IsNaN(Simultaneous12DofHuberDeltaPx) ||
                double.IsInfinity(Simultaneous12DofHuberDeltaPx))
            {
                throw new ArgumentException($"Simultaneous12DofHuberDeltaPx가 올바르지 않습니다. Simultaneous12DofHuberDeltaPx={Simultaneous12DofHuberDeltaPx}");
            }
        }
    }


    /// <summary>
    /// 로봇 제어기(메이커)별로 상이한 TCP 오일러 각(Euler Angle)의 회전 순서 및 축 기준을 정의합니다.
    /// </summary>
    public enum EEulerSequence
    {
        Extrinsic_XYZ,
        Extrinsic_XZY,
        Extrinsic_YXZ,
        Extrinsic_YZX,
        Extrinsic_ZXY,
        Extrinsic_ZYX,
        Extrinsic_ZYZ,
        Intrinsic_XYZ,
        Intrinsic_XZY,
        Intrinsic_YXZ,
        Intrinsic_YZX,
        Intrinsic_ZXY,
        Intrinsic_ZYX,
        Intrinsic_ZYZ,
        RotVecDegree,
        RotVecRadian
    }

    /// <summary>
    /// Euler 각을 회전 행렬로 만드는 해석 방식입니다.
    /// ForceLogic 같은 상용 SW의 Int_ZYZ 정의가 공개되어 있지 않을 때,
    /// 같은 Rx/Ry/Rz 값에 대해 곱셈 순서/수동 회전/부호 반전을 실험적으로 비교합니다.
    /// </summary>
    public enum EulerMatrixConvention
    {
        /// <summary>기존 코드 방식. Intrinsic: R1*R2*R3, Extrinsic: R3*R2*R1.</summary>
        DefaultActive,

        /// <summary>기존 방식의 곱셈 순서를 반대로 적용합니다.</summary>
        ReverseOrderActive,

        /// <summary>기존 방식으로 만든 회전 행렬을 transpose하여 passive/좌표계 회전 해석을 시험합니다.</summary>
        DefaultPassive,

        /// <summary>반대 곱셈 순서 결과를 transpose합니다.</summary>
        ReverseOrderPassive,

        /// <summary>각도 부호를 모두 반전한 뒤 기존 곱셈 순서를 적용합니다.</summary>
        NegatedDefaultActive,

        /// <summary>각도 부호를 모두 반전한 뒤 반대 곱셈 순서를 적용합니다.</summary>
        NegatedReverseOrderActive,

        /// <summary>각도 부호 반전 + 기존 곱셈 순서 + transpose입니다.</summary>
        NegatedDefaultPassive,

        /// <summary>각도 부호 반전 + 반대 곱셈 순서 + transpose입니다.</summary>
        NegatedReverseOrderPassive
    }

    /// <summary>
    /// CSV의 Rx/Ry/Rz 세 값을 Euler sequence의 첫 번째/두 번째/세 번째 각도로 넣는 순서입니다.
    /// 예를 들어 Intrinsic_ZYZ에서 RxRyRz는 (A=Rx, B=Ry, C=Rz), RzRyRx는 (A=Rz, B=Ry, C=Rx)로 해석합니다.
    /// </summary>
    public enum RobotAngleInputOrder
    {
        RxRyRz,
        RxRzRy,
        RyRxRz,
        RyRzRx,
        RzRxRy,
        RzRyRx
    }

    public class Simultaneous12DofSeed
    {
        public string Name { get; set; }
        public double[] Parameters { get; set; }
    }

    public class Simultaneous12DofSeedScore
    {
        public Simultaneous12DofSeed Seed { get; set; }
        public double OptimizeCost { get; set; }
        public double TrueCost { get; set; }
        public double OptimizeRms { get { return SafeSqrtStatic(OptimizeCost); } }
        public double TrueRms { get { return SafeSqrtStatic(TrueCost); } }

        public static double SafeSqrtStatic(double cost)
        {
            if (double.IsNaN(cost) || double.IsInfinity(cost) || cost < 0.0 || cost >= double.MaxValue / 8.0)
                return double.NaN;

            return Math.Sqrt(cost);
        }
    }


    /// <summary>
    /// 로봇 제어기에서 전달받는 회전각의 단위를 정의합니다.
    /// </summary>
    public enum EAngleUnit
    {
        Degree,
        Radian
    }

    /// <summary>
    /// 카메라의 장착 위치에 따른 Hand-Eye 캘리브레이션 물리적 모드를 정의합니다.
    /// </summary>
    public enum HandEyeMode
    {
        /// <summary> 카메라는 외부에 고정되어 있고, 로봇(그리퍼)이 타겟을 들고 움직이는 환경 </summary>
        EyeToHand,
        /// <summary> 카메라가 로봇(그리퍼)의 끝단에 부착되어 로봇과 함께 움직이는 환경 </summary>
        EyeInHand
    }

    public enum RobotPoseTransformDirection
    {
        /// <summary>CSV pose가 T_gripper2base, 즉 TCP/Gripper frame에서 Base frame으로 가는 변환입니다.</summary>
        GripperToBase,

        /// <summary>CSV pose가 T_base2gripper, 즉 Base frame에서 TCP/Gripper frame으로 가는 변환입니다.</summary>
        BaseToGripper
    }

    public enum EndEffectorHandedness
    {
        Right,
        Left
    }

    public enum OptimizationMethod
    {
        Linear,
        Nonlinear,
        Linear_Reprojection,
        Nonlinear_Reprojection
    }

    public enum CoordianteConvert
    {
        withCameraOffset,
        withBaseOffset,
        withoutOffset
    }


    public class LeaveOneOutResult
    {
        public int RemovedPoseNumber { get; set; }
        public List<RobotTargetPose> RobotList { get; set; }
        public List<CameraTargetPose> CameraList { get; set; }
        public double[,] Matrix { get; set; }
        public HandEyeValidationResult Validation { get; set; }
    }

    public class PosePairSet
    {
        public List<RobotTargetPose> RobotList { get; set; } = new List<RobotTargetPose>();
        public List<CameraTargetPose> CameraList { get; set; } = new List<CameraTargetPose>();
        public bool MatchedByExplicitIndex { get; set; }
        public string PairSelectionNote { get; set; } = "Original";
    }

    public class PosePairSelectionCandidate
    {
        public string Name { get; set; }
        public List<RobotTargetPose> RobotList { get; set; }
        public List<CameraTargetPose> CameraList { get; set; }
        public HandEyeValidationResult Validation { get; set; }
        public bool IsOriginal { get; set; }
    }

    public class OutlierSelectionResult
    {
        public List<RobotTargetPose> RobotList { get; set; }
        public List<CameraTargetPose> CameraList { get; set; }
        public List<int> RemovedPoseNumbers { get; set; } = new List<int>();
        public HandEyeValidationResult Validation { get; set; }
        public double[,] Matrix { get; set; }
    }

    public class CheckerboardDirectionEdge
    {
        public int I { get; set; }
        public int J { get; set; }
        public double[,] Cost { get; set; }
    }

    public class CheckerboardDirectionSolution
    {
        public int[] Labels { get; set; }
        public double Cost { get; set; }
        public int FlipCount { get; set; }
    }

    public class EulerConventionTestResult
    {
        public EulerMatrixConvention Convention { get; set; }
        public RobotAngleInputOrder AngleInputOrder { get; set; }
        public RobotPoseTransformDirection RobotPoseDirection { get; set; }
        public double RmsError { get; set; }
        public double MaxError { get; set; }
    }

    public class FullConventionFallbackCandidate
    {
        public HandEyeSweepResult Result { get; set; }
        public EndEffectorHandedness Handedness { get; set; }
        public RobotPoseTransformDirection RobotPoseDirection { get; set; }
    }

    /// <summary>
    /// 모든 회전 시퀀스 및 알고리즘 조합(Sweep)을 테스트한 후 도출되는 단일 검증 결과 모델입니다.
    /// </summary>
    public class HandEyeSweepResult
    {
        public EEulerSequence RotationType { get; set; }
        public EulerMatrixConvention EulerConvention { get; set; } = EulerMatrixConvention.DefaultActive;
        public RobotAngleInputOrder AngleInputOrder { get; set; } = RobotAngleInputOrder.RxRyRz;
        public bool InvertRobotPose { get; set; }
        public bool InvertCameraPose { get; set; }
        public HandEyeCalibrationMethod Method { get; set; }

        public double RmsError { get; set; }
        public double MaxError { get; set; }
        public double StdDevX { get; set; }
        public double StdDevY { get; set; }
        public double StdDevZ { get; set; }

        public double CamToGripperX { get; set; }
        public double CamToGripperY { get; set; }
        public double CamToGripperZ { get; set; }
        public double CamToGripperNorm { get; set; }

        public double[,] T_Cam2Gripper { get; set; }
    }

    /// <summary>
    /// 캘리브레이션 완료 후, 단일 로봇 포즈에 대한 타겟의 절대 좌표 및 편차(오차)를 담는 모델입니다.
    /// </summary>
    public class HandEyeValidationPose
    {
        public int PoseIndex { get; set; }

        // Robot Gripper in Base
        public double[,] T_Gripper2Base { get; set; }

        // Camera in Base, calculated by Hand-Eye
        public double[,] T_Cam2Base { get; set; }

        // Target in Base, should converge to one point
        public double[,] T_Target2Base { get; set; }

        public double[,] T_Target2Gripper { get; set; }

        public double TargetX { get; set; }
        public double TargetY { get; set; }
        public double TargetZ { get; set; }

        public double ErrorX { get; set; }
        public double ErrorY { get; set; }
        public double ErrorZ { get; set; }
        public double ErrorNorm { get; set; }
    }

    /// <summary>
    /// 전체 포즈 세트에 대한 캘리브레이션 종합 검증 및 통계(오차율) 결과를 제공합니다.
    /// </summary>
    public class HandEyeValidationResult
    {
        public List<HandEyeValidationPose> Poses { get; set; } = new List<HandEyeValidationPose>();

        public double MeanTargetX { get; set; }
        public double MeanTargetY { get; set; }
        public double MeanTargetZ { get; set; }

        public double RmsError { get; set; }
        public double MaxError { get; set; }

        public double StdDevX { get; set; }
        public double StdDevY { get; set; }
        public double StdDevZ { get; set; }
    }

    /// <summary>
    /// AX = XB motion equation 잔차입니다. Translation 단위는 현재 pose 데이터 단위(mm)입니다.
    /// </summary>
    public class HandEyeAxXbResidualResult
    {
        public string PairMode { get; set; }
        public int PoseCount { get; set; }
        public int PairCount { get; set; }

        public double RotationRmseRad { get; set; }
        public double RotationMaxRad { get; set; }
        public double TranslationRmse { get; set; }
        public double TranslationMax { get; set; }

        public double RotationRmseDeg
        {
            get { return RotationRmseRad * 180.0 / Math.PI; }
        }

        public double RotationMaxDeg
        {
            get { return RotationMaxRad * 180.0 / Math.PI; }
        }
    }

    /// <summary>
    /// 캘리브레이션 검증에 대한 통계 지표(평균 및 표준편차) 모델입니다.
    /// </summary>
    public class CalibrationMetrics
    {
        public double MeanX { get; set; }
        public double MeanY { get; set; }
        public double MeanZ { get; set; }

        public double StdDevX { get; set; }
        public double StdDevY { get; set; }
        public double StdDevZ { get; set; }

        public override string ToString()
        {
            return $"[오차 표준편차] X: {StdDevX:F3}mm, Y: {StdDevY:F3}mm, Z: {StdDevZ:F3}mm\n" +
                   $"[타겟 평균좌표] X: {MeanX:F3}, Y: {MeanY:F3}, Z: {MeanZ:F3}";
        }
    }

    /// <summary>
    /// Helix Toolkit 등 3D 뷰어 렌더링에 필요한 로봇 End Effector의 4x4 Transform 정보를 제공합니다.
    /// </summary>
    public class UIEndEffectorData
    {
        public int PoseIndex { get; set; }
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public string Label { get; set; }
        public string InfoText { get; set; }
        public double[,] TransformMatrix4x4 { get; set; } // 3D 엔진(Helix Toolkit 등) Transform3D용
    }

    /// <summary>
    /// 비전 시스템 측에서 PnP(Perspective-n-Point)를 통해 검출한 타겟(캘리브레이션 보드)의 공간 좌표 모델입니다.
    /// </summary>
    public class CameraTargetPose
    {
        public double X { get; set; }   // mm
        public double Y { get; set; }   // mm
        public double Z { get; set; }   // mm

        public double Rx { get; set; }  // deg
        public double Ry { get; set; }  // deg
        public double Rz { get; set; }  // deg

        public double[,] RotationMatrix { get; set; }

        public double ReprojectionRmsPx { get; set; }
        public double ReprojectionMaxPx { get; set; }

        public Point2f[] ImagePoints { get; set; }
        public Point3f[] ObjectPoints { get; set; }

        /// <summary> 체커보드 대칭성 때문에 가능한 target frame 후보들입니다. Normal/RotZ180/FlipX/FlipY 등을 포함할 수 있습니다. </summary>
        public List<CameraTargetPose> FrameCandidates { get; set; }

        /// <summary> 현재 선택된 체커보드 target frame 후보 이름입니다. </summary>
        public string FrameCandidateName { get; set; } = "Normal";

        public int SourcePoseIndex { get; set; }

        /// <summary>
        /// SourcePoseIndex가 파일명 또는 CSV의 PoseIndex 열처럼 원본 데이터에서 명시적으로 얻어진 값인지 여부입니다.
        /// false인 경우에는 단순 로드 순서 기반 자동 번호입니다.
        /// </summary>
        public bool HasExplicitPoseIndex { get; set; }
    }

    public class RobotTargetPose
    {
        public double robotTx { get; set; }
        public double robotTy { get; set; }
        public double robotTz { get; set; }
        public double robotRx { get; set; }
        public double robotRy { get; set; }
        public double robotRz { get; set; }

        public int SourcePoseIndex { get; set; }

        /// <summary>
        /// SourcePoseIndex가 CSV의 PoseIndex 열처럼 원본 데이터에서 명시적으로 얻어진 값인지 여부입니다.
        /// false인 경우에는 단순 로드 순서 기반 자동 번호입니다.
        /// </summary>
        public bool HasExplicitPoseIndex { get; set; }
    }

    public class Intrinsic
    {
        /// <summary> Width of the image in pixels </summary>
        public double width;

        /// <summary> Height of the image in pixels </summary>
        public double height;

        /// <summary> Horizontal coordinate of the principal point of the image, as a pixel offset from the left edge </summary>
        public double ppx;

        /// <summary> Vertical coordinate of the principal point of the image, as a pixel offset from the top edge </summary>
        public double ppy;

        /// <summary> Focal length of the image plane, as a multiple of pixel width </summary>
        public double fx;

        /// <summary> Focal length of the image plane, as a multiple of pixel height </summary>
        public double fy;

        /// <summary> Distortion coefficients, order: k1, k2, p1, p2, k3 </summary>
        public double[] coeffs;

        /// <summary>
        /// Gets the horizontal and vertical field of view, based on video intrinsics
        /// </summary>
        /// <value>horizontal and vertical field of view in degrees</value>
    }

    public class CalibrationDataException : Exception
    {
        public CalibrationDataException(string message) : base(message) { }
    }
}
