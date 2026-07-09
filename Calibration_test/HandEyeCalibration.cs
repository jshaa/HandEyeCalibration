using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.Optimization;
using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Calibration_test
{
    /// <summary>
    /// 로봇의 End Effector(TCP)와 비전 카메라 간의 관계 행렬(Transformation Matrix)을
    /// 도출하고 검증하는 Hand-Eye Calibration 메인 클래스입니다.
    /// </summary>
    public class HandEyeCalibration : IDisposable
    {
        private readonly List<Mat> R_gripper2base = new List<Mat>();
        private readonly List<Mat> t_gripper2base = new List<Mat>();
        private readonly List<Mat> R_target2cam = new List<Mat>();
        private readonly List<Mat> t_target2cam = new List<Mat>();


        private readonly List<int> poseIndexList = new List<int>();

        private HandEyeParams HandEyeParams;
        private MathUtils MathUtils;
        private CalibrationOptimizer Optimizer;

        public HandEyeParams Params
        {
            get { return HandEyeParams; }
        }

        public MathUtils Utils
        {
            get { return MathUtils; }
        }

        public CalibrationOptimizer OptimizerService
        {
            get { return Optimizer; }
        }

        public HandEyeCalibration()
            : this(new HandEyeParams(), null, null)
        {
        }

        public HandEyeCalibration(HandEyeParams parameters)
            : this(parameters, null, null)
        {
        }

        public HandEyeCalibration(
            HandEyeParams parameters,
            MathUtils mathUtils,
            CalibrationOptimizer optimizer)
        {
            SetDependencies(parameters, mathUtils, optimizer);
        }

        private void SetDependencies(
            HandEyeParams parameters,
            MathUtils mathUtils,
            CalibrationOptimizer optimizer)
        {
            HandEyeParams = parameters ?? throw new ArgumentNullException(nameof(parameters));
            MathUtils = mathUtils ?? new MathUtils(HandEyeParams);
            MathUtils.SetParams(HandEyeParams);

            Optimizer = optimizer ?? new CalibrationOptimizer(HandEyeParams, MathUtils);
            Optimizer.SetDependencies(HandEyeParams, MathUtils);
        }

        // 기존 Form1 코드와의 호환을 위한 wrapper 프로퍼티입니다.
        public EEulerSequence RobotEulerSequence
        {
            get { return HandEyeParams.RobotEulerSequence; }
            set { HandEyeParams.RobotEulerSequence = value; }
        }

        public EulerMatrixConvention RobotEulerMatrixConvention
        {
            get { return HandEyeParams.RobotEulerMatrixConvention; }
            set { HandEyeParams.RobotEulerMatrixConvention = value; }
        }

        public RobotAngleInputOrder RobotAngleInputOrder
        {
            get { return HandEyeParams.RobotAngleInput; }
            set { HandEyeParams.RobotAngleInput = value; }
        }

        public bool AutoResolveRobotAngleDefinition
        {
            get { return HandEyeParams.AutoResolveRobotAngleDefinition; }
            set { HandEyeParams.AutoResolveRobotAngleDefinition = value; }
        }

        public bool AutoResolveRobotEulerMatrixConvention
        {
            get { return HandEyeParams.AutoResolveRobotEulerMatrixConvention; }
            set { HandEyeParams.AutoResolveRobotEulerMatrixConvention = value; }
        }

        public EAngleUnit RobotAngleUnit
        {
            get { return HandEyeParams.RobotAngleUnit; }
            set { HandEyeParams.RobotAngleUnit = value; }
        }

        public HandEyeMode mode
        {
            get { return HandEyeParams.mode; }
            set { HandEyeParams.mode = value; }
        }

        public HandEyeCalibrationMethod CalibrationMethod
        {
            get { return HandEyeParams.CalibrationMethod; }
            set { HandEyeParams.CalibrationMethod = value; }
        }

        public double scale
        {
            get { return HandEyeParams.scale; }
            set { HandEyeParams.scale = value; }
        }

        public Intrinsic CameraIntrinsic
        {
            get { return HandEyeParams.CameraIntrinsic; }
            set { HandEyeParams.CameraIntrinsic = value; }
        }

        public OptimizationMethod OptimizationMethod
        {
            get { return HandEyeParams.Optimization; }
            set { HandEyeParams.Optimization = value; }
        }

        public bool UseNonlinearOptimization
        {
            get { return HandEyeParams.UseNonlinearOptimization; }
            set { HandEyeParams.UseNonlinearOptimization = value; }
        }

        public double[,] LastTarget2Gripper
        {
            get { return HandEyeParams.LastTarget2Gripper; }
        }

        public double LastNonlinearReprojectionRmsPx
        {
            get { return HandEyeParams.LastNonlinearReprojectionRmsPx; }
        }

        public bool LastNonlinearAccepted
        {
            get { return HandEyeParams.LastNonlinearAccepted; }
        }

        public EndEffectorHandedness Handedness
        {
            get { return HandEyeParams.Handedness; }
            set { HandEyeParams.Handedness = value; }
        }

        public CoordianteConvert CoordConvertMode
        {
            get { return HandEyeParams.CoordConvertMode; }
            set { HandEyeParams.CoordConvertMode = value; }
        }

        public double[,] CameraOffsetMatrix
        {
            get { return HandEyeParams.CameraOffsetMatrix; }
            set { HandEyeParams.CameraOffsetMatrix = value; }
        }

        public double[,] BaseOffsetMatrix
        {
            get { return HandEyeParams.BaseOffsetMatrix; }
            set { HandEyeParams.BaseOffsetMatrix = value; }
        }

        public double OptimalSolutionAccuracy
        {
            get { return HandEyeParams.OptimalSolutionAccuracy; }
            set { HandEyeParams.OptimalSolutionAccuracy = value; }
        }

        public double MaxAcceptReprojectionRmsPx
        {
            get { return HandEyeParams.MaxAcceptReprojectionRmsPx; }
            set { HandEyeParams.MaxAcceptReprojectionRmsPx = value; }
        }

        /// <summary>
        /// 메모리에 누적된 OpenCV Mat 포즈 데이터와 리스트를 모두 해제하고 초기화합니다.
        /// </summary>
        public void Clear()
        {
            DisposeList(R_gripper2base);
            DisposeList(t_gripper2base);
            DisposeList(R_target2cam);
            DisposeList(t_target2cam);

            poseIndexList.Clear();
        }
        private void DisposeList(List<Mat> list)
        {
            foreach (Mat mat in list)
                mat?.Dispose();

            list.Clear();
        }
        public void Dispose()
        {
            Clear();
        }

        /// <summary>
        /// Sweep / Leave-One-Out 반복 시 Mat 객체가 누적(Memory Leak)되지 않도록 기존 데이터를 삭제합니다.
        /// </summary>
        public void CLEAR_POSE()
        {
            // Sweep / Leave-One-Out 반복 시 Mat 객체가 누적되지 않도록 Dispose 후 Clear합니다.
            Clear();
        }

        public void SET_PARAMS(HandEyeParams _HandEyeParams)
        {
            SetDependencies(
                _HandEyeParams ?? new HandEyeParams(),
                MathUtils,
                Optimizer);
        }

        private int ResolvePoseIndex(RobotTargetPose r_pose, CameraTargetPose c_pose)
        {
            // 명시적으로 얻은 PoseIndex를 우선 사용합니다.
            // Robot CSV에 PoseIndex 열이 없고 이미지 파일명만 2.png, 3.png처럼 실제 번호를 가진 경우,
            // 자동 생성된 Robot index보다 Camera의 명시 index가 우선되어야 합니다.
            if (r_pose != null && r_pose.HasExplicitPoseIndex && r_pose.SourcePoseIndex > 0)
                return r_pose.SourcePoseIndex;

            if (c_pose != null && c_pose.HasExplicitPoseIndex && c_pose.SourcePoseIndex > 0)
                return c_pose.SourcePoseIndex;

            if (r_pose != null && r_pose.SourcePoseIndex > 0)
                return r_pose.SourcePoseIndex;

            if (c_pose != null && c_pose.SourcePoseIndex > 0)
                return c_pose.SourcePoseIndex;

            return poseIndexList.Count + 1;
        }

        /// <summary>
        /// 캘리브레이션을 위한 로봇 TCP 포즈와 카메라 비전 포즈 데이터를 1세트 누적합니다. (최소 10세트 권장)
        /// </summary>
        /// <param name="r_pose">로봇 제어기에서 수신한 X, Y, Z, Rx, Ry, Rz 포즈 객체</param>
        /// <param name="c_pose">비전 PnP로 획득한 체스보드 타겟의 변환 행렬 객체</param>
        public void AddPoseData(
            RobotTargetPose r_pose,
             CameraTargetPose c_pose)
        {
            if (c_pose == null || r_pose == null)
                return;

            // 로봇 좌표계 변환 (Euler/RotVec -> 3x3 Rotation Matrix, 3x1 Translation Vector)
            // 기존 AddPoseData 경로에서도 RotVec/Handedness 설정이 누락되지 않도록 AddPoseDataByConvention과 동일한 변환을 사용합니다.
            Mat robotR = Optimizer.RobotRotationToMatrix(
                r_pose.robotRx,
                r_pose.robotRy,
                r_pose.robotRz,
                HandEyeParams.RobotEulerSequence);

            Mat robotT = new Mat(3, 1, MatType.CV_64FC1, new double[]
            {
                r_pose.robotTx,
                r_pose.robotTy,
                r_pose.robotTz
            });

            Optimizer.ApplyHandednessTransformation(ref robotR, ref robotT, HandEyeParams.Handedness);

            R_gripper2base.Add(robotR);
            t_gripper2base.Add(robotT);

            // 카메라 좌표계 변환
            Mat camR = new Mat(3, 3, MatType.CV_64FC1);
            var idx = camR.GetGenericIndexer<double>();
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    idx[i, j] = c_pose.RotationMatrix[i, j]; // PnP에서 구한 행렬 바로 대입
                }
            }
            R_target2cam.Add(camR);
            t_target2cam.Add(new Mat(3, 1, MatType.CV_64FC1, new double[] { c_pose.X, c_pose.Y, c_pose.Z }));

            poseIndexList.Add(ResolvePoseIndex(r_pose, c_pose));
            //R_target2cam.Add(EulerToMatrix(c_pose.Rx, c_pose.Ry, c_pose.Rz));
            //t_target2cam.Add(new Mat(3, 1, MatType.CV_64FC1, new double[] { c_pose.X, c_pose.Y, c_pose.Z }));
        }

        /// <summary>
        /// 누적된 포즈 데이터를 바탕으로 OpenCV 내부 함수를 호출하여 
        /// 초기 선형 Hand-Eye Calibration 변환 행렬을 도출합니다.
        /// </summary>
        /// <returns>카메라와 그리퍼(또는 베이스) 간의 4x4 동차 변환 행렬(Homogeneous Matrix)</returns>
        public double[,] ExecuteCalibration()
        {
            List<Mat> tGrpScaled = new List<Mat>();
            List<Mat> tCamScaled = new List<Mat>();

            try
            {
                HandEyeParams.ValidateBasic();

                int minPoseCount = HandEyeParams.MinPoseCount;
                if (R_gripper2base.Count < minPoseCount)
                {
                    throw new CalibrationDataException($"캘리브레이션을 수행하려면 최소 {minPoseCount}개 이상의 자세 데이터가 필요합니다.");
                }

                if (R_gripper2base.Count != t_gripper2base.Count ||
    R_target2cam.Count != t_target2cam.Count ||
    R_gripper2base.Count != R_target2cam.Count)
                {
                    throw new CalibrationDataException(
                        $"내부 Pose 데이터 개수가 일치하지 않습니다. " +
                        $"R_robot={R_gripper2base.Count}, t_robot={t_gripper2base.Count}, " +
                        $"R_cam={R_target2cam.Count}, t_cam={t_target2cam.Count}");
                }

                if (HandEyeParams.scale <= 0.0 || double.IsNaN(HandEyeParams.scale) || double.IsInfinity(HandEyeParams.scale))
                {
                    throw new CalibrationDataException($"Scale factor가 올바르지 않습니다. scale={HandEyeParams.scale}");
                }

                using (Mat R_cam2gripper = new Mat())
                using (Mat t_cam2gripper = new Mat())
                {
                    foreach (Mat t in t_gripper2base)
                        tGrpScaled.Add(t * HandEyeParams.scale);

                    foreach (Mat t in t_target2cam)
                        tCamScaled.Add(t * HandEyeParams.scale);

                    Cv2.CalibrateHandEye(
                        R_gripper2base, tGrpScaled,
                        R_target2cam, tCamScaled,
                        R_cam2gripper, t_cam2gripper,
                        HandEyeParams.CalibrationMethod
                    );

                    // 스케일 복원 (m -> mm). 검증을 위해 반드시 복원해야 함.
                    using (Mat restoredTranslation = t_cam2gripper * (1.0 / HandEyeParams.scale))
                    {
                        double[,] resultMatrix = Build4x4Matrix(R_cam2gripper, restoredTranslation);
                        MathUtils.ValidateHomogeneousMatrix(resultMatrix, "CalibrateHandEye result");
                        return resultMatrix;
                    }

                    //Cv2.CalibrateHandEye(
                    //               R_gripper2base, t_gripper2base,
                    //               R_target2cam, t_target2cam,
                    //               R_cam2gripper, t_cam2gripper,
                    //               CalibrationMethod
                    //           );

                    //return Build4x4Matrix(R_cam2gripper, t_cam2gripper);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
                return null;
            }
            finally
            {
                foreach (Mat t in tGrpScaled)
                    t?.Dispose();

                foreach (Mat t in tCamScaled)
                    t?.Dispose();
            }
        }

        /// <summary>
        /// Helix Toolkit 등 3D 뷰어 렌더링에 필요한 로봇 End Effector의 4x4 Transform 정보를 제공합니다.
        /// </summary>
        public double[,] ExecuteEyeToHandCalibration(
            List<RobotTargetPose> robotList,
            List<CameraTargetPose> cameraList,
            EEulerSequence robotRotType,
            HandEyeCalibrationMethod method)
        {
            if (robotList == null || cameraList == null)
                throw new ArgumentNullException("robotList/cameraList");

            if (robotList.Count != cameraList.Count)
                throw new ArgumentException($"Pose count mismatch: Robot={robotList.Count}, Camera={cameraList.Count}");

            if (robotList.Count < HandEyeParams.MinPoseCount)
                throw new ArgumentException($"Eye-to-Hand calibration requires at least {HandEyeParams.MinPoseCount} pose pairs.");

            CLEAR_POSE();

            HandEyeParams.CalibrationMethod = method;
            HandEyeParams.RobotEulerSequence = robotRotType;
            HandEyeParams.mode = HandEyeMode.EyeToHand;

            bool invertRobotPoseForOpenCv =
                HandEyeParams.EyeToHandRobotPoseInputDirection == RobotPoseTransformDirection.GripperToBase;

            for (int i = 0; i < robotList.Count; i++)
            {
                // Eye-to-Hand 구조:
                // - Camera fixed
                // - Target/Checkerboard attached to robot gripper
                // - SolvePnP 결과는 T_target2cam 그대로 사용합니다.
                // - OpenCV에는 T_base2gripper 의미가 들어가야 T_cam2base가 출력됩니다.
                // - CSV가 T_gripper2base이면 inverse하고, 이미 T_base2gripper이면 그대로 사용합니다.
                AddPoseDataByConvention(
                    robotList[i],
                    cameraList[i],
                    robotRotType,
                    invertRobotPose: invertRobotPoseForOpenCv,
                    invertCameraPose: false);
            }

            double[,] initialMatrix = ExecuteCalibration();

            // 여기서는 closed-form 초기해만 반환합니다.
            // 12-DoF Nonlinear Reprojection은 최종 pose set이 결정된 후 Form1에서 한 번만 실행합니다.
            return initialMatrix;
        }

        public double[,] ExecuteEyeInHandCalibration(
    List<RobotTargetPose> robotList,
    List<CameraTargetPose> cameraList,
    EEulerSequence robotRotType,
    HandEyeCalibrationMethod method)
        {
            if (robotList == null || cameraList == null)
                throw new ArgumentNullException("robotList/cameraList");

            if (robotList.Count != cameraList.Count)
                throw new ArgumentException($"Pose count mismatch: Robot={robotList.Count}, Camera={cameraList.Count}");

            if (robotList.Count < HandEyeParams.MinPoseCount)
                throw new ArgumentException($"Eye-in-Hand calibration requires at least {HandEyeParams.MinPoseCount} pose pairs.");

            CLEAR_POSE();

            HandEyeParams.CalibrationMethod = method;
            HandEyeParams.RobotEulerSequence = robotRotType;
            HandEyeParams.mode = HandEyeMode.EyeInHand;

            for (int i = 0; i < robotList.Count; i++)
            {
                // Eye-in-Hand 구조:
                // - Camera attached to robot gripper
                // - Robot pose는 T_gripper2base 그대로 사용
                // - SolvePnP 결과는 T_target2cam 그대로 사용
                AddPoseDataByConvention(
                    robotList[i],
                    cameraList[i],
                    robotRotType,
                    invertRobotPose: false,
                    invertCameraPose: false);
            }

            double[,] initialMatrix = ExecuteCalibration();

            // 현재 12-DoF Nonlinear Reprojection은 EyeToHand 경로에서만 최종 단계로 실행합니다.
            return initialMatrix;
        }

        public double[,] ExecuteHandEyeCalibration(
    List<RobotTargetPose> robotList,
    List<CameraTargetPose> cameraList,
    EEulerSequence robotRotType,
    HandEyeCalibrationMethod method,
    HandEyeMode handEyeMode)
        {
            if (handEyeMode == HandEyeMode.EyeToHand)
                return ExecuteEyeToHandCalibration(robotList, cameraList, robotRotType, method);

            return ExecuteEyeInHandCalibration(robotList, cameraList, robotRotType, method);
        }

        public HandEyeValidationResult BuildValidationResultByMode(
            double[,] handEyeMatrix,
            HandEyeMode handEyeMode)
        {
            MathUtils.ValidateHomogeneousMatrix(handEyeMatrix, nameof(handEyeMatrix));

            if (handEyeMode == HandEyeMode.EyeToHand)
                return BuildEyeToHandValidationResult(handEyeMatrix);

            return BuildValidationResult(handEyeMatrix);
        }

        public HandEyeAxXbResidualResult EvaluateAxXbResidual(
            double[,] handEyeMatrix,
            bool adjacentOnly)
        {
            MathUtils.ValidateHomogeneousMatrix(handEyeMatrix, nameof(handEyeMatrix));

            if (R_gripper2base.Count < 2)
                throw new CalibrationDataException("AX=XB 잔차를 계산하려면 최소 2개 이상의 포즈가 필요합니다.");

            if (R_gripper2base.Count != t_gripper2base.Count ||
                R_target2cam.Count != t_target2cam.Count ||
                R_gripper2base.Count != R_target2cam.Count)
            {
                throw new CalibrationDataException(
                    $"내부 Pose 데이터 개수가 일치하지 않습니다. " +
                    $"R_robot={R_gripper2base.Count}, t_robot={t_gripper2base.Count}, " +
                    $"R_cam={R_target2cam.Count}, t_cam={t_target2cam.Count}");
            }

            int pairCount = 0;
            double rotSumSq = 0.0;
            double transSumSq = 0.0;
            double rotMax = 0.0;
            double transMax = 0.0;

            using (Mat X = MathUtils.ArrayToMat4x4(handEyeMatrix))
            {
                int poseCount = R_gripper2base.Count;

                for (int i = 0; i < poseCount - 1; i++)
                {
                    int jStart = i + 1;
                    int jEnd = adjacentOnly ? i + 2 : poseCount;

                    for (int j = jStart; j < jEnd; j++)
                    {
                        using (Mat H_i = MathUtils.Build4x4MatFromRt(R_gripper2base[i], t_gripper2base[i]))
                        using (Mat H_j = MathUtils.Build4x4MatFromRt(R_gripper2base[j], t_gripper2base[j]))
                        using (Mat C_i = MathUtils.Build4x4MatFromRt(R_target2cam[i], t_target2cam[i]))
                        using (Mat C_j = MathUtils.Build4x4MatFromRt(R_target2cam[j], t_target2cam[j]))
                        using (Mat H_j_inv = H_j.Inv())
                        using (Mat C_i_inv = C_i.Inv())
                        using (Mat A = H_j_inv * H_i)
                        using (Mat B = C_j * C_i_inv)
                        using (Mat AX = A * X)
                        using (Mat XB = X * B)
                        using (Mat XB_inv = XB.Inv())
                        using (Mat residual = XB_inv * AX)
                        {
                            double rot = GetRotationAngleRad(residual);
                            double trans = GetTranslationNorm(residual);

                            if (!IsFinite(rot) || !IsFinite(trans))
                                continue;

                            pairCount++;
                            rotSumSq += rot * rot;
                            transSumSq += trans * trans;

                            if (rot > rotMax)
                                rotMax = rot;

                            if (trans > transMax)
                                transMax = trans;
                        }
                    }
                }
            }

            if (pairCount == 0)
                throw new CalibrationDataException("AX=XB 잔차를 계산할 유효 pair가 없습니다.");

            return new HandEyeAxXbResidualResult
            {
                PairMode = adjacentOnly ? "Adjacent" : "AllPairs",
                PoseCount = R_gripper2base.Count,
                PairCount = pairCount,
                RotationRmseRad = Math.Sqrt(rotSumSq / pairCount),
                RotationMaxRad = rotMax,
                TranslationRmse = Math.Sqrt(transSumSq / pairCount),
                TranslationMax = transMax
            };
        }

        public EEulerSequence[] GetSupportedEulerSequences()
        {
            return new EEulerSequence[]
            {
         EEulerSequence.Extrinsic_XYZ,
         EEulerSequence.Extrinsic_XZY,
         EEulerSequence.Extrinsic_YXZ,
         EEulerSequence.Extrinsic_YZX,
         EEulerSequence.Extrinsic_ZXY,
         EEulerSequence.Extrinsic_ZYX,
         EEulerSequence.Extrinsic_ZYZ,
         EEulerSequence.Intrinsic_XYZ,
         EEulerSequence.Intrinsic_XZY,
         EEulerSequence.Intrinsic_YXZ,
         EEulerSequence.Intrinsic_YZX,
         EEulerSequence.Intrinsic_ZXY,
         EEulerSequence.Intrinsic_ZYX,
         EEulerSequence.Intrinsic_ZYZ,
         EEulerSequence.RotVecDegree,
         EEulerSequence.RotVecRadian
            };
        }

        public HandEyeCalibrationMethod[] GetSupportedMethods()
        {
            return new HandEyeCalibrationMethod[]
            {
        HandEyeCalibrationMethod.TSAI,
        HandEyeCalibrationMethod.PARK,
        HandEyeCalibrationMethod.HORAUD,
        HandEyeCalibrationMethod.ANDREFF,
        HandEyeCalibrationMethod.DANIILIDIS
            };
        }

        public EulerMatrixConvention[] GetSupportedEulerMatrixConventions()
        {
            return new EulerMatrixConvention[]
            {
                EulerMatrixConvention.DefaultActive,
                EulerMatrixConvention.ReverseOrderActive,
                EulerMatrixConvention.DefaultPassive,
                EulerMatrixConvention.ReverseOrderPassive,
                EulerMatrixConvention.NegatedDefaultActive,
                EulerMatrixConvention.NegatedReverseOrderActive,
                EulerMatrixConvention.NegatedDefaultPassive,
                EulerMatrixConvention.NegatedReverseOrderPassive
            };
        }


        public double[,] GetOutputMatrix(double[,] calibMatrix)
        {
            if (calibMatrix == null)
                return null;

            MathUtils.ValidateHomogeneousMatrix(calibMatrix, nameof(calibMatrix));
            return ApplyCoordinateOffset(calibMatrix);
        }

        /// <summary>
        /// <returns>TCP에서 카메라로의 4x4 동차 변환 행렬 (T_tcp_cam)</returns>
        /// </summary>
        private double[,] Build4x4Matrix(Mat R, Mat t)
        {
            double[,] T = new double[4, 4];

            var indexerR = R.GetGenericIndexer<double>();
            var indexert = t.GetGenericIndexer<double>();

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    T[i, j] = indexerR[i, j];
                }
                T[i, 3] = indexert[i, 0];
                T[3, i] = 0;
            }
            T[3, 3] = 1;

            return T;
        }

        /// <summary>
        /// Hand-Eye 캘리브레이션 행렬을 바탕으로 X, Y, Z 오차의 평균과 표준편차를 계산 (내부 일관성 확인)
        /// 공식: T_target_to_base = T_gripper_to_base * T_cam_to_gripper * T_target_to_cam
        /// </summary>
        /// <param name="T_cam2gripper_Array">ExecuteCalibration()에서 도출된 4x4 변환 행렬</param>
        /// <returns>오차 지표가 담긴 CalibrationMetrics 객체</returns>
        public CalibrationMetrics ValidateCalibration(double[,] T_cam2gripper_Array)
        {
            if (R_gripper2base.Count == 0)
                throw new CalibrationDataException("검증할 포즈 데이터가 없습니다.");

            // double[,] 배열을 OpenCV Mat (4x4)으로 변환
            using (Mat T_cam2gripper = MathUtils.ArrayToMat4x4(T_cam2gripper_Array))
            {
                var indexer_cam2grp = T_cam2gripper.GetGenericIndexer<double>();
                for (int r = 0; r < 4; r++)
                {
                    for (int c = 0; c < 4; c++)
                    {
                        indexer_cam2grp[r, c] = T_cam2gripper_Array[r, c];
                    }
                }

                List<double> x_values = new List<double>();
                List<double> y_values = new List<double>();
                List<double> z_values = new List<double>();

                // 누적된 모든 데이터 세트에 대해 타겟의 베이스 기준 절대 좌표 역산
                for (int i = 0; i < R_gripper2base.Count; i++)
                {
                    using (Mat T_gripper2base = MathUtils.Build4x4MatFromRt(R_gripper2base[i], t_gripper2base[i]))
                    using (Mat T_target2cam = MathUtils.Build4x4MatFromRt(R_target2cam[i], t_target2cam[i]))

                    // 핵심 수식: T_target_to_base 도출
                    using (Mat T_target2base = T_gripper2base * T_cam2gripper * T_target2cam)
                    {
                        var indexer_tgt2base = T_target2base.GetGenericIndexer<double>();
                        x_values.Add(indexer_tgt2base[0, 3]);
                        y_values.Add(indexer_tgt2base[1, 3]);
                        z_values.Add(indexer_tgt2base[2, 3]);
                    }
                }

                // 통계적 오차 지표 (평균 및 표준편차) 계산
                CalibrationMetrics metrics = new CalibrationMetrics
                {
                    MeanX = x_values.Average(),
                    MeanY = y_values.Average(),
                    MeanZ = z_values.Average(),

                    StdDevX = CalculateStandardDeviation(x_values),
                    StdDevY = CalculateStandardDeviation(y_values),
                    StdDevZ = CalculateStandardDeviation(z_values)
                };

                Console.WriteLine("=== Hand-Eye Calibration 검증 완료 ===");
                Console.WriteLine(metrics.ToString());

                return metrics;
            }
        }



        /// <summary>
        /// 로봇의 좌표계가 Left-Handed일 경우 OpenCV(Right-Handed)와 맞추기 위해 Y축(또는 Z축)을 반전합니다.
        /// </summary>


        public HandEyeValidationResult BuildEyeToHandValidationResult(double[,] T_cam2base_Array)
        {
            if (R_gripper2base.Count == 0)
                throw new CalibrationDataException("검증할 포즈 데이터가 없습니다.");

            HandEyeValidationResult result = new HandEyeValidationResult();

            using (Mat T_cam2base = MathUtils.ArrayToMat4x4(T_cam2base_Array))
            {
                for (int i = 0; i < R_gripper2base.Count; i++)
                {
                    // 주의:
                    // Eye-to-Hand 모드에서는 이 리스트가 실제로 T_base2gripper 의미여야 함
                    using (Mat T_base2gripper = MathUtils.Build4x4MatFromRt(R_gripper2base[i], t_gripper2base[i]))
                    using (Mat T_target2cam = MathUtils.Build4x4MatFromRt(R_target2cam[i], t_target2cam[i]))
                    using (Mat T_target2gripper = T_base2gripper * T_cam2base * T_target2cam)
                    {
                        double[,] T_Target2Gripper = MathUtils.MatToArray4x4(T_target2gripper);

                        result.Poses.Add(new HandEyeValidationPose
                        {
                            PoseIndex = poseIndexList.Count > i ? poseIndexList[i] : i + 1,
                            T_Target2Gripper = T_Target2Gripper,

                            TargetX = T_Target2Gripper[0, 3],
                            TargetY = T_Target2Gripper[1, 3],
                            TargetZ = T_Target2Gripper[2, 3]
                        });
                    }
                }
            }

            result.MeanTargetX = result.Poses.Average(p => p.TargetX);
            result.MeanTargetY = result.Poses.Average(p => p.TargetY);
            result.MeanTargetZ = result.Poses.Average(p => p.TargetZ);

            double sumSq = 0;
            double maxErr = 0;

            foreach (var p in result.Poses)
            {
                p.ErrorX = p.TargetX - result.MeanTargetX;
                p.ErrorY = p.TargetY - result.MeanTargetY;
                p.ErrorZ = p.TargetZ - result.MeanTargetZ;

                p.ErrorNorm = Math.Sqrt(
                    p.ErrorX * p.ErrorX +
                    p.ErrorY * p.ErrorY +
                    p.ErrorZ * p.ErrorZ);

                sumSq += p.ErrorNorm * p.ErrorNorm;

                if (p.ErrorNorm > maxErr)
                    maxErr = p.ErrorNorm;
            }

            result.RmsError = Math.Sqrt(sumSq / result.Poses.Count);
            result.MaxError = maxErr;

            result.StdDevX = CalculateStandardDeviation(result.Poses.Select(p => p.TargetX).ToList());
            result.StdDevY = CalculateStandardDeviation(result.Poses.Select(p => p.TargetY).ToList());
            result.StdDevZ = CalculateStandardDeviation(result.Poses.Select(p => p.TargetZ).ToList());

            return result;
        }

        public HandEyeValidationResult BuildValidationResult(double[,] T_cam2gripper_Array)
        {
            if (R_gripper2base.Count == 0)
                throw new CalibrationDataException("검증할 포즈 데이터가 없습니다.");

            HandEyeValidationResult result = new HandEyeValidationResult();

            using (Mat T_cam2gripper = MathUtils.ArrayToMat4x4(T_cam2gripper_Array))
            {
                for (int i = 0; i < R_gripper2base.Count; i++)
                {
                    using (Mat T_gripper2base = MathUtils.Build4x4MatFromRt(R_gripper2base[i], t_gripper2base[i]))
                    using (Mat T_target2cam = MathUtils.Build4x4MatFromRt(R_target2cam[i], t_target2cam[i]))
                    using (Mat T_cam2base = T_gripper2base * T_cam2gripper)
                    using (Mat T_target2base = T_cam2base * T_target2cam)
                    {
                        double[,] arrGripper = MathUtils.MatToArray4x4(T_gripper2base);
                        double[,] arrCam = MathUtils.MatToArray4x4(T_cam2base);
                        double[,] arrTarget = MathUtils.MatToArray4x4(T_target2base);

                        result.Poses.Add(new HandEyeValidationPose
                        {
                            PoseIndex = poseIndexList.Count > i ? poseIndexList[i] : i + 1,

                            T_Gripper2Base = arrGripper,
                            T_Cam2Base = arrCam,
                            T_Target2Base = arrTarget,

                            TargetX = arrTarget[0, 3],
                            TargetY = arrTarget[1, 3],
                            TargetZ = arrTarget[2, 3]
                        });
                    }
                }
            }

            result.MeanTargetX = result.Poses.Average(p => p.TargetX);
            result.MeanTargetY = result.Poses.Average(p => p.TargetY);
            result.MeanTargetZ = result.Poses.Average(p => p.TargetZ);

            double sumSq = 0;
            double maxErr = 0;

            foreach (var p in result.Poses)
            {
                p.ErrorX = p.TargetX - result.MeanTargetX;
                p.ErrorY = p.TargetY - result.MeanTargetY;
                p.ErrorZ = p.TargetZ - result.MeanTargetZ;

                p.ErrorNorm = Math.Sqrt(
                    p.ErrorX * p.ErrorX +
                    p.ErrorY * p.ErrorY +
                    p.ErrorZ * p.ErrorZ);

                sumSq += p.ErrorNorm * p.ErrorNorm;

                if (p.ErrorNorm > maxErr)
                    maxErr = p.ErrorNorm;
            }

            result.RmsError = Math.Sqrt(sumSq / result.Poses.Count);
            result.MaxError = maxErr;

            result.StdDevX = CalculateStandardDeviation(result.Poses.Select(p => p.TargetX).ToList());
            result.StdDevY = CalculateStandardDeviation(result.Poses.Select(p => p.TargetY).ToList());
            result.StdDevZ = CalculateStandardDeviation(result.Poses.Select(p => p.TargetZ).ToList());

            return result;
        }

        public List<UIEndEffectorData> BuildValidationUiData(HandEyeValidationResult validation, HandEyeMode mode)
        {
            if (validation == null)
                return new List<UIEndEffectorData>();

            bool isEyeToHand = mode == HandEyeMode.EyeToHand;

            return validation.Poses
                .Select(p => new UIEndEffectorData
                {
                    PoseIndex = p.PoseIndex,
                    X = p.TargetX,
                    Y = p.TargetY,
                    Z = p.TargetZ,
                    TransformMatrix4x4 = isEyeToHand
                        ? (p.T_Target2Gripper ?? p.T_Target2Base)
                        : (p.T_Target2Base ?? p.T_Target2Gripper)
                })
                .ToList();
        }

        public LeaveOneOutResult RunLeaveOneOutBest(List<RobotTargetPose> robotList, List<CameraTargetPose> cameraList,
  EEulerSequence sequence, HandEyeCalibrationMethod method, HandEyeMode mode)
        {
            LeaveOneOutResult best = null;

            Console.WriteLine("===== LEAVE-ONE-OUT TEST =====");

            if (robotList == null || cameraList == null || robotList.Count != cameraList.Count)
            {
                Console.WriteLine("LOO 생략: Robot/Camera pose list가 비어 있거나 개수가 맞지 않습니다.");
                return null;
            }

            // ExecuteCalibration()의 최소 pose 조건을 만족해야 하므로,
            // 전체 pose가 MinPoseCount 이하이면 하나를 제거한 subset은 계산할 수 없습니다.
            int minPoseCount = HandEyeParams.MinPoseCount;
            if (robotList.Count <= minPoseCount)
            {
                Console.WriteLine($"LOO 생략: Pose Count={robotList.Count}. Leave-one-out 후 최소 {minPoseCount} pose 조건을 만족하지 않습니다.");
                return null;
            }

            for (int removeIndex = 0; removeIndex < robotList.Count; removeIndex++)
            {
                int removedPoseNumber = GetPoseDisplayIndex(robotList, cameraList, removeIndex);

                var rSub = robotList
                    .Where((p, idx) => idx != removeIndex)
                    .ToList();

                var cSub = cameraList
                    .Where((p, idx) => idx != removeIndex)
                    .ToList();

                if (rSub.Count < minPoseCount || cSub.Count < minPoseCount)
                {
                    Console.WriteLine($"Remove Pose {removedPoseNumber:D2} -> SKIP: subset pose count < {minPoseCount}");
                    continue;
                }

                try
                {
                    HandEyeValidationResult validation =
                        CalibrateAndValidate(
                            rSub,
                            cSub,
                            sequence,
                            method,
                            mode,
                            out double[,] matrix);

                    Console.WriteLine(
                        $"Remove Pose {removedPoseNumber:D2} -> " +
                        $"RMS={validation.RmsError:F3} mm, MAX={validation.MaxError:F3} mm, " +
                        $"STD=({validation.StdDevX:F3}, {validation.StdDevY:F3}, {validation.StdDevZ:F3})");

                    if (best == null || validation.RmsError < best.Validation.RmsError)
                    {
                        best = new LeaveOneOutResult
                        {
                            RemovedPoseNumber = removedPoseNumber,
                            RobotList = rSub,
                            CameraList = cSub,
                            Matrix = matrix,
                            Validation = validation
                        };
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Remove Pose {removedPoseNumber:D2} -> FAIL: {ex.Message}");
                    continue;
                }
            }

            return best;
        }

        public int GetPoseDisplayIndex(List<RobotTargetPose> robotList, List<CameraTargetPose> cameraList, int listIndex)
        {
            RobotTargetPose robotPose =
                robotList != null && listIndex >= 0 && listIndex < robotList.Count
                    ? robotList[listIndex]
                    : null;

            CameraTargetPose cameraPose =
                cameraList != null && listIndex >= 0 && listIndex < cameraList.Count
                    ? cameraList[listIndex]
                    : null;

            if (robotPose != null && robotPose.HasExplicitPoseIndex && robotPose.SourcePoseIndex > 0)
                return robotPose.SourcePoseIndex;

            if (cameraPose != null && cameraPose.HasExplicitPoseIndex && cameraPose.SourcePoseIndex > 0)
                return cameraPose.SourcePoseIndex;

            if (robotPose != null && robotPose.SourcePoseIndex > 0)
                return robotPose.SourcePoseIndex;

            if (cameraPose != null && cameraPose.SourcePoseIndex > 0)
                return cameraPose.SourcePoseIndex;

            return listIndex + 1;
        }

        public bool TryAutoResolveFullConventionIfNeeded(
            PosePairSet posePairs, HandEyeMode mode, ref EEulerSequence selectedSequence, ref HandEyeCalibrationMethod selectedMethod,
            ref RobotAngleInputOrder selectedAngleInputOrder, ref EulerMatrixConvention selectedEulerConvention,
            ref RobotPoseTransformDirection selectedRobotPoseDirection, ref HandEyeValidationResult validation,
            ref double[,] matrix)
        {
            if (!HandEyeParams.AutoResolveFullConventionWhenHighError ||
                posePairs == null ||
                validation == null ||
                validation.RmsError <= HandEyeParams.MaxRecommendedRmsErrorMm)
            {
                return false;
            }

            Console.WriteLine("===== FULL CONVENTION FALLBACK SWEEP =====");
            Console.WriteLine(
                $"Current RMS={validation.RmsError:F3} mm가 권장 기준 {HandEyeParams.MaxRecommendedRmsErrorMm:F3} mm를 초과하여 " +
                "Euler sequence/method/handedness/pose direction 전체 후보를 비교합니다.");

            EndEffectorHandedness originalHandedness = HandEyeParams.Handedness;
            List<EndEffectorHandedness> handednessCandidates = HandEyeParams.AutoResolveHandednessWhenHighError
                ? Enum.GetValues(typeof(EndEffectorHandedness)).Cast<EndEffectorHandedness>().ToList()
                : new List<EndEffectorHandedness> { originalHandedness };

            List<FullConventionFallbackCandidate> candidates = new List<FullConventionFallbackCandidate>();

            foreach (EndEffectorHandedness handedness in handednessCandidates)
            {
                try
                {
                    HandEyeParams.Handedness = handedness;

                    List<HandEyeSweepResult> sweepResults = RunConventionSweep(
                        posePairs.RobotList,
                        posePairs.CameraList,
                        mode);

                    foreach (HandEyeSweepResult result in sweepResults)
                    {
                        if (!IsUsableFullConventionCandidate(result, mode))
                            continue;

                        candidates.Add(new FullConventionFallbackCandidate
                        {
                            Result = result,
                            Handedness = handedness,
                            RobotPoseDirection = mode == HandEyeMode.EyeToHand
                                ? (result.InvertRobotPose
                                    ? RobotPoseTransformDirection.GripperToBase
                                    : RobotPoseTransformDirection.BaseToGripper)
                                : selectedRobotPoseDirection
                        });
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Full convention sweep 실패: Handedness={handedness}, Error={ex.Message}");
                }
            }

            List<FullConventionFallbackCandidate> orderedCandidates = candidates
                .Where(c => c != null && c.Result != null)
                .OrderBy(c => c.Result.RmsError)
                .ToList();

            if (orderedCandidates.Count == 0)
            {
                RestoreSelectedConvention(
                    originalHandedness,
                    selectedSequence,
                    selectedMethod,
                    selectedAngleInputOrder,
                    selectedEulerConvention,
                    selectedRobotPoseDirection);
                Console.WriteLine("[Full Convention Fallback Skip] 유효 후보가 없습니다.");
                return false;
            }

            int topCount = Math.Min(HandEyeParams.FullConventionFallbackTopCount, orderedCandidates.Count);
            for (int i = 0; i < topCount; i++)
            {
                FullConventionFallbackCandidate c = orderedCandidates[i];
                HandEyeSweepResult r = c.Result;
                Console.WriteLine(
                    $"[Full Convention Rank {i + 1:D2}] RMS={r.RmsError:F3} mm, MAX={r.MaxError:F3} mm, " +
                    $"Handedness={c.Handedness}, Rot={r.RotationType}, Method={r.Method}, " +
                    $"AngleOrder={r.AngleInputOrder}, EulerConv={r.EulerConvention}, " +
                    $"RobotPoseDirection={c.RobotPoseDirection}, InvRobot={r.InvertRobotPose}, InvCam={r.InvertCameraPose}");
            }

            FullConventionFallbackCandidate best = orderedCandidates[0];
            double improvementRatio = (validation.RmsError - best.Result.RmsError) / Math.Max(validation.RmsError, 1e-9);

            if (best.Result.RmsError >= validation.RmsError ||
                improvementRatio < HandEyeParams.MinFullConventionImprovementRatio)
            {
                RestoreSelectedConvention(
                    originalHandedness,
                    selectedSequence,
                    selectedMethod,
                    selectedAngleInputOrder,
                    selectedEulerConvention,
                    selectedRobotPoseDirection);
                Console.WriteLine(
                    $"[Full Convention Fallback Keep] Best RMS={best.Result.RmsError:F3} mm, " +
                    $"개선율={improvementRatio:P1}, 채택 기준={HandEyeParams.MinFullConventionImprovementRatio:P1}");
                return false;
            }

            selectedSequence = best.Result.RotationType;
            selectedMethod = best.Result.Method;
            selectedAngleInputOrder = best.Result.AngleInputOrder;
            selectedEulerConvention = best.Result.EulerConvention;
            selectedRobotPoseDirection = best.RobotPoseDirection;

            RestoreSelectedConvention(
                best.Handedness,
                selectedSequence,
                selectedMethod,
                selectedAngleInputOrder,
                selectedEulerConvention,
                selectedRobotPoseDirection);

            validation = CalibrateAndValidate(
                posePairs.RobotList,
                posePairs.CameraList,
                selectedSequence,
                selectedMethod,
                mode,
                out matrix);

            Console.WriteLine(
                $"[Full Convention Fallback Selected] RMS {validation.RmsError:F3} mm, 개선율={improvementRatio:P1}, " +
                $"Handedness={best.Handedness}, Rot={selectedSequence}, Method={selectedMethod}, " +
                $"AngleOrder={selectedAngleInputOrder}, EulerConv={selectedEulerConvention}, " +
                $"RobotPoseDirection={selectedRobotPoseDirection}");

            return true;
        }

        private bool IsUsableFullConventionCandidate(HandEyeSweepResult result, HandEyeMode mode)
        {
            if (result == null ||
                !IsFinite(result.RmsError) ||
                !IsFinite(result.MaxError) ||
                !IsFinite(result.CamToGripperNorm))
            {
                return false;
            }

            // SolvePnP는 T_target2cam이므로 자동 채택에서는 camera pose inverse 후보를 제외합니다.
            if (result.InvertCameraPose)
                return false;

            // Eye-in-Hand의 물리 입력은 T_gripper2base이므로 robot inverse 후보를 자동 채택하지 않습니다.
            if (mode == HandEyeMode.EyeInHand && result.InvertRobotPose)
                return false;

            return true;
        }

        private void RestoreSelectedConvention(
            EndEffectorHandedness handedness,
            EEulerSequence sequence,
            HandEyeCalibrationMethod method,
            RobotAngleInputOrder angleInputOrder,
            EulerMatrixConvention eulerConvention,
            RobotPoseTransformDirection robotPoseDirection)
        {
            HandEyeParams.Handedness = handedness;
            HandEyeParams.RobotEulerSequence = sequence;
            HandEyeParams.CalibrationMethod = method;
            HandEyeParams.RobotAngleInput = angleInputOrder;
            HandEyeParams.RobotEulerMatrixConvention = eulerConvention;
            HandEyeParams.EyeToHandRobotPoseInputDirection = robotPoseDirection;
        }

        public List<int> GetDuplicateExplicitPoseIndices<T>(IEnumerable<T> poses, Func<T, int> getPoseIndex, Func<T, bool> hasExplicitPoseIndex)
        {
            if (poses == null)
                return new List<int>();

            return poses
                .Where(p => p != null && hasExplicitPoseIndex(p) && getPoseIndex(p) > 0)
                .GroupBy(getPoseIndex)
                .Where(g => g.Count() > 1)
                .Select(g => g.Key)
                .OrderBy(index => index)
                .ToList();
        }

        public PosePairSet ResolveCheckerboardFrameAmbiguity(
            PosePairSet original,
            EEulerSequence sequence,
            HandEyeCalibrationMethod method,
            HandEyeMode mode)
        {
            if (original == null ||
                original.RobotList == null ||
                original.CameraList == null ||
                original.RobotList.Count != original.CameraList.Count)
            {
                return original;
            }

            PosePairSet directionResolved;
            if (TryResolveCheckerboardDirectionByRelativeMotion(
                original,
                sequence,
                method,
                mode,
                out directionResolved))
            {
                original = directionResolved;
            }

            if (!HandEyeParams.EnableCheckerboardFrameAutoResolve ||
                HandEyeParams.CheckerboardFrameResolveMaxPasses <= 0)
            {
                Console.WriteLine("기존 greedy 체커보드 frame 자동 보정은 비활성화되어 있습니다. 방향 식별 결과만 사용합니다.");
                return original;
            }

            List<List<CameraTargetPose>> candidateTable = original.CameraList
                .Select(p => p != null && p.FrameCandidates != null && p.FrameCandidates.Count > 1
                    ? p.FrameCandidates
                    : new List<CameraTargetPose> { p })
                .ToList();

            bool hasAnyAlternative = candidateTable.Any(list => list != null && list.Count > 1);
            if (!hasAnyAlternative)
                return original;

            Console.WriteLine("===== CHECKERBOARD FRAME AMBIGUITY RESOLVE =====");
            Console.WriteLine("일반 checkerboard는 시작 코너가 180도 뒤집혀도 PnP RMS가 낮게 나올 수 있어, hand-eye 3D RMS 기준으로 target frame 후보를 검증합니다.");

            List<CameraTargetPose> currentCameraList = original.CameraList.ToList();
            HandEyeValidationResult currentValidation = null;

            try
            {
                currentValidation = CalibrateAndValidate(
                    original.RobotList,
                    currentCameraList,
                    sequence,
                    method,
                    mode,
                    out double[,] _);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[Checkerboard Frame Resolve Skip] 초기 검증 실패: {ex.Message}");
                return original;
            }

            double currentRms = currentValidation.RmsError;
            Console.WriteLine($"Initial Checkerboard Frame RMS={currentRms:F3} mm");

            bool changedAny = false;
            int maxPasses = Math.Max(1, HandEyeParams.CheckerboardFrameResolveMaxPasses);

            for (int pass = 0; pass < maxPasses; pass++)
            {
                bool changedThisPass = false;

                for (int poseIndex = 0; poseIndex < currentCameraList.Count; poseIndex++)
                {
                    List<CameraTargetPose> candidates = candidateTable[poseIndex];
                    if (candidates == null || candidates.Count <= 1)
                        continue;

                    CameraTargetPose originalCandidate = currentCameraList[poseIndex];
                    CameraTargetPose bestCandidate = originalCandidate;
                    HandEyeValidationResult bestValidation = currentValidation;
                    double bestRms = currentRms;

                    foreach (CameraTargetPose candidate in candidates)
                    {
                        if (candidate == null || ReferenceEquals(candidate, originalCandidate))
                            continue;

                        List<CameraTargetPose> testCameraList = currentCameraList.ToList();
                        testCameraList[poseIndex] = candidate;

                        try
                        {
                            HandEyeValidationResult validation = CalibrateAndValidate(
                                original.RobotList,
                                testCameraList,
                                sequence,
                                method,
                                mode,
                                out double[,] _);

                            if (validation != null && validation.RmsError < bestRms)
                            {
                                bestRms = validation.RmsError;
                                bestValidation = validation;
                                bestCandidate = candidate;
                            }
                        }
                        catch
                        {
                            continue;
                        }
                    }

                    double improvementRatio = currentRms > 0.0
                        ? (currentRms - bestRms) / currentRms
                        : 0.0;

                    if (!ReferenceEquals(bestCandidate, originalCandidate) &&
                        bestRms < currentRms &&
                        improvementRatio >= HandEyeParams.MinCheckerboardFrameImprovementRatio)
                    {
                        int displayPoseIndex = GetPoseDisplayIndex(original.RobotList, currentCameraList, poseIndex);
                        string oldName = string.IsNullOrWhiteSpace(originalCandidate.FrameCandidateName)
                            ? "Normal"
                            : originalCandidate.FrameCandidateName;
                        string newName = string.IsNullOrWhiteSpace(bestCandidate.FrameCandidateName)
                            ? "Unknown"
                            : bestCandidate.FrameCandidateName;

                        Console.WriteLine(
                            $"[Checkerboard Frame Pass {pass + 1}] Pose {displayPoseIndex:D2}: " +
                            $"{oldName} -> {newName}, RMS {currentRms:F3} -> {bestRms:F3} mm, 개선율={improvementRatio:P1}");

                        currentCameraList[poseIndex] = bestCandidate;
                        currentValidation = bestValidation;
                        currentRms = bestRms;
                        changedThisPass = true;
                        changedAny = true;
                    }
                }

                if (!changedThisPass)
                    break;
            }

            if (!changedAny)
            {
                Console.WriteLine("[Checkerboard Frame Keep] 다른 frame 후보가 전체 RMS를 의미 있게 개선하지 않아 원래 frame을 유지합니다.");
                return original;
            }

            Console.WriteLine($"[Checkerboard Frame Selected] Final RMS={currentRms:F3} mm");
            for (int i = 0; i < currentCameraList.Count; i++)
            {
                CameraTargetPose pose = currentCameraList[i];
                if (pose != null && !string.IsNullOrWhiteSpace(pose.FrameCandidateName) && pose.FrameCandidateName != "Normal")
                {
                    int displayPoseIndex = GetPoseDisplayIndex(original.RobotList, currentCameraList, i);
                    Console.WriteLine($"  Pose {displayPoseIndex:D2}: {pose.FrameCandidateName}");
                }
            }

            return new PosePairSet
            {
                RobotList = original.RobotList,
                CameraList = currentCameraList,
                MatchedByExplicitIndex = original.MatchedByExplicitIndex,
                PairSelectionNote = original.PairSelectionNote + ", CheckerboardFrameResolved"
            };
        }

        public PosePairSet BuildPosePairSet(List<RobotTargetPose> r_pose_list, List<CameraTargetPose> c_pose_list)
        {
            if (r_pose_list == null || c_pose_list == null ||
                r_pose_list.Count == 0 || c_pose_list.Count == 0)
            {
                throw new InvalidOperationException("Robot Pose와 Camera Pose를 먼저 로드하세요.");
            }

            bool robotHasExplicitIndex = r_pose_list.Any(p =>
                p != null && p.HasExplicitPoseIndex && p.SourcePoseIndex > 0);

            bool cameraHasExplicitIndex = c_pose_list.Any(p =>
                p != null && p.HasExplicitPoseIndex && p.SourcePoseIndex > 0);

            List<int> duplicateRobotIndices = GetDuplicateExplicitPoseIndices(
                r_pose_list,
                p => p.SourcePoseIndex,
                p => p.HasExplicitPoseIndex);

            List<int> duplicateCameraIndices = GetDuplicateExplicitPoseIndices(
                c_pose_list,
                p => p.SourcePoseIndex,
                p => p.HasExplicitPoseIndex);

            if (duplicateRobotIndices.Count > 0)
            {
                throw new InvalidOperationException(
                    $"Robot CSV에 중복 PoseIndex가 있습니다: {string.Join(",", duplicateRobotIndices)}");
            }

            if (duplicateCameraIndices.Count > 0)
            {
                throw new InvalidOperationException(
                    $"Camera/Image에 중복 PoseIndex가 있습니다: {string.Join(",", duplicateCameraIndices)}");
            }

            // 양쪽 모두 원본 PoseIndex가 명시되어 있으면 index 기준으로 inner join합니다.
            // 한쪽만 명시 index가 있는 경우에는 기존처럼 로드 순서를 유지합니다.
            // 예: Robot CSV에는 PoseIndex 열이 없고 이미지 파일만 2.png부터 시작하는 경우가 여기에 해당합니다.
            if (robotHasExplicitIndex && cameraHasExplicitIndex)
            {
                var robotByIndex = r_pose_list
                    .Where(p => p != null && p.HasExplicitPoseIndex && p.SourcePoseIndex > 0)
                    .GroupBy(p => p.SourcePoseIndex)
                    .ToDictionary(g => g.Key, g => g.First());

                var cameraByIndex = c_pose_list
                    .Where(p => p != null && p.HasExplicitPoseIndex && p.SourcePoseIndex > 0)
                    .GroupBy(p => p.SourcePoseIndex)
                    .ToDictionary(g => g.Key, g => g.First());

                var commonIndices = robotByIndex.Keys
                    .Intersect(cameraByIndex.Keys)
                    .OrderBy(index => index)
                    .ToList();

                var robotOnly = robotByIndex.Keys.Except(cameraByIndex.Keys).OrderBy(index => index).ToList();
                var cameraOnly = cameraByIndex.Keys.Except(robotByIndex.Keys).OrderBy(index => index).ToList();

                if (robotOnly.Count > 0)
                    Console.WriteLine($"Robot에만 있는 PoseIndex: {string.Join(",", robotOnly)}");

                if (cameraOnly.Count > 0)
                    Console.WriteLine($"Camera에만 있는 PoseIndex: {string.Join(",", cameraOnly)}");

                if (commonIndices.Count == 0)
                    throw new InvalidOperationException("Robot/Camera 사이에 일치하는 PoseIndex가 없습니다.");

                Console.WriteLine($"PoseIndex 기준 매칭 적용: Pair Count={commonIndices.Count}");

                return new PosePairSet
                {
                    RobotList = commonIndices.Select(index => robotByIndex[index]).ToList(),
                    CameraList = commonIndices.Select(index => cameraByIndex[index]).ToList(),
                    MatchedByExplicitIndex = true
                };
            }

            if (r_pose_list.Count != c_pose_list.Count)
            {
                throw new InvalidOperationException(
                    $"Pose Count 불일치: Robot={r_pose_list.Count}, Camera={c_pose_list.Count}. " +
                    "양쪽 모두 명시 PoseIndex가 있으면 자동 매칭할 수 있지만, 현재는 순서 매칭만 가능합니다.");
            }

            if (robotHasExplicitIndex || cameraHasExplicitIndex)
            {
                Console.WriteLine(
                    "한쪽에만 명시 PoseIndex가 있어 로드 순서 기준으로 매칭합니다. " +
                    "출력 PoseIndex는 명시된 쪽의 번호를 우선 사용합니다.");
            }
            else
            {
                Console.WriteLine("명시 PoseIndex가 없어 로드 순서 기준으로 매칭합니다.");
            }

            return new PosePairSet
            {
                RobotList = r_pose_list.ToList(),
                CameraList = c_pose_list.ToList(),
                MatchedByExplicitIndex = false
            };
        }

        private static bool IsValidRotationMatrix(double[,] matrix)
        {
            if (matrix == null || matrix.GetLength(0) != 3 || matrix.GetLength(1) != 3)
                return false;

            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++)
                {
                    if (!IsFinite(matrix[r, c]))
                        return false;
                }
            }

            return true;
        }

        public HandEyeValidationResult CalibrateAndValidate(List<RobotTargetPose> robotList, List<CameraTargetPose> cameraList,
     EEulerSequence sequence, HandEyeCalibrationMethod method, HandEyeMode mode, out double[,] matrix)
        {
            matrix = ExecuteHandEyeCalibration(
                robotList,
                cameraList,
                sequence,
                method,
                mode);

            if (matrix == null)
                throw new InvalidOperationException("Hand-Eye calibration matrix 계산 실패");

            return BuildValidationResultByMode(matrix, mode);
        }

        public void ValidatePosePairContent(PosePairSet posePairs, bool requireImagePoints)
        {
            if (posePairs == null ||
                posePairs.RobotList == null ||
                posePairs.CameraList == null ||
                posePairs.RobotList.Count != posePairs.CameraList.Count)
            {
                throw new InvalidOperationException("Robot/Camera Pose Pair가 비어 있거나 개수가 맞지 않습니다.");
            }

            for (int i = 0; i < posePairs.RobotList.Count; i++)
            {
                RobotTargetPose robotPose = posePairs.RobotList[i];
                CameraTargetPose cameraPose = posePairs.CameraList[i];
                int poseIndex = GetPoseDisplayIndex(posePairs.RobotList, posePairs.CameraList, i);

                if (robotPose == null || cameraPose == null)
                    throw new InvalidOperationException($"Pose {poseIndex:D2}: Robot 또는 Camera pose가 null입니다.");

                if (!IsFinite(robotPose.robotTx) || !IsFinite(robotPose.robotTy) || !IsFinite(robotPose.robotTz) ||
                    !IsFinite(robotPose.robotRx) || !IsFinite(robotPose.robotRy) || !IsFinite(robotPose.robotRz))
                {
                    throw new InvalidOperationException($"Pose {poseIndex:D2}: Robot pose에 NaN/Infinity가 포함되어 있습니다.");
                }

                if (!IsFinite(cameraPose.X) || !IsFinite(cameraPose.Y) || !IsFinite(cameraPose.Z) ||
                    !IsFinite(cameraPose.Rx) || !IsFinite(cameraPose.Ry) || !IsFinite(cameraPose.Rz) ||
                    !IsFinite(cameraPose.ReprojectionRmsPx) || !IsFinite(cameraPose.ReprojectionMaxPx))
                {
                    throw new InvalidOperationException($"Pose {poseIndex:D2}: Camera pose에 NaN/Infinity가 포함되어 있습니다.");
                }

                if (!IsValidRotationMatrix(cameraPose.RotationMatrix))
                    throw new InvalidOperationException($"Pose {poseIndex:D2}: Camera RotationMatrix가 비어 있거나 비정상입니다.");

                if (requireImagePoints)
                {
                    if (cameraPose.ImagePoints == null ||
                        cameraPose.ObjectPoints == null ||
                        cameraPose.ImagePoints.Length == 0 ||
                        cameraPose.ImagePoints.Length != cameraPose.ObjectPoints.Length)
                    {
                        throw new InvalidOperationException(
                            $"Pose {poseIndex:D2}: Nonlinear Reprojection에 필요한 ImagePoints/ObjectPoints가 비어 있거나 개수가 맞지 않습니다.");
                    }
                }
            }
        }

        public void PrintCalibrationHealthDiagnostics(List<int> bestExcludedPoseNumbers, HandEyeValidationResult validation, HandEyeMode mode, OptimizationMethod optimizationMethod)
        {
            if (validation == null)
                return;

            Console.WriteLine("===== CALIBRATION HEALTH CHECK =====");

            bool rmsTooHigh = validation.RmsError > HandEyeParams.MaxRecommendedRmsErrorMm;
            bool maxTooHigh = validation.MaxError > HandEyeParams.MaxRecommendedMaxErrorMm;

            if (!rmsTooHigh && !maxTooHigh)
            {
                Console.WriteLine("3D validation error가 권장 기준 이내입니다.");
                return;
            }

            Console.WriteLine(
                $"[WARNING] 현재 결과는 정상 hand-eye calibration 수준으로 보기 어렵습니다. " +
                $"RMS={validation.RmsError:F3} mm / 권장≤{HandEyeParams.MaxRecommendedRmsErrorMm:F3} mm, " +
                $"MAX={validation.MaxError:F3} mm / 권장≤{HandEyeParams.MaxRecommendedMaxErrorMm:F3} mm");

            var topOutliers = validation.Poses
                .OrderByDescending(p => p.ErrorNorm)
                .Take(5)
                .Select(p => $"{p.PoseIndex:D2}:{p.ErrorNorm:F1}mm")
                .ToList();

            Console.WriteLine($"Top Outlier Pose = {string.Join(", ", topOutliers)}");

            if (validation.RmsError > 20.0)
            {
                Console.WriteLine("[판정] 수 mm 수준의 미세 오차가 아니라 pose pair/convention/TCP/PnP 좌표 정의 오류 가능성이 큽니다.");
                Console.WriteLine("확인 우선순위: 1) Robot CSV row와 이미지 번호 매칭, 2) TCP 사용 여부, 3) Robot pose 좌표계(Base/User/Work), 4) Euler sequence/각도 단위, 5) Checkerboard object point 원점/방향.");
            }

            if (optimizationMethod == OptimizationMethod.Nonlinear_Reprojection &&
                IsFinite(HandEyeParams.LastNonlinearReprojectionRmsPx) &&
                HandEyeParams.LastNonlinearReprojectionRmsPx > HandEyeParams.MaxAcceptReprojectionRmsPx)
            {
                Console.WriteLine(
                    $"[Reprojection WARNING] 선형/초기 reprojection RMS={HandEyeParams.LastNonlinearReprojectionRmsPx:F3}px로 " +
                    $"채택 기준 {HandEyeParams.MaxAcceptReprojectionRmsPx:F3}px보다 큽니다. " +
                    "이 경우 Nonlinear로 억지 보정하기보다 pose pair와 좌표계 정의를 먼저 점검해야 합니다.");
            }

            if (bestExcludedPoseNumbers.Count >= HandEyeParams.MaxAutoExcludedPoseCount &&
                HandEyeParams.MaxAutoExcludedPoseCount > 0)
            {
                Console.WriteLine(
                    $"[Outlier WARNING] 자동 제외 pose가 최대 허용 개수({HandEyeParams.MaxAutoExcludedPoseCount})에 도달했습니다. " +
                    "더 많은 pose를 제거해야만 RMS가 내려간다면 데이터 일부 문제가 아니라 convention 자체가 틀렸을 가능성이 큽니다.");
            }
        }

        public List<UIEndEffectorData> BuildViewerSceneData(double[,] bestHandEyeMatrix, HandEyeMode bestResultMode)
        {
            if (bestHandEyeMatrix == null)
                return new List<UIEndEffectorData>();

            try
            {
                List<UIEndEffectorData> sceneData = BuildCameraOriginViewerData(bestHandEyeMatrix, bestResultMode);

                if (sceneData != null)
                    return sceneData;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Viewer scene data 생성 실패: {ex.Message}");
            }

            return new List<UIEndEffectorData>();
        }

        public List<UIEndEffectorData> BuildCameraOriginViewerData(double[,] handEyeMatrix, HandEyeMode handEyeMode)
        {
            MathUtils.ValidateHomogeneousMatrix(handEyeMatrix, nameof(handEyeMatrix));

            List<UIEndEffectorData> uiDataList = new List<UIEndEffectorData>();

            if (R_gripper2base.Count == 0)
                return uiDataList;

            if (handEyeMode == HandEyeMode.EyeToHand)
            {
                using (Mat T_cam2base = MathUtils.ArrayToMat4x4(handEyeMatrix))
                {
                    for (int i = 0; i < R_gripper2base.Count; i++)
                    {
                        // Eye-to-Hand 검증에서 사용한 것과 동일한 의미의 로봇 pose를 사용합니다.
                        // 이 pose는 내부적으로 Base -> Gripper 의미로 취급됩니다.
                        using (Mat T_base2gripper = MathUtils.Build4x4MatFromRt(R_gripper2base[i], t_gripper2base[i]))
                        using (Mat T_cam2gripper = T_base2gripper * T_cam2base)
                        using (Mat T_gripper2cam = T_cam2gripper.Inv())
                        {
                            double[,] renderMatrix = MathUtils.MatToArray4x4(T_gripper2cam);
                            int poseIndex = poseIndexList.Count > i ? poseIndexList[i] : i + 1;

                            uiDataList.Add(new UIEndEffectorData
                            {
                                PoseIndex = poseIndex,
                                X = renderMatrix[0, 3],
                                Y = renderMatrix[1, 3],
                                Z = renderMatrix[2, 3],
                                Label = $"End Effector {poseIndex}",
                                InfoText = $"({renderMatrix[0, 3]:F2}, {renderMatrix[1, 3]:F2}, {renderMatrix[2, 3]:F2})",
                                TransformMatrix4x4 = renderMatrix
                            });
                        }
                    }
                }

                return uiDataList;
            }

            using (Mat T_cam2gripper = MathUtils.ArrayToMat4x4(handEyeMatrix))
            using (Mat T_gripper2cam = T_cam2gripper.Inv())
            {
                for (int i = 0; i < R_gripper2base.Count; i++)
                {
                    using (Mat T_target2cam = MathUtils.Build4x4MatFromRt(R_target2cam[i], t_target2cam[i]))
                    using (Mat T_cam2target = T_target2cam.Inv())
                    using (Mat T_target2base = CalculateTargetToBase(i, T_cam2gripper))
                    using (Mat T_base2target = T_target2base.Inv())
                    using (Mat T_gripper2base = MathUtils.Build4x4MatFromRt(R_gripper2base[i], t_gripper2base[i]))
                    using (Mat T_temp = T_cam2target * T_base2target)
                    using (Mat T_cam2gripper_visual = T_temp * T_gripper2base)
                    {
                        double[,] renderMatrix = MathUtils.MatToArray4x4(T_cam2gripper_visual);
                        int poseIndex = poseIndexList.Count > i ? poseIndexList[i] : i + 1;

                        uiDataList.Add(new UIEndEffectorData
                        {
                            PoseIndex = poseIndex,
                            X = renderMatrix[0, 3],
                            Y = renderMatrix[1, 3],
                            Z = renderMatrix[2, 3],
                            Label = $"End Effector {poseIndex}",
                            InfoText = $"({renderMatrix[0, 3]:F2}, {renderMatrix[1, 3]:F2}, {renderMatrix[2, 3]:F2})",
                            TransformMatrix4x4 = renderMatrix
                        });
                    }
                }
            }

            return uiDataList;
        }

        public void PrintPoseErrors(HandEyeValidationResult validation)
        {
            Console.WriteLine("===== Per Pose Validation Error =====");

            foreach (var p in validation.Poses.OrderByDescending(x => x.ErrorNorm))
            {
                Console.WriteLine(
                    $"Pose {p.PoseIndex:D2} | " +
                    $"ErrNorm={p.ErrorNorm:F3} mm | " +
                    $"ErrX={p.ErrorX:F3}, " +
                    $"ErrY={p.ErrorY:F3}, " +
                    $"ErrZ={p.ErrorZ:F3} | " +
                    $"Target=({p.TargetX:F3}, {p.TargetY:F3}, {p.TargetZ:F3})");
            }
        }

        private double GetRotationAngleRad(Mat homogeneousResidual)
        {
            var idx = homogeneousResidual.GetGenericIndexer<double>();
            double trace = idx[0, 0] + idx[1, 1] + idx[2, 2];
            double cos = (trace - 1.0) * 0.5;

            if (cos > 1.0)
                cos = 1.0;
            else if (cos < -1.0)
                cos = -1.0;

            return Math.Acos(cos);
        }

        private double GetTranslationNorm(Mat homogeneousResidual)
        {
            var idx = homogeneousResidual.GetGenericIndexer<double>();
            double x = idx[0, 3];
            double y = idx[1, 3];
            double z = idx[2, 3];
            return Math.Sqrt(x * x + y * y + z * z);
        }

        public RobotAngleInputOrder[] GetSupportedRobotAngleInputOrders()
        {
            return new RobotAngleInputOrder[]
            {
                RobotAngleInputOrder.RxRyRz,
                RobotAngleInputOrder.RxRzRy,
                RobotAngleInputOrder.RyRxRz,
                RobotAngleInputOrder.RyRzRx,
                RobotAngleInputOrder.RzRxRy,
                RobotAngleInputOrder.RzRyRx
            };
        }



        /// <summary>
        /// 표본 표준편차(Standard Deviation)를 계산합니다.
        /// </summary>
        private double CalculateStandardDeviation(List<double> values)
        {
            if (values.Count <= 1) return 0;

            double avg = values.Average();
            double sumOfSquaresOfDifferences = values.Select(val => (val - avg) * (val - avg)).Sum();
            return Math.Sqrt(sumOfSquaresOfDifferences / (values.Count - 1));
        }


        // 유틸리티: 3x3 R과 3x1 t로 4x4 행렬 만들기
        private Mat CreateHomogeneousMatrix(Mat R, Mat t)
        {
            Mat T = Mat.Eye(4, 4, MatType.CV_64FC1);
            Mat R_roi = new Mat(T, new Rect(0, 0, 3, 3));
            Mat t_roi = new Mat(T, new Rect(3, 0, 1, 3));
            R.CopyTo(R_roi);
            t.CopyTo(t_roi);
            return T;
        }


        private Mat InvertTransform(Mat R, Mat t, out Mat invR, out Mat invT)
        {
            invR = R.T();

            Mat temp = invR * t;
            invT = -temp;
            temp.Dispose();

            return null;
        }

        private void InvertRt(Mat R, Mat t, out Mat RInv, out Mat tInv)
        {
            RInv = R.T();

            using (Mat temp = RInv * t)
            {
                tInv = -temp;
            }
        }

        public void AddPoseDataByConvention(RobotTargetPose r_pose, CameraTargetPose c_pose,
    EEulerSequence robotRotType, bool invertRobotPose, bool invertCameraPose)
        {
            if (c_pose == null || r_pose == null)
                return;

            Mat R_robot = Optimizer.RobotRotationToMatrix(
                r_pose.robotRx,
                r_pose.robotRy,
                r_pose.robotRz,
                robotRotType);

            Mat t_robot = new Mat(3, 1, MatType.CV_64FC1, new double[]
            {
        r_pose.robotTx,
        r_pose.robotTy,
        r_pose.robotTz
            });

            Optimizer.ApplyHandednessTransformation(ref R_robot, ref t_robot, HandEyeParams.Handedness);

            if (invertRobotPose)
            {
                InvertRt(R_robot, t_robot, out Mat R_inv, out Mat t_inv);

                R_robot.Dispose();
                t_robot.Dispose();

                R_robot = R_inv;
                t_robot = t_inv;
            }

            R_gripper2base.Add(R_robot);
            t_gripper2base.Add(t_robot);

            Mat R_cam = new Mat(3, 3, MatType.CV_64FC1);
            var idx = R_cam.GetGenericIndexer<double>();

            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++)
                {
                    idx[r, c] = c_pose.RotationMatrix[r, c];
                }
            }

            Mat t_cam = new Mat(3, 1, MatType.CV_64FC1, new double[]
            {
        c_pose.X,
        c_pose.Y,
        c_pose.Z
            });

            if (invertCameraPose)
            {
                InvertRt(R_cam, t_cam, out Mat R_inv, out Mat t_inv);

                R_cam.Dispose();
                t_cam.Dispose();

                R_cam = R_inv;
                t_cam = t_inv;
            }

            R_target2cam.Add(R_cam);
            t_target2cam.Add(t_cam);

            poseIndexList.Add(ResolvePoseIndex(r_pose, c_pose));
        }

        public List<HandEyeSweepResult> RunConventionSweep(
    List<RobotTargetPose> robotList, List<CameraTargetPose> cameraList, HandEyeMode mode)
        {
            List<HandEyeSweepResult> results = new List<HandEyeSweepResult>();

            EEulerSequence originalSequence = HandEyeParams.RobotEulerSequence;
            HandEyeCalibrationMethod originalMethod = HandEyeParams.CalibrationMethod;
            HandEyeMode originalMode = HandEyeParams.mode;
            EulerMatrixConvention originalEulerConvention = HandEyeParams.RobotEulerMatrixConvention;
            RobotAngleInputOrder originalAngleOrder = HandEyeParams.RobotAngleInput;

            try
            {
                HandEyeParams.ValidateBasic();
                HandEyeParams.mode = mode;

                EEulerSequence[] rotTypes = GetSupportedEulerSequences();

                HandEyeCalibrationMethod[] methods =
                {
        HandEyeCalibrationMethod.TSAI,
        HandEyeCalibrationMethod.PARK,
        HandEyeCalibrationMethod.HORAUD,
        HandEyeCalibrationMethod.ANDREFF,
        // DANIILIDIS는 불안정하면 일단 제외 권장
        // HandEyeCalibrationMethod.DANIILIDIS
    };

                bool[] invertOptions = { false, true };

                foreach (EEulerSequence rotType in rotTypes)
                {
                    // 상용 SW에서 주로 문제가 되는 ZYZ 계열에 대해서만 행렬 해석 후보와 CSV 각도 열 매핑 후보를 확장합니다.
                    // 모든 Euler sequence에 곱하면 sweep 시간이 과도하게 늘어나고 해석도 흐려집니다.
                    EulerMatrixConvention[] eulerConventions = (!IsRotationVectorSequence(rotType) && IsZyzSequence(rotType))
                        ? GetSupportedEulerMatrixConventions()
                        : new EulerMatrixConvention[] { EulerMatrixConvention.DefaultActive };

                    RobotAngleInputOrder[] angleOrders = (!IsRotationVectorSequence(rotType) && IsZyzSequence(rotType))
                        ? GetSupportedRobotAngleInputOrders()
                        : new RobotAngleInputOrder[] { RobotAngleInputOrder.RxRyRz };

                    foreach (RobotAngleInputOrder angleOrder in angleOrders)
                    {
                        HandEyeParams.RobotAngleInput = angleOrder;

                        foreach (EulerMatrixConvention eulerConvention in eulerConventions)
                        {
                            foreach (bool invertRobot in invertOptions)
                            {
                                foreach (bool invertCamera in invertOptions)
                                {
                                    foreach (HandEyeCalibrationMethod method in methods)
                                    {
                                        try
                                        {
                                            CLEAR_POSE();

                                            HandEyeParams.CalibrationMethod = method;
                                            HandEyeParams.RobotEulerSequence = rotType;
                                            HandEyeParams.RobotEulerMatrixConvention = eulerConvention;
                                            HandEyeParams.RobotAngleInput = angleOrder;

                                            for (int i = 0; i < robotList.Count; i++)
                                            {
                                                AddPoseDataByConvention(
                                                    robotList[i],
                                                    cameraList[i],
                                                    rotType,
                                                    invertRobot,
                                                    invertCamera);
                                            }

                                            double[,] T = ExecuteCalibration();

                                            if (T == null)
                                                continue;

                                            HandEyeValidationResult validation =
                                                mode == HandEyeMode.EyeToHand
                                                    ? BuildEyeToHandValidationResult(T)
                                                    : BuildValidationResult(T);

                                            double tx = T[0, 3];
                                            double ty = T[1, 3];
                                            double tz = T[2, 3];
                                            double norm = Math.Sqrt(tx * tx + ty * ty + tz * tz);

                                            if (double.IsNaN(validation.RmsError) ||
                                                double.IsInfinity(validation.RmsError) ||
                                                double.IsNaN(norm) ||
                                                double.IsInfinity(norm))
                                                continue;

                                            results.Add(new HandEyeSweepResult
                                            {
                                                RotationType = rotType,
                                                EulerConvention = eulerConvention,
                                                AngleInputOrder = angleOrder,
                                                InvertRobotPose = invertRobot,
                                                InvertCameraPose = invertCamera,
                                                Method = method,

                                                RmsError = validation.RmsError,
                                                MaxError = validation.MaxError,
                                                StdDevX = validation.StdDevX,
                                                StdDevY = validation.StdDevY,
                                                StdDevZ = validation.StdDevZ,

                                                CamToGripperX = tx,
                                                CamToGripperY = ty,
                                                CamToGripperZ = tz,
                                                CamToGripperNorm = norm,

                                                T_Cam2Gripper = T
                                            });
                                        }
                                        catch (Exception ex)
                                        {
                                            Console.WriteLine(
                                                $"ACTUAL-MM SWEEP FAIL | " +
                                                $"Mode={mode}, Rot={rotType}, EulerConv={eulerConvention}, AngleOrder={angleOrder}, " +
                                                $"InvRobot={invertRobot}, InvCam={invertCamera}, " +
                                                $"Method={method}, Error={ex.Message}");
                                            continue;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            finally
            {
                CLEAR_POSE();
                HandEyeParams.RobotEulerSequence = originalSequence;
                HandEyeParams.RobotEulerMatrixConvention = originalEulerConvention;
                HandEyeParams.RobotAngleInput = originalAngleOrder;
                HandEyeParams.CalibrationMethod = originalMethod;
                HandEyeParams.mode = originalMode;
            }

            return results
                .OrderBy(r => r.RmsError)
                .ToList();
        }

        public void PrintActualMmSweepResults(
    List<HandEyeSweepResult> results,
    int topCount = 30)
        {
            Console.WriteLine("===== ACTUAL MM CONVENTION SWEEP RESULT =====");

            foreach (HandEyeSweepResult r in results.Take(topCount))
            {
                Console.WriteLine(
                    $"RMS={r.RmsError:F3} mm, " +
                    $"MAX={r.MaxError:F3} mm, " +
                    $"STD=({r.StdDevX:F3}, {r.StdDevY:F3}, {r.StdDevZ:F3}) mm, " +
                    $"Tnorm={r.CamToGripperNorm:F3} mm, " +
                    $"Rot={r.RotationType}, " +
                    $"EulerConv={r.EulerConvention}, " +
                    $"AngleOrder={r.AngleInputOrder}, " +
                    $"InvRobot={r.InvertRobotPose}, " +
                    $"InvCam={r.InvertCameraPose}, " +
                    $"Method={r.Method}, " +
                    $"T=({r.CamToGripperX:F3}, {r.CamToGripperY:F3}, {r.CamToGripperZ:F3}) mm");
            }
        }


        public HandEyeSweepResult CreateSingleResult(
            EEulerSequence sequence,
            HandEyeCalibrationMethod method,
            HandEyeMode mode,
            double[,] matrix,
            HandEyeValidationResult validation)
        {
            if (validation == null)
                throw new ArgumentNullException(nameof(validation));

            MathUtils.ValidateHomogeneousMatrix(matrix, nameof(matrix));

            double tx = matrix[0, 3];
            double ty = matrix[1, 3];
            double tz = matrix[2, 3];
            double norm = Math.Sqrt(tx * tx + ty * ty + tz * tz);

            return new HandEyeSweepResult
            {
                RotationType = sequence,
                EulerConvention = HandEyeParams.RobotEulerMatrixConvention,
                AngleInputOrder = HandEyeParams.RobotAngleInput,
                Method = method,
                InvertRobotPose = mode == HandEyeMode.EyeToHand &&
                    HandEyeParams.EyeToHandRobotPoseInputDirection == RobotPoseTransformDirection.GripperToBase,
                InvertCameraPose = false,
                RmsError = validation.RmsError,
                MaxError = validation.MaxError,
                StdDevX = validation.StdDevX,
                StdDevY = validation.StdDevY,
                StdDevZ = validation.StdDevZ,
                CamToGripperX = tx,
                CamToGripperY = ty,
                CamToGripperZ = tz,
                CamToGripperNorm = norm,
                T_Cam2Gripper = matrix
            };
        }

        /// <summary>
        /// 도출된 캘리브레이션 4x4 결과 행렬에 지정된 Offset 모드를 적용합니다.
        /// </summary>
        private double[,] ApplyCoordinateOffset(double[,] calibMatrix)
        {
            using (Mat X = MathUtils.ArrayToMat4x4(calibMatrix))
            {
                switch (HandEyeParams.CoordConvertMode)
                {
                    case CoordianteConvert.withoutOffset:
                        return MathUtils.MatToArray4x4(X);

                    case CoordianteConvert.withCameraOffset:
                        if (HandEyeParams.CameraOffsetMatrix == null)
                            throw new CalibrationDataException("CameraOffsetMatrix가 null입니다.");

                        MathUtils.ValidateHomogeneousMatrix(HandEyeParams.CameraOffsetMatrix, nameof(HandEyeParams.CameraOffsetMatrix));

                        using (Mat cameraOffset = MathUtils.ArrayToMat4x4(HandEyeParams.CameraOffsetMatrix))
                        using (Mat converted = X * cameraOffset)
                        {
                            return MathUtils.MatToArray4x4(converted);
                        }

                    case CoordianteConvert.withBaseOffset:
                        if (HandEyeParams.BaseOffsetMatrix == null)
                            throw new CalibrationDataException("BaseOffsetMatrix가 null입니다.");

                        MathUtils.ValidateHomogeneousMatrix(HandEyeParams.BaseOffsetMatrix, nameof(HandEyeParams.BaseOffsetMatrix));

                        using (Mat baseOffset = MathUtils.ArrayToMat4x4(HandEyeParams.BaseOffsetMatrix))
                        using (Mat converted = baseOffset * X)
                        {
                            return MathUtils.MatToArray4x4(converted);
                        }

                    default:
                        throw new ArgumentOutOfRangeException(
                            nameof(HandEyeParams.CoordConvertMode),
                            HandEyeParams.CoordConvertMode,
                            "지원하지 않는 CoordianteConvert 모드입니다.");
                }
            }
        }

        private int[] BuildAnchorBasedDirectionLabels(
           List<CheckerboardDirectionEdge> edges,
           int count,
           int anchor,
           int anchorLabel)
        {
            int[] labels = new int[count];
            for (int i = 0; i < count; i++)
                labels[i] = 0;

            if (anchor < 0 || anchor >= count)
                return labels;

            labels[anchor] = anchorLabel;

            for (int i = 0; i < count; i++)
            {
                if (i == anchor)
                    continue;

                double cost0 = 0.0;
                double cost1 = 0.0;
                int edgeCount = 0;

                foreach (CheckerboardDirectionEdge edge in edges)
                {
                    if (edge.I == anchor && edge.J == i)
                    {
                        cost0 += edge.Cost[anchorLabel, 0];
                        cost1 += edge.Cost[anchorLabel, 1];
                        edgeCount++;
                    }
                    else if (edge.I == i && edge.J == anchor)
                    {
                        cost0 += edge.Cost[0, anchorLabel];
                        cost1 += edge.Cost[1, anchorLabel];
                        edgeCount++;
                    }
                }

                labels[i] = edgeCount > 0 && cost1 < cost0 ? 1 : 0;
            }

            return labels;
        }

        private void AddOptimizedDirectionSolution(
            List<CheckerboardDirectionSolution> solutions,
            int[] seedLabels,
            List<CheckerboardDirectionEdge> edges,
            int count)
        {
            if (seedLabels == null || seedLabels.Length != count)
                return;

            int[] labels = seedLabels.ToArray();
            OptimizeDirectionLabelsCoordinateDescent(labels, edges, count, 30);

            double cost = ComputeDirectionLabelCost(labels, edges);
            CheckerboardDirectionSolution solution = new CheckerboardDirectionSolution
            {
                Labels = labels,
                Cost = cost,
                FlipCount = CountDirectionFlips(labels)
            };

            bool exists = solutions.Any(s => SameLabels(s.Labels, solution.Labels));
            if (!exists)
                solutions.Add(solution);
        }

        private List<CheckerboardDirectionEdge> BuildCheckerboardDirectionEdges(
           double[][,] robotRotations,
           double[][,] cameraRotations)
        {
            List<CheckerboardDirectionEdge> edges = new List<CheckerboardDirectionEdge>();
            int n = robotRotations.Length;
            double minAngleRad = HandEyeParams.CheckerboardDirectionMinRelativeRotationDeg * Math.PI / 180.0;

            for (int i = 0; i < n - 1; i++)
            {
                for (int j = i + 1; j < n; j++)
                {
                    double[,] rA = MathUtils.Multiply3x3(MathUtils.Transpose3x3(robotRotations[j]), robotRotations[i]);
                    double angleA = MathUtils.RotationAngleRad(rA);
                    if (!IsFinite(angleA) || angleA < minAngleRad)
                        continue;

                    CheckerboardDirectionEdge edge = new CheckerboardDirectionEdge
                    {
                        I = i,
                        J = j,
                        Cost = new double[2, 2]
                    };

                    for (int si = 0; si <= 1; si++)
                    {
                        for (int sj = 0; sj <= 1; sj++)
                        {
                            double[,] rB = MathUtils.Multiply3x3(cameraRotations[j * 2 + sj], MathUtils.Transpose3x3(cameraRotations[i * 2 + si]));
                            double angleB = MathUtils.RotationAngleRad(rB);
                            double diff = Math.Abs(angleA - angleB);
                            edge.Cost[si, sj] = diff * diff;
                        }
                    }

                    edges.Add(edge);
                }
            }

            return edges;
        }

        private char[] GetEulerAxesLocal(EEulerSequence sequence)
        {
            switch (sequence)
            {
                case EEulerSequence.Extrinsic_XYZ:
                case EEulerSequence.Intrinsic_XYZ:
                    return new[] { 'X', 'Y', 'Z' };
                case EEulerSequence.Extrinsic_XZY:
                case EEulerSequence.Intrinsic_XZY:
                    return new[] { 'X', 'Z', 'Y' };
                case EEulerSequence.Extrinsic_YXZ:
                case EEulerSequence.Intrinsic_YXZ:
                    return new[] { 'Y', 'X', 'Z' };
                case EEulerSequence.Extrinsic_YZX:
                case EEulerSequence.Intrinsic_YZX:
                    return new[] { 'Y', 'Z', 'X' };
                case EEulerSequence.Extrinsic_ZXY:
                case EEulerSequence.Intrinsic_ZXY:
                    return new[] { 'Z', 'X', 'Y' };
                case EEulerSequence.Extrinsic_ZYX:
                case EEulerSequence.Intrinsic_ZYX:
                    return new[] { 'Z', 'Y', 'X' };
                case EEulerSequence.Extrinsic_ZYZ:
                case EEulerSequence.Intrinsic_ZYZ:
                    return new[] { 'Z', 'Y', 'Z' };
                default:
                    throw new ArgumentOutOfRangeException(nameof(sequence), sequence, "지원하지 않는 Euler sequence입니다.");
            }
        }

        private double[,] AxisRotationArray(char axis, double rad)
        {
            double c = Math.Cos(rad);
            double s = Math.Sin(rad);

            switch (axis)
            {
                case 'X':
                    return new double[,] { { 1, 0, 0 }, { 0, c, -s }, { 0, s, c } };
                case 'Y':
                    return new double[,] { { c, 0, s }, { 0, 1, 0 }, { -s, 0, c } };
                case 'Z':
                    return new double[,] { { c, -s, 0 }, { s, c, 0 }, { 0, 0, 1 } };
                default:
                    throw new ArgumentException($"지원하지 않는 회전축입니다: {axis}");
            }
        }

        private void PrintSelectedResult(HandEyeSweepResult result, HandEyeMode mode)
        {
            if (result == null)
                return;

            string transformName = mode == HandEyeMode.EyeToHand
                ? "T_cam2base"
                : "T_cam2gripper";

            Console.WriteLine(
                $"RMS={result.RmsError:F3} mm, " +
                $"MAX={result.MaxError:F3} mm, " +
                $"{transformName}=({result.CamToGripperX:F6}, " +
                $"{result.CamToGripperY:F6}, {result.CamToGripperZ:F6}), " +
                $"Tnorm={result.CamToGripperNorm:F6}, " +
                $"Rot={result.RotationType}, AngleOrder={result.AngleInputOrder}, " +
                $"EulerConv={result.EulerConvention}, Handedness={HandEyeParams.Handedness}, " +
                $"RobotPoseDirection={HandEyeParams.EyeToHandRobotPoseInputDirection}, Method={result.Method}");
        }

        private void PrintValidationSummary(HandEyeValidationResult validation, HandEyeMode mode)
        {
            if (validation == null)
                return;

            string targetName = mode == HandEyeMode.EyeToHand
                ? "Mean Target in Gripper"
                : "Mean Target in Base";

            Console.WriteLine($"Pose Count = {validation.Poses.Count}");
            Console.WriteLine(
                $"{targetName} = " +
                $"X:{validation.MeanTargetX:F3}, " +
                $"Y:{validation.MeanTargetY:F3}, " +
                $"Z:{validation.MeanTargetZ:F3}");
            Console.WriteLine($"RMS Error = {validation.RmsError:F3} mm");
            Console.WriteLine($"Max Error = {validation.MaxError:F3} mm");
            Console.WriteLine(
                $"StdDev = ({validation.StdDevX:F3}, " +
                $"{validation.StdDevY:F3}, {validation.StdDevZ:F3}) mm");
        }

        private void PrintAxXbResidualSummary(double[,] handEyeMatrix)
        {
            if (handEyeMatrix == null)
                return;

            try
            {
                Console.WriteLine("===== AX=XB RESIDUAL CHECK =====");
                PrintSingleAxXbResidual(EvaluateAxXbResidual(handEyeMatrix, adjacentOnly: false));
                PrintSingleAxXbResidual(EvaluateAxXbResidual(handEyeMatrix, adjacentOnly: true));
            }
            catch (Exception ex)
            {
                Console.WriteLine($"AX=XB residual 계산 실패: {ex.Message}");
            }
        }

        private void PrintSingleAxXbResidual(HandEyeAxXbResidualResult residual)
        {
            if (residual == null)
                return;

            Console.WriteLine(
                $"{residual.PairMode}: Pose={residual.PoseCount}, Pair={residual.PairCount}, " +
                $"Rotation RMSE={residual.RotationRmseRad:F10} rad ({residual.RotationRmseDeg:F6} deg), " +
                $"Rotation MAX={residual.RotationMaxRad:F10} rad ({residual.RotationMaxDeg:F6} deg), " +
                $"Translation RMSE={residual.TranslationRmse:F6} mm, " +
                $"Translation MAX={residual.TranslationMax:F6} mm");
        }

        public void PrintViewerSceneSummary(List<UIEndEffectorData> sceneData)
        {
            if (sceneData == null || sceneData.Count == 0)
                return;

            Console.WriteLine("===== Viewer Camera-Origin Pose Summary =====");
            Console.WriteLine("Camera Origin = (0.000, 0.000, 0.000)");

            foreach (UIEndEffectorData data in sceneData.OrderBy(d => d.PoseIndex))
            {
                double[,] m = data.TransformMatrix4x4;
                if (m == null || m.GetLength(0) != 4 || m.GetLength(1) != 4)
                    continue;

                Console.WriteLine(
                    $"{data.Label ?? $"End Effector {data.PoseIndex}"} | " +
                    $"Pos=({m[0, 3]:F3}, {m[1, 3]:F3}, {m[2, 3]:F3}) mm | " +
                    $"Xdir=({m[0, 0]:F3}, {m[1, 0]:F3}, {m[2, 0]:F3}) | " +
                    $"Ydir=({m[0, 1]:F3}, {m[1, 1]:F3}, {m[2, 1]:F3}) | " +
                    $"Zdir=({m[0, 2]:F3}, {m[1, 2]:F3}, {m[2, 2]:F3})");
            }
        }

        public void RunSelectedHandEyeCalibrationCore(List<RobotTargetPose> r_pose_list, List<CameraTargetPose> c_pose_list,
            Intrinsic intrinsic_data, HandEyeSweepResult bestHandEyeResult, List<int> bestExcludedPoseNumbers,
            HandEyeValidationResult bestValidationResult,
            out double[,] bestHandEyeMatrix, out double[,] bestOutputHandEyeMatrix,
            out HandEyeMode bestResultMode, out EEulerSequence bestResultEulerSequence, out EulerMatrixConvention bestResultEulerConvention,
            out RobotAngleInputOrder bestResultAngleInputOrder, out RobotPoseTransformDirection bestResultRobotPoseDirection,
            out EndEffectorHandedness bestResultHandedness, out HandEyeCalibrationMethod bestResultMethod,
            out OptimizationMethod bestResultOptimizationMethod)
        {
            bestHandEyeMatrix = null;
            bestOutputHandEyeMatrix = null;
            bestResultMode = HandEyeMode.EyeToHand;
            bestResultEulerSequence = EEulerSequence.Intrinsic_ZYZ;
            bestResultEulerConvention = EulerMatrixConvention.DefaultActive;
            bestResultAngleInputOrder = RobotAngleInputOrder.RxRyRz;
            bestResultRobotPoseDirection = RobotPoseTransformDirection.GripperToBase;
            bestResultHandedness = EndEffectorHandedness.Right;
            bestResultMethod = HandEyeCalibrationMethod.ANDREFF;
            bestResultOptimizationMethod = OptimizationMethod.Nonlinear_Reprojection;

            Console.WriteLine("===== BUILD VERSION: final26 / 3D-validation-priority candidate selection enabled =====");

            HandEyeParams.RobotAngleUnit = EAngleUnit.Degree;
            HandEyeParams.ValidateBasic();
            HandEyeParams.ResetRuntimeResults();

            // 새 계산을 시작할 때 이전 결과를 먼저 비웁니다.
            // 입력 검증 실패 또는 계산 중 예외가 발생해도 저장/Viewer 버튼에서 이전 결과가 재사용되지 않도록 합니다.
            if (r_pose_list == null || c_pose_list == null ||
                r_pose_list.Count == 0 || c_pose_list.Count == 0)
            {
                Console.WriteLine("Robot Pose와 Camera Pose를 먼저 로드하세요.");
                return;
            }

            PosePairSet posePairs;

            try
            {
                posePairs = BuildPosePairSet(r_pose_list, c_pose_list);
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
                return;
            }

            if (posePairs.RobotList.Count < HandEyeParams.MinPoseCount)
            {
                Console.WriteLine(
                    $"유효한 Pose Pair가 부족합니다. " +
                    $"Pair={posePairs.RobotList.Count}, Min={HandEyeParams.MinPoseCount}");
                return;
            }

            try
            {
                ValidatePosePairContent(posePairs, requireImagePoints: false);
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
                return;
            }

            EEulerSequence selectedSequence = HandEyeParams.RobotEulerSequence;
            HandEyeCalibrationMethod selectedMethod = HandEyeParams.CalibrationMethod;
            HandEyeMode selectedMode = HandEyeParams.mode;
            OptimizationMethod selectedOptimization = HandEyeParams.Optimization;

            ResolveRobotAngleDefinition(
                posePairs,
                selectedSequence,
                selectedMethod,
                selectedMode,
                out RobotAngleInputOrder selectedAngleInputOrder,
                out EulerMatrixConvention selectedEulerConvention,
                out RobotPoseTransformDirection selectedRobotPoseDirection);

            posePairs = MaybeSelectBetterShiftedPosePairSet(
                posePairs,
                selectedSequence,
                selectedMethod,
                selectedMode);

            posePairs = ResolveCheckerboardFrameAmbiguity(
                posePairs,
                selectedSequence,
                selectedMethod,
                selectedMode);

            if (posePairs.RobotList.Count < HandEyeParams.MinPoseCount)
            {
                Console.WriteLine($"유효한 Pose Pair가 부족합니다. " + $"Pair={posePairs.RobotList.Count}, " +
                    $"Min={HandEyeParams.MinPoseCount}");
                return;
            }

            try
            {
                ValidatePosePairContent(posePairs, requireImagePoints: false);
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
                return;
            }

            Console.WriteLine("===== HAND-EYE CALIBRATION START =====");
            Console.WriteLine($"Mode = {selectedMode}");
            Console.WriteLine($"Euler Sequence = {selectedSequence}");
            Console.WriteLine($"Robot Angle Input Order = {selectedAngleInputOrder}");
            Console.WriteLine($"Euler Matrix Convention = {selectedEulerConvention}");
            if (selectedMode == HandEyeMode.EyeToHand)
                Console.WriteLine($"Eye-to-Hand Robot Pose Direction = {selectedRobotPoseDirection}");
            Console.WriteLine($"Method = {selectedMethod}");
            Console.WriteLine($"Optimization = {selectedOptimization}");
            Console.WriteLine($"Pose Count = {posePairs.RobotList.Count}");
            Console.WriteLine($"Pair Selection = {posePairs.PairSelectionNote}");

            // Closed-form/LOO 단계에서는 Nonlinear를 끕니다.
            // Nonlinear는 최종 선택된 pose set에 대해 한 번만 수행해야 합니다.

            List<RobotTargetPose> finalRobotList = posePairs.RobotList;
            List<CameraTargetPose> finalCameraList = posePairs.CameraList;

            try
            {
                HandEyeValidationResult fullValidation =
                    CalibrateAndValidate(
                        posePairs.RobotList,
                        posePairs.CameraList,
                        selectedSequence,
                        selectedMethod,
                        selectedMode,
                        out double[,] fullMatrix);

                bool fullConventionChanged = TryAutoResolveFullConventionIfNeeded(
                    posePairs,
                    selectedMode,
                    ref selectedSequence,
                    ref selectedMethod,
                    ref selectedAngleInputOrder,
                    ref selectedEulerConvention,
                    ref selectedRobotPoseDirection,
                    ref fullValidation,
                    ref fullMatrix);

                if (fullConventionChanged)
                {
                    posePairs = ResolveCheckerboardFrameAmbiguity(
                        posePairs,
                        selectedSequence,
                        selectedMethod,
                        selectedMode);

                    fullValidation = CalibrateAndValidate(
                        posePairs.RobotList,
                        posePairs.CameraList,
                        selectedSequence,
                        selectedMethod,
                        selectedMode,
                        out fullMatrix);
                }

                finalRobotList = posePairs.RobotList;
                finalCameraList = posePairs.CameraList;

                bestHandEyeResult = CreateSingleResult(
                    selectedSequence,
                    selectedMethod,
                    selectedMode,
                    fullMatrix,
                    fullValidation);

                Console.WriteLine("===== SELECTED HAND-EYE RESULT =====");
                PrintSelectedResult(bestHandEyeResult, selectedMode);

                Console.WriteLine("===== FULL POSE VALIDATION =====");
                PrintValidationSummary(fullValidation, selectedMode);

                OutlierSelectionResult outlierSelection = RunIterativeOutlierSelection(
                    posePairs.RobotList,
                    posePairs.CameraList,
                    selectedSequence,
                    selectedMethod,
                    selectedMode,
                    fullValidation,
                    fullMatrix);

                bestExcludedPoseNumbers = outlierSelection.RemovedPoseNumbers;
                finalRobotList = outlierSelection.RobotList;
                finalCameraList = outlierSelection.CameraList;

                // LOO 테스트는 내부 pose buffer를 마지막 subset 상태로 바꿔놓을 수 있습니다.
                // 따라서 최종 선택된 pose set으로 반드시 다시 계산해 내부 buffer와 검증 결과를 동기화합니다.
                bestValidationResult = CalibrateAndValidate(
                    finalRobotList,
                    finalCameraList,
                    selectedSequence,
                    selectedMethod,
                    selectedMode,
                    out bestHandEyeMatrix);

                switch (selectedOptimization)
                {
                    case OptimizationMethod.Linear:
                        Console.WriteLine("OptimizationMethod=Linear -> Closed-form/LOO 결과만 사용합니다.");
                        break;

                    case OptimizationMethod.Linear_Reprojection:
                        if (intrinsic_data == null)
                        {
                            Console.WriteLine("카메라 내부 파라미터가 없어 Linear_Reprojection 평가를 생략합니다.");
                            break;
                        }

                        if (selectedMode != HandEyeMode.EyeToHand)
                        {
                            Console.WriteLine("현재 Linear_Reprojection 평가는 EyeToHand 경로만 구현되어 있어 EyeInHand에서는 선형 결과만 사용합니다.");
                            break;
                        }

                        Console.WriteLine("===== 선형 결과 2D 재투영 RMS 평가 시작 =====");
                        using (Mat camMat = MathUtils.BuildCameraMatrix(intrinsic_data))
                        using (Mat dist = MathUtils.BuildDistCoeffs(intrinsic_data))
                        {
                            try
                            {
                                ValidatePosePairContent(
                                    new PosePairSet
                                    {
                                        RobotList = finalRobotList,
                                        CameraList = finalCameraList
                                    },
                                    requireImagePoints: true);

                                double linearRmsPx = EvaluateEyeToHandReprojectionRms(
                                    bestHandEyeMatrix,
                                    finalRobotList,
                                    finalCameraList,
                                    camMat,
                                    dist,
                                    selectedSequence);

                                Console.WriteLine($"Linear_Reprojection RMS={linearRmsPx:F3} px. 행렬은 선형/LOO 결과를 유지합니다.");
                            }
                            catch (Exception ex)
                            {
                                Console.WriteLine($"Linear_Reprojection 평가 실패: {ex.Message}");
                                Console.WriteLine("선형/LOO 결과를 그대로 사용합니다.");
                            }
                        }
                        break;

                    case OptimizationMethod.Nonlinear:
                        Console.WriteLine("===== 3D 일관성 기반 6-DoF 비선형 최적화 시작 =====");
                        try
                        {
                            bestHandEyeMatrix = RefineCalibrationNonlinearConsistency6DoF(
                                bestHandEyeMatrix,
                                finalRobotList,
                                finalCameraList,
                                selectedSequence,
                                selectedMode);

                            if (HandEyeParams.LastNonlinearAccepted)
                            {
                                Console.WriteLine($"3D 비선형 최적화 채택 완료! Consistency RMS: {HandEyeParams.LastNonlinearConsistencyRmsMm:F3} mm");
                            }
                            else
                            {
                                Console.WriteLine($"3D 비선형 최적화 미채택. 선형/LOO 결과 유지. 기준 RMS: {HandEyeParams.LastNonlinearConsistencyRmsMm:F3} mm");
                            }

                            bestValidationResult = BuildValidationResultByMode(
                                bestHandEyeMatrix,
                                selectedMode);
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine($"3D Nonlinear Optimization 실패: {ex.Message}");
                            Console.WriteLine("선형/LOO 결과를 그대로 사용합니다.");
                        }
                        break;

                    case OptimizationMethod.Nonlinear_Reprojection:
                        if (intrinsic_data == null)
                        {
                            Console.WriteLine("카메라 내부 파라미터가 없어 Nonlinear_Reprojection을 생략합니다.");
                            break;
                        }

                        if (selectedMode != HandEyeMode.EyeToHand)
                        {
                            Console.WriteLine("현재 12-DoF Nonlinear_Reprojection은 EyeToHand만 구현되어 있어 EyeInHand에서는 생략합니다.");
                            break;
                        }

                        Console.WriteLine("===== 12-DoF 비선형 재투영 오차 최적화 시작 =====");
                        using (Mat camMat = MathUtils.BuildCameraMatrix(intrinsic_data))
                        using (Mat dist = MathUtils.BuildDistCoeffs(intrinsic_data))
                        {
                            try
                            {
                                ValidatePosePairContent(
                                    new PosePairSet
                                    {
                                        RobotList = finalRobotList,
                                        CameraList = finalCameraList
                                    },
                                    requireImagePoints: true);

                                bestHandEyeMatrix = Optimizer.RefineCalibrationNonlinear12DoF(
                                    bestHandEyeMatrix,
                                    finalRobotList,
                                    finalCameraList,
                                    camMat,
                                    dist,
                                    selectedSequence);

                                if (HandEyeParams.LastNonlinearAccepted)
                                {
                                    Console.WriteLine($"재투영 비선형 최적화 채택 완료! Reprojection RMS: {HandEyeParams.LastNonlinearReprojectionRmsPx:F3} px");
                                    if (HandEyeParams.LastTarget2Gripper != null)
                                    {
                                        Console.WriteLine(
                                            $"최적화 Target2Gripper T=({HandEyeParams.LastTarget2Gripper[0, 3]:F3}, " +
                                            $"{HandEyeParams.LastTarget2Gripper[1, 3]:F3}, {HandEyeParams.LastTarget2Gripper[2, 3]:F3}) mm");
                                    }
                                }
                                else
                                {
                                    Console.WriteLine($"재투영 비선형 최적화 미채택. 선형/LOO 결과 유지. 기준 RMS: {HandEyeParams.LastNonlinearReprojectionRmsPx:F3} px");
                                }

                                bestValidationResult = BuildValidationResultByMode(
                                    bestHandEyeMatrix,
                                    selectedMode);
                            }
                            catch (Exception ex)
                            {
                                Console.WriteLine($"12-DoF Nonlinear_Reprojection 실패: {ex.Message}");
                                Console.WriteLine("선형/LOO 결과를 그대로 사용합니다.");
                            }
                        }
                        break;

                    default:
                        Console.WriteLine($"지원하지 않는 OptimizationMethod={selectedOptimization}. 선형/LOO 결과를 사용합니다.");
                        break;
                }

                bestHandEyeResult = CreateSingleResult(
                    selectedSequence,
                    selectedMethod,
                    selectedMode,
                    bestHandEyeMatrix,
                    bestValidationResult);

                // Solver/Validation에는 raw matrix를 사용하고, CoordianteConvert는 외부 출력용 행렬에만 적용합니다.
                bestOutputHandEyeMatrix = GetOutputMatrix(bestHandEyeMatrix);
                Console.WriteLine($"CoordianteConvert={HandEyeParams.CoordConvertMode} -> Output Matrix 생성 완료");

                bestResultMode = selectedMode;
                bestResultEulerSequence = selectedSequence;
                bestResultEulerConvention = selectedEulerConvention;
                bestResultAngleInputOrder = selectedAngleInputOrder;
                bestResultRobotPoseDirection = selectedRobotPoseDirection;
                bestResultHandedness = HandEyeParams.Handedness;
                bestResultMethod = selectedMethod;
                bestResultOptimizationMethod = selectedOptimization;
            }
            finally
            {
            }

            Console.WriteLine("===== FINAL HAND-EYE VALIDATION =====");
            Console.WriteLine($"Mode = {selectedMode}");
            Console.WriteLine(
                $"Excluded Pose = {(bestExcludedPoseNumbers.Count == 0 ? "None" : string.Join(",", bestExcludedPoseNumbers))}");
            PrintValidationSummary(bestValidationResult, selectedMode);
            PrintAxXbResidualSummary(bestHandEyeMatrix);
            PrintPoseErrors(bestValidationResult);
            PrintCalibrationHealthDiagnostics(bestExcludedPoseNumbers, bestValidationResult, selectedMode, selectedOptimization);
        }

        private void ResolveRobotAngleDefinition(
            PosePairSet posePairs, EEulerSequence sequence, HandEyeCalibrationMethod method, HandEyeMode mode,
            out RobotAngleInputOrder selectedAngleInputOrder, out EulerMatrixConvention selectedEulerConvention,
            out RobotPoseTransformDirection selectedRobotPoseDirection)
        {
            RobotAngleInputOrder originalAngleOrder = HandEyeParams.RobotAngleInput;
            EulerMatrixConvention originalConvention = HandEyeParams.RobotEulerMatrixConvention;
            RobotPoseTransformDirection originalRobotPoseDirection = HandEyeParams.EyeToHandRobotPoseInputDirection;

            selectedAngleInputOrder = originalAngleOrder;
            selectedEulerConvention = originalConvention;
            selectedRobotPoseDirection = originalRobotPoseDirection;

            bool canResolveAngleDefinition =
                HandEyeParams.AutoResolveRobotAngleDefinition && !IsRotationVectorSequence(sequence);
            bool canResolveRobotPoseDirection =
                mode == HandEyeMode.EyeToHand && HandEyeParams.AutoResolveEyeToHandRobotPoseInputDirection;

            if (!canResolveAngleDefinition && !canResolveRobotPoseDirection)
            {
                Console.WriteLine(
                    $"Robot definition 자동 비교 생략: AngleOrder={originalAngleOrder}, " +
                    $"EulerConv={originalConvention}, RobotPoseDirection={originalRobotPoseDirection}");
                return;
            }

            Console.WriteLine("===== ROBOT ANGLE DEFINITION DIAGNOSTIC =====");
            Console.WriteLine(
                $"Sequence={sequence}, Method={method}, Mode={mode}, " +
                $"OriginalAngleOrder={originalAngleOrder}, OriginalEulerConv={originalConvention}, " +
                $"OriginalRobotPoseDirection={originalRobotPoseDirection}");
            Console.WriteLine("CSV의 Rx/Ry/Rz 열 순서, Euler 행렬 정의, Eye-to-Hand 로봇 pose 방향을 함께 비교합니다.");

            List<EulerConventionTestResult> results = new List<EulerConventionTestResult>();

            RobotAngleInputOrder[] angleOrders = canResolveAngleDefinition
                ? GetSupportedRobotAngleInputOrders()
                : new RobotAngleInputOrder[] { originalAngleOrder };

            EulerMatrixConvention[] conventions = canResolveAngleDefinition && HandEyeParams.AutoResolveRobotEulerMatrixConvention
                ? GetSupportedEulerMatrixConventions()
                : new EulerMatrixConvention[] { originalConvention };

            RobotPoseTransformDirection[] robotPoseDirections = canResolveRobotPoseDirection
                ? new RobotPoseTransformDirection[]
                {
                    RobotPoseTransformDirection.GripperToBase,
                    RobotPoseTransformDirection.BaseToGripper
                }
                : new RobotPoseTransformDirection[] { originalRobotPoseDirection };

            foreach (RobotAngleInputOrder angleOrder in angleOrders)
            {
                foreach (EulerMatrixConvention convention in conventions)
                {
                    foreach (RobotPoseTransformDirection robotPoseDirection in robotPoseDirections)
                    {
                        try
                        {
                            HandEyeParams.RobotAngleInput = angleOrder;
                            HandEyeParams.RobotEulerMatrixConvention = convention;
                            HandEyeParams.EyeToHandRobotPoseInputDirection = robotPoseDirection;

                            HandEyeValidationResult validation = CalibrateAndValidate(
                                posePairs.RobotList,
                                posePairs.CameraList,
                                sequence,
                                method,
                                mode,
                                out double[,] matrix);

                            results.Add(new EulerConventionTestResult
                            {
                                AngleInputOrder = angleOrder,
                                Convention = convention,
                                RobotPoseDirection = robotPoseDirection,
                                RmsError = validation.RmsError,
                                MaxError = validation.MaxError
                            });

                            Console.WriteLine(
                                $"AngleOrder={angleOrder}, EulerConv={convention}, RobotPoseDirection={robotPoseDirection} -> " +
                                $"RMS={validation.RmsError:F3} mm, MAX={validation.MaxError:F3} mm");
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine(
                                $"AngleOrder={angleOrder}, EulerConv={convention}, RobotPoseDirection={robotPoseDirection} -> FAIL: {ex.Message}");
                        }
                    }
                }
            }

            if (results.Count == 0)
            {
                HandEyeParams.RobotAngleInput = originalAngleOrder;
                HandEyeParams.RobotEulerMatrixConvention = originalConvention;
                HandEyeParams.EyeToHandRobotPoseInputDirection = originalRobotPoseDirection;
                Console.WriteLine(
                    $"[Robot Definition Keep] 유효 결과 없음. AngleOrder={originalAngleOrder}, " +
                    $"EulerConv={originalConvention}, RobotPoseDirection={originalRobotPoseDirection} 유지");
                return;
            }

            EulerConventionTestResult best = results.OrderBy(r => r.RmsError).First();
            EulerConventionTestResult original = results.FirstOrDefault(r =>
                r.AngleInputOrder == originalAngleOrder &&
                r.Convention == originalConvention &&
                r.RobotPoseDirection == originalRobotPoseDirection);

            if (original == null)
            {
                HandEyeParams.RobotAngleInput = best.AngleInputOrder;
                HandEyeParams.RobotEulerMatrixConvention = best.Convention;
                HandEyeParams.EyeToHandRobotPoseInputDirection = best.RobotPoseDirection;
                selectedAngleInputOrder = best.AngleInputOrder;
                selectedEulerConvention = best.Convention;
                selectedRobotPoseDirection = best.RobotPoseDirection;
                Console.WriteLine(
                    $"[Robot Definition Selected] AngleOrder={best.AngleInputOrder}, " +
                    $"EulerConv={best.Convention}, RobotPoseDirection={best.RobotPoseDirection}, RMS={best.RmsError:F3} mm");
                return;
            }

            double improvementRatio = (original.RmsError - best.RmsError) / Math.Max(original.RmsError, 1e-9);

            if ((best.AngleInputOrder != originalAngleOrder ||
                 best.Convention != originalConvention ||
                 best.RobotPoseDirection != originalRobotPoseDirection) &&
                improvementRatio >= HandEyeParams.MinEulerConventionImprovementRatio)
            {
                HandEyeParams.RobotAngleInput = best.AngleInputOrder;
                HandEyeParams.RobotEulerMatrixConvention = best.Convention;
                HandEyeParams.EyeToHandRobotPoseInputDirection = best.RobotPoseDirection;
                selectedAngleInputOrder = best.AngleInputOrder;
                selectedEulerConvention = best.Convention;
                selectedRobotPoseDirection = best.RobotPoseDirection;
                Console.WriteLine(
                    $"[Robot Definition Selected] " +
                    $"AngleOrder {originalAngleOrder} -> {best.AngleInputOrder}, " +
                    $"EulerConv {originalConvention} -> {best.Convention}, " +
                    $"RobotPoseDirection {originalRobotPoseDirection} -> {best.RobotPoseDirection}, " +
                    $"RMS {original.RmsError:F3} -> {best.RmsError:F3} mm, 개선율={improvementRatio * 100.0:F1}%");
                return;
            }

            HandEyeParams.RobotAngleInput = originalAngleOrder;
            HandEyeParams.RobotEulerMatrixConvention = originalConvention;
            HandEyeParams.EyeToHandRobotPoseInputDirection = originalRobotPoseDirection;
            selectedAngleInputOrder = originalAngleOrder;
            selectedEulerConvention = originalConvention;
            selectedRobotPoseDirection = originalRobotPoseDirection;
            Console.WriteLine(
                $"[Robot Definition Keep] AngleOrder={originalAngleOrder}, EulerConv={originalConvention}, " +
                $"RobotPoseDirection={originalRobotPoseDirection} 유지. " +
                $"Best AngleOrder={best.AngleInputOrder}, EulerConv={best.Convention}, RobotPoseDirection={best.RobotPoseDirection}, " +
                $"RMS {original.RmsError:F3} -> {best.RmsError:F3} mm, 개선율={improvementRatio * 100.0:F1}%");
        }

        private PosePairSet MaybeSelectBetterShiftedPosePairSet(
            PosePairSet original,
            EEulerSequence sequence,
            HandEyeCalibrationMethod method,
            HandEyeMode mode)
        {
            if (original == null ||
                original.RobotList == null ||
                original.CameraList == null ||
                original.RobotList.Count != original.CameraList.Count)
            {
                return original;
            }

            if (!HandEyeParams.EnableAutoPairShiftSelection)
                return original;

            bool robotHasExplicitIndex = original.RobotList.Any(p =>
                p != null && p.HasExplicitPoseIndex && p.SourcePoseIndex > 0);

            bool cameraHasExplicitIndex = original.CameraList.Any(p =>
                p != null && p.HasExplicitPoseIndex && p.SourcePoseIndex > 0);

            // 양쪽 모두 명시 index가 있으면 이미 index join을 사용하고 있으므로 shift 추정은 하지 않습니다.
            // 양쪽 모두 명시 index가 없으면 어느 쪽이 밀렸는지 판단 근거가 부족합니다.
            if (robotHasExplicitIndex == cameraHasExplicitIndex)
                return original;

            int count = original.RobotList.Count;
            if (count <= HandEyeParams.MinPoseCount + 1)
                return original;

            Console.WriteLine("===== POSE PAIR SHIFT DIAGNOSTIC =====");
            Console.WriteLine("한쪽에만 명시 PoseIndex가 있어 한 칸 밀림 후보를 추가 검증합니다.");

            List<PosePairSelectionCandidate> candidates = new List<PosePairSelectionCandidate>
            {
                new PosePairSelectionCandidate
                {
                    Name = "Original order",
                    RobotList = original.RobotList.ToList(),
                    CameraList = original.CameraList.ToList(),
                    IsOriginal = true
                },
                new PosePairSelectionCandidate
                {
                    Name = "Drop first Robot / drop last Camera",
                    RobotList = original.RobotList.Skip(1).ToList(),
                    CameraList = original.CameraList.Take(count - 1).ToList(),
                    IsOriginal = false
                },
                new PosePairSelectionCandidate
                {
                    Name = "Drop last Robot / drop first Camera",
                    RobotList = original.RobotList.Take(count - 1).ToList(),
                    CameraList = original.CameraList.Skip(1).ToList(),
                    IsOriginal = false
                }
            };

            foreach (PosePairSelectionCandidate candidate in candidates)
            {
                if (candidate.RobotList.Count < HandEyeParams.MinPoseCount ||
                    candidate.CameraList.Count < HandEyeParams.MinPoseCount)
                {
                    Console.WriteLine($"{candidate.Name} -> SKIP: Pair Count={candidate.RobotList.Count}");
                    continue;
                }

                try
                {
                    candidate.Validation = CalibrateAndValidate(
                        candidate.RobotList,
                        candidate.CameraList,
                        sequence,
                        method,
                        mode,
                        out double[,] _);

                    Console.WriteLine(
                        $"{candidate.Name} -> Pair={candidate.RobotList.Count}, " +
                        $"RMS={candidate.Validation.RmsError:F3} mm, MAX={candidate.Validation.MaxError:F3} mm");
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"{candidate.Name} -> FAIL: {ex.Message}");
                    candidate.Validation = null;
                }
            }

            PosePairSelectionCandidate originalCandidate = candidates
                .FirstOrDefault(c => c.IsOriginal && c.Validation != null);

            PosePairSelectionCandidate bestCandidate = candidates
                .Where(c => c.Validation != null)
                .OrderBy(c => c.Validation.RmsError)
                .FirstOrDefault();

            if (originalCandidate == null || bestCandidate == null)
                return original;

            double originalRms = originalCandidate.Validation.RmsError;
            double bestRms = bestCandidate.Validation.RmsError;
            double improvementRatio = originalRms > 0.0
                ? (originalRms - bestRms) / originalRms
                : 0.0;

            if (!bestCandidate.IsOriginal &&
                bestRms < originalRms &&
                improvementRatio >= HandEyeParams.AutoPairShiftMinImprovementRatio)
            {
                Console.WriteLine(
                    $"[Pose Pair Shift Auto Select] {bestCandidate.Name} 채택. " +
                    $"RMS {originalRms:F3} -> {bestRms:F3} mm, 개선율={improvementRatio:P1}");

                return new PosePairSet
                {
                    RobotList = bestCandidate.RobotList,
                    CameraList = bestCandidate.CameraList,
                    MatchedByExplicitIndex = original.MatchedByExplicitIndex,
                    PairSelectionNote = bestCandidate.Name
                };
            }

            if (!bestCandidate.IsOriginal && bestRms < originalRms)
            {
                Console.WriteLine(
                    $"[Pose Pair Shift Keep Original] shift 후보가 더 좋지만 개선율={improvementRatio:P1}로 " +
                    $"자동 채택 기준 {HandEyeParams.AutoPairShiftMinImprovementRatio:P1} 미만입니다.");
            }
            else
            {
                Console.WriteLine("[Pose Pair Shift Keep Original] 원래 순서 매칭이 가장 낫거나 차이가 충분하지 않습니다.");
            }

            return original;
        }

        public OutlierSelectionResult RunIterativeOutlierSelection(
            List<RobotTargetPose> robotList,
            List<CameraTargetPose> cameraList,
            EEulerSequence sequence,
            HandEyeCalibrationMethod method,
            HandEyeMode mode,
            HandEyeValidationResult initialValidation,
            double[,] initialMatrix)
        {
            OutlierSelectionResult result = new OutlierSelectionResult
            {
                RobotList = robotList != null ? robotList.ToList() : new List<RobotTargetPose>(),
                CameraList = cameraList != null ? cameraList.ToList() : new List<CameraTargetPose>(),
                Validation = initialValidation,
                Matrix = initialMatrix
            };

            if (!HandEyeParams.EnableIterativeOutlierRejection ||
                HandEyeParams.MaxAutoExcludedPoseCount <= 0)
            {
                Console.WriteLine("자동 반복 LOO가 비활성화되어 전체 pose set을 유지합니다.");
                return result;
            }

            Console.WriteLine("===== ITERATIVE LEAVE-ONE-OUT OUTLIER SELECTION =====");

            int maxRemoveByCount = Math.Max(0, result.RobotList.Count - HandEyeParams.MinPoseCount);
            int maxRemove = Math.Min(HandEyeParams.MaxAutoExcludedPoseCount, maxRemoveByCount);

            if (maxRemove <= 0)
            {
                Console.WriteLine("반복 LOO 생략: 최소 pose 개수 조건 때문에 더 이상 제외할 수 없습니다.");
                return result;
            }

            for (int step = 0; step < maxRemove; step++)
            {
                if (result.Validation == null)
                    break;

                double baselineRms = result.Validation.RmsError;
                Console.WriteLine($"[LOO Step {step + 1}] Baseline RMS={baselineRms:F3} mm, Pair Count={result.RobotList.Count}");

                LeaveOneOutResult looBest = RunLeaveOneOutBest(
                    result.RobotList,
                    result.CameraList,
                    sequence,
                    method,
                    mode);

                if (looBest == null || looBest.Validation == null)
                {
                    Console.WriteLine($"[LOO Step {step + 1}] 유효한 제거 후보가 없습니다.");
                    break;
                }

                double candidateRms = looBest.Validation.RmsError;
                double improvement = baselineRms - candidateRms;
                double improvementRatio = baselineRms > 0.0 ? improvement / baselineRms : 0.0;

                if (candidateRms < baselineRms &&
                    improvementRatio >= HandEyeParams.MinOutlierImprovementRatio)
                {
                    result.RemovedPoseNumbers.Add(looBest.RemovedPoseNumber);
                    result.RobotList = looBest.RobotList;
                    result.CameraList = looBest.CameraList;
                    result.Validation = looBest.Validation;
                    result.Matrix = looBest.Matrix;

                    Console.WriteLine(
                        $"[LOO Step {step + 1} Accept] Remove Pose {looBest.RemovedPoseNumber:D2}, " +
                        $"RMS {baselineRms:F3} -> {candidateRms:F3} mm, 개선율={improvementRatio:P1}");
                }
                else
                {
                    Console.WriteLine(
                        $"[LOO Step {step + 1} Stop] Best Remove Pose {looBest.RemovedPoseNumber:D2}, " +
                        $"RMS {baselineRms:F3} -> {candidateRms:F3} mm, 개선율={improvementRatio:P1}. " +
                        $"채택 기준={HandEyeParams.MinOutlierImprovementRatio:P1}");
                    break;
                }
            }

            Console.WriteLine(
                $"반복 LOO 결과: 제외 Pose=" +
                $"{(result.RemovedPoseNumbers.Count == 0 ? "None" : string.Join(",", result.RemovedPoseNumbers))}, " +
                $"최종 Pair Count={result.RobotList.Count}, " +
                $"예상 RMS={(result.Validation == null ? double.NaN : result.Validation.RmsError):F3} mm");

            return result;
        }

        private bool TryGetNormalAndRotZ180Candidates(
         CameraTargetPose pose,
         out CameraTargetPose normal,
         out CameraTargetPose rot180)
        {
            normal = pose;
            rot180 = null;

            if (pose == null)
                return false;

            if (pose.FrameCandidates == null || pose.FrameCandidates.Count == 0)
                return false;

            foreach (CameraTargetPose candidate in pose.FrameCandidates)
            {
                if (candidate == null)
                    continue;

                string name = string.IsNullOrWhiteSpace(candidate.FrameCandidateName)
                    ? "Normal"
                    : candidate.FrameCandidateName;

                if (string.Equals(name, "Normal", StringComparison.OrdinalIgnoreCase))
                    normal = candidate;
                else if (string.Equals(name, "RotZ180", StringComparison.OrdinalIgnoreCase))
                    rot180 = candidate;
            }

            return normal != null && rot180 != null;
        }

        private void OptimizeDirectionLabelsCoordinateDescent(
            int[] labels,
            List<CheckerboardDirectionEdge> edges,
            int count,
            int maxPasses)
        {
            for (int pass = 0; pass < maxPasses; pass++)
            {
                bool changed = false;

                for (int pose = 0; pose < count; pose++)
                {
                    double cost0 = ComputeSingleLabelCost(pose, 0, labels, edges);
                    double cost1 = ComputeSingleLabelCost(pose, 1, labels, edges);
                    int bestLabel = cost1 < cost0 ? 1 : 0;

                    if (bestLabel != labels[pose])
                    {
                        labels[pose] = bestLabel;
                        changed = true;
                    }
                }

                if (!changed)
                    break;
            }
        }

        private double ComputeSingleLabelCost(
            int pose,
            int label,
            int[] labels,
            List<CheckerboardDirectionEdge> edges)
        {
            double sum = 0.0;

            foreach (CheckerboardDirectionEdge edge in edges)
            {
                if (edge.I == pose)
                    sum += edge.Cost[label, labels[edge.J]];
                else if (edge.J == pose)
                    sum += edge.Cost[labels[edge.I], label];
            }

            return sum;
        }

        private double ComputeDirectionLabelCost(int[] labels, List<CheckerboardDirectionEdge> edges)
        {
            double sum = 0.0;

            foreach (CheckerboardDirectionEdge edge in edges)
                sum += edge.Cost[labels[edge.I], labels[edge.J]];

            return sum;
        }

        private double DirectionCostToDegRms(double cost, int edgeCount)
        {
            if (edgeCount <= 0)
                return double.NaN;

            return Math.Sqrt(cost / edgeCount) * 180.0 / Math.PI;
        }

        private int CountDirectionFlips(int[] labels)
        {
            if (labels == null)
                return 0;

            int count = 0;
            for (int i = 0; i < labels.Length; i++)
            {
                if (labels[i] == 1)
                    count++;
            }
            return count;
        }

        private bool SameLabels(int[] a, int[] b)
        {
            if (a == null || b == null || a.Length != b.Length)
                return false;

            for (int i = 0; i < a.Length; i++)
            {
                if (a[i] != b[i])
                    return false;
            }
            return true;
        }

        private double[,] BuildRobotRotationForDirectionDiagnostic(
            RobotTargetPose pose,
            EEulerSequence sequence,
            HandEyeMode mode)
        {
            if (pose == null)
                throw new ArgumentNullException(nameof(pose));

            double[,] R = RobotRotationToMatrixArray(
                pose.robotRx,
                pose.robotRy,
                pose.robotRz,
                sequence);

            if (HandEyeParams.Handedness == EndEffectorHandedness.Left)
            {
                double[,] S = new double[,] { { 1, 0, 0 }, { 0, -1, 0 }, { 0, 0, 1 } };
                R = MathUtils.Multiply3x3(MathUtils.Multiply3x3(S, R), S);
            }

            if (mode == HandEyeMode.EyeToHand &&
                HandEyeParams.EyeToHandRobotPoseInputDirection == RobotPoseTransformDirection.GripperToBase)
            {
                R = MathUtils.Transpose3x3(R);
            }

            return R;
        }



        private double[,] RobotRotationToMatrixArray(
            double rx,
            double ry,
            double rz,
            EEulerSequence sequence)
        {
            if (sequence == EEulerSequence.RotVecDegree || sequence == EEulerSequence.RotVecRadian)
            {
                double scale = sequence == EEulerSequence.RotVecDegree ? Math.PI / 180.0 : 1.0;
                using (Mat rvec = new Mat(3, 1, MatType.CV_64FC1, new double[] { rx * scale, ry * scale, rz * scale }))
                using (Mat R = new Mat())
                {
                    Cv2.Rodrigues(rvec, R);
                    return MathUtils.Mat3x3ToArrayLocal(R);
                }
            }

            MapRobotEulerAnglesLocal(rx, ry, rz, HandEyeParams.RobotAngleInput,
                out double a1, out double a2, out double a3);

            return EulerToMatrixArray(a1, a2, a3,
                sequence, HandEyeParams.RobotAngleUnit, HandEyeParams.RobotEulerMatrixConvention);
        }

        private void MapRobotEulerAnglesLocal(double rx, double ry, double rz, RobotAngleInputOrder order,
            out double a1, out double a2, out double a3)
        {
            switch (order)
            {
                case RobotAngleInputOrder.RxRyRz:
                    a1 = rx; a2 = ry; a3 = rz; return;
                case RobotAngleInputOrder.RxRzRy:
                    a1 = rx; a2 = rz; a3 = ry; return;
                case RobotAngleInputOrder.RyRxRz:
                    a1 = ry; a2 = rx; a3 = rz; return;
                case RobotAngleInputOrder.RyRzRx:
                    a1 = ry; a2 = rz; a3 = rx; return;
                case RobotAngleInputOrder.RzRxRy:
                    a1 = rz; a2 = rx; a3 = ry; return;
                case RobotAngleInputOrder.RzRyRx:
                    a1 = rz; a2 = ry; a3 = rx; return;
                default:
                    throw new ArgumentOutOfRangeException(nameof(order), order, "지원하지 않는 RobotAngleInputOrder입니다.");
            }
        }

        private double[,] EulerToMatrixArray(double a1, double a2, double a3,
            EEulerSequence sequence, EAngleUnit angleUnit, EulerMatrixConvention convention)
        {
            double r1 = angleUnit == EAngleUnit.Degree ? a1 * Math.PI / 180.0 : a1;
            double r2 = angleUnit == EAngleUnit.Degree ? a2 * Math.PI / 180.0 : a2;
            double r3 = angleUnit == EAngleUnit.Degree ? a3 * Math.PI / 180.0 : a3;

            if (IsNegatedEulerConventionLocal(convention))
            {
                r1 = -r1;
                r2 = -r2;
                r3 = -r3;
            }

            char[] axes = GetEulerAxesLocal(sequence);
            double[,] R1 = AxisRotationArray(axes[0], r1);
            double[,] R2 = AxisRotationArray(axes[1], r2);
            double[,] R3 = AxisRotationArray(axes[2], r3);

            bool useReverseOrder = IsReverseOrderEulerConventionLocal(convention);
            bool useIntrinsicOrder = IsIntrinsicLocal(sequence);

            if (useReverseOrder)
                useIntrinsicOrder = !useIntrinsicOrder;

            double[,] active = useIntrinsicOrder
                ? MathUtils.Multiply3x3(MathUtils.Multiply3x3(R1, R2), R3)
                : MathUtils.Multiply3x3(MathUtils.Multiply3x3(R3, R2), R1);

            if (IsPassiveEulerConventionLocal(convention))
                return MathUtils.Transpose3x3(active);

            return active;
        }

        public void PrintTopSweepResults(List<HandEyeSweepResult> results, int topCount = 20)
        {
            Console.WriteLine("===== Hand-Eye Convention Sweep Result =====");

            var validResults = results
                .Where(r =>
                    !double.IsNaN(r.RmsError) &&
                    !double.IsInfinity(r.RmsError) &&
                    !double.IsNaN(r.CamToGripperNorm) &&
                    !double.IsInfinity(r.CamToGripperNorm))
                .Take(topCount);

            foreach (var r in validResults)
            {
                Console.WriteLine(
                    $"RMS={r.RmsError:F3} mm, " +
                    $"MAX={r.MaxError:F3} mm, " +
                    $"STD=({r.StdDevX:F3}, {r.StdDevY:F3}, {r.StdDevZ:F3}), " +
                    $"Tnorm={r.CamToGripperNorm:F3} mm, " +
                    $"Rot={r.RotationType}, " +
                    $"EulerConv={r.EulerConvention}, " +
                    $"AngleOrder={r.AngleInputOrder}, " +
                    $"InvRobot={r.InvertRobotPose}, " +
                    $"InvCam={r.InvertCameraPose}, " +
                    $"Method={r.Method}, " +
                    $"T=({r.CamToGripperX:F3}, {r.CamToGripperY:F3}, {r.CamToGripperZ:F3})");
            }
        }

        private bool TryResolveCheckerboardDirectionByRelativeMotion(
           PosePairSet original, EEulerSequence sequence, HandEyeCalibrationMethod method, HandEyeMode mode, out PosePairSet resolved)
        {
            resolved = null;

            if (!HandEyeParams.EnableCheckerboardDirectionDisambiguation)
            {
                Console.WriteLine("체커보드 180도 방향 식별이 비활성화되어 있습니다.");
                return false;
            }

            if (mode != HandEyeMode.EyeToHand)
            {
                Console.WriteLine("체커보드 180도 방향 식별은 현재 EyeToHand 경로에서만 수행합니다.");
                return false;
            }

            if (original == null || original.RobotList == null || original.CameraList == null ||
                original.RobotList.Count != original.CameraList.Count || original.CameraList.Count < 2)
            {
                return false;
            }

            int n = original.CameraList.Count;
            CameraTargetPose[,] candidates = new CameraTargetPose[n, 2];

            for (int i = 0; i < n; i++)
            {
                CameraTargetPose normal;
                CameraTargetPose rot180;
                if (!TryGetNormalAndRotZ180Candidates(original.CameraList[i], out normal, out rot180))
                {
                    Console.WriteLine("[Checkerboard Direction Skip] Normal/RotZ180 후보가 없는 pose가 있어 방향 식별을 생략합니다.");
                    return false;
                }

                candidates[i, 0] = normal;
                candidates[i, 1] = rot180;
            }

            Console.WriteLine("===== CHECKERBOARD 180-DEG DIRECTION DISAMBIGUATION =====");
            Console.WriteLine("PnP RMS가 낮아도 대칭 checkerboard의 corner[0]이 pose마다 반대 코너가 될 수 있으므로,");
            Console.WriteLine("로봇 상대 회전각과 카메라 상대 회전각의 일관성으로 Normal/RotZ180을 전역 선택합니다.");

            double[][,] robotRotations = new double[n][,];
            double[][,] cameraRotations = new double[n * 2][,];

            for (int i = 0; i < n; i++)
            {
                robotRotations[i] = BuildRobotRotationForDirectionDiagnostic(original.RobotList[i], sequence, mode);
                cameraRotations[i * 2 + 0] = candidates[i, 0].RotationMatrix;
                cameraRotations[i * 2 + 1] = candidates[i, 1].RotationMatrix;
            }

            List<CheckerboardDirectionEdge> edges = BuildCheckerboardDirectionEdges(robotRotations, cameraRotations);
            if (edges.Count == 0)
            {
                Console.WriteLine("[Checkerboard Direction Skip] 판정 가능한 상대 회전 pair가 없습니다.");
                return false;
            }

            int[] normalLabels = new int[n];
            double normalCost = ComputeDirectionLabelCost(normalLabels, edges);
            double normalCostDeg = DirectionCostToDegRms(normalCost, edges.Count);

            List<CheckerboardDirectionSolution> solutions = new List<CheckerboardDirectionSolution>();
            AddOptimizedDirectionSolution(solutions, normalLabels, edges, n);

            int[] allRot = Enumerable.Repeat(1, n).ToArray();
            AddOptimizedDirectionSolution(solutions, allRot, edges, n);

            AddOptimizedDirectionSolution(solutions, BuildAnchorBasedDirectionLabels(edges, n, 0, 0), edges, n);
            AddOptimizedDirectionSolution(solutions, BuildAnchorBasedDirectionLabels(edges, n, 0, 1), edges, n);

            for (int anchor = 1; anchor < Math.Min(n, 6); anchor++)
            {
                AddOptimizedDirectionSolution(solutions, BuildAnchorBasedDirectionLabels(edges, n, anchor, 0), edges, n);
                AddOptimizedDirectionSolution(solutions, BuildAnchorBasedDirectionLabels(edges, n, anchor, 1), edges, n);
            }

            CheckerboardDirectionSolution best = solutions
                .OrderBy(s => s.Cost)
                .ThenBy(s => s.FlipCount)
                .FirstOrDefault();

            if (best == null || best.Labels == null)
                return false;

            // 전체를 180도 돌린 해는 같은 물리 좌표계를 다른 원점으로 표현한 것에 가깝습니다.
            // 기존 Normal 기준에서 바뀌는 pose 수가 적은 쪽으로 정규화합니다.
            if (best.FlipCount > n / 2)
            {
                int[] inverted = best.Labels.Select(v => v == 0 ? 1 : 0).ToArray();
                best.Labels = inverted;
                best.Cost = ComputeDirectionLabelCost(inverted, edges);
                best.FlipCount = CountDirectionFlips(inverted);
            }

            double bestCostDeg = DirectionCostToDegRms(best.Cost, edges.Count);
            double improvementRatio = normalCost > 0.0
                ? (normalCost - best.Cost) / normalCost
                : 0.0;

            Console.WriteLine(
                $"[Checkerboard Direction Cost] Pair={edges.Count}, " +
                $"Normal={normalCostDeg:F3}deg, Best={bestCostDeg:F3}deg, " +
                $"개선율={improvementRatio:P1}, RotZ180 Count={best.FlipCount}/{n}");

            if (best.FlipCount == 0)
            {
                Console.WriteLine("[Checkerboard Direction Keep] 모든 pose가 Normal 방향으로 일관됩니다.");
                return false;
            }

            if (improvementRatio < HandEyeParams.MinCheckerboardDirectionCostImprovementRatio)
            {
                Console.WriteLine(
                    $"[Checkerboard Direction Keep] 상대운동 방향 cost 개선율이 " +
                    $"채택 기준 {HandEyeParams.MinCheckerboardDirectionCostImprovementRatio:P1} 미만입니다.");
                return false;
            }

            List<CameraTargetPose> selectedCameraList = new List<CameraTargetPose>();
            for (int i = 0; i < n; i++)
                selectedCameraList.Add(candidates[i, best.Labels[i]]);

            HandEyeValidationResult originalValidation = null;
            HandEyeValidationResult selectedValidation = null;

            try
            {
                originalValidation = CalibrateAndValidate(
                    original.RobotList,
                    original.CameraList,
                    sequence,
                    method,
                    mode,
                    out double[,] _);

                selectedValidation = CalibrateAndValidate(
                    original.RobotList,
                    selectedCameraList,
                    sequence,
                    method,
                    mode,
                    out double[,] _);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[Checkerboard Direction Validation Skip] {ex.Message}");
                return false;
            }

            double allowedRms = originalValidation.RmsError * HandEyeParams.MaxCheckerboardDirectionValidationWorseningRatio;
            if (selectedValidation.RmsError > allowedRms)
            {
                Console.WriteLine(
                    $"[Checkerboard Direction Reject] 상대운동 방향은 개선됐지만 3D validation이 과도하게 악화됩니다. " +
                    $"RMS {originalValidation.RmsError:F3} -> {selectedValidation.RmsError:F3} mm, " +
                    $"허용={allowedRms:F3} mm");
                return false;
            }

            Console.WriteLine(
                $"[Checkerboard Direction Selected] 3D RMS {originalValidation.RmsError:F3} -> {selectedValidation.RmsError:F3} mm, " +
                $"MAX {originalValidation.MaxError:F3} -> {selectedValidation.MaxError:F3} mm");

            for (int i = 0; i < n; i++)
            {
                if (best.Labels[i] == 1)
                {
                    int displayPoseIndex = GetPoseDisplayIndex(original.RobotList, selectedCameraList, i);
                    Console.WriteLine($"  Pose {displayPoseIndex:D2}: Normal -> RotZ180");
                }
            }

            resolved = new PosePairSet
            {
                RobotList = original.RobotList,
                CameraList = selectedCameraList,
                MatchedByExplicitIndex = original.MatchedByExplicitIndex,
                PairSelectionNote = original.PairSelectionNote + ", CheckerboardDirectionResolved"
            };

            return true;
        }

        private bool IsIntrinsicLocal(EEulerSequence sequence)
        {
            switch (sequence)
            {
                case EEulerSequence.Intrinsic_XYZ:
                case EEulerSequence.Intrinsic_XZY:
                case EEulerSequence.Intrinsic_YXZ:
                case EEulerSequence.Intrinsic_YZX:
                case EEulerSequence.Intrinsic_ZXY:
                case EEulerSequence.Intrinsic_ZYX:
                case EEulerSequence.Intrinsic_ZYZ:
                    return true;
                default:
                    return false;
            }
        }

        private bool IsReverseOrderEulerConventionLocal(EulerMatrixConvention convention)
        {
            switch (convention)
            {
                case EulerMatrixConvention.ReverseOrderActive:
                case EulerMatrixConvention.ReverseOrderPassive:
                case EulerMatrixConvention.NegatedReverseOrderActive:
                case EulerMatrixConvention.NegatedReverseOrderPassive:
                    return true;
                default:
                    return false;
            }
        }

        private bool IsPassiveEulerConventionLocal(EulerMatrixConvention convention)
        {
            switch (convention)
            {
                case EulerMatrixConvention.DefaultPassive:
                case EulerMatrixConvention.ReverseOrderPassive:
                case EulerMatrixConvention.NegatedDefaultPassive:
                case EulerMatrixConvention.NegatedReverseOrderPassive:
                    return true;
                default:
                    return false;
            }
        }

        private bool IsNegatedEulerConventionLocal(EulerMatrixConvention convention)
        {
            switch (convention)
            {
                case EulerMatrixConvention.NegatedDefaultActive:
                case EulerMatrixConvention.NegatedReverseOrderActive:
                case EulerMatrixConvention.NegatedDefaultPassive:
                case EulerMatrixConvention.NegatedReverseOrderPassive:
                    return true;
                default:
                    return false;
            }
        }


        private bool IsRotationVectorSequence(EEulerSequence sequence)
        {
            return sequence == EEulerSequence.RotVecDegree ||
                   sequence == EEulerSequence.RotVecRadian;
        }

        private bool IsZyzSequence(EEulerSequence sequence)
        {
            return sequence == EEulerSequence.Intrinsic_ZYZ ||
                   sequence == EEulerSequence.Extrinsic_ZYZ;
        }

        private bool IsValidNumber(double v)
        {
            return !double.IsNaN(v) && !double.IsInfinity(v);
        }

        private bool IsValidResult(HandEyeSweepResult r)
        {
            return r != null
                && IsValidNumber(r.RmsError)
                && IsValidNumber(r.MaxError)
                && IsValidNumber(r.CamToGripperNorm)
                && IsValidNumber(r.CamToGripperX)
                && IsValidNumber(r.CamToGripperY)
                && IsValidNumber(r.CamToGripperZ);
        }

        public void SetPoseDataByConvention(
            List<RobotTargetPose> robotList,
            List<CameraTargetPose> cameraList,
            EEulerSequence robotRotType,
            bool invertRobotPose,
            bool invertCameraPose,
            HandEyeCalibrationMethod method)
        {
            CLEAR_POSE();

            HandEyeParams.CalibrationMethod = method;

            for (int i = 0; i < robotList.Count; i++)
            {
                AddPoseDataByConvention(
                    robotList[i],
                    cameraList[i],
                    robotRotType,
                    invertRobotPose,
                    invertCameraPose);
            }
        }

        public double EvaluateEyeToHandReprojectionRms(
            double[,] cam2base,
            List<RobotTargetPose> robotPoses,
            List<CameraTargetPose> cameraPoses,
            Mat camMatrix,
            Mat dist,
            EEulerSequence robotRotType)
        {
            if (camMatrix == null || camMatrix.Empty())
                throw new CalibrationDataException("카메라 내부 파라미터(Camera Matrix)가 필요합니다.");

            if (cam2base == null)
                throw new ArgumentNullException(nameof(cam2base));

            MathUtils.ValidateHomogeneousMatrix(cam2base, nameof(cam2base));

            if (robotPoses == null || cameraPoses == null || robotPoses.Count != cameraPoses.Count || robotPoses.Count == 0)
                throw new ArgumentException("Robot/Camera pose list가 비어 있거나 개수가 맞지 않습니다.");

            for (int i = 0; i < cameraPoses.Count; i++)
            {
                if (cameraPoses[i].ImagePoints == null || cameraPoses[i].ObjectPoints == null ||
                    cameraPoses[i].ImagePoints.Length == 0 ||
                    cameraPoses[i].ImagePoints.Length != cameraPoses[i].ObjectPoints.Length)
                {
                    throw new CalibrationDataException($"Pose {i + 1}의 ImagePoints/ObjectPoints가 비어 있거나 개수가 맞지 않습니다.");
                }
            }

            int bestPoseIndex = -1;
            double bestRms = double.MaxValue;

            for (int i = 0; i < cameraPoses.Count; i++)
            {
                double poseRms = cameraPoses[i].ReprojectionRmsPx;
                if (IsFinite(poseRms) && poseRms < bestRms)
                {
                    bestRms = poseRms;
                    bestPoseIndex = i;
                }
            }

            if (bestPoseIndex < 0)
                throw new CalibrationDataException("재투영 RMS를 계산할 수 없습니다. 모든 Camera Pose의 Reprojection RMS가 비정상입니다.");

            double[,] target2Gripper = Optimizer.EstimateTarget2GripperInitialAverage(
                cam2base,
                robotPoses,
                cameraPoses,
                robotRotType);

            double[] x = MathUtils.MatrixToParamsScaled(cam2base);
            double[] y = MathUtils.MatrixToParamsScaled(target2Gripper);
            double[] parameters = x.Concat(y).ToArray();

            double cost = Optimizer.ComputeEyeToHandReprojectionCost12Raw(
                parameters,
                robotPoses,
                cameraPoses,
                camMatrix,
                dist,
                robotRotType);

            if (!IsFinite(cost) || cost >= double.MaxValue / 8.0)
                throw new CalibrationDataException("선형 결과의 재투영 RMS 계산에 실패했습니다.");

            double rms = Math.Sqrt(cost);
            HandEyeParams.LastLinearReprojectionRmsPx = rms;
            HandEyeParams.LastTarget2Gripper = target2Gripper;

            return rms;
        }

        public double[,] RefineCalibrationNonlinearConsistency6DoF(
            double[,] initialMatrix,
            List<RobotTargetPose> robotPoses,
            List<CameraTargetPose> cameraPoses,
            EEulerSequence robotRotType,
            HandEyeMode handEyeMode)
        {
            HandEyeParams.LastNonlinearAccepted = false;

            if (initialMatrix == null)
                throw new ArgumentNullException(nameof(initialMatrix));

            MathUtils.ValidateHomogeneousMatrix(initialMatrix, nameof(initialMatrix));

            if (robotPoses == null || cameraPoses == null || robotPoses.Count != cameraPoses.Count || robotPoses.Count == 0)
                throw new ArgumentException("Robot/Camera pose list가 비어 있거나 개수가 맞지 않습니다.");

            if (!HandEyeParams.ShouldRunNonlinearConsistency)
            {
                Console.WriteLine($"OptimizationMethod={HandEyeParams.Optimization} -> 3D Consistency Nonlinear 생략");
                return initialMatrix;
            }

            HandEyeParams.ValidateBasic();
            HandEyeParams.ValidateOptimizerScale();

            double[] x0 = MathUtils.MatrixToParamsScaled(initialMatrix);
            var initialGuess = DenseVector.OfArray(x0);

            double initialCost = ComputePoseConsistencyCost6(
                x0,
                robotPoses,
                cameraPoses,
                robotRotType,
                handEyeMode);

            double initialRms = Math.Sqrt(initialCost);

            if (!IsFinite(initialRms) || initialCost >= double.MaxValue / 8.0)
            {
                HandEyeParams.LastNonlinearConsistencyRmsMm = double.NaN;
                HandEyeParams.LastNonlinearAccepted = false;
                Console.WriteLine("[Nonlinear 3D Skip] 초기 3D consistency cost가 유효하지 않아 선형/LOO 결과를 유지합니다.");
                return initialMatrix;
            }

            var objectiveFunction = ObjectiveFunction.Value(
                v => ComputePoseConsistencyCost6(
                    v.ToArray(),
                    robotPoses,
                    cameraPoses,
                    robotRotType,
                    handEyeMode));

            var result = NelderMeadSimplex.Minimum(
                objectiveFunction,
                initialGuess,
                HandEyeParams.OptimalSolutionAccuracy,
                5000);

            double finalCost = result.FunctionInfoAtMinimum.Value;
            double finalRms = Math.Sqrt(finalCost);

            if (!IsFinite(finalRms) || finalRms >= initialRms)
            {
                HandEyeParams.LastNonlinearConsistencyRmsMm = initialRms;
                HandEyeParams.LastNonlinearAccepted = false;

                Console.WriteLine(
                    $"[Nonlinear 3D Reject] Initial={initialRms:F3}mm, Candidate={finalRms:F3}mm");

                return initialMatrix;
            }

            double[] opt = result.MinimizingPoint.ToArray();

            using (Mat T_optimized = MathUtils.ParamsToMatrixMatScaled(opt, 0))
            {
                double[,] optimized = MathUtils.MatToArray4x4(T_optimized);
                MathUtils.ValidateHomogeneousMatrix(optimized, nameof(optimized));

                HandEyeParams.LastNonlinearConsistencyRmsMm = finalRms;
                HandEyeParams.LastNonlinearAccepted = true;

                Console.WriteLine(
                    $"[Nonlinear 3D Accept] Initial={initialRms:F3}mm, Final={finalRms:F3}mm");

                return optimized;
            }
        }

        private double ComputePoseConsistencyCost6(
            double[] parameters,
            List<RobotTargetPose> robotPoses,
            List<CameraTargetPose> cameraPoses,
            EEulerSequence robotRotType,
            HandEyeMode handEyeMode)
        {
            if (parameters == null || parameters.Length < 6 ||
                robotPoses == null || cameraPoses == null ||
                robotPoses.Count != cameraPoses.Count ||
                robotPoses.Count == 0)
            {
                return double.MaxValue / 4.0;
            }

            for (int i = 0; i < 6; i++)
            {
                if (!IsFinite(parameters[i]))
                    return double.MaxValue / 4.0;
            }

            List<double[]> points = new List<double[]>();

            using (Mat T_handEye = MathUtils.ParamsToMatrixMatScaled(parameters, 0))
            {
                for (int i = 0; i < robotPoses.Count; i++)
                {
                    try
                    {
                        using (Mat T_target2cam = Optimizer.BuildCameraPoseMat(cameraPoses[i]))
                        {
                            if (handEyeMode == HandEyeMode.EyeToHand)
                            {
                                using (Mat T_base2gripper = Optimizer.BuildEyeToHandBase2GripperPoseMat(robotPoses[i], robotRotType))
                                using (Mat temp = T_base2gripper * T_handEye)
                                using (Mat T_target2gripper = temp * T_target2cam)
                                {
                                    AddTranslationPoint(points, T_target2gripper);
                                }
                            }
                            else
                            {
                                using (Mat T_gripper2base = Optimizer.BuildRobotPoseMat(robotPoses[i], robotRotType))
                                using (Mat temp = T_gripper2base * T_handEye)
                                using (Mat T_target2base = temp * T_target2cam)
                                {
                                    AddTranslationPoint(points, T_target2base);
                                }
                            }
                        }
                    }
                    catch
                    {
                        return double.MaxValue / 4.0;
                    }
                }
            }

            if (points.Count == 0)
                return double.MaxValue / 4.0;

            double meanX = points.Average(p => p[0]);
            double meanY = points.Average(p => p[1]);
            double meanZ = points.Average(p => p[2]);

            double totalErrorSq = 0.0;

            foreach (double[] p in points)
            {
                double dx = p[0] - meanX;
                double dy = p[1] - meanY;
                double dz = p[2] - meanZ;

                if (!IsFinite(dx) || !IsFinite(dy) || !IsFinite(dz))
                    return double.MaxValue / 4.0;

                totalErrorSq += dx * dx + dy * dy + dz * dz;

                if (!IsFinite(totalErrorSq) || totalErrorSq >= double.MaxValue / 16.0)
                    return double.MaxValue / 4.0;
            }

            return totalErrorSq / points.Count;
        }

        private void AddTranslationPoint(List<double[]> points, Mat transform)
        {
            var idx = transform.GetGenericIndexer<double>();
            double x = idx[0, 3];
            double y = idx[1, 3];
            double z = idx[2, 3];

            if (!IsFinite(x) || !IsFinite(y) || !IsFinite(z))
                throw new CalibrationDataException("비정상 translation 값입니다.");

            points.Add(new double[] { x, y, z });
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }

        /// <summary>
        /// 3D 뷰어에서 '카메라를 원점(0,0,0)'으로 고정했을 때, 
        /// 각 캘리브레이션 스텝별 End Effector의 위치와 자세 행렬을 산출합니다.
        /// 이전 이미지와 동일한 방사형(Cyan 선) 검증 UI를 구현할 때 사용합니다.
        /// </summary>
        /// <param name="T_cam2gripper_Array">ExecuteCalibration()에서 도출된 4x4 행렬</param>
        /// <returns>UI에 바인딩할 End Effector 데이터 리스트</returns>
        public List<UIEndEffectorData> EndEffectorPosesForCameraOrigin(double[,] T_cam2gripper_Array)
        {
            List<UIEndEffectorData> uiDataList = new List<UIEndEffectorData>();

            if (R_gripper2base.Count == 0)
                return uiDataList;

            using (Mat T_cam2gripper = MathUtils.ArrayToMat4x4(T_cam2gripper_Array))
            using (Mat T_gripper2cam = T_cam2gripper.Inv()) // T_cam_to_gripper의 역행렬
            {
                for (int i = 0; i < R_gripper2base.Count; i++)
                {
                    // UI의 기준이 Camera(Origin)이므로, 카메라 대비 타겟의 위치(T_cam2target)를 역산할 필요 없이,
                    // 캘리브레이션 품질을 시각화하기 위해 '측정된 타겟 위치'를 기준으로 역산된 End Effector 위치를 구합니다.

                    using (Mat T_target2cam = MathUtils.Build4x4MatFromRt(R_target2cam[i], t_target2cam[i]))
                    using (Mat T_cam2target = T_target2cam.Inv())
                    {
                        // 로직: 카메라 기준 타겟 위치에, 실제 베이스-그리퍼 역변환을 곱해 시각화할 수도 있으나,
                        // 첨부된 이미지는 가장 직관적인 T_cam -> T_gripper 관계를 뿌려주는 형태입니다.
                        // Eye-in-Hand에서 카메라는 그리퍼에 고정되어 있으므로, 
                        // 수집된 각 포즈(T_cam2target) 관점에서 타겟을 고정시켰을 때 카메라와 그리퍼가 어떻게 움직였는지를 
                        // 뒤집어서(카메라 중심) 표현하기 위한 행렬입니다.

                        // 카메라 원점 기준 End Effector의 4x4 행렬 생성
                        // (Eye-in-Hand에서는 사실상 T_gripper2cam가 고정이지만, 각도에 따른 편차를 시각화하기 위해
                        //  타겟 좌표계를 거쳐 역산한 편차 데이터를 그립니다.)
                        using (Mat T_target2base = CalculateTargetToBase(i, T_cam2gripper))
                        using (Mat T_base2target = T_target2base.Inv())
                        using (Mat T_gripper2base = MathUtils.Build4x4MatFromRt(R_gripper2base[i], t_gripper2base[i]))
                        using (Mat T_base2gripper = T_gripper2base.Inv())
                        using (Mat T_temp = T_cam2target * T_base2target) // T_cam -> Target -> Base
                        using (Mat T_cam2gripper_visual = T_temp * T_gripper2base) // Base -> Gripper
                        {
                            double[,] renderMatrix = new double[4, 4];
                            var indexer = T_cam2gripper_visual.GetGenericIndexer<double>();

                            for (int r = 0; r < 4; r++)
                            {
                                for (int c = 0; c < 4; c++)
                                {
                                    renderMatrix[r, c] = indexer[r, c];
                                }
                            }

                            uiDataList.Add(new UIEndEffectorData
                            {
                                PoseIndex = i + 1,
                                X = renderMatrix[0, 3],
                                Y = renderMatrix[1, 3],
                                Z = renderMatrix[2, 3],
                                TransformMatrix4x4 = renderMatrix
                            });
                        }
                    }
                }
            }
            return uiDataList;
        }

        // 내부 헬퍼 메서드 (ValidateCalibration 내부 로직과 동일하게 분리)
        private Mat CalculateTargetToBase(int index, Mat T_cam2gripper)
        {
            Mat T_gripper2base = MathUtils.Build4x4MatFromRt(R_gripper2base[index], t_gripper2base[index]);
            Mat T_target2cam = MathUtils.Build4x4MatFromRt(R_target2cam[index], t_target2cam[index]);

            Mat temp = T_gripper2base * T_cam2gripper;
            Mat T_target2base = temp * T_target2cam;

            temp.Dispose();
            T_gripper2base.Dispose();
            T_target2cam.Dispose();

            return T_target2base;
        }

        public Intrinsic load_cam_intrinsic(string filepath)
        {
            Intrinsic data = new Intrinsic();

            data.fx = 0;
            data.fy = 0;
            data.ppx = 0;
            data.ppy = 0;
            data.coeffs = null;
            data.height = 0;
            data.width = 0;

            return data;
        }

        public double[,] LIST_to_ARRAY(List<UIEndEffectorData> dataList)
        {
            if (dataList == null || dataList.Count == 0)
                return new double[0, 0];

            // PoseIndex, X, Y, Z + 4x4 Matrix 16개 = 총 20개
            double[,] array = new double[dataList.Count, 20];

            for (int i = 0; i < dataList.Count; i++)
            {
                UIEndEffectorData data = dataList[i];

                array[i, 0] = data.PoseIndex;
                array[i, 1] = data.X;
                array[i, 2] = data.Y;
                array[i, 3] = data.Z;

                double[,] m = data.TransformMatrix4x4;

                int col = 4;

                if (m != null && m.GetLength(0) == 4 && m.GetLength(1) == 4)
                {
                    for (int r = 0; r < 4; r++)
                    {
                        for (int c = 0; c < 4; c++)
                        {
                            array[i, col++] = m[r, c];
                        }
                    }
                }
                else
                {
                    // Matrix가 없으면 Identity Matrix로 대체
                    double[,] identity = HandEyeParams.CreateIdentity4x4();

                    for (int r = 0; r < 4; r++)
                    {
                        for (int c = 0; c < 4; c++)
                        {
                            array[i, col++] = identity[r, c];
                        }
                    }
                }
            }

            return array;
        }

    }
}