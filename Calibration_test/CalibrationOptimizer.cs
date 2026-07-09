using MathNet.Numerics.Optimization;
using MathNet.Numerics.LinearAlgebra.Double;
using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Calibration_test
{
    /// <summary>
    /// Hand-Eye 초기해 이후의 12-DoF 비선형 최적화, pose 행렬 생성, 재투영 cost 계산을 담당합니다.
    /// </summary>
    public class CalibrationOptimizer
    {
        private HandEyeParams HandEyeParams;
        private MathUtils MathUtils;

        public HandEyeParams Params
        {
            get { return HandEyeParams; }
        }

        public MathUtils Utils
        {
            get { return MathUtils; }
        }

        /// <summary>
        /// 기본 파라미터와 MathUtils 인스턴스로 최적화 서비스를 생성합니다.
        /// </summary>
        public CalibrationOptimizer()
            : this(new HandEyeParams(), null)
        {
        }

        /// <summary>
        /// 지정한 파라미터와 수학 유틸리티 인스턴스를 공유하도록 최적화 서비스를 생성합니다.
        /// </summary>
        /// <param name="parameters">공유할 Hand-Eye 파라미터입니다.</param>
        /// <param name="mathUtils">공유할 MathUtils입니다. null이면 새 인스턴스를 생성합니다.</param>
        public CalibrationOptimizer(HandEyeParams parameters, MathUtils mathUtils = null)
        {
            SetDependencies(parameters, mathUtils);
        }

        /// <summary>
        /// 최적화 서비스가 참조하는 파라미터와 수학 유틸리티 인스턴스를 교체합니다.
        /// </summary>
        /// <param name="parameters">공유할 Hand-Eye 파라미터입니다.</param>
        /// <param name="mathUtils">공유할 MathUtils입니다. null이면 새 인스턴스를 생성합니다.</param>
        public void SetDependencies(HandEyeParams parameters, MathUtils mathUtils = null)
        {
            HandEyeParams = parameters ?? throw new ArgumentNullException(nameof(parameters));
            MathUtils = mathUtils ?? new MathUtils(HandEyeParams);
            MathUtils.SetParams(HandEyeParams);
        }

        /// <summary>
        /// 설정에 따라 전용 12-DoF 동시 재투영 최적화를 실행하거나 closed-form 초기해를 유지합니다.
        /// </summary>
        /// <returns>채택된 Camera-to-Base 4x4 행렬입니다.</returns>
        public double[,] RefineCalibrationNonlinear12DoF(double[,] initialCam2Base, List<RobotTargetPose> robotPoses,
   List<CameraTargetPose> cameraPoses, Mat camMatrix, Mat dist, EEulerSequence robotRotType)
        {
            if (initialCam2Base == null)
                throw new ArgumentNullException(nameof(initialCam2Base));

            MathUtils.ValidateHomogeneousMatrix(initialCam2Base, nameof(initialCam2Base));

            if (!HandEyeParams.UseDedicatedSimultaneousReprojection12DoF)
            {
                Console.WriteLine("UseDedicatedSimultaneousReprojection12DoF=false -> 전용 12-DoF를 생략하고 closed-form 결과를 유지합니다.");
                HandEyeParams.LastNonlinearAccepted = false;
                return initialCam2Base;
            }

            // final18: OpenCV closed-form 결과를 단순 보정하는 방식이 아니라,
            // Camera2Base(X)와 Target2Gripper(Y)를 동시에 12-DoF로 재투영 오차 최소화합니다.
            // final16 대비 seed 범위를 넓히고 Huber robust loss를 추가했습니다.
            return RefineCalibrationSimultaneousReprojection12DoF(
                initialCam2Base,
                robotPoses,
                cameraPoses,
                camMatrix,
                dist,
                robotRotType);
        }

        /// <summary>
        /// Camera2Base(X)와 Target2Gripper(Y)를 동시에 12-DoF로 최적화해 2D 재투영 RMS를 최소화합니다.
        /// </summary>
        /// <returns>채택 조건을 만족하면 최적화된 Camera-to-Base 행렬, 아니면 초기 행렬입니다.</returns>
        public double[,] RefineCalibrationSimultaneousReprojection12DoF(double[,] initialCam2Base,
            List<RobotTargetPose> robotPoses, List<CameraTargetPose> cameraPoses,
            Mat camMatrix, Mat dist, EEulerSequence robotRotType)
        {
            HandEyeParams.LastNonlinearAccepted = false;

            if (camMatrix == null || camMatrix.Empty())
                throw new Exception("카메라 내부 파라미터(Camera Matrix)가 필요합니다.");

            if (initialCam2Base == null)
                throw new ArgumentNullException(nameof(initialCam2Base));

            MathUtils.ValidateHomogeneousMatrix(initialCam2Base, nameof(initialCam2Base));

            if (robotPoses == null || cameraPoses == null || robotPoses.Count != cameraPoses.Count || robotPoses.Count == 0)
                throw new ArgumentException("Robot/Camera pose list가 비어 있거나 개수가 맞지 않습니다.");

            if (!HandEyeParams.ShouldRunNonlinearReprojection)
            {
                Console.WriteLine($"OptimizationMethod={HandEyeParams.Optimization} -> Simultaneous 12-DoF Reprojection 생략");
                return initialCam2Base;
            }

            HandEyeParams.ValidateBasic();
            HandEyeParams.ValidateOptimizerScale();

            ValidateReprojectionPoseData(robotPoses, cameraPoses);

            Console.WriteLine("===== SIMULTANEOUS 12-DOF REPROJECTION OPTIMIZATION START =====");
            Console.WriteLine("Model:  T_target2cam_pred = inv(T_cam2base) * T_gripper2base_i * T_target2gripper");
            Console.WriteLine("Solve:  X=T_cam2base(6DoF), Y=T_target2gripper(6DoF), total 12DoF");
            Console.WriteLine($"RobotPoseDirection = {HandEyeParams.EyeToHandRobotPoseInputDirection}");
            Console.WriteLine(
                $"GeneratedSeeds={HandEyeParams.Simultaneous12DofMaxGeneratedSeedCount}, " +
                $"OptimizedSeeds={HandEyeParams.Simultaneous12DofMaxOptimizedSeedCount}, " +
                $"PatternIter={HandEyeParams.Simultaneous12DofMaxPatternIterations}, " +
                $"AcceptImprovedAboveLimit={HandEyeParams.AcceptImprovedReprojectionEvenAboveLimit}, " +
                $"RobustLoss={HandEyeParams.Simultaneous12DofUseRobustLoss}, HuberDelta={HandEyeParams.Simultaneous12DofHuberDeltaPx:F3}px");

            List<Simultaneous12DofSeed> seedCandidates = CreateSimultaneous12DofSeeds(
                initialCam2Base,
                robotPoses,
                cameraPoses,
                robotRotType,
                HandEyeParams.Simultaneous12DofMaxGeneratedSeedCount);

            if (seedCandidates.Count == 0)
                throw new Exception("Simultaneous 12-DoF 초기 seed를 만들 수 없습니다.");

            Func<double[], double> optimizeCostFunc = p => ComputeEyeToHandReprojectionCost12(
                p,
                robotPoses,
                cameraPoses,
                camMatrix,
                dist,
                robotRotType);

            Func<double[], double> trueCostFunc = p => ComputeEyeToHandReprojectionCost12Raw(
                p,
                robotPoses,
                cameraPoses,
                camMatrix,
                dist,
                robotRotType);

            List<Simultaneous12DofSeedScore> scoredSeeds = new List<Simultaneous12DofSeedScore>();

            foreach (Simultaneous12DofSeed seed in seedCandidates)
            {
                double optimizeCost = optimizeCostFunc(seed.Parameters);
                double trueCost = trueCostFunc(seed.Parameters);

                if (!IsFinite(optimizeCost) || optimizeCost >= double.MaxValue / 8.0 ||
                    !IsFinite(trueCost) || trueCost >= double.MaxValue / 8.0)
                {
                    Console.WriteLine($"[Sim12 Seed] {seed.Name} -> invalid");
                    continue;
                }

                scoredSeeds.Add(new Simultaneous12DofSeedScore
                {
                    Seed = seed,
                    OptimizeCost = optimizeCost,
                    TrueCost = trueCost
                });
            }

            if (scoredSeeds.Count == 0)
            {
                HandEyeParams.LastNonlinearReprojectionRmsPx = double.NaN;
                HandEyeParams.LastTarget2Gripper = null;
                Console.WriteLine("[Sim12 Skip] 모든 초기 seed의 재투영 cost가 유효하지 않습니다.");
                return initialCam2Base;
            }

            scoredSeeds = scoredSeeds
                .OrderBy(s => s.OptimizeCost)
                .ThenBy(s => s.TrueCost)
                .ToList();

            int optimizeSeedCount = Math.Min(
                Math.Max(1, HandEyeParams.Simultaneous12DofMaxOptimizedSeedCount),
                scoredSeeds.Count);

            List<Simultaneous12DofSeedScore> selectedSeeds = scoredSeeds
                .Take(optimizeSeedCount)
                .ToList();

            Console.WriteLine(
                $"[Sim12 Seed Screening] Generated={seedCandidates.Count}, Valid={scoredSeeds.Count}, Optimizing={selectedSeeds.Count}");

            for (int i = 0; i < Math.Min(scoredSeeds.Count, 10); i++)
            {
                Simultaneous12DofSeedScore s = scoredSeeds[i];
                Console.WriteLine(
                    $"[Sim12 Seed Rank {i + 1:D2}] {s.Seed.Name} -> " +
                    $"RobustRMS={FormatFinite(SafeSqrtCost(s.OptimizeCost))} px, " +
                    $"TrueRMS={FormatFinite(SafeSqrtCost(s.TrueCost))} px");
            }

            Simultaneous12DofSeedScore bestInitialTrueSeed = scoredSeeds
                .OrderBy(s => s.TrueCost)
                .First();

            double bestInitialTrueCost = bestInitialTrueSeed.TrueCost;
            double bestInitialTrueRms = SafeSqrtCost(bestInitialTrueCost);

            Console.WriteLine(
                $"[Sim12 Initial Best] Seed={bestInitialTrueSeed.Seed.Name}, TrueRMS={bestInitialTrueRms:F3} px, " +
                $"RobustRMS={FormatFinite(SafeSqrtCost(bestInitialTrueSeed.OptimizeCost))} px");

            if (!IsFinite(bestInitialTrueRms) ||
                bestInitialTrueRms > HandEyeParams.MaxStartReprojectionRmsForNonlinearPx)
            {
                HandEyeParams.LastNonlinearReprojectionRmsPx = bestInitialTrueRms;
                HandEyeParams.LastTarget2Gripper = null;
                HandEyeParams.LastNonlinearAccepted = false;

                Console.WriteLine(
                    $"[Sim12 Skip] 초기/seed 재투영 TrueRMS={FormatFinite(bestInitialTrueRms)}px가 " +
                    $"시작 허용 기준 {HandEyeParams.MaxStartReprojectionRmsForNonlinearPx:F3}px보다 큽니다. " +
                    "좌표계 방향, pose pair, TCP, checkerboard frame을 먼저 맞춘 뒤 최적화를 실행해야 합니다.");
                return initialCam2Base;
            }

            double bestFinalTrueCost = double.MaxValue;
            double bestFinalOptimizeCost = double.MaxValue;
            double[] bestFinalParameters = null;
            string bestFinalSeedName = string.Empty;
            int totalPatternEvaluations = 0;

            foreach (Simultaneous12DofSeedScore seedScore in selectedSeeds)
            {
                Simultaneous12DofSeed seed = seedScore.Seed;

                double patternCost;
                int patternEvaluations;
                double[] patternParameters = OptimizeByPatternSearch12D(
                    seed.Parameters,
                    optimizeCostFunc,
                    out patternCost,
                    out patternEvaluations);

                totalPatternEvaluations += patternEvaluations;

                double[] candidateParameters = patternParameters;
                double candidateOptimizeCost = patternCost;

                try
                {
                    var objectiveFunction = ObjectiveFunction.Value(v => optimizeCostFunc(v.ToArray()));
                    var result = NelderMeadSimplex.Minimum(
                        objectiveFunction,
                        DenseVector.OfArray(patternParameters),
                        HandEyeParams.OptimalSolutionAccuracy,
                        8000);

                    double nmCost = result.FunctionInfoAtMinimum.Value;
                    if (IsFinite(nmCost) && nmCost < candidateOptimizeCost)
                    {
                        candidateOptimizeCost = nmCost;
                        candidateParameters = result.MinimizingPoint.ToArray();
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[Sim12 NelderMead Warning] Seed={seed.Name}, {ex.Message}. PatternSearch 결과를 사용합니다.");
                }

                double candidateTrueCost = trueCostFunc(candidateParameters);

                double startRobustRms = SafeSqrtCost(seedScore.OptimizeCost);
                double startTrueRms = SafeSqrtCost(seedScore.TrueCost);
                double patternRobustRms = SafeSqrtCost(patternCost);
                double finalRobustRms = SafeSqrtCost(candidateOptimizeCost);
                double finalTrueRms = SafeSqrtCost(candidateTrueCost);

                Console.WriteLine(
                    $"[Sim12 Optimize] Seed={seed.Name}, " +
                    $"StartTrue={FormatFinite(startTrueRms)}px, StartRobust={FormatFinite(startRobustRms)}px, " +
                    $"PatternRobust={FormatFinite(patternRobustRms)}px, " +
                    $"FinalTrue={FormatFinite(finalTrueRms)}px, FinalRobust={FormatFinite(finalRobustRms)}px, Eval={patternEvaluations}");

                // Robust loss는 탐색 안정화 용도로만 사용하고,
                // 실제 채택/최종 선택은 순수 L2 재투영 RMS(TrueRMS)를 기준으로 합니다.
                if (IsFinite(candidateTrueCost) && candidateTrueCost < bestFinalTrueCost)
                {
                    bestFinalTrueCost = candidateTrueCost;
                    bestFinalOptimizeCost = candidateOptimizeCost;
                    bestFinalParameters = candidateParameters;
                    bestFinalSeedName = seed.Name;

                    if (finalTrueRms <= HandEyeParams.MaxAcceptReprojectionRmsPx)
                    {
                        Console.WriteLine(
                            $"[Sim12 Early Stop] Seed={seed.Name}에서 TrueRMS={finalTrueRms:F3}px가 " +
                            $"허용 기준 {HandEyeParams.MaxAcceptReprojectionRmsPx:F3}px 이하입니다.");
                        break;
                    }
                }
            }

            if (bestFinalParameters == null ||
                !IsFinite(bestFinalTrueCost) ||
                bestFinalTrueCost >= double.MaxValue / 8.0)
            {
                HandEyeParams.LastNonlinearReprojectionRmsPx = bestInitialTrueRms;
                HandEyeParams.LastTarget2Gripper = null;
                HandEyeParams.LastNonlinearAccepted = false;
                Console.WriteLine("[Sim12 Reject] 유효한 최적화 결과가 없습니다. Closed-form 결과를 유지합니다.");
                return initialCam2Base;
            }

            double bestFinalTrueRms = SafeSqrtCost(bestFinalTrueCost);
            double bestFinalRobustRms = SafeSqrtCost(bestFinalOptimizeCost);
            double improvementRatio = bestInitialTrueRms > 1e-12
                ? (bestInitialTrueRms - bestFinalTrueRms) / bestInitialTrueRms
                : 0.0;

            bool improvedEnough = bestFinalTrueRms < bestInitialTrueRms &&
                improvementRatio >= HandEyeParams.MinSimultaneous12DofImprovementRatio;
            bool underAcceptLimit = bestFinalTrueRms <= HandEyeParams.MaxAcceptReprojectionRmsPx;
            bool accept = underAcceptLimit ||
                (improvedEnough && HandEyeParams.AcceptImprovedReprojectionEvenAboveLimit);

            Console.WriteLine(
                $"[Sim12 Best] Seed={bestFinalSeedName}, InitialTrue={bestInitialTrueRms:F3}px, " +
                $"FinalTrue={bestFinalTrueRms:F3}px, FinalRobust={FormatFinite(bestFinalRobustRms)}px, " +
                $"Improvement={improvementRatio * 100.0:F2}%, Limit={HandEyeParams.MaxAcceptReprojectionRmsPx:F3}px, " +
                $"TotalPatternEval={totalPatternEvaluations}");

            if (!accept)
            {
                double[,] rejectedTarget = null;
                try
                {
                    using (Mat T_rejectedTarget = MathUtils.ParamsToMatrixMatScaled(bestFinalParameters, 6))
                    {
                        rejectedTarget = MathUtils.MatToArray4x4(T_rejectedTarget);
                    }
                }
                catch
                {
                    rejectedTarget = null;
                }

                HandEyeParams.LastNonlinearReprojectionRmsPx = bestInitialTrueRms;
                HandEyeParams.LastTarget2Gripper = rejectedTarget;
                HandEyeParams.LastNonlinearAccepted = false;

                Console.WriteLine("[Sim12 Reject] TrueRMS 기준으로 허용 조건을 만족하지 않아 Closed-form 결과를 유지합니다.");
                return initialCam2Base;
            }

            using (Mat T_cam2base = MathUtils.ParamsToMatrixMatScaled(bestFinalParameters, 0))
            using (Mat T_target2gripper = MathUtils.ParamsToMatrixMatScaled(bestFinalParameters, 6))
            {
                double[,] optimizedCam2Base = MathUtils.MatToArray4x4(T_cam2base);
                double[,] optimizedTarget2Gripper = MathUtils.MatToArray4x4(T_target2gripper);

                MathUtils.ValidateHomogeneousMatrix(optimizedCam2Base, nameof(optimizedCam2Base));
                MathUtils.ValidateHomogeneousMatrix(optimizedTarget2Gripper, nameof(optimizedTarget2Gripper));

                HandEyeParams.LastTarget2Gripper = optimizedTarget2Gripper;
                HandEyeParams.LastNonlinearReprojectionRmsPx = bestFinalTrueRms;
                HandEyeParams.LastNonlinearAccepted = true;

                if (!underAcceptLimit)
                {
                    Console.WriteLine(
                        $"[Sim12 Accept Warning] TrueRMS={bestFinalTrueRms:F3}px가 허용 기준 {HandEyeParams.MaxAcceptReprojectionRmsPx:F3}px보다 크지만, " +
                        "전용 12-DoF 최적화가 기존보다 개선되어 진단 목적으로 채택합니다.");
                }
                else
                {
                    Console.WriteLine($"[Sim12 Accept] TrueRMS={bestFinalTrueRms:F3}px");
                }

                Console.WriteLine(
                    $"[Sim12 Output] T_cam2base=({optimizedCam2Base[0, 3]:F3}, {optimizedCam2Base[1, 3]:F3}, {optimizedCam2Base[2, 3]:F3}) mm, " +
                    $"T_target2gripper=({optimizedTarget2Gripper[0, 3]:F3}, {optimizedTarget2Gripper[1, 3]:F3}, {optimizedTarget2Gripper[2, 3]:F3}) mm");

                return optimizedCam2Base;
            }
        }

        private double[] OptimizeByPatternSearch12D(
         double[] start,
         Func<double[], double> costFunc,
         out double bestCost,
         out int evaluations)
        {
            double[] best = (double[])start.Clone();
            bestCost = costFunc(best);
            evaluations = 1;

            double[] step = new double[12];
            for (int i = 0; i < 12; i++)
            {
                bool rotationParameter = (i % 6) < 3;
                step[i] = rotationParameter
                    ? HandEyeParams.Simultaneous12DofInitialRotationStepRad
                    : HandEyeParams.Simultaneous12DofInitialTranslationStepScaled;
            }

            int maxIterations = HandEyeParams.Simultaneous12DofMaxPatternIterations;
            double minStep = HandEyeParams.Simultaneous12DofMinStep;

            for (int iter = 0; iter < maxIterations; iter++)
            {
                bool improved = false;

                for (int dim = 0; dim < 12; dim++)
                {
                    double original = best[dim];

                    best[dim] = original + step[dim];
                    double plusCost = costFunc(best);
                    evaluations++;

                    if (IsFinite(plusCost) && plusCost < bestCost)
                    {
                        bestCost = plusCost;
                        improved = true;
                        continue;
                    }

                    best[dim] = original - step[dim];
                    double minusCost = costFunc(best);
                    evaluations++;

                    if (IsFinite(minusCost) && minusCost < bestCost)
                    {
                        bestCost = minusCost;
                        improved = true;
                        continue;
                    }

                    best[dim] = original;
                }

                if (!improved)
                {
                    double maxStep = 0.0;
                    for (int i = 0; i < step.Length; i++)
                    {
                        step[i] *= 0.5;
                        if (step[i] > maxStep)
                            maxStep = step[i];
                    }

                    if (maxStep < minStep)
                        break;
                }
            }

            return best;
        }
        /// <summary>
        /// robust loss 없이 순수 L2 2D 재투영 cost를 계산합니다.
        /// </summary>
        /// <returns>평균 squared pixel error입니다. 입력이 유효하지 않으면 큰 sentinel 값을 반환합니다.</returns>
        public double ComputeEyeToHandReprojectionCost12Raw(
           double[] parameters,
           List<RobotTargetPose> robotPoses,
           List<CameraTargetPose> cameraPoses,
           Mat camMatrix,
           Mat dist,
           EEulerSequence robotRotType)
        {
            return ComputeEyeToHandReprojectionCost12Internal(
                parameters,
                robotPoses,
                cameraPoses,
                camMatrix,
                dist,
                robotRotType,
                useRobustLoss: false);
        }

        private double ComputeEyeToHandReprojectionCost12Internal(
            double[] parameters,
            List<RobotTargetPose> robotPoses,
            List<CameraTargetPose> cameraPoses,
            Mat camMatrix,
            Mat dist,
            EEulerSequence robotRotType,
            bool useRobustLoss)
        {
            if (parameters == null || parameters.Length < 12 ||
                robotPoses == null || cameraPoses == null ||
                robotPoses.Count != cameraPoses.Count)
            {
                return double.MaxValue / 4.0;
            }

            for (int i = 0; i < 12; i++)
            {
                if (!IsFinite(parameters[i]))
                    return double.MaxValue / 4.0;
            }

            double totalErrorSq = 0.0;
            int totalPoints = 0;

            using (Mat T_cam2base = MathUtils.ParamsToMatrixMatScaled(parameters, 0))
            using (Mat T_target2gripper = MathUtils.ParamsToMatrixMatScaled(parameters, 6))
            using (Mat T_base2cam = T_cam2base.Inv()) // inv(X)
            {
                for (int i = 0; i < robotPoses.Count; i++)
                {
                    try
                    {
                        using (Mat T_gripper2base = BuildEyeToHandGripper2BasePoseMat(robotPoses[i], robotRotType))

                        // T_target2cam_pred = inv(X) * G_i * Y
                        using (Mat temp = T_base2cam * T_gripper2base)
                        using (Mat T_target2cam_pred = temp * T_target2gripper)

                        // 메모리 연속성(.Clone()) 보장하여 에러 방지
                        using (Mat R = T_target2cam_pred.SubMat(0, 3, 0, 3).Clone())
                        using (Mat t = T_target2cam_pred.SubMat(0, 3, 3, 4).Clone())
                        using (Mat rvec = new Mat())
                        using (var objInput = InputArray.Create(cameraPoses[i].ObjectPoints))
                        using (Mat projectedMat = new Mat())
                        {
                            Cv2.Rodrigues(R, rvec);

                            Cv2.ProjectPoints(objInput, rvec, t, camMatrix, dist, projectedMat);

                            Point2f[] projected = ReadPoint2fArray(projectedMat, cameraPoses[i].ObjectPoints.Length);
                            Point2f[] observed = cameraPoses[i].ImagePoints;

                            if (projected == null || observed == null || projected.Length != observed.Length)
                                return double.MaxValue / 4.0;

                            for (int j = 0; j < observed.Length; j++)
                            {
                                double dx = observed[j].X - projected[j].X;
                                double dy = observed[j].Y - projected[j].Y;

                                if (!IsFinite(dx) || !IsFinite(dy))
                                    return double.MaxValue / 4.0;

                                double pointErrorSq = dx * dx + dy * dy;
                                double accumulatedError = useRobustLoss
                                    ? ApplyReprojectionLoss(pointErrorSq)
                                    : pointErrorSq;

                                totalErrorSq += accumulatedError;

                                if (!IsFinite(totalErrorSq) || totalErrorSq >= double.MaxValue / 16.0)
                                    return double.MaxValue / 4.0;

                                totalPoints++;
                            }
                        }
                    }
                    catch
                    {
                        return double.MaxValue / 4.0;
                    }
                }
            }

            return totalPoints > 0 ? (totalErrorSq / totalPoints) : double.MaxValue;
        }

        private double ApplyReprojectionLoss(double pointErrorSq)
        {
            if (!IsFinite(pointErrorSq) || pointErrorSq < 0.0)
                return double.MaxValue / 4.0;

            if (!HandEyeParams.Simultaneous12DofUseRobustLoss)
                return pointErrorSq;

            double delta = HandEyeParams.Simultaneous12DofHuberDeltaPx;
            if (delta <= 0.0 || double.IsNaN(delta) || double.IsInfinity(delta))
                return pointErrorSq;

            double error = Math.Sqrt(pointErrorSq);
            if (error <= delta)
                return pointErrorSq;

            // Huber loss rho(e) = 2*delta*e - delta^2 를 squared-error와 같은 스케일로 반환합니다.
            return 2.0 * delta * error - delta * delta;
        }

        /// <summary>
        /// OpenCV ProjectPoints 결과 Mat을 Point2f 배열로 변환합니다.
        /// </summary>
        /// <param name="mat">Point2f가 들어 있는 OpenCV Mat입니다.</param>
        /// <param name="count">읽을 포인트 개수입니다.</param>
        /// <returns>변환된 Point2f 배열입니다. 실패 시 빈 배열입니다.</returns>
        public static Point2f[] ReadPoint2fArray(Mat mat, int count)
        {
            if (count <= 0)
                return new Point2f[0];

            Point2f[] points = new Point2f[count];

            if (mat == null || mat.Empty())
            {
                Console.WriteLine("Point2f Mat is null or empty.");
                return new Point2f[0];
            }

            long total = mat.Total();
            if (total < count)
            {
                Console.WriteLine($"Point2f Mat size mismatch. Total={total}, Required={count}");
                return new Point2f[0];
            }

            try
            {
                for (int i = 0; i < count; i++)
                {
                    if (mat.Rows == count && mat.Cols == 1)
                    {
                        points[i] = mat.Get<Point2f>(i, 0);
                    }
                    else if (mat.Rows == 1 && mat.Cols == count)
                    {
                        points[i] = mat.Get<Point2f>(0, i);
                    }
                    else
                    {
                        // fallback
                        points[i] = mat.Get<Point2f>(i);
                    }
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Point2f 배열 변환 실패: {ex.Message}");
                return new Point2f[0];
            }

            return points;
        }

        private Mat BuildEyeToHandGripper2BasePoseMat(RobotTargetPose pose, EEulerSequence rotType)
        {
            using (Mat rawRobotPose = BuildRobotPoseMat(pose, rotType))
            {
                if (HandEyeParams.EyeToHandRobotPoseInputDirection == RobotPoseTransformDirection.GripperToBase)
                    return rawRobotPose.Clone();

                return rawRobotPose.Inv();
            }
        }

        /// <summary>
        /// RobotTargetPose의 병진값과 회전 정의를 4x4 robot pose Mat으로 변환합니다.
        /// </summary>
        /// <returns>CV_64FC1 타입의 4x4 동차변환 Mat입니다. 호출자가 Dispose해야 합니다.</returns>
        public Mat BuildRobotPoseMat(RobotTargetPose pose, EEulerSequence rotType)
        {
            ValidateRobotPose(pose, nameof(pose));

            Mat R = null;
            Mat t = null;

            try
            {
                R = RobotRotationToMatrix(
       pose.robotRx,
       pose.robotRy,
       pose.robotRz,
       rotType);

                t = new Mat(3, 1, MatType.CV_64FC1, new double[]
                {
        pose.robotTx,
        pose.robotTy,
        pose.robotTz
                });

                ApplyHandednessTransformation(ref R, ref t, HandEyeParams.Handedness);

                return MathUtils.Build4x4MatFromRt(R, t);
            }
            finally
            {
                if (R != null)
                    R.Dispose();
                if (t != null)
                    t.Dispose();
            }
        }

        /// <summary>
        /// Left-handed 로봇 좌표계를 사용할 때 Y축 반사 행렬로 회전과 병진을 보정합니다.
        /// </summary>
        public void ApplyHandednessTransformation(ref Mat R, ref Mat t, EndEffectorHandedness handedness)
        {
            if (R == null || R.Empty() || R.Rows != 3 || R.Cols != 3)
                throw new ArgumentException("R은 3x3 회전행렬이어야 합니다.", nameof(R));

            if (t == null || t.Empty() || t.Rows != 3 || t.Cols != 1)
                throw new ArgumentException("t는 3x1 병진벡터여야 합니다.", nameof(t));

            if (handedness == EndEffectorHandedness.Left)
            {
                // Y축 대칭 변환 행렬 S = diag(1, -1, 1)
                using (Mat S = Mat.Eye(3, 3, MatType.CV_64FC1))
                {
                    S.Set<double>(1, 1, -1.0);

                    // R' = S * R * S
                    Mat newR = S * R * S;
                    // t' = S * t
                    Mat newT = S * t;

                    R.Dispose();
                    t.Dispose();

                    R = newR;
                    t = newT;
                }
            }
        }

        /// <summary>
        /// 로봇 회전 입력(Euler 또는 rotation vector)을 현재 파라미터 convention에 맞는 3x3 회전행렬로 변환합니다.
        /// </summary>
        /// <returns>CV_64FC1 타입의 3x3 회전행렬 Mat입니다. 호출자가 Dispose해야 합니다.</returns>
        public Mat RobotRotationToMatrix(double rx, double ry, double rz, EEulerSequence type)
        {
            ValidateFinitePoseValue(rx, nameof(rx));
            ValidateFinitePoseValue(ry, nameof(ry));
            ValidateFinitePoseValue(rz, nameof(rz));

            // RotVec는 벡터 성분 자체가 축 성분이므로 CSV 열 순서를 바꾸면 의미가 달라질 수 있습니다.
            // Euler sequence 후보 탐색에서만 RobotAngleInputOrder를 적용합니다.
            switch (type)
            {
                case EEulerSequence.RotVecDegree:
                    return MathUtils.RotVecToMatrix(rx, ry, rz, true);

                case EEulerSequence.RotVecRadian:
                    return MathUtils.RotVecToMatrix(rx, ry, rz, false);

                default:
                    MapRobotEulerAngles(rx, ry, rz, HandEyeParams.RobotAngleInput,
                        out double a1, out double a2, out double a3);
                    return EulerToMatrix(a1, a2, a3, type, HandEyeParams.RobotAngleUnit, HandEyeParams.RobotEulerMatrixConvention);
            }
        }
        private void MapRobotEulerAngles(double rx, double ry, double rz,
            RobotAngleInputOrder order, out double a1, out double a2, out double a3)
        {
            switch (order)
            {
                case RobotAngleInputOrder.RxRyRz:
                    a1 = rx; a2 = ry; a3 = rz;
                    return;
                case RobotAngleInputOrder.RxRzRy:
                    a1 = rx; a2 = rz; a3 = ry;
                    return;
                case RobotAngleInputOrder.RyRxRz:
                    a1 = ry; a2 = rx; a3 = rz;
                    return;
                case RobotAngleInputOrder.RyRzRx:
                    a1 = ry; a2 = rz; a3 = rx;
                    return;
                case RobotAngleInputOrder.RzRxRy:
                    a1 = rz; a2 = rx; a3 = ry;
                    return;
                case RobotAngleInputOrder.RzRyRx:
                    a1 = rz; a2 = ry; a3 = rx;
                    return;
                default:
                    throw new ArgumentOutOfRangeException(nameof(order), order, "지원하지 않는 RobotAngleInputOrder입니다.");
            }
        }

        private Mat EulerToMatrix(double rx, double ry, double rz, EEulerSequence eeuler, EAngleUnit angle)
        {
            return EulerToMatrix(rx, ry, rz, eeuler, angle, HandEyeParams.RobotEulerMatrixConvention);
        }
        private Mat EulerToMatrix(double rx, double ry, double rz,
            EEulerSequence eeuler, EAngleUnit angle, EulerMatrixConvention convention)
        {
            double r1 = MathUtils.ToRadians(rx, angle);
            double r2 = MathUtils.ToRadians(ry, angle);
            double r3 = MathUtils.ToRadians(rz, angle);

            if (IsNegatedEulerConvention(convention))
            {
                r1 = -r1;
                r2 = -r2;
                r3 = -r3;
            }

            char[] axes = GetEulerAxes(eeuler);

            using (Mat R1 = MathUtils.AxisRotation(axes[0], r1))
            using (Mat R2 = MathUtils.AxisRotation(axes[1], r2))
            using (Mat R3 = MathUtils.AxisRotation(axes[2], r3))
            {
                bool useReverseOrder = IsReverseOrderEulerConvention(convention);
                bool useIntrinsicOrder = IsIntrinsic(eeuler);

                // DefaultActive는 기존 코드와 동일합니다.
                // Intrinsic: R1*R2*R3, Extrinsic: R3*R2*R1
                // ReverseOrder 계열은 blackbox 상용 SW가 반대 곱셈 순서를 쓰는 경우를 실험적으로 찾기 위한 후보입니다.
                if (useReverseOrder)
                    useIntrinsicOrder = !useIntrinsicOrder;

                Mat activeR = useIntrinsicOrder
                    ? MathUtils.Multiply3(R1, R2, R3)
                    : MathUtils.Multiply3(R3, R2, R1);

                if (IsPassiveEulerConvention(convention))
                {
                    using (activeR)
                    {
                        Mat passiveR = new Mat();
                        Cv2.Transpose(activeR, passiveR);
                        return passiveR;
                    }
                }

                return activeR;
            }
        }

        private void ValidateReprojectionPoseData(
            List<RobotTargetPose> robotPoses,
            List<CameraTargetPose> cameraPoses)
        {
            if (robotPoses == null || cameraPoses == null || robotPoses.Count != cameraPoses.Count || robotPoses.Count == 0)
                throw new ArgumentException("Robot/Camera pose list가 비어 있거나 개수가 맞지 않습니다.");

            for (int i = 0; i < cameraPoses.Count; i++)
            {
                if (robotPoses[i] == null || cameraPoses[i] == null)
                    throw new Exception($"Pose {i + 1}가 null입니다.");

                if (cameraPoses[i].ImagePoints == null || cameraPoses[i].ObjectPoints == null ||
                    cameraPoses[i].ImagePoints.Length == 0 ||
                    cameraPoses[i].ImagePoints.Length != cameraPoses[i].ObjectPoints.Length)
                {
                    throw new Exception($"Pose {i + 1}의 ImagePoints/ObjectPoints가 비어 있거나 개수가 맞지 않습니다.");
                }
            }
        }

        private List<Simultaneous12DofSeed> CreateSimultaneous12DofSeeds(
            double[,] initialCam2Base,
            List<RobotTargetPose> robotPoses,
            List<CameraTargetPose> cameraPoses,
            EEulerSequence robotRotType,
            int maxSeedCount)
        {
            List<Simultaneous12DofSeed> seeds = new List<Simultaneous12DofSeed>();

            double[] x0 = MathUtils.MatrixToParamsScaled(initialCam2Base);
            double[,] avgTarget2Gripper = EstimateTarget2GripperInitialAverage(
                initialCam2Base,
                robotPoses,
                cameraPoses,
                robotRotType);
            double[] yAvg = MathUtils.MatrixToParamsScaled(avgTarget2Gripper);

            AddSimultaneousSeed(seeds, "ClosedFormX + MedianTargetY", x0.Concat(yAvg).ToArray(), maxSeedCount);

            var rankedPoseIndexes = cameraPoses
                .Select((pose, idx) => new
                {
                    Index = idx,
                    Rms = IsFinite(pose.ReprojectionRmsPx) ? pose.ReprojectionRmsPx : double.MaxValue,
                    PoseNumber = pose.SourcePoseIndex > 0 ? pose.SourcePoseIndex : idx + 1
                })
                .OrderBy(x => x.Rms)
                .Take(Math.Max(1, Math.Min(maxSeedCount - seeds.Count, 6)))
                .ToList();

            foreach (var item in rankedPoseIndexes)
            {
                try
                {
                    double[,] singleTarget2Gripper = EstimateTarget2GripperInitial(
                        initialCam2Base,
                        robotPoses[item.Index],
                        cameraPoses[item.Index],
                        robotRotType);
                    double[] ySingle = MathUtils.MatrixToParamsScaled(singleTarget2Gripper);
                    AddSimultaneousSeed(
                        seeds,
                        $"ClosedFormX + SinglePoseY(Pose {item.PoseNumber})",
                        x0.Concat(ySingle).ToArray(),
                        maxSeedCount);
                }
                catch
                {
                }
            }

            if (seeds.Count < maxSeedCount)
            {
                double[] baseSeed = x0.Concat(yAvg).ToArray();

                // final18:
                // final16은 X(Camera2Base) translation ±100mm 정도만 seed로 추가했습니다.
                // closed-form이 수십~수백 mm 벗어나면 pattern search가 같은 지역해에 갇힐 수 있으므로,
                // X/Y의 회전/이동 seed를 넓게 추가합니다. translation 파라미터는 scaled 단위입니다.
                AddAxisPerturbationSeeds(seeds, baseSeed, "X", 0, 6, 0.20, 0.300, maxSeedCount);
                AddAxisPerturbationSeeds(seeds, baseSeed, "Y", 6, 12, 0.20, 0.300, maxSeedCount);
                AddAxisPerturbationSeeds(seeds, baseSeed, "X-wide", 0, 6, 0.45, 0.600, maxSeedCount);
                AddAxisPerturbationSeeds(seeds, baseSeed, "Y-wide", 6, 12, 0.45, 0.600, maxSeedCount);
            }

            return seeds;
        }

        private double[,] EstimateTarget2GripperInitial(
        double[,] initialCam2Base, RobotTargetPose robotPose, CameraTargetPose camPose,
        EEulerSequence robotRotType)
        {
            using (Mat X_cam2base = MathUtils.ArrayToMat4x4(initialCam2Base))
            using (Mat base2gripper = BuildEyeToHandBase2GripperPoseMat(robotPose, robotRotType))
            using (Mat P_target2cam = BuildCameraPoseMat(camPose))
            using (Mat Y_target2gripper = base2gripper * X_cam2base * P_target2cam)
            {
                return MathUtils.MatToArray4x4(Y_target2gripper);
            }
        }

        /// <summary>
        /// CameraTargetPose의 회전행렬과 병진값을 4x4 Target-to-Camera pose Mat으로 변환합니다.
        /// </summary>
        /// <returns>CV_64FC1 타입의 4x4 동차변환 Mat입니다. 호출자가 Dispose해야 합니다.</returns>
        public Mat BuildCameraPoseMat(CameraTargetPose pose)
        {
            ValidateCameraPose(pose, nameof(pose));

            using (Mat R = new Mat(3, 3, MatType.CV_64FC1))
            using (Mat t = new Mat(3, 1, MatType.CV_64FC1, new double[] { pose.X, pose.Y, pose.Z }))
            {
                for (int r = 0; r < 3; r++)
                    for (int c = 0; c < 3; c++)
                        R.Set(r, c, pose.RotationMatrix[r, c]);

                return MathUtils.Build4x4MatFromRt(R, t);
            }
        }

        /// <summary>
        /// Eye-to-Hand 설정에서 robot pose 입력 방향에 따라 Base-to-Gripper pose Mat을 생성합니다.
        /// </summary>
        /// <returns>CV_64FC1 타입의 4x4 동차변환 Mat입니다. 호출자가 Dispose해야 합니다.</returns>
        public Mat BuildEyeToHandBase2GripperPoseMat(RobotTargetPose pose, EEulerSequence rotType)
        {
            ValidateRobotPose(pose, nameof(pose));

            using (Mat rawRobotPose = BuildRobotPoseMat(pose, rotType))
            {
                if (HandEyeParams.EyeToHandRobotPoseInputDirection == RobotPoseTransformDirection.BaseToGripper)
                    return rawRobotPose.Clone();

                return rawRobotPose.Inv();
            }
        }


        /// <summary>
        /// 초기 Camera2Base와 각 pose pair로부터 Target2Gripper 후보를 만들고 회전 quaternion/병진 median으로 평균 초기값을 계산합니다.
        /// </summary>
        /// <returns>Target-to-Gripper 4x4 초기 행렬입니다.</returns>
        public double[,] EstimateTarget2GripperInitialAverage(double[,] initialCam2Base, List<RobotTargetPose> robotPoses, List<CameraTargetPose> cameraPoses, EEulerSequence robotRotType)
        {
            if (initialCam2Base == null)
                throw new ArgumentNullException(nameof(initialCam2Base));

            MathUtils.ValidateHomogeneousMatrix(initialCam2Base, nameof(initialCam2Base));

            if (robotPoses == null || cameraPoses == null || robotPoses.Count != cameraPoses.Count || robotPoses.Count == 0)
                throw new ArgumentException("Target2Gripper 평균 초기값을 만들 Robot/Camera pose list가 올바르지 않습니다.");

            List<double[,]> target2GripperList = new List<double[,]>();

            using (Mat X_cam2base = MathUtils.ArrayToMat4x4(initialCam2Base))
            {
                for (int i = 0; i < robotPoses.Count; i++)
                {
                    using (Mat base2gripper = BuildEyeToHandBase2GripperPoseMat(robotPoses[i], robotRotType))
                    using (Mat P_target2cam = BuildCameraPoseMat(cameraPoses[i]))
                    using (Mat temp = base2gripper * X_cam2base)
                    using (Mat Y_target2gripper = temp * P_target2cam)
                    {
                        double[,] y = MathUtils.MatToArray4x4(Y_target2gripper);
                        try
                        {
                            MathUtils.ValidateHomogeneousMatrix(y, "Target2Gripper candidate");
                            target2GripperList.Add(y);
                        }
                        catch
                        {
                            // 비정상 후보는 평균 초기값 계산에서 제외합니다.
                        }
                    }
                }
            }

            if (target2GripperList.Count == 0)
                throw new Exception("Target2Gripper 평균 초기값을 만들 수 없습니다.");

            double[,] averaged = AverageHomogeneousMatrices(target2GripperList);
            MathUtils.ValidateHomogeneousMatrix(averaged, nameof(averaged));
            return averaged;
        }

        private double[,] AverageHomogeneousMatrices(List<double[,]> transforms)
        {
            if (transforms == null || transforms.Count == 0)
                throw new ArgumentException("평균낼 transform이 없습니다.");

            List<double> xs = new List<double>();
            List<double> ys = new List<double>();
            List<double> zs = new List<double>();

            double[] qSum = new double[4];
            double[] qRef = null;

            foreach (double[,] t in transforms)
            {
                xs.Add(t[0, 3]);
                ys.Add(t[1, 3]);
                zs.Add(t[2, 3]);

                double[] q = MathUtils.RotationMatrixToQuaternion(t);
                if (qRef == null)
                    qRef = q;
                else if (MathUtils.QuaternionDot(qRef, q) < 0.0)
                {
                    for (int i = 0; i < 4; i++)
                        q[i] = -q[i];
                }

                for (int i = 0; i < 4; i++)
                    qSum[i] += q[i];
            }

            MathUtils.NormalizeQuaternion(qSum);
            double[,] R = MathUtils.QuaternionToRotationMatrix(qSum);

            double[,] result = new double[4, 4];
            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++)
                    result[r, c] = R[r, c];
            }

            // Translation은 평균보다 중앙값이 outlier 초기값에 덜 흔들립니다.
            result[0, 3] = MathUtils.Median(xs);
            result[1, 3] = MathUtils.Median(ys);
            result[2, 3] = MathUtils.Median(zs);
            result[3, 3] = 1.0;

            return result;
        }

        private double ComputeEyeToHandReprojectionCost12(
          double[] parameters, List<RobotTargetPose> robotPoses, List<CameraTargetPose> cameraPoses,
          Mat camMatrix, Mat dist, EEulerSequence robotRotType)
        {
            return ComputeEyeToHandReprojectionCost12Internal(
                parameters,
                robotPoses,
                cameraPoses,
                camMatrix,
                dist,
                robotRotType,
                HandEyeParams.Simultaneous12DofUseRobustLoss);
        }

        private void AddAxisPerturbationSeeds(List<Simultaneous12DofSeed> seeds, double[] source, string label,
            int startIndex, int endIndex, double rotationDeltaRad, double translationDeltaScaled, int maxSeedCount)
        {
            if (seeds.Count >= maxSeedCount || source == null)
                return;

            for (int i = startIndex; i < endIndex && seeds.Count < maxSeedCount; i++)
            {
                bool rotationParameter = ((i - startIndex) < 3);
                double delta = rotationParameter ? rotationDeltaRad : translationDeltaScaled;
                string unit = rotationParameter ? "rad" : "scaled";
                AddPerturbedSeed(seeds, $"{label} p{(i - startIndex)} +{delta:F3}{unit}", source, i, +delta, maxSeedCount);
                AddPerturbedSeed(seeds, $"{label} p{(i - startIndex)} -{delta:F3}{unit}", source, i, -delta, maxSeedCount);
            }
        }

        private char[] GetEulerAxes(EEulerSequence sequence)
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

        private void AddPerturbedSeed(
           List<Simultaneous12DofSeed> seeds,
           string name,
           double[] source,
           int parameterIndex,
           double delta,
           int maxSeedCount)
        {
            if (seeds.Count >= maxSeedCount)
                return;

            double[] copy = (double[])source.Clone();
            copy[parameterIndex] += delta;
            AddSimultaneousSeed(seeds, name, copy, maxSeedCount);
        }

        private void AddSimultaneousSeed(
            List<Simultaneous12DofSeed> seeds,
            string name,
            double[] parameters,
            int maxSeedCount)
        {
            if (seeds.Count >= maxSeedCount || parameters == null || parameters.Length < 12)
                return;

            for (int i = 0; i < 12; i++)
            {
                if (!IsFinite(parameters[i]))
                    return;
            }

            seeds.Add(new Simultaneous12DofSeed
            {
                Name = name,
                Parameters = (double[])parameters.Clone()
            });
        }

        private double SafeSqrtCost(double cost)
        {
            return Simultaneous12DofSeedScore.SafeSqrtStatic(cost);
        }


        private string FormatFinite(double value)
        {
            return IsFinite(value) ? value.ToString("F3") : "NaN";
        }

        private bool IsReverseOrderEulerConvention(EulerMatrixConvention convention)
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

        private bool IsPassiveEulerConvention(EulerMatrixConvention convention)
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

        private bool IsNegatedEulerConvention(EulerMatrixConvention convention)
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


        private bool IsIntrinsic(EEulerSequence sequence)
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

        private static void ValidateRobotPose(RobotTargetPose pose, string name)
        {
            if (pose == null)
                throw new ArgumentNullException(name);

            ValidateFinitePoseValue(pose.robotTx, $"{name}.robotTx");
            ValidateFinitePoseValue(pose.robotTy, $"{name}.robotTy");
            ValidateFinitePoseValue(pose.robotTz, $"{name}.robotTz");
            ValidateFinitePoseValue(pose.robotRx, $"{name}.robotRx");
            ValidateFinitePoseValue(pose.robotRy, $"{name}.robotRy");
            ValidateFinitePoseValue(pose.robotRz, $"{name}.robotRz");
        }

        private static void ValidateCameraPose(CameraTargetPose pose, string name)
        {
            if (pose == null)
                throw new ArgumentNullException(name);

            ValidateFinitePoseValue(pose.X, $"{name}.X");
            ValidateFinitePoseValue(pose.Y, $"{name}.Y");
            ValidateFinitePoseValue(pose.Z, $"{name}.Z");
            MathUtils.ValidateRotationMatrix(pose.RotationMatrix, $"{name}.RotationMatrix");
        }

        private static void ValidateFinitePoseValue(double value, string name)
        {
            if (!IsFinite(value))
                throw new ArgumentException($"{name} 값이 NaN/Infinity입니다.");
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}
