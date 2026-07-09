using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Calibration_test
{
    /// <summary>
    /// 체커보드 영상에서 코너를 검출하고 SolvePnP로 CameraTargetPose를 계산하는 비전 포즈 추정 서비스입니다.
    /// DLL 사용 시 외부에서 전달한 HandEyeParams와 MathUtils 인스턴스를 공유할 수 있습니다.
    /// </summary>
    public class BoardDetector
    {
        private readonly HandEyeParams HandEyeParams;
        private readonly MathUtils MathUtils;

        /// <summary>
        /// 지정한 파라미터 인스턴스로 BoardDetector를 생성합니다.
        /// </summary>
        /// <param name="parameters">공유할 Hand-Eye 파라미터입니다.</param>
        public BoardDetector(HandEyeParams parameters)
            : this(parameters, null)
        {
        }

        /// <summary>
        /// 지정한 파라미터와 수학 유틸리티 인스턴스를 공유하도록 BoardDetector를 생성합니다.
        /// </summary>
        /// <param name="parameters">공유할 Hand-Eye 파라미터입니다.</param>
        /// <param name="mathUtils">공유할 MathUtils입니다. null이면 새 인스턴스를 생성합니다.</param>
        public BoardDetector(HandEyeParams parameters, MathUtils mathUtils)
        {
            HandEyeParams = parameters ?? throw new ArgumentNullException(nameof(parameters));
            MathUtils = mathUtils ?? new MathUtils(HandEyeParams);
            MathUtils.SetParams(HandEyeParams);
        }

        /// <summary>
        /// 영상에서 체커보드 내부 코너를 찾고 PnP로 타겟의 카메라 기준 6-DoF pose를 계산합니다.
        /// PnP RMS가 파라미터 허용 기준을 넘으면 pose를 생성하지 않습니다.
        /// </summary>
        /// <param name="colorMat">입력 영상 Mat입니다. 검출 코너와 축이 그려질 수 있습니다.</param>
        /// <param name="intrinsics">카메라 내부 파라미터입니다.</param>
        /// <param name="patternCols">체커보드 내부 코너 column 개수입니다.</param>
        /// <param name="patternRows">체커보드 내부 코너 row 개수입니다.</param>
        /// <param name="squareSizeMm">체커보드 한 칸의 실제 크기(mm)입니다.</param>
        /// <param name="index">원본 이미지 순번입니다. SourcePoseIndex는 index+1로 저장됩니다.</param>
        /// <param name="pose">성공 시 계산된 CameraTargetPose입니다.</param>
        /// <returns>검출, PnP, 재투영 검증을 모두 통과하면 true입니다.</returns>
        public bool TryGetCheckerboardTargetPose(Mat colorMat, Intrinsic intrinsics,
                        int patternCols, int patternRows, float squareSizeMm, int index, out CameraTargetPose pose)
        {
            pose = null;

            if (!TryValidateCheckerboardInput(colorMat, intrinsics, patternCols, patternRows, squareSizeMm, index))
                return false;

            Size patternSize = new Size(patternCols, patternRows);

            Point2f[] imagePoints;

            bool found = Cv2.FindChessboardCorners(
                colorMat,
                patternSize,
                out imagePoints,
                ChessboardFlags.AdaptiveThresh | ChessboardFlags.NormalizeImage);

            if (!found)
            {
                Console.WriteLine("Checkerboard 코너를 찾지 못했습니다.");
                return false;
            }

            using (Mat gray = new Mat())
            {
                if (colorMat.Channels() == 3)
                    Cv2.CvtColor(colorMat, gray, ColorConversionCodes.BGR2GRAY);
                else
                    colorMat.CopyTo(gray);

                Cv2.CornerSubPix(gray, imagePoints,
                    new Size(11, 11), new Size(-1, -1),
                    new TermCriteria(CriteriaTypes.Eps | CriteriaTypes.MaxIter, 30, 0.001));
            }

            Cv2.DrawChessboardCorners(colorMat, patternSize, imagePoints, found);

            Point3f[] objectPoints = CreateCheckerboardObjectPoints(
                patternCols, patternRows, squareSizeMm);

            using (Mat cameraMatrix = MathUtils.BuildCameraMatrix(intrinsics))
            using (Mat distCoeffs = MathUtils.BuildDistCoeffs(intrinsics))
            using (Mat rvec = new Mat())
            using (Mat tvec = new Mat())
            using (InputArray objectPointsInput = InputArray.Create(objectPoints))
            using (InputArray imagePointsInput = InputArray.Create(imagePoints))
            {
                try
                {
                    Cv2.SolvePnP(
                   objectPointsInput,
                   imagePointsInput,
                   cameraMatrix,
                   distCoeffs,
                   rvec,
                   tvec,
                   false,
                   SolvePnPFlags.Iterative);
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"SolvePnP 예외 발생: {ex.Message}");
                    return false;
                }

                if (rvec.Empty() || tvec.Empty())
                {
                    Console.WriteLine("SolvePnP 실패: rvec 또는 tvec가 비어 있습니다.");
                    return false;
                }

                double rmsError = CalculateRMSError(objectPoints, imagePoints, rvec, tvec,
                    cameraMatrix, distCoeffs, out double maxError);

                if (!IsFinite(rmsError) || !IsFinite(maxError) || rmsError >= double.MaxValue / 8.0)
                {
                    Console.WriteLine($"PnP Reprojection 계산 실패 또는 비정상 값: RMS={rmsError}, MAX={maxError}");
                    return false;
                }

                if (rmsError > HandEyeParams.MaxAcceptPnPPoseReprojectionRmsPx)
                {
                    Console.WriteLine(
                        $"PnP Reprojection RMS가 허용 기준을 초과하여 pose를 제외합니다. " +
                        $"RMS={rmsError:F3}px, Limit={HandEyeParams.MaxAcceptPnPPoseReprojectionRmsPx:F3}px");
                    return false;
                }

                Console.WriteLine($"[PnP Reprojection] RMS={rmsError:F3}px, MAX={maxError:F3}px");

                DrawPoseAxis(colorMat, rvec, tvec, cameraMatrix, distCoeffs, squareSizeMm * 2.0f);

                using (Mat R = new Mat())
                {
                    Cv2.Rodrigues(rvec, R);

                    RotationMatrixToEulerZYX(R, out double rx, out double ry, out double rz);

                    var tidx = tvec.GetGenericIndexer<double>();

                    pose = new CameraTargetPose
                    {
                        X = tidx[0, 0],
                        Y = tidx[1, 0],
                        Z = tidx[2, 0],
                        Rx = rx,
                        Ry = ry,
                        Rz = rz,
                        RotationMatrix = MathUtils.Mat3x3ToArray(R),

                        ReprojectionRmsPx = rmsError,
                        ReprojectionMaxPx = maxError,

                        ImagePoints = imagePoints.ToArray(),
                        ObjectPoints = objectPoints.ToArray(),

                        SourcePoseIndex = index + 1,
                        HasExplicitPoseIndex = false,
                        FrameCandidateName = "Normal"
                    };

                    pose.FrameCandidates = BuildCheckerboardFrameCandidates(
                        pose,
                        patternCols,
                        patternRows,
                        squareSizeMm);

                    Console.WriteLine("===== Target Pose in Camera =====");
                    Console.WriteLine($"T: X={pose.X:F3}, Y={pose.Y:F3}, Z={pose.Z:F3} mm");
                    Console.WriteLine($"R: Rx={pose.Rx:F3}, Ry={pose.Ry:F3}, Rz={pose.Rz:F3} deg");
                    Console.WriteLine("=================================");

                    return true;
                }
            }
        }

        /// <summary>
        /// 3x3 회전행렬을 ZYX 순서의 Euler 각(degree)으로 변환합니다.
        /// Gimbal lock 근처에서는 Rx를 0으로 두고 Rz를 계산합니다.
        /// </summary>
        private void RotationMatrixToEulerZYX(Mat R, out double rxDeg, out double ryDeg, out double rzDeg)
        {
            MathUtils.Mat3x3ToArray(R);
            var m = R.GetGenericIndexer<double>();

            double r00 = m[0, 0];
            double r10 = m[1, 0];
            double r20 = m[2, 0];
            double r21 = m[2, 1];
            double r22 = m[2, 2];
            double r01 = m[0, 1];
            double r11 = m[1, 1];

            double ry = Math.Asin(-r20);
            double cy = Math.Cos(ry);

            double rx;
            double rz;

            if (Math.Abs(cy) > 1e-6)
            {
                rx = Math.Atan2(r21, r22);
                rz = Math.Atan2(r10, r00);
            }
            else
            {
                // Gimbal lock 근처
                rx = 0;
                rz = Math.Atan2(-r01, r11);
            }

            rxDeg = rx * 180.0 / Math.PI;
            ryDeg = ry * 180.0 / Math.PI;
            rzDeg = rz * 180.0 / Math.PI;
        }

        /// <summary>
        /// CameraTargetPose의 값 배열을 복제하되 FrameCandidates 순환 참조는 복사하지 않습니다.
        /// </summary>
        private CameraTargetPose CloneCameraTargetPose(CameraTargetPose src)
        {
            if (src == null)
                return null;

            return new CameraTargetPose
            {
                X = src.X,
                Y = src.Y,
                Z = src.Z,
                Rx = src.Rx,
                Ry = src.Ry,
                Rz = src.Rz,
                RotationMatrix = src.RotationMatrix != null ? (double[,])src.RotationMatrix.Clone() : null,
                ReprojectionRmsPx = src.ReprojectionRmsPx,
                ReprojectionMaxPx = src.ReprojectionMaxPx,
                ImagePoints = src.ImagePoints != null ? src.ImagePoints.ToArray() : null,
                ObjectPoints = src.ObjectPoints != null ? src.ObjectPoints.ToArray() : null,
                SourcePoseIndex = src.SourcePoseIndex,
                HasExplicitPoseIndex = src.HasExplicitPoseIndex,
                FrameCandidateName = src.FrameCandidateName
            };
        }

        /// <summary>
        /// CameraTargetPose의 회전행렬과 병진값을 4x4 동차변환 배열로 결합합니다.
        /// </summary>
        private double[,] Build4x4ArrayFromCameraPose(CameraTargetPose pose)
        {
            ValidateCameraTargetPose(pose, nameof(pose));

            double[,] T = Calibration_test.HandEyeParams.CreateIdentity4x4();

            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++)
                {
                    T[r, c] = pose.RotationMatrix[r, c];
                }
            }

            T[0, 3] = pose.X;
            T[1, 3] = pose.Y;
            T[2, 3] = pose.Z;

            return T;
        }

        /// <summary>
        /// 기존 카메라 pose에 타겟 프레임 후보 변환을 곱해 대칭 체커보드 후보 pose를 생성합니다.
        /// </summary>
        private CameraTargetPose TransformCameraTargetPoseFrame(
          CameraTargetPose src,
          string candidateName,
          double[,] targetFrameTransform,
          Point3f[] candidateObjectPoints)
        {
            if (src == null)
                return null;

            MathUtils.ValidateHomogeneousMatrix(targetFrameTransform, nameof(targetFrameTransform));

            using (Mat T_current = MathUtils.ArrayToMat4x4(Build4x4ArrayFromCameraPose(src)))
            using (Mat T_delta = MathUtils.ArrayToMat4x4(targetFrameTransform))
            using (Mat T_candidate = T_current * T_delta)
            {
                double[,] arr = MathUtils.MatToArray4x4(T_candidate);

                using (Mat R = new Mat(3, 3, MatType.CV_64FC1))
                {
                    var ridx = R.GetGenericIndexer<double>();
                    for (int r = 0; r < 3; r++)
                    {
                        for (int c = 0; c < 3; c++)
                        {
                            ridx[r, c] = arr[r, c];
                        }
                    }

                    RotationMatrixToEulerZYX(R, out double rx, out double ry, out double rz);

                    CameraTargetPose candidate = CloneCameraTargetPose(src);
                    candidate.X = arr[0, 3];
                    candidate.Y = arr[1, 3];
                    candidate.Z = arr[2, 3];
                    candidate.Rx = rx;
                    candidate.Ry = ry;
                    candidate.Rz = rz;
                    candidate.RotationMatrix = new double[3, 3];
                    for (int r = 0; r < 3; r++)
                    {
                        for (int c = 0; c < 3; c++)
                        {
                            candidate.RotationMatrix[r, c] = arr[r, c];
                        }
                    }
                    candidate.ObjectPoints = candidateObjectPoints != null ? candidateObjectPoints.ToArray() : src.ObjectPoints;
                    candidate.FrameCandidateName = candidateName;
                    candidate.FrameCandidates = null;
                    return candidate;
                }
            }
        }

        /// <summary>
        /// 회전행렬과 병진값으로 타겟 프레임 후보 변환을 생성합니다.
        /// </summary>
        private double[,] CreateFrameTransform(double[,] R, double tx, double ty, double tz)
        {
            MathUtils.ValidateRotationMatrix(R, nameof(R));

            if (!IsFinite(tx) || !IsFinite(ty) || !IsFinite(tz))
                throw new ArgumentException("프레임 변환 병진값에 NaN/Infinity가 포함되어 있습니다.");

            double[,] T = Calibration_test.HandEyeParams.CreateIdentity4x4();
            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++)
                {
                    T[r, c] = R[r, c];
                }
            }
            T[0, 3] = tx;
            T[1, 3] = ty;
            T[2, 3] = tz;
            return T;
        }

        /// <summary>
        /// 대칭 체커보드에서 가능한 target frame 후보(Normal/RotZ180/FlipX/FlipY)를 생성합니다.
        /// </summary>
        private List<CameraTargetPose> BuildCheckerboardFrameCandidates(
    CameraTargetPose normalPose,
    int patternCols,
    int patternRows,
    float squareSizeMm)
        {
            ValidateCheckerboardGeometry(patternCols, patternRows, squareSizeMm);

            List<CameraTargetPose> candidates = new List<CameraTargetPose>();

            if (normalPose == null)
                return candidates;

            float width = (patternCols - 1) * squareSizeMm;
            float height = (patternRows - 1) * squareSizeMm;

            CameraTargetPose normal = CloneCameraTargetPose(normalPose);
            normal.FrameCandidateName = "Normal";
            normal.ObjectPoints = CreateCheckerboardObjectPointsVariant(patternCols, patternRows, squareSizeMm, false, false);
            candidates.Add(normal);

            // OpenCV의 일반 checkerboard는 180도 회전된 시작 코너를 구분하지 못할 수 있습니다.
            // PnP RMS는 낮아도 target frame이 반대 코너가 되면 hand-eye RMS가 크게 튑니다.
            candidates.Add(TransformCameraTargetPoseFrame(
                normalPose,
                "RotZ180",
                CreateFrameTransform(
                    new double[,] { { -1, 0, 0 }, { 0, -1, 0 }, { 0, 0, 1 } },
                    width,
                    height,
                    0),
                CreateCheckerboardObjectPointsVariant(patternCols, patternRows, squareSizeMm, true, true)));

            // 일부 보드/검출 방향에서는 시작 코너가 X 또는 Y 방향으로만 뒤집힌 것처럼 보일 수 있습니다.
            // 평면 PnP에서는 법선 방향까지 함께 바뀌는 180도 회전 후보로 취급합니다.
            candidates.Add(TransformCameraTargetPoseFrame(
                normalPose,
                "FlipX_RotY180",
                CreateFrameTransform(
                    new double[,] { { -1, 0, 0 }, { 0, 1, 0 }, { 0, 0, -1 } },
                    width,
                    0,
                    0),
                CreateCheckerboardObjectPointsVariant(patternCols, patternRows, squareSizeMm, true, false)));

            candidates.Add(TransformCameraTargetPoseFrame(
                normalPose,
                "FlipY_RotX180",
                CreateFrameTransform(
                    new double[,] { { 1, 0, 0 }, { 0, -1, 0 }, { 0, 0, -1 } },
                    0,
                    height,
                    0),
                CreateCheckerboardObjectPointsVariant(patternCols, patternRows, squareSizeMm, false, true)));

            candidates = candidates.Where(c => c != null).ToList();

            // 각 후보에서 다시 전체 후보 목록에 접근할 필요가 있으므로 동일한 목록을 공유합니다.
            foreach (CameraTargetPose candidate in candidates)
                candidate.FrameCandidates = candidates;

            return candidates;
        }

        /// <summary>
        /// X/Y 방향 반전 옵션을 적용한 체커보드 3D object point 배열을 생성합니다.
        /// </summary>
        private Point3f[] CreateCheckerboardObjectPointsVariant(
           int patternCols,
           int patternRows,
           float squareSizeMm,
           bool reverseX,
           bool reverseY)
        {
            ValidateCheckerboardGeometry(patternCols, patternRows, squareSizeMm);

            List<Point3f> points = new List<Point3f>();
            float maxX = (patternCols - 1) * squareSizeMm;
            float maxY = (patternRows - 1) * squareSizeMm;

            for (int y = 0; y < patternRows; y++)
            {
                for (int x = 0; x < patternCols; x++)
                {
                    float px = x * squareSizeMm;
                    float py = y * squareSizeMm;

                    if (reverseX)
                        px = maxX - px;
                    if (reverseY)
                        py = maxY - py;

                    points.Add(new Point3f(px, py, 0));
                }
            }

            return points.ToArray();
        }


        /// <summary>
        /// PnP 결과를 다시 투영해 관측 코너와의 RMS/Max 픽셀 오차를 계산합니다.
        /// </summary>
        private double CalculateRMSError(
    Point3f[] objectPoints, Point2f[] imagePoints,
    Mat rvec, Mat tvec, Mat cameraMatrix, Mat distCoeffs,
    out double maxError)
        {
            maxError = 0;

            if (objectPoints == null || imagePoints == null)
                return double.MaxValue;

            if (objectPoints.Length == 0 || objectPoints.Length != imagePoints.Length)
                return double.MaxValue;

            using (Mat objectMat = Mat.FromArray(objectPoints))
            using (Mat projectedMat = new Mat())
            {
                try
                {
                    Cv2.ProjectPoints(
                        objectMat,
                        rvec,
                        tvec,
                        cameraMatrix,
                        distCoeffs,
                        projectedMat);
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Reprojection 계산 실패: {ex.Message}");
                    return double.MaxValue;
                }

                Point2f[] projectedPoints = CalibrationOptimizer.ReadPoint2fArray(projectedMat, objectPoints.Length); /* new Point2f[objectPoints.Length];*/

                if (projectedPoints == null || projectedPoints.Length != imagePoints.Length)
                    return double.MaxValue;

                double sumSq = 0;

                for (int i = 0; i < imagePoints.Length; i++)
                {
                    double dx = projectedPoints[i].X - imagePoints[i].X;
                    double dy = projectedPoints[i].Y - imagePoints[i].Y;

                    if (!IsFinite(dx) || !IsFinite(dy))
                    {
                        return double.MaxValue;
                    }

                    double err = Math.Sqrt(dx * dx + dy * dy);

                    if (!IsFinite(err))
                        return double.MaxValue;

                    if (err > maxError)
                        maxError = err;

                    sumSq += err * err;

                    if (!IsFinite(sumSq) || sumSq >= double.MaxValue / 16.0)
                    {
                        maxError = double.MaxValue;
                        return double.MaxValue;
                    }
                }

                return Math.Sqrt(sumSq / imagePoints.Length);
            }
        }

        /// <summary>
        /// 체커보드 내부 코너의 3D object point 배열을 보드 좌상단 기준 XY 평면 위에 생성합니다.
        /// </summary>
        private Point3f[] CreateCheckerboardObjectPoints(int patternCols, int patternRows, float squareSizeMm)
        {
            ValidateCheckerboardGeometry(patternCols, patternRows, squareSizeMm);

            List<Point3f> points = new List<Point3f>();

            for (int y = 0; y < patternRows; y++)
            {
                for (int x = 0; x < patternCols; x++)
                {
                    points.Add(new Point3f(
                        x * squareSizeMm,
                        y * squareSizeMm,
                        0));
                }
            }

            return points.ToArray();
        }

        /// <summary>
        /// 검출된 pose 축을 영상 위에 투영해 X/Y/Z 방향을 표시합니다.
        /// </summary>
        private static void DrawPoseAxis(Mat image, Mat rvec, Mat tvec,
                     Mat cameraMatrix, Mat distCoeffs, float axisLengthMm)
        {
            if (image == null || image.Empty() || rvec == null || rvec.Empty() ||
                tvec == null || tvec.Empty() || cameraMatrix == null || cameraMatrix.Empty() ||
                distCoeffs == null || distCoeffs.Empty() || axisLengthMm <= 0.0f)
            {
                return;
            }

            Point3f[] axisPoints =
            {
        new Point3f(0, 0, 0),
        new Point3f(axisLengthMm, 0, 0),
        new Point3f(0, axisLengthMm, 0),
        new Point3f(0, 0, -axisLengthMm)
    };

            using (Mat axisMat = Mat.FromArray(axisPoints))
            using (Mat projectedMat = new Mat())
            {
                try
                {
                    Cv2.ProjectPoints(
                        axisMat,
                        rvec,
                        tvec,
                        cameraMatrix,
                        distCoeffs,
                        projectedMat);
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Pose Axis 투영 실패: {ex.Message}");
                    return;
                }

                Point2f[] projectedPoints = CalibrationOptimizer.ReadPoint2fArray(projectedMat, axisPoints.Length);/*new Point2f[axisPoints.Length];*/

                //projectedMat.GetArray(out projectedPoints);

                if (projectedPoints.Length < 4)
                    return;

                if (!projectedPoints.Take(4).All(IsFinite))
                {
                    Console.WriteLine("Pose Axis 투영 결과에 NaN/Infinity가 있어 축 표시를 생략합니다.");
                    return;
                }

                Point p0 = new Point(
                    (int)projectedPoints[0].X,
                    (int)projectedPoints[0].Y);

                Point px = new Point(
                    (int)projectedPoints[1].X,
                    (int)projectedPoints[1].Y);

                Point py = new Point(
                    (int)projectedPoints[2].X,
                    (int)projectedPoints[2].Y);

                Point pz = new Point(
                    (int)projectedPoints[3].X,
                    (int)projectedPoints[3].Y);

                Cv2.Line(image, p0, px, Scalar.Red, 3);
                Cv2.Line(image, p0, py, Scalar.Green, 3);
                Cv2.Line(image, p0, pz, Scalar.Blue, 3);

                Cv2.PutText(image, "X", px, HersheyFonts.HersheySimplex, 0.7, Scalar.Red, 2);
                Cv2.PutText(image, "Y", py, HersheyFonts.HersheySimplex, 0.7, Scalar.Green, 2);
                Cv2.PutText(image, "Z", pz, HersheyFonts.HersheySimplex, 0.7, Scalar.Blue, 2);
            }
        }

        /// <summary>
        /// 체커보드 검출 입력값이 수학적으로 유효한지 확인합니다.
        /// Try 계열 API 정책에 맞춰 실패 시 false를 반환합니다.
        /// </summary>
        private bool TryValidateCheckerboardInput(
            Mat colorMat,
            Intrinsic intrinsics,
            int patternCols,
            int patternRows,
            float squareSizeMm,
            int index)
        {
            if (colorMat == null || colorMat.Empty())
            {
                Console.WriteLine("입력 이미지 Mat이 비어 있습니다.");
                return false;
            }

            if (index < 0)
            {
                Console.WriteLine($"이미지 index가 올바르지 않습니다. Index={index}");
                return false;
            }

            try
            {
                ValidateCheckerboardGeometry(patternCols, patternRows, squareSizeMm);
                MathUtils.ValidateIntrinsic(intrinsics);
                HandEyeParams.ValidateBasic();
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Checkerboard 입력 검증 실패: {ex.Message}");
                return false;
            }
        }

        /// <summary>
        /// 체커보드 코너 수와 square size가 PnP 입력으로 유효한지 검증합니다.
        /// </summary>
        private static void ValidateCheckerboardGeometry(int patternCols, int patternRows, float squareSizeMm)
        {
            if (patternCols <= 1 || patternRows <= 1)
                throw new ArgumentException($"체커보드 내부 코너 수가 올바르지 않습니다. Cols={patternCols}, Rows={patternRows}");

            if (squareSizeMm <= 0.0f || float.IsNaN(squareSizeMm) || float.IsInfinity(squareSizeMm))
                throw new ArgumentException($"체커보드 squareSizeMm가 올바르지 않습니다. SquareSizeMm={squareSizeMm}");
        }

        /// <summary>
        /// CameraTargetPose가 4x4 변환 생성에 필요한 유한한 병진값과 유효 회전행렬을 갖는지 검증합니다.
        /// </summary>
        private static void ValidateCameraTargetPose(CameraTargetPose pose, string name)
        {
            if (pose == null)
                throw new ArgumentNullException(name);

            if (!IsFinite(pose.X) || !IsFinite(pose.Y) || !IsFinite(pose.Z))
                throw new ArgumentException($"{name} 병진값에 NaN/Infinity가 포함되어 있습니다.");

            MathUtils.ValidateRotationMatrix(pose.RotationMatrix, $"{name}.RotationMatrix");
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }

        private static bool IsFinite(Point2f point)
        {
            return IsFinite(point.X) && IsFinite(point.Y);
        }
    }
}
