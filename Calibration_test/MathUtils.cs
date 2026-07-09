using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Calibration_test
{
    /// <summary>
    /// Hand-Eye Calibration에서 사용하는 행렬, 회전, 카메라 파라미터 변환 유틸리티입니다.
    /// 모든 공개 함수는 DLL 외부 호출을 고려해 입력 크기, 유한값, 동차변환 형식을 검증합니다.
    /// </summary>
    public class MathUtils
    {
        private const double RotationValidationTolerance = 1e-3;
        private HandEyeParams HandEyeParams;

        /// <summary>
        /// 현재 수학 유틸리티가 참조하는 캘리브레이션 파라미터 인스턴스입니다.
        /// </summary>
        public HandEyeParams Params
        {
            get { return HandEyeParams; }
        }

        /// <summary>
        /// 기본 파라미터 인스턴스를 생성해 유틸리티를 초기화합니다.
        /// </summary>
        public MathUtils()
            : this(new HandEyeParams())
        {
        }

        /// <summary>
        /// 외부에서 전달한 파라미터 인스턴스를 공유하도록 유틸리티를 초기화합니다.
        /// </summary>
        /// <param name="parameters">공유할 Hand-Eye 파라미터 인스턴스입니다.</param>
        public MathUtils(HandEyeParams parameters)
        {
            SetParams(parameters);
        }

        /// <summary>
        /// 유틸리티가 참조하는 파라미터 인스턴스를 교체합니다.
        /// </summary>
        /// <param name="parameters">공유할 Hand-Eye 파라미터 인스턴스입니다.</param>
        public void SetParams(HandEyeParams parameters)
        {
            HandEyeParams = parameters ?? throw new ArgumentNullException(nameof(parameters));
        }

        /// <summary>
        /// 4x4 동차변환 배열을 OpenCV 64-bit Mat으로 변환합니다.
        /// 마지막 행과 회전 블록의 수학적 유효성을 함께 검증합니다.
        /// </summary>
        /// <param name="array">4x4 동차변환 배열입니다.</param>
        /// <returns>CV_64FC1 타입의 4x4 Mat입니다. 호출자가 Dispose해야 합니다.</returns>
        public Mat ArrayToMat4x4(double[,] array)
        {
            ValidateHomogeneousMatrix(array, nameof(array));

            Mat mat = new Mat(4, 4, MatType.CV_64FC1);
            var idx = mat.GetGenericIndexer<double>();

            for (int r = 0; r < 4; r++)
            {
                for (int c = 0; c < 4; c++)
                {
                    idx[r, c] = array[r, c];
                }
            }

            return mat;
        }

        /// <summary>
        /// 3x3 회전행렬과 3x1 병진벡터를 4x4 동차 변환 행렬(Homogeneous Transformation Matrix)로 결합합니다.
        /// </summary>
        /// <param name="R">3x3 회전행렬 Mat입니다.</param>
        /// <param name="t">3x1 병진벡터 Mat입니다.</param>
        /// <returns>CV_64FC1 타입의 4x4 동차변환 Mat입니다. 호출자가 Dispose해야 합니다.</returns>
        public Mat Build4x4MatFromRt(Mat R, Mat t)
        {
            ValidateRotationMat(R, nameof(R));
            ValidateTranslationMat(t, nameof(t));

            Mat T = Mat.Eye(4, 4, MatType.CV_64FC1);
            var indexerT = T.GetGenericIndexer<double>();
            var indexerR = R.GetGenericIndexer<double>();
            var indexert = t.GetGenericIndexer<double>();

            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++)
                {
                    indexerT[r, c] = indexerR[r, c];
                }
                indexerT[r, 3] = indexert[r, 0];
            }
            return T;
        }

        /// <summary>
        /// 4x4 동차변환 Mat을 double 배열로 변환합니다.
        /// 마지막 행과 회전 블록의 수학적 유효성을 함께 검증합니다.
        /// </summary>
        /// <param name="mat">CV_64FC1 타입의 4x4 Mat입니다.</param>
        /// <returns>4x4 동차변환 배열입니다.</returns>
        public double[,] MatToArray4x4(Mat mat)
        {
            ValidateMatShape(mat, 4, 4, nameof(mat));

            double[,] arr = new double[4, 4];
            var idx = mat.GetGenericIndexer<double>();

            for (int r = 0; r < 4; r++)
            {
                for (int c = 0; c < 4; c++)
                {
                    arr[r, c] = idx[r, c];
                }
            }

            ValidateHomogeneousMatrix(arr, nameof(mat));
            return arr;
        }

        /// <summary>
        /// 두 3x3 행렬의 곱 a*b를 계산합니다.
        /// </summary>
        /// <param name="a">왼쪽 3x3 행렬입니다.</param>
        /// <param name="b">오른쪽 3x3 행렬입니다.</param>
        /// <returns>행렬 곱 결과입니다.</returns>
        public double[,] Multiply3x3(double[,] a, double[,] b)
        {
            ValidateFiniteArray(a, 3, 3, nameof(a));
            ValidateFiniteArray(b, 3, 3, nameof(b));

            double[,] r = new double[3, 3];
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    double sum = 0.0;
                    for (int k = 0; k < 3; k++)
                        sum += a[i, k] * b[k, j];

                    if (!IsFinite(sum))
                        throw new ArgumentException("3x3 행렬 곱 결과에 NaN/Infinity가 발생했습니다.");

                    r[i, j] = sum;
                }
            }
            return r;
        }

        /// <summary>
        /// 3x3 행렬의 전치행렬을 계산합니다.
        /// </summary>
        /// <param name="a">전치할 3x3 행렬입니다.</param>
        /// <returns>전치행렬입니다.</returns>
        public double[,] Transpose3x3(double[,] a)
        {
            ValidateFiniteArray(a, 3, 3, nameof(a));

            double[,] r = new double[3, 3];
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                    r[i, j] = a[j, i];
            }
            return r;
        }

        /// <summary>
        /// 회전행렬의 trace로부터 회전각(rad)을 계산합니다.
        /// acos 입력은 부동소수 오차를 고려해 [-1, 1]로 clamp합니다.
        /// </summary>
        /// <param name="r">3x3 회전행렬 또는 좌상단 3x3 회전 블록을 가진 행렬입니다.</param>
        /// <returns>회전각(rad)입니다.</returns>
        public double RotationAngleRad(double[,] r)
        {
            ValidateRotationMatrix(r, nameof(r));

            double trace = r[0, 0] + r[1, 1] + r[2, 2];
            double cos = (trace - 1.0) * 0.5;
            if (cos > 1.0) cos = 1.0;
            if (cos < -1.0) cos = -1.0;
            return Math.Acos(cos);
        }

        /// <summary>
        /// 유한한 double 값 목록의 중앙값을 계산합니다.
        /// </summary>
        /// <param name="values">중앙값 계산 대상 값 목록입니다.</param>
        /// <returns>정렬 기준 중앙값입니다.</returns>
        public double Median(List<double> values)
        {
            if (values == null || values.Count == 0)
                throw new ArgumentException("Median 계산 대상이 없습니다.");

            for (int i = 0; i < values.Count; i++)
            {
                if (!IsFinite(values[i]))
                    throw new ArgumentException($"Median 계산 대상에 NaN/Infinity가 포함되어 있습니다. Index={i}");
            }

            List<double> sorted = values.OrderBy(v => v).ToList();
            int mid = sorted.Count / 2;

            if (sorted.Count % 2 == 1)
                return sorted[mid];

            return 0.5 * (sorted[mid - 1] + sorted[mid]);
        }

        /// <summary>
        /// 회전행렬을 단위 quaternion(w, x, y, z)으로 변환합니다.
        /// 4x4 동차변환을 전달한 경우 좌상단 3x3 회전 블록만 사용합니다.
        /// </summary>
        /// <param name="m">3x3 회전행렬 또는 4x4 동차변환 배열입니다.</param>
        /// <returns>정규화된 quaternion 배열입니다.</returns>
        public double[] RotationMatrixToQuaternion(double[,] m)
        {
            ValidateRotationMatrix(m, nameof(m));

            double trace = m[0, 0] + m[1, 1] + m[2, 2];
            double w, x, y, z;

            if (trace > 0.0)
            {
                double s = Math.Sqrt(trace + 1.0) * 2.0;
                w = 0.25 * s;
                x = (m[2, 1] - m[1, 2]) / s;
                y = (m[0, 2] - m[2, 0]) / s;
                z = (m[1, 0] - m[0, 1]) / s;
            }
            else if (m[0, 0] > m[1, 1] && m[0, 0] > m[2, 2])
            {
                double s = Math.Sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0;
                w = (m[2, 1] - m[1, 2]) / s;
                x = 0.25 * s;
                y = (m[0, 1] + m[1, 0]) / s;
                z = (m[0, 2] + m[2, 0]) / s;
            }
            else if (m[1, 1] > m[2, 2])
            {
                double s = Math.Sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0;
                w = (m[0, 2] - m[2, 0]) / s;
                x = (m[0, 1] + m[1, 0]) / s;
                y = 0.25 * s;
                z = (m[1, 2] + m[2, 1]) / s;
            }
            else
            {
                double s = Math.Sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0;
                w = (m[1, 0] - m[0, 1]) / s;
                x = (m[0, 2] + m[2, 0]) / s;
                y = (m[1, 2] + m[2, 1]) / s;
                z = 0.25 * s;
            }

            double[] q = new double[] { w, x, y, z };
            NormalizeQuaternion(q);
            return q;
        }

        /// <summary>
        /// quaternion(w, x, y, z)을 3x3 회전행렬로 변환합니다.
        /// 입력 quaternion은 내부에서 정규화합니다.
        /// </summary>
        /// <param name="q">quaternion(w, x, y, z) 배열입니다.</param>
        /// <returns>3x3 회전행렬입니다.</returns>
        public double[,] QuaternionToRotationMatrix(double[] q)
        {
            ValidateFiniteVector(q, 4, nameof(q));

            double[] normalized = (double[])q.Clone();
            NormalizeQuaternion(normalized);

            double w = normalized[0];
            double x = normalized[1];
            double y = normalized[2];
            double z = normalized[3];

            double[,] R = new double[3, 3];

            R[0, 0] = 1.0 - 2.0 * (y * y + z * z);
            R[0, 1] = 2.0 * (x * y - z * w);
            R[0, 2] = 2.0 * (x * z + y * w);

            R[1, 0] = 2.0 * (x * y + z * w);
            R[1, 1] = 1.0 - 2.0 * (x * x + z * z);
            R[1, 2] = 2.0 * (y * z - x * w);

            R[2, 0] = 2.0 * (x * z - y * w);
            R[2, 1] = 2.0 * (y * z + x * w);
            R[2, 2] = 1.0 - 2.0 * (x * x + y * y);

            ValidateRotationMatrix(R, nameof(R));
            return R;
        }

        /// <summary>
        /// 4x4 동차변환 행렬을 Rodrigues 회전벡터 3개와 병진 3개의 6-DoF 파라미터로 변환합니다.
        /// </summary>
        /// <param name="matrix">4x4 동차변환 배열입니다.</param>
        /// <returns>[rx, ry, rz, tx, ty, tz] 순서의 파라미터입니다.</returns>
        public double[] MatrixToParams(double[,] matrix)
        {
            ValidateHomogeneousMatrix(matrix, nameof(matrix));

            using (Mat T = ArrayToMat4x4(matrix))
            using (Mat R = T.SubMat(0, 3, 0, 3).Clone())
            using (Mat t = T.SubMat(0, 3, 3, 4).Clone())
            using (Mat rvec = new Mat())
            {
                Cv2.Rodrigues(R, rvec);
                return new double[]
                {
            rvec.At<double>(0, 0), rvec.At<double>(1, 0), rvec.At<double>(2, 0),
            t.At<double>(0, 0), t.At<double>(1, 0), t.At<double>(2, 0)
                };
            }
        }

        /// <summary>
        /// 6-DoF 파라미터 배열의 지정 offset에서 Rodrigues 회전벡터와 병진벡터를 읽어 4x4 Mat으로 변환합니다.
        /// </summary>
        /// <param name="p">[rx, ry, rz, tx, ty, tz] 파라미터 배열입니다.</param>
        /// <param name="offset">읽기 시작 위치입니다.</param>
        /// <returns>CV_64FC1 타입의 4x4 동차변환 Mat입니다. 호출자가 Dispose해야 합니다.</returns>
        public Mat ParamsToMatrixMat(double[] p, int offset)
        {
            if (offset < 0)
                throw new ArgumentOutOfRangeException(nameof(offset), offset, "offset은 0 이상이어야 합니다.");

            ValidateFiniteVector(p, offset + 6, nameof(p));

            using (Mat rvec = new Mat(3, 1, MatType.CV_64FC1, new double[] { p[offset + 0], p[offset + 1], p[offset + 2] }))
            using (Mat tvec = new Mat(3, 1, MatType.CV_64FC1, new double[] { p[offset + 3], p[offset + 4], p[offset + 5] }))
            using (Mat R = new Mat())
            {
                Cv2.Rodrigues(rvec, R);
                return Build4x4MatFromRt(R, tvec);
            }
        }

        /// <summary>
        /// 6-DoF 파라미터 배열을 4x4 동차변환 배열로 변환합니다.
        /// </summary>
        /// <param name="p">[rx, ry, rz, tx, ty, tz] 파라미터 배열입니다.</param>
        /// <returns>4x4 동차변환 배열입니다.</returns>
        public double[,] ParamsToMatrix(double[] p)
        {
            using (Mat mat = ParamsToMatrixMat(p))
            {
                return MatToArray4x4(mat);
            }
        }

        /// <summary>
        /// 카메라 내부 파라미터에서 OpenCV camera matrix K를 생성합니다.
        /// </summary>
        /// <param name="intr">fx, fy, ppx, ppy, distortion coefficients를 가진 내부 파라미터입니다.</param>
        /// <returns>CV_64FC1 타입의 3x3 camera matrix입니다. 호출자가 Dispose해야 합니다.</returns>
        public Mat BuildCameraMatrix(Intrinsic intr)
        {
            ValidateIntrinsic(intr);

            Mat cameraMatrix = Mat.Eye(3, 3, MatType.CV_64FC1);
            var idx = cameraMatrix.GetGenericIndexer<double>();

            idx[0, 0] = intr.fx;
            idx[0, 1] = 0;
            idx[0, 2] = intr.ppx;

            idx[1, 0] = 0;
            idx[1, 1] = intr.fy;
            idx[1, 2] = intr.ppy;

            idx[2, 0] = 0;
            idx[2, 1] = 0;
            idx[2, 2] = 1;

            return cameraMatrix;
        }

        /// <summary>
        /// 카메라 왜곡계수(k1, k2, p1, p2, k3)를 OpenCV 1x5 Mat으로 생성합니다.
        /// </summary>
        /// <param name="intr">왜곡계수 배열을 포함한 내부 파라미터입니다.</param>
        /// <returns>CV_64FC1 타입의 1x5 distortion coefficient Mat입니다. 호출자가 Dispose해야 합니다.</returns>
        public Mat BuildDistCoeffs(Intrinsic intr)
        {
            ValidateIntrinsic(intr);

            Mat distCoeffs = new Mat(1, 5, MatType.CV_64FC1);
            var idx = distCoeffs.GetGenericIndexer<double>();

            idx[0, 0] = intr.coeffs[0]; // k1
            idx[0, 1] = intr.coeffs[1]; // k2
            idx[0, 2] = intr.coeffs[2]; // p1
            idx[0, 3] = intr.coeffs[3]; // p2
            idx[0, 4] = intr.coeffs[4]; // k3

            return distCoeffs;
        }

        /// <summary>
        /// 6-DoF 파라미터 배열을 4x4 동차변환 Mat으로 변환합니다.
        /// </summary>
        /// <param name="p">[rx, ry, rz, tx, ty, tz] 파라미터 배열입니다.</param>
        /// <returns>CV_64FC1 타입의 4x4 동차변환 Mat입니다. 호출자가 Dispose해야 합니다.</returns>
        public Mat ParamsToMatrixMat(double[] p)
        {
            ValidateFiniteVector(p, 6, nameof(p));

            using (Mat rvec = new Mat(3, 1, MatType.CV_64FC1, new double[] { p[0], p[1], p[2] }))
            using (Mat tvec = new Mat(3, 1, MatType.CV_64FC1, new double[] { p[3], p[4], p[5] }))
            using (Mat R = new Mat())
            {
                Cv2.Rodrigues(rvec, R);
                return Build4x4MatFromRt(R, tvec);
            }
        }

        /// <summary>
        /// quaternion(w, x, y, z)을 제자리에서 단위 길이로 정규화합니다.
        /// </summary>
        /// <param name="q">정규화할 quaternion 배열입니다.</param>
        public void NormalizeQuaternion(double[] q)
        {
            ValidateFiniteVector(q, 4, nameof(q));

            double norm = Math.Sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
            if (norm <= 0.0 || double.IsNaN(norm) || double.IsInfinity(norm))
                throw new Exception("Quaternion 정규화 실패");

            for (int i = 0; i < 4; i++)
                q[i] /= norm;
        }

        /// <summary>
        /// 두 quaternion(w, x, y, z)의 내적을 계산합니다.
        /// </summary>
        /// <param name="a">첫 번째 quaternion입니다.</param>
        /// <param name="b">두 번째 quaternion입니다.</param>
        /// <returns>quaternion 내적입니다.</returns>
        public double QuaternionDot(double[] a, double[] b)
        {
            ValidateFiniteVector(a, 4, nameof(a));
            ValidateFiniteVector(b, 4, nameof(b));

            return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
        }

        /// <summary>
        /// 4x4 동차변환을 최적화용 6-DoF 파라미터로 변환하고 병진 성분에 OptimizerTranslationScale을 적용합니다.
        /// </summary>
        /// <param name="matrix">4x4 동차변환 배열입니다.</param>
        /// <returns>[rx, ry, rz, scaled tx, scaled ty, scaled tz] 파라미터입니다.</returns>
        public double[] MatrixToParamsScaled(double[,] matrix)
        {
            ValidateHomogeneousMatrix(matrix, nameof(matrix));
            HandEyeParams.ValidateOptimizerScale();

            using (Mat T = ArrayToMat4x4(matrix))
            using (Mat R = T.SubMat(0, 3, 0, 3).Clone())
            using (Mat t = T.SubMat(0, 3, 3, 4).Clone())
            using (Mat rvec = new Mat())
            {
                Cv2.Rodrigues(R, rvec);

                return new double[]
                {
            rvec.At<double>(0, 0),
            rvec.At<double>(1, 0),
            rvec.At<double>(2, 0),

            t.At<double>(0, 0) * OptimizerTranslationScale,
            t.At<double>(1, 0) * OptimizerTranslationScale,
            t.At<double>(2, 0) * OptimizerTranslationScale
                };
            }
        }

        /// <summary>
        /// 최적화 내부 병진 축소 계수입니다.
        /// </summary>
        public double OptimizerTranslationScale
        {
            get { return HandEyeParams.OptimizerTranslationScale; }
            set { HandEyeParams.OptimizerTranslationScale = value; }
        }

        /// <summary>
        /// 3x3 Mat을 double 배열로 변환합니다. 기존 호출 호환용 별칭입니다.
        /// </summary>
        /// <param name="mat">CV_64FC1 타입의 3x3 Mat입니다.</param>
        /// <returns>3x3 배열입니다.</returns>
        public double[,] Mat3x3ToArrayLocal(Mat mat)
        {
            return Mat3x3ToArray(mat);
        }

        /// <summary>
        /// 3x3 Mat을 double 배열로 변환하고 회전행렬 유효성을 검증합니다.
        /// </summary>
        /// <param name="mat">CV_64FC1 타입의 3x3 Mat입니다.</param>
        /// <returns>3x3 회전행렬 배열입니다.</returns>
        public double[,] Mat3x3ToArray(Mat mat)
        {
            ValidateRotationMat(mat, nameof(mat));

            double[,] array = new double[3, 3];
            var idx = mat.GetGenericIndexer<double>();

            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++)
                {
                    array[r, c] = idx[r, c];
                }
            }

            ValidateRotationMatrix(array, nameof(mat));
            return array;
        }

        /// <summary>
        /// 세 3x3 Mat의 곱 a*b*c를 계산합니다.
        /// </summary>
        /// <param name="a">첫 번째 3x3 행렬입니다.</param>
        /// <param name="b">두 번째 3x3 행렬입니다.</param>
        /// <param name="c">세 번째 3x3 행렬입니다.</param>
        /// <returns>CV_64FC1 타입의 3x3 곱 결과 Mat입니다. 호출자가 Dispose해야 합니다.</returns>
        public Mat Multiply3(Mat a, Mat b, Mat c)
        {
            ValidateRotationMat(a, nameof(a));
            ValidateRotationMat(b, nameof(b));
            ValidateRotationMat(c, nameof(c));

            using (Mat ab = a * b)
            {
                return ab * c;
            }
        }

        /// <summary>
        /// 최적화용 scaled 6-DoF 파라미터를 4x4 동차변환 Mat으로 복원합니다.
        /// </summary>
        /// <param name="p">scaled 파라미터 배열입니다.</param>
        /// <param name="offset">읽기 시작 위치입니다.</param>
        /// <returns>CV_64FC1 타입의 4x4 동차변환 Mat입니다. 호출자가 Dispose해야 합니다.</returns>
        public Mat ParamsToMatrixMatScaled(double[] p, int offset)
        {
            if (offset < 0)
                throw new ArgumentOutOfRangeException(nameof(offset), offset, "offset은 0 이상이어야 합니다.");

            ValidateFiniteVector(p, offset + 6, nameof(p));

            HandEyeParams.ValidateOptimizerScale();

            using (Mat rvec = new Mat(3, 1, MatType.CV_64FC1, new double[]
            {
        p[offset + 0],
        p[offset + 1],
        p[offset + 2]
            }))
            using (Mat tvec = new Mat(3, 1, MatType.CV_64FC1, new double[]
            {
        p[offset + 3] * OptimizerTranslationRestore,
        p[offset + 4] * OptimizerTranslationRestore,
        p[offset + 5] * OptimizerTranslationRestore
            }))
            using (Mat R = new Mat())
            {
                Cv2.Rodrigues(rvec, R);
                return Build4x4MatFromRt(R, tvec);
            }
        }

        /// <summary>
        /// Rodrigues 회전벡터를 3x3 회전행렬 Mat으로 변환합니다.
        /// </summary>
        /// <param name="rx">회전벡터 X 성분입니다.</param>
        /// <param name="ry">회전벡터 Y 성분입니다.</param>
        /// <param name="rz">회전벡터 Z 성분입니다.</param>
        /// <param name="degree">true이면 입력을 degree로 보고 radian으로 변환합니다.</param>
        /// <returns>CV_64FC1 타입의 3x3 회전행렬 Mat입니다. 호출자가 Dispose해야 합니다.</returns>
        public Mat RotVecToMatrix(double rx, double ry, double rz, bool degree)
        {
            ValidateFinite(rx, nameof(rx));
            ValidateFinite(ry, nameof(ry));
            ValidateFinite(rz, nameof(rz));

            double scale = degree ? Math.PI / 180.0 : 1.0;

            using (Mat rvec = new Mat(3, 1, MatType.CV_64FC1, new double[]
            {
        rx * scale,
        ry * scale,
        rz * scale
            }))
            {
                Mat R = new Mat();
                Cv2.Rodrigues(rvec, R);
                return R;
            }
        }

        /// <summary>
        /// 단일 축 X/Y/Z에 대한 기본 회전행렬을 생성합니다.
        /// </summary>
        /// <param name="axis">회전축 문자 X, Y, Z입니다.</param>
        /// <param name="rad">회전각(rad)입니다.</param>
        /// <returns>CV_64FC1 타입의 3x3 회전행렬 Mat입니다. 호출자가 Dispose해야 합니다.</returns>
        public Mat AxisRotation(char axis, double rad)
        {
            ValidateFinite(rad, nameof(rad));
            axis = char.ToUpperInvariant(axis);

            double c = Math.Cos(rad);
            double s = Math.Sin(rad);

            switch (axis)
            {
                case 'X':
                    return new Mat(3, 3, MatType.CV_64FC1, new double[]
                    {
                1, 0, 0,
                0, c, -s,
                0, s, c
                    });

                case 'Y':
                    return new Mat(3, 3, MatType.CV_64FC1, new double[]
                    {
                c, 0, s,
                0, 1, 0,
                -s, 0, c
                    });

                case 'Z':
                    return new Mat(3, 3, MatType.CV_64FC1, new double[]
                    {
                c, -s, 0,
                s, c, 0,
                0, 0, 1
                    });

                default:
                    throw new ArgumentException($"지원하지 않는 회전축입니다: {axis}");
            }
        }

        /// <summary>
        /// 각도 단위 설정에 따라 입력 각도를 radian으로 변환합니다.
        /// </summary>
        /// <param name="angle">입력 각도입니다.</param>
        /// <param name="unit">입력 각도 단위입니다.</param>
        /// <returns>radian 단위 각도입니다.</returns>
        public double ToRadians(double angle, EAngleUnit unit)
        {
            ValidateFinite(angle, nameof(angle));

            switch (unit)
            {
                case EAngleUnit.Degree:
                    return angle * Math.PI / 180.0;
                case EAngleUnit.Radian:
                    return angle;
                default:
                    throw new ArgumentOutOfRangeException(nameof(unit), unit, "지원하지 않는 각도 단위입니다.");
            }
        }

        /// <summary>
        /// 최적화 결과 병진 복원 계수입니다.
        /// </summary>
        public double OptimizerTranslationRestore
        {
            get { return HandEyeParams.OptimizerTranslationRestore; }
            set { HandEyeParams.OptimizerTranslationRestore = value; }
        }

        /// <summary>
        /// 4x4 동차변환 배열의 크기, 유한값, 마지막 행, 회전 블록의 정규성을 검증합니다.
        /// </summary>
        /// <param name="matrix">검증할 4x4 동차변환 배열입니다.</param>
        /// <param name="name">예외 메시지에 표시할 인자 이름입니다.</param>
        public static void ValidateHomogeneousMatrix(double[,] matrix, string name)
        {
            if (!IsFiniteMatrix(matrix, 4, 4))
                throw new ArgumentException($"{name} 4x4 행렬이 null이거나 NaN/Infinity를 포함합니다.");

            const double eps = 1e-9;
            if (Math.Abs(matrix[3, 0]) > eps ||
                Math.Abs(matrix[3, 1]) > eps ||
                Math.Abs(matrix[3, 2]) > eps ||
                Math.Abs(matrix[3, 3] - 1.0) > eps)
            {
                throw new ArgumentException(
                    $"{name} 4x4 행렬의 마지막 행이 동차 변환 형식이 아닙니다. " +
                    $"LastRow=({matrix[3, 0]}, {matrix[3, 1]}, {matrix[3, 2]}, {matrix[3, 3]})");
            }

            ValidateRotationMatrix(matrix, name);
        }

        /// <summary>
        /// 카메라 내부 파라미터가 OpenCV 투영 함수에 사용할 수 있는 값인지 검증합니다.
        /// </summary>
        /// <param name="intr">검증할 카메라 내부 파라미터입니다.</param>
        public static void ValidateIntrinsic(Intrinsic intr)
        {
            if (intr == null)
                throw new ArgumentNullException(nameof(intr));

            ValidateFinite(intr.fx, nameof(intr.fx));
            ValidateFinite(intr.fy, nameof(intr.fy));
            ValidateFinite(intr.ppx, nameof(intr.ppx));
            ValidateFinite(intr.ppy, nameof(intr.ppy));

            if (intr.fx <= 0.0 || intr.fy <= 0.0)
                throw new ArgumentException($"카메라 focal length가 올바르지 않습니다. fx={intr.fx}, fy={intr.fy}");

            if (intr.width < 0.0 || intr.height < 0.0 ||
                !IsFinite(intr.width) || !IsFinite(intr.height))
            {
                throw new ArgumentException($"카메라 이미지 크기가 올바르지 않습니다. width={intr.width}, height={intr.height}");
            }

            if (intr.coeffs == null || intr.coeffs.Length < 5)
                throw new ArgumentException("카메라 왜곡계수는 최소 5개(k1, k2, p1, p2, k3)가 필요합니다.");

            for (int i = 0; i < 5; i++)
            {
                if (!IsFinite(intr.coeffs[i]))
                    throw new ArgumentException($"카메라 왜곡계수에 NaN/Infinity가 포함되어 있습니다. Index={i}");
            }
        }

        /// <summary>
        /// 3x3 회전행렬 또는 4x4 동차변환의 좌상단 회전 블록이 SO(3)에 가까운지 검증합니다.
        /// </summary>
        /// <param name="matrix">검증할 행렬입니다.</param>
        /// <param name="name">예외 메시지에 표시할 인자 이름입니다.</param>
        public static void ValidateRotationMatrix(double[,] matrix, string name)
        {
            if (matrix == null || matrix.GetLength(0) < 3 || matrix.GetLength(1) < 3)
                throw new ArgumentException($"{name} 회전행렬은 최소 3x3 크기여야 합니다.");

            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++)
                {
                    if (!IsFinite(matrix[r, c]))
                        throw new ArgumentException($"{name} 회전행렬에 NaN/Infinity가 포함되어 있습니다.");
                }
            }

            for (int row = 0; row < 3; row++)
            {
                double normSq = 0.0;
                for (int c = 0; c < 3; c++)
                    normSq += matrix[row, c] * matrix[row, c];

                if (Math.Abs(normSq - 1.0) > RotationValidationTolerance)
                    throw new ArgumentException($"{name} 회전행렬의 {row}행 norm이 1이 아닙니다. NormSq={normSq}");
            }

            for (int r1 = 0; r1 < 3; r1++)
            {
                for (int r2 = r1 + 1; r2 < 3; r2++)
                {
                    double dot = 0.0;
                    for (int c = 0; c < 3; c++)
                        dot += matrix[r1, c] * matrix[r2, c];

                    if (Math.Abs(dot) > RotationValidationTolerance)
                        throw new ArgumentException($"{name} 회전행렬의 행들이 직교하지 않습니다. Dot({r1},{r2})={dot}");
                }
            }

            double det = Determinant3x3(matrix);
            if (Math.Abs(det - 1.0) > RotationValidationTolerance)
                throw new ArgumentException($"{name} 회전행렬 determinant가 +1이 아닙니다. Det={det}");
        }

        private static bool IsFiniteMatrix(double[,] matrix, int rows, int cols)
        {
            if (matrix == null ||
                matrix.GetLength(0) != rows ||
                matrix.GetLength(1) != cols)
            {
                return false;
            }

            for (int r = 0; r < rows; r++)
            {
                for (int c = 0; c < cols; c++)
                {
                    if (!IsFinite(matrix[r, c]))
                        return false;
                }
            }

            return true;
        }

        private static void ValidateFiniteArray(double[,] matrix, int rows, int cols, string name)
        {
            if (!IsFiniteMatrix(matrix, rows, cols))
                throw new ArgumentException($"{name} 행렬은 {rows}x{cols} 크기여야 하며 NaN/Infinity를 포함할 수 없습니다.");
        }

        private static void ValidateMatShape(Mat mat, int rows, int cols, string name)
        {
            if (mat == null || mat.Empty() || mat.Rows != rows || mat.Cols != cols)
                throw new ArgumentException($"{name} Mat은 {rows}x{cols} 크기여야 합니다.");

            if (mat.Type() != MatType.CV_64FC1)
                throw new ArgumentException($"{name} Mat은 CV_64FC1 타입이어야 합니다. Type={mat.Type()}");
        }

        private static void ValidateRotationMat(Mat mat, string name)
        {
            ValidateMatShape(mat, 3, 3, name);

            double[,] arr = new double[3, 3];
            var idx = mat.GetGenericIndexer<double>();
            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++)
                    arr[r, c] = idx[r, c];
            }

            ValidateRotationMatrix(arr, name);
        }

        private static void ValidateTranslationMat(Mat mat, string name)
        {
            ValidateMatShape(mat, 3, 1, name);

            var idx = mat.GetGenericIndexer<double>();
            for (int r = 0; r < 3; r++)
            {
                if (!IsFinite(idx[r, 0]))
                    throw new ArgumentException($"{name} 병진벡터에 NaN/Infinity가 포함되어 있습니다.");
            }
        }

        private static void ValidateFiniteVector(double[] values, int requiredLength, string name)
        {
            if (values == null || values.Length < requiredLength)
                throw new ArgumentException($"{name} 배열은 최소 {requiredLength}개 요소가 필요합니다.");

            for (int i = 0; i < requiredLength; i++)
            {
                if (!IsFinite(values[i]))
                    throw new ArgumentException($"{name} 배열에 NaN/Infinity가 포함되어 있습니다. Index={i}");
            }
        }

        private static void ValidateFinite(double value, string name)
        {
            if (!IsFinite(value))
                throw new ArgumentException($"{name} 값이 NaN/Infinity입니다.");
        }

        private static double Determinant3x3(double[,] m)
        {
            return
                m[0, 0] * (m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1]) -
                m[0, 1] * (m[1, 0] * m[2, 2] - m[1, 2] * m[2, 0]) +
                m[0, 2] * (m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]);
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}
