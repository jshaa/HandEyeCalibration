using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Calibration_test
{
    public class CSV_Helper
    {
        public void Save_csv_Effector(List<UIEndEffectorData> dataList, string filePath)
        {
            if (dataList == null || dataList.Count == 0)
            {
                Console.WriteLine("저장할 데이터가 없습니다.");
                return;
            }

            try
            {
                // 엑셀에서 열 때 문자열 깨짐을 방지하기 위해 UTF8(BOM 포함) 인코딩 사용
                using (StreamWriter sw = new StreamWriter(filePath, false, new UTF8Encoding(true)))
                {
                    // 1. CSV 헤더(첫 줄) 작성
                    StringBuilder header = new StringBuilder();
                    header.Append("PoseIndex,X,Y,Z");

                    // 4x4 행렬의 각 요소 이름(M00 ~ M33)을 헤더에 추가
                    for (int r = 0; r < 4; r++)
                    {
                        for (int c = 0; c < 4; c++)
                        {
                            header.Append($",M{r}{c}");
                        }
                    }
                    sw.WriteLine(header.ToString());

                    // 2. 리스트 데이터 순회 및 작성
                    foreach (var data in dataList)
                    {
                        StringBuilder row = new StringBuilder();

                        // 기본 데이터 작성 - CSV는 문화권에 따라 소수점이 콤마로 바뀌면 깨질 수 있으므로 InvariantCulture를 사용합니다.
                        row.Append(data.PoseIndex.ToString(CultureInfo.InvariantCulture));
                        row.Append(',');
                        row.Append(data.X.ToString("G17", CultureInfo.InvariantCulture));
                        row.Append(',');
                        row.Append(data.Y.ToString("G17", CultureInfo.InvariantCulture));
                        row.Append(',');
                        row.Append(data.Z.ToString("G17", CultureInfo.InvariantCulture));

                        // 4x4 배열 데이터를 가로(Row) 방향으로 평탄화하여 쉼표로 연결
                        if (data.TransformMatrix4x4 != null)
                        {
                            for (int r = 0; r < 4; r++)
                            {
                                for (int c = 0; c < 4; c++)
                                {
                                    row.Append(',');
                                    row.Append(data.TransformMatrix4x4[r, c].ToString("G17", CultureInfo.InvariantCulture));
                                }
                            }
                        }
                        else
                        {
                            // 만약 Matrix 데이터가 비어있다면 빈 칸(쉼표) 16개 추가
                            row.Append(',', 16);
                        }

                        sw.WriteLine(row.ToString());
                    }
                }
                Console.WriteLine($"CSV 파일 저장 성공: {filePath}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"CSV 저장 중 오류 발생: {ex.Message}");
                throw; // UI 단으로 에러를 던져서 팝업창 등을 띄울 수 있도록 함
            }
        }

        public List<UIEndEffectorData> Load_csv_Effector(string filePath)
        {
            List<UIEndEffectorData> loadedDataList = new List<UIEndEffectorData>();

            if (!File.Exists(filePath))
            {
                Console.WriteLine("지정된 파일을 찾을 수 없습니다.");
                return loadedDataList;
            }

            try
            {
                // 저장할 때와 동일하게 UTF8 인코딩으로 읽기
                using (StreamReader sr = new StreamReader(filePath, Encoding.UTF8))
                {
                    // 첫 줄(헤더)은 건너뛰기
                    string header = sr.ReadLine();

                    string line;
                    while ((line = sr.ReadLine()) != null)
                    {
                        // 빈 줄 무시
                        if (string.IsNullOrWhiteSpace(line))
                            continue;

                        string[] values = line.Split(',');

                        // 데이터 개수가 맞는지 검증 (PoseIndex, X, Y, Z + 16개 Matrix 값 = 총 20개)
                        if (values.Length < 20)
                        {
                            Console.WriteLine("CSV 행의 데이터 형식이 올바르지 않아 건너뜁니다.");
                            continue;
                        }

                        UIEndEffectorData data = new UIEndEffectorData();

                        // 기본 좌표 데이터 복원
                        data.PoseIndex = int.Parse(values[0], CultureInfo.InvariantCulture);
                        data.X = double.Parse(values[1], CultureInfo.InvariantCulture);
                        data.Y = double.Parse(values[2], CultureInfo.InvariantCulture);
                        data.Z = double.Parse(values[3], CultureInfo.InvariantCulture);

                        // 4x4 동차 변환 행렬 복원
                        double[,] matrix = new double[4, 4];
                        int valueIndex = 4; // Matrix 데이터는 5번째 열(인덱스 4)부터 시작

                        for (int r = 0; r < 4; r++)
                        {
                            for (int c = 0; c < 4; c++)
                            {
                                // 빈 칸 처리 (만약 빈칸으로 저장되었을 경우 0으로 처리)
                                if (string.IsNullOrWhiteSpace(values[valueIndex]))
                                {
                                    matrix[r, c] = 0.0;
                                }
                                else
                                {
                                    matrix[r, c] = double.Parse(values[valueIndex], CultureInfo.InvariantCulture);
                                }
                                valueIndex++;
                            }
                        }

                        data.TransformMatrix4x4 = matrix;
                        loadedDataList.Add(data);
                    }
                }

                Console.WriteLine($"CSV 파일 로드 성공: 총 {loadedDataList.Count}개의 포즈 데이터를 불러왔습니다.");
                return loadedDataList;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"CSV 로드 중 오류 발생: {ex.Message}");
                throw;
            }
        }

        public void Save_csv_CAM(List<CameraTargetPose> poselist, string filePath)
        {
            if (poselist == null || poselist.Count == 0)
            {
                Console.WriteLine("저장할 데이터가 없습니다.");
                return;
            }

            try
            {
                using (StreamWriter sw = new StreamWriter(filePath, false, new UTF8Encoding(true)))
                {
                    // CSV 헤더 작성 (X, Y, Z, Rx, Ry, Rz 추가)
                    StringBuilder header = new StringBuilder();
                    header.Append("PoseIndex,X,Y,Z,Rx,Ry,Rz");

                    for (int r = 0; r < 3; r++)
                    {
                        for (int c = 0; c < 3; c++)
                        {
                            header.Append($",M{r}{c}");
                        }
                    }
                    sw.WriteLine(header.ToString());

                    // 리스트 데이터 순회 및 작성
                    for (int i = 0; i < poselist.Count; i++)
                    {
                        var data = poselist[i];
                        StringBuilder row = new StringBuilder();

                        // 기본 6자유도 데이터 작성
                        int poseIndex = data.SourcePoseIndex > 0 ? data.SourcePoseIndex : i + 1;
                        row.Append(poseIndex.ToString(CultureInfo.InvariantCulture));
                        row.Append(',');
                        row.Append(data.X.ToString("G17", CultureInfo.InvariantCulture));
                        row.Append(',');
                        row.Append(data.Y.ToString("G17", CultureInfo.InvariantCulture));
                        row.Append(',');
                        row.Append(data.Z.ToString("G17", CultureInfo.InvariantCulture));
                        row.Append(',');
                        row.Append(data.Rx.ToString("G17", CultureInfo.InvariantCulture));
                        row.Append(',');
                        row.Append(data.Ry.ToString("G17", CultureInfo.InvariantCulture));
                        row.Append(',');
                        row.Append(data.Rz.ToString("G17", CultureInfo.InvariantCulture));

                        // 3x3 회전 행렬 배열 데이터를 가로(Row) 방향으로 평탄화하여 쉼표로 연결
                        if (data.RotationMatrix != null)
                        {
                            for (int r = 0; r < 3; r++)
                            {
                                for (int c = 0; c < 3; c++)
                                {
                                    row.Append(',');
                                    row.Append(data.RotationMatrix[r, c].ToString("G17", CultureInfo.InvariantCulture));
                                }
                            }
                        }
                        else
                        {
                            // 행렬 데이터가 비어있다면 빈 칸(쉼표) 9개 추가
                            row.Append(',', 9);
                        }

                        sw.WriteLine(row.ToString());
                    }
                }

                Console.WriteLine($"카메라 좌표 CSV 파일 저장 성공: {filePath}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"CSV 저장 중 오류 발생: {ex.Message}");
                throw;
            }
        }

        public List<RobotTargetPose> LoadRobotPoseCsv(string csvPath)
        {
            try
            {
                List<RobotTargetPose> poseList = new List<RobotTargetPose>();

                if (!File.Exists(csvPath))
                    throw new FileNotFoundException("Robot Pose CSV 파일을 찾을 수 없습니다.", csvPath);

                string[] lines = File.ReadAllLines(csvPath, Encoding.UTF8);

                bool headerChecked = false;
                bool hasPoseIndexColumn = false;

                for (int i = 0; i < lines.Length; i++)
                {
                    string line = lines[i].Trim();

                    if (string.IsNullOrWhiteSpace(line))
                        continue;

                    if (line.StartsWith("#"))
                        continue;

                    string[] tokens = line.Split(',').Select(t => t.Trim()).ToArray();

                    if (!headerChecked)
                    {
                        headerChecked = true;
                        string firstTokenLower = tokens.Length > 0 ? tokens[0].ToLowerInvariant() : string.Empty;

                        if (firstTokenLower.Contains("pose") ||
                            firstTokenLower.Contains("index") ||
                            firstTokenLower == "no" ||
                            firstTokenLower == "idx")
                        {
                            hasPoseIndexColumn = true;
                            Console.WriteLine($"CSV {i + 1}번째 줄을 헤더로 판단하여 건너뜁니다. line = {line}");
                            continue;
                        }
                    }

                    if (tokens.Length < 6)
                    {
                        Console.WriteLine($"CSV {i + 1}번째 줄의 데이터 개수가 부족하여 건너뜁니다. line = {line}");
                        continue;
                    }

                    double tx, ty, tz, rx, ry, rz;
                    int sourcePoseIndex = poseList.Count + 1;
                    bool hasExplicitPoseIndex = false;

                    int startIndex = hasPoseIndexColumn ? 1 : 0;

                    // 헤더가 없는 CSV라도 PoseIndex,X,Y,Z,Rx,Ry,Rz 형식을 우선 허용합니다.
                    // 첫 pose 번호가 1이 아닌 경우(예: 이미지 #2부터 시작)에도 7열 파일을 잘못 6열로 읽지 않도록
                    // 첫 번째 토큰이 정수이고 1~6열이 정상 pose이면 PoseIndex 열로 판단합니다.
                    if (!hasPoseIndexColumn && tokens.Length >= 7)
                    {
                        double poseIndexValue;
                        bool firstTokenIsInteger =
                            double.TryParse(tokens[0], NumberStyles.Float, CultureInfo.InvariantCulture, out poseIndexValue) &&
                            Math.Abs(poseIndexValue - Math.Round(poseIndexValue)) < 1e-9;

                        double tx1, ty1, tz1, rx1, ry1, rz1;
                        bool parseAsIndexedPose = firstTokenIsInteger &&
                            TryParseRobotPoseTokens(tokens, 1, out tx1, out ty1, out tz1, out rx1, out ry1, out rz1);

                        if (parseAsIndexedPose)
                        {
                            startIndex = 1;
                            sourcePoseIndex = (int)Math.Round(poseIndexValue);
                            hasExplicitPoseIndex = true;
                        }
                    }

                    bool parseOk = TryParseRobotPoseTokens(
                        tokens, startIndex,
                        out tx, out ty, out tz, out rx, out ry, out rz);

                    if (parseOk && startIndex == 1)
                    {
                        double poseIndexValue;
                        if (double.TryParse(tokens[0], NumberStyles.Float, CultureInfo.InvariantCulture, out poseIndexValue) &&
                            Math.Abs(poseIndexValue - Math.Round(poseIndexValue)) < 1e-9)
                        {
                            sourcePoseIndex = (int)Math.Round(poseIndexValue);
                            hasExplicitPoseIndex = true;
                        }
                    }

                    if (!parseOk)
                    {
                        if (poseList.Count == 0)
                        {
                            Console.WriteLine($"CSV {i + 1}번째 줄을 헤더 또는 비수치 행으로 판단하여 건너뜁니다. line = {line}");
                            continue;
                        }

                        Console.WriteLine($"CSV {i + 1}번째 줄 파싱 실패로 건너뜁니다. line = {line}");
                        continue;
                    }

                    RobotTargetPose pose = new RobotTargetPose
                    {
                        robotTx = tx,
                        robotTy = ty,
                        robotTz = tz,
                        robotRx = rx,
                        robotRy = ry,
                        robotRz = rz,
                        SourcePoseIndex = sourcePoseIndex,
                        HasExplicitPoseIndex = hasExplicitPoseIndex
                    };

                    poseList.Add(pose);

                    //Console.WriteLine($"Robot Pose {i + 1} 개 저장");
                }
                return poseList;

            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
                return null;
            }
        }

        public void SaveAcceptedHandEyeResultCsv(string filePath, HandEyeParams HandEyeParams, HandEyeValidationResult bestValidationResult, double[,] bestHandEyeMatrix, double[,] bestOutputHandEyeMatrix)
        {
            if (string.IsNullOrWhiteSpace(filePath))
                throw new ArgumentException("저장 경로가 비어 있습니다.", nameof(filePath));

            using (StreamWriter sw = new StreamWriter(filePath, false, new System.Text.UTF8Encoding(true)))
            {
                StringBuilder header = new StringBuilder();
                header.Append("Kind");
                for (int r = 0; r < 4; r++)
                {
                    for (int c = 0; c < 4; c++)
                        header.Append($",M{r}{c}");
                }
                header.Append(",Tx,Ty,Tz");
                sw.WriteLine(header.ToString());

                AppendMatrixCsvRow(sw, "RawAcceptedHandEye", bestHandEyeMatrix);
                AppendMatrixCsvRow(sw, "OutputAcceptedHandEye", bestOutputHandEyeMatrix);

                if (HandEyeParams != null && HandEyeParams.LastTarget2Gripper != null)
                    AppendMatrixCsvRow(sw, "OptimizedTarget2Gripper", HandEyeParams.LastTarget2Gripper);

                sw.WriteLine();
                sw.WriteLine("Metric,Value");
                if (HandEyeParams != null)
                {
                    sw.WriteLine("NonlinearAccepted," + HandEyeParams.LastNonlinearAccepted.ToString());
                    sw.WriteLine("NonlinearReprojectionRmsPx," + HandEyeParams.LastNonlinearReprojectionRmsPx.ToString("G17", CultureInfo.InvariantCulture));
                    sw.WriteLine("MaxAcceptReprojectionRmsPx," + HandEyeParams.MaxAcceptReprojectionRmsPx.ToString("G17", CultureInfo.InvariantCulture));
                    sw.WriteLine("Consistency3DReprojectionLimitPx," + HandEyeParams.Simultaneous12Dof3DRefineMaxReprojectionRmsPx.ToString("G17", CultureInfo.InvariantCulture));
                }

                if (bestValidationResult != null)
                {
                    sw.WriteLine("ValidationRmsMm," + bestValidationResult.RmsError.ToString("G17", CultureInfo.InvariantCulture));
                    sw.WriteLine("ValidationMaxMm," + bestValidationResult.MaxError.ToString("G17", CultureInfo.InvariantCulture));
                }
            }
        }

        private void AppendMatrixCsvRow(StreamWriter sw, string kind, double[,] matrix)
        {
            if (sw == null)
                throw new ArgumentNullException(nameof(sw));

            StringBuilder row = new StringBuilder();
            row.Append(kind ?? string.Empty);

            if (matrix != null && matrix.GetLength(0) == 4 && matrix.GetLength(1) == 4)
            {
                for (int r = 0; r < 4; r++)
                {
                    for (int c = 0; c < 4; c++)
                    {
                        row.Append(',');
                        row.Append(matrix[r, c].ToString("G17", System.Globalization.CultureInfo.InvariantCulture));
                    }
                }

                row.Append(',');
                row.Append(matrix[0, 3].ToString("G17", System.Globalization.CultureInfo.InvariantCulture));
                row.Append(',');
                row.Append(matrix[1, 3].ToString("G17", System.Globalization.CultureInfo.InvariantCulture));
                row.Append(',');
                row.Append(matrix[2, 3].ToString("G17", System.Globalization.CultureInfo.InvariantCulture));
            }
            else
            {
                // M00~M33 + Tx/Ty/Tz = 19 columns
                for (int i = 0; i < 19; i++)
                    row.Append(',');
            }

            sw.WriteLine(row.ToString());
        }

        private bool TryParseRobotPoseTokens(
            string[] tokens,
            int startIndex,
            out double tx,
            out double ty,
            out double tz,
            out double rx,
            out double ry,
            out double rz)
        {
            tx = ty = tz = rx = ry = rz = 0.0;

            if (tokens == null || tokens.Length < startIndex + 6)
                return false;

            return
                double.TryParse(tokens[startIndex + 0].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out tx) &&
                double.TryParse(tokens[startIndex + 1].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out ty) &&
                double.TryParse(tokens[startIndex + 2].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out tz) &&
                double.TryParse(tokens[startIndex + 3].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out rx) &&
                double.TryParse(tokens[startIndex + 4].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out ry) &&
                double.TryParse(tokens[startIndex + 5].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out rz);
        }
    }
}
