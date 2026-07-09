using OpenCvSharp;
using OpenCvSharp.Extensions;
using System;
using System.Collections.Generic;
using System.Data;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using System.Text.RegularExpressions;
using System.Windows.Forms;
using System.Windows.Forms.Integration;
using System.Text;

namespace Calibration_test
{
    public partial class Form1 : Form
    {
        HandEyeCalibration hand_Eye_Calibration = new HandEyeCalibration();
        CSV_Helper CSV_Helper = new CSV_Helper();

        private HandEyeSweepResult bestHandEyeResult = null;
        private HandEyeValidationResult bestValidationResult = null;
        private List<int> bestExcludedPoseNumbers = new List<int>();
        private double[,] bestHandEyeMatrix = null;
        private double[,] bestOutputHandEyeMatrix = null;

        // 캘리브레이션 실행 당시의 설정 스냅샷입니다.
        // UI 콤보박스를 나중에 변경해도 저장/Viewer 출력이 이전 결과와 섞이지 않도록 별도로 보관합니다.
        private HandEyeMode bestResultMode = HandEyeMode.EyeToHand;
        private EEulerSequence bestResultEulerSequence = EEulerSequence.Intrinsic_ZYZ;
        private EulerMatrixConvention bestResultEulerConvention = EulerMatrixConvention.DefaultActive;
        private RobotAngleInputOrder bestResultAngleInputOrder = RobotAngleInputOrder.RxRyRz;
        private RobotPoseTransformDirection bestResultRobotPoseDirection = RobotPoseTransformDirection.GripperToBase;
        private EndEffectorHandedness bestResultHandedness = EndEffectorHandedness.Right;
        private HandEyeCalibrationMethod bestResultMethod = HandEyeCalibrationMethod.ANDREFF;
        private OptimizationMethod bestResultOptimizationMethod = OptimizationMethod.Nonlinear_Reprojection;

        ElementHost ElementHost;
        Viewer viewer;

        Intrinsic intrinsic_data;

        List<RobotTargetPose> r_pose_list = new List<RobotTargetPose>();
        List<CameraTargetPose> c_pose_list = new List<CameraTargetPose>();

        HandEyeParams HandEyeParams;
        BoardDetector BoardDetector;

        string[] filepath = null;

        public Form1()
        {
            InitializeComponent();

            SET_PARAMS();

            INIT_ElementHosts();

            INIT_CalibrationComboBoxes();

            intrinsic_init();
        }

        private void SET_PARAMS()
        {
            HandEyeParams = hand_Eye_Calibration.Params;

            BoardDetector = new BoardDetector(HandEyeParams, hand_Eye_Calibration.Utils);
        }

        private void EnsureParams()
        {
            if (HandEyeParams == null)
                HandEyeParams = hand_Eye_Calibration.Params;
        }

        private bool ShouldRunNonlinearReprojection()
        {
            EnsureParams();
            return HandEyeParams.ShouldRunNonlinearReprojection;
        }

        private void ShowMessageSafe(string message)
        {
            if (IsDisposed)
                return;

            if (InvokeRequired && IsHandleCreated)
            {
                BeginInvoke(new Action(() => System.Windows.Forms.MessageBox.Show(this, message)));
                return;
            }

            System.Windows.Forms.MessageBox.Show(this, message);
        }

        

        private void INIT_CalibrationComboBoxes()
        {
            comboBox1.Items.Clear();
            comboBox2.Items.Clear();
            comboBox3.Items.Clear();
            comboBox4.Items.Clear();
            comboBox5.Items.Clear();
            comboBox6.Items.Clear();

            foreach (string name in Enum.GetNames(typeof(EEulerSequence)))
                comboBox1.Items.Add(name);

            foreach (string name in Enum.GetNames(typeof(HandEyeCalibrationMethod)))
                comboBox2.Items.Add(name);

            foreach (string name in Enum.GetNames(typeof(HandEyeMode)))
                comboBox3.Items.Add(name);

            foreach (string name in Enum.GetNames(typeof(EndEffectorHandedness)))
                comboBox4.Items.Add(name);

            foreach (string name in Enum.GetNames(typeof(CoordianteConvert)))
                comboBox5.Items.Add(name);

            foreach (string name in Enum.GetNames(typeof(OptimizationMethod)))
                comboBox6.Items.Add(name);

            comboBox1.SelectedItem = EEulerSequence.Intrinsic_ZYZ.ToString();
            comboBox2.SelectedItem = HandEyeCalibrationMethod.ANDREFF.ToString();
            comboBox3.SelectedItem = HandEyeMode.EyeToHand.ToString();
            comboBox4.SelectedItem = EndEffectorHandedness.Right.ToString();
            comboBox5.SelectedItem = CoordianteConvert.withoutOffset.ToString();
            comboBox6.SelectedItem = OptimizationMethod.Nonlinear_Reprojection.ToString();

            ApplySelectedCalibrationSettings();
        }
        private void ApplySelectedCalibrationSettings()
        {
            EnsureParams();

            if (comboBox1.SelectedItem != null &&
                Enum.TryParse(comboBox1.SelectedItem.ToString(), out EEulerSequence sequence))
            {
                HandEyeParams.RobotEulerSequence = sequence;
            }

            // 1. TSAI      2.PARK      3.HORAUD      4.ANDREFF      5.DANIILIDIS
            if (comboBox2.SelectedItem != null &&
                Enum.TryParse(comboBox2.SelectedItem.ToString(), out HandEyeCalibrationMethod method))
            {
                HandEyeParams.CalibrationMethod = method;
            }

            // 1. EyeToHand     2. EyeInHand
            if (comboBox3.SelectedItem != null &&
                Enum.TryParse(comboBox3.SelectedItem.ToString(), out HandEyeMode mode))
            {
                HandEyeParams.mode = mode;
            }

            // 1. Right     2. Left
            if (comboBox4.SelectedItem != null &&
                Enum.TryParse(comboBox4.SelectedItem.ToString(), out EndEffectorHandedness handedness))
            {
                HandEyeParams.Handedness = handedness;
            }

            // 1. withCameraOffset      2. withBaseOffset       3. withoutOffset
            if (comboBox5.SelectedItem != null &&
                Enum.TryParse(comboBox5.SelectedItem.ToString(), out CoordianteConvert convert))
            {
                HandEyeParams.CoordConvertMode = convert;
            }

            if (comboBox6.SelectedItem != null &&
                Enum.TryParse(comboBox6.SelectedItem.ToString(), out OptimizationMethod O_method))
            {
                HandEyeParams.Optimization = O_method;
            }
        }

        private void INIT_ElementHosts()
        {
            viewer = new Viewer();

            ElementHost = new ElementHost { Dock = DockStyle.Fill };
            ElementHost.Child = viewer;
            panel1.Controls.Add(ElementHost);
        }
        private void ClearBestCalibrationResult()
        {
            bestHandEyeResult = null;
            bestValidationResult = null;
            bestHandEyeMatrix = null;
            bestOutputHandEyeMatrix = null;
            bestExcludedPoseNumbers.Clear();
        }

        private void intrinsic_init()
        {
            EnsureParams();

            double Focal_Length_X = 1551.017;
            double Focal_Length_Y = 1550.621;
            double Principal_Point_X = 691.5576;
            double Principal_Point_Y = 556.4161;
            double K1 = -0.3530728;
            double K2 = 0.1943659;
            double P1 = 8.996115E-05;
            double P2 = -0.0001524744;
            double K3 = -0.07252458;

            intrinsic_data = new Intrinsic();

            // 현재 캘리브레이션 TIF 이미지 해상도 기준입니다.
            // CameraMatrix 자체에는 width/height가 직접 들어가지 않지만,
            // 로그/검증/향후 해상도 검증에서 기준값으로 사용하기 위해 명시합니다.
            intrinsic_data.width = 1464;
            intrinsic_data.height = 1096;

            intrinsic_data.fx = Focal_Length_X;
            intrinsic_data.fy = Focal_Length_Y;
            intrinsic_data.ppx = Principal_Point_X;
            intrinsic_data.ppy = Principal_Point_Y;
            intrinsic_data.coeffs = new double[] { K1, K2, P1, P2, K3 };

            HandEyeParams.CameraIntrinsic = intrinsic_data;
        }

        private void button2_Click(object sender, EventArgs e)
        {
            string filepath = @"D:\[REAL_SENSE]\CalibData3.csv";
            r_pose_list = CSV_Helper.LoadRobotPoseCsv(filepath);

            //using (OpenFileDialog ofd = new OpenFileDialog())
            //{
            //    ofd.Filter = "CSV files (*.csv)|*.csv|All files (*.*)|*.*";
            //    ofd.Title = "로봇 포즈 CSV 파일 열기";
            //    ofd.InitialDirectory = @"D:\[REAL_SENSE]";
            //    if (ofd.ShowDialog() == DialogResult.OK)
            //    {
            //        r_pose_list = hand_Eye_Calibration.LoadRobotPoseCsv(ofd.FileName);
            //        if (r_pose_list != null)
            //        {
            //            Console.WriteLine($"로봇 포즈 {r_pose_list.Count}개 로드 완료.");
            //        }
            //    }
            //}
        }

        private void button4_Click(object sender, EventArgs e)
        {
            OpenFileDialog ofd = new OpenFileDialog();
            ofd.Multiselect = true;

            if (ofd.ShowDialog() == DialogResult.OK)
            {
                filepath = ofd.FileNames
                    .OrderBy(f => GetPoseIndexFromFilePath(f, int.MaxValue))
                    .ThenBy(f => Path.GetFileName(f))
                    .ToArray();

                if (pictureBox1.Image != null)
                {
                    pictureBox1.Image.Dispose();
                }

                pictureBox1.Image = new Bitmap(filepath[0]);
                pictureBox1.SizeMode = PictureBoxSizeMode.Zoom;
            }
        }

        private bool TryGetPoseIndexFromFilePath(string path, out int poseIndex)
        {
            poseIndex = 0;

            if (string.IsNullOrWhiteSpace(path))
                return false;

            string name = Path.GetFileNameWithoutExtension(path);

            // 기존 방식처럼 파일명 안의 모든 숫자를 단순 연결하면
            // calib_20260708_pose_02.png -> 2026070802 처럼 잘못 해석될 수 있습니다.
            // 우선 pose/img/image/view/step 키워드 뒤의 숫자를 찾고,
            // 없으면 파일명의 마지막 숫자 그룹을 pose 번호로 사용합니다.
            Match semanticMatch = Regex.Match(
                name,
                @"(?:pose|pos|img|image|view|step)[_\-\s]*(\d+)",
                RegexOptions.IgnoreCase);

            string numberString = null;

            if (semanticMatch.Success && semanticMatch.Groups.Count > 1)
            {
                numberString = semanticMatch.Groups[1].Value;
            }
            else
            {
                MatchCollection matches = Regex.Matches(name, @"\d+");
                if (matches.Count == 0)
                    return false;

                numberString = matches[matches.Count - 1].Value;
            }

            return int.TryParse(numberString, out poseIndex) && poseIndex > 0;
        }

        private int GetPoseIndexFromFilePath(string path, int fallbackIndex)
        {
            return TryGetPoseIndexFromFilePath(path, out int poseIndex)
                ? poseIndex
                : fallbackIndex;
        }

        private void button3_Click(object sender, EventArgs e)
        {
            if (filepath == null || filepath.Length == 0)
            {
                ShowMessageSafe("먼저 이미지를 선택하세요.");
                return;
            }

            c_pose_list.Clear();

            for (int i = 0; i < filepath.Length; i++)
            {
                using (Bitmap bmp = new Bitmap(filepath[i]))
                using (Mat mat = BitmapConverter.ToMat(bmp))
                {
                    var oldImage = pictureBox1.Image;
                    pictureBox1.Image = new Bitmap(bmp);
                    oldImage?.Dispose();
                    pictureBox1.SizeMode = PictureBoxSizeMode.Zoom;

                    string file = Path.GetFileNameWithoutExtension(filepath[i]);

                    CameraTargetPose pose = new CameraTargetPose();
                    bool ok = BoardDetector.TryGetCheckerboardTargetPose(
                        mat, intrinsic_data, 7, 5, 24.0f, i, out pose);

                    if (!ok || pose == null)
                    {
                        Console.WriteLine($"[{i + 1}] 체커보드 검출 실패: {filepath[i]}");
                        continue;
                    }

                    // 파일명이 2.png, pose_02.bmp처럼 실제 pose 번호를 포함하는 경우
                    // 검증/LOO/CSV 출력의 PoseIndex가 원본 데이터 번호와 맞도록 보정합니다.
                    if (TryGetPoseIndexFromFilePath(filepath[i], out int filePoseIndex))
                    {
                        pose.SourcePoseIndex = filePoseIndex;
                        pose.HasExplicitPoseIndex = true;
                    }
                    else
                    {
                        pose.SourcePoseIndex = i + 1;
                        pose.HasExplicitPoseIndex = false;
                    }

                    if (pose.FrameCandidates != null)
                    {
                        foreach (CameraTargetPose candidate in pose.FrameCandidates)
                        {
                            if (candidate == null)
                                continue;

                            candidate.SourcePoseIndex = pose.SourcePoseIndex;
                            candidate.HasExplicitPoseIndex = pose.HasExplicitPoseIndex;
                        }
                    }

                    c_pose_list.Add(pose);
                }
            }
            Console.WriteLine($"Camera Pose Count = {c_pose_list.Count}");
        }

        private async void button5_Click(object sender, EventArgs e)
        {
            if (button5.Enabled == false)
                return;

            try
            {
                EnsureParams();

                // UI 컨트롤 ComboBox 접근은 반드시 UI thread에서 먼저 처리합니다.
                // 이후 실제 계산은 background thread에서 수행해 STA 메시지 펌프가 막히지 않도록 합니다.
                HandEyeParams.RobotAngleUnit = EAngleUnit.Degree;
                ApplySelectedCalibrationSettings();

                button5.Enabled = false;
                UseWaitCursor = true;

                Console.WriteLine("===== HAND-EYE CALIBRATION BACKGROUND TASK START =====");
                await Task.Run(() => hand_Eye_Calibration.RunSelectedHandEyeCalibrationCore(r_pose_list, c_pose_list, intrinsic_data,
                    bestHandEyeResult, bestExcludedPoseNumbers, bestValidationResult, out bestHandEyeMatrix, out bestOutputHandEyeMatrix,
                    out bestResultMode, out bestResultEulerSequence, out bestResultEulerConvention, out bestResultAngleInputOrder,
                    out bestResultRobotPoseDirection,out bestResultHandedness, out bestResultMethod, out bestResultOptimizationMethod));
                Console.WriteLine("===== HAND-EYE CALIBRATION BACKGROUND TASK DONE =====");
            }
            catch (Exception ex)
            {
                ClearBestCalibrationResult();
                Console.WriteLine(ex.ToString());
                ShowMessageSafe($"Hand-Eye Calibration 실패: {ex.Message}");
            }
            finally
            {
                UseWaitCursor = false;
                button5.Enabled = true;
            }
        }

        private async void button7_Click(object sender, EventArgs e)  //test
        {
            EnsureParams();
            HandEyeParams.RobotAngleUnit = EAngleUnit.Degree;
            ApplySelectedCalibrationSettings();

            if (r_pose_list == null || c_pose_list == null ||
                r_pose_list.Count == 0 || c_pose_list.Count == 0)
            {
                ShowMessageSafe("Robot Pose와 Camera Pose를 먼저 로드하세요.");
                return;
            }

            PosePairSet posePairs;

            try
            {
                posePairs = hand_Eye_Calibration.BuildPosePairSet(r_pose_list, c_pose_list);
            }
            catch (Exception ex)
            {
                ShowMessageSafe(ex.Message);
                return;
            }

            if (posePairs.RobotList.Count < HandEyeParams.MinPoseCount)
            {
                ShowMessageSafe(
                    $"유효한 Pose Pair가 부족합니다. " +
                    $"Pair={posePairs.RobotList.Count}, Min={HandEyeParams.MinPoseCount}");
                return;
            }

            try
            {
                hand_Eye_Calibration.ValidatePosePairContent(posePairs, requireImagePoints: false);
            }
            catch (Exception ex)
            {
                ShowMessageSafe(ex.Message);
                return;
            }

            List<HandEyeSweepResult> results = null;

            try
            {
                HandEyeParams.ValidateBasic();
                HandEyeMode selectedMode = HandEyeParams.mode;

                Console.WriteLine("===== ACTUAL MM SWEEP START =====");
                Console.WriteLine($"Robot Pose Count = {posePairs.RobotList.Count}");
                Console.WriteLine($"Camera Pose Count = {posePairs.CameraList.Count}");
                Console.WriteLine($"Scale = {HandEyeParams.scale}");
                Console.WriteLine($"Mode = {selectedMode}");

                results = await Task.Run(() =>
                    hand_Eye_Calibration.RunConventionSweep(
                        posePairs.RobotList,
                        posePairs.CameraList,
                        selectedMode));

                Console.WriteLine("RunConventionSweep DONE");

                if (results == null || results.Count == 0)
                {
                    ShowMessageSafe("유효한 Sweep 결과가 없습니다.");
                    return;
                }

                hand_Eye_Calibration.PrintActualMmSweepResults(results, 50);

                HandEyeSweepResult best = results[0];

                Console.WriteLine("===== BEST ACTUAL MM RESULT =====");
                Console.WriteLine(
                    $"RMS={best.RmsError:F3} mm, " +
                    $"MAX={best.MaxError:F3} mm, " +
                    $"Rot={best.RotationType}, " +
                    $"EulerConv={best.EulerConvention}, " +
                    $"AngleOrder={best.AngleInputOrder}, " +
                    $"InvRobot={best.InvertRobotPose}, " +
                    $"InvCam={best.InvertCameraPose}, " +
                    $"Method={best.Method}");
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.ToString());
                ShowMessageSafe($"Sweep 테스트 실패: {ex.Message}");
            }
            finally
            {
            }
        }

       
        private void button6_Click(object sender, EventArgs e)
        {
            EnsureParams();

            if (bestHandEyeMatrix == null)
            {
                ShowMessageSafe("먼저 Hand-Eye Calibration을 실행하세요.");
                return;
            }

            // button6은 반드시 마지막으로 채택된 raw hand-eye matrix를 기준으로
            // validation/output matrix를 다시 생성합니다. 이전 캐시가 남아 있어
            // closed-form 또는 reject된 중간 결과가 저장/표시되는 상황을 방지합니다.
            try
            {
                bestValidationResult = hand_Eye_Calibration.BuildValidationResultByMode(
                    bestHandEyeMatrix,
                    bestResultMode);

                bestOutputHandEyeMatrix = hand_Eye_Calibration.GetOutputMatrix(bestHandEyeMatrix);
            }
            catch (Exception ex)
            {
                ShowMessageSafe($"채택된 Hand-Eye 결과로 validation을 재생성하지 못했습니다: {ex.Message}");
                return;
            }

            if (bestValidationResult == null)
            {
                ShowMessageSafe("채택된 Hand-Eye 결과의 validation이 비어 있습니다.");
                return;
            }

            List<UIEndEffectorData> Data = hand_Eye_Calibration.BuildValidationUiData(bestValidationResult, bestResultMode);

            if (Data.Count == 0 || Data.Any(d => d.TransformMatrix4x4 == null))
            {
                ShowMessageSafe("저장할 Transform Matrix가 비어 있습니다. Calibration 결과를 확인하세요.");
                return;
            }

            string outputDir = @"D:\[REAL_SENSE]";
            string baseName =
                $"CALIB_{bestResultEulerSequence}_{bestResultAngleInputOrder}_{bestResultEulerConvention}_{bestResultHandedness}_{bestResultRobotPoseDirection}_{bestResultMode}_{bestResultMethod}_{bestResultOptimizationMethod}";

            string validationPath = Path.Combine(outputDir, baseName + ".csv");
            string acceptedResultPath = Path.Combine(outputDir, baseName + "_AcceptedResult.csv");

            try
            {
                Directory.CreateDirectory(outputDir);

                // 기존 파일명은 호환성을 위해 pose별 validation scatter를 저장합니다.
                CSV_Helper.Save_csv_Effector(Data, validationPath);

                // 실제 채택된 보정 결과는 별도 CSV에 명시적으로 저장합니다.
                CSV_Helper.SaveAcceptedHandEyeResultCsv(acceptedResultPath, HandEyeParams, bestValidationResult, bestHandEyeMatrix, bestOutputHandEyeMatrix);
            }
            catch (Exception ex)
            {
                ShowMessageSafe($"CSV 저장 실패: {ex.Message}");
                return;
            }

            Console.WriteLine("===== BUTTON6 OUTPUT USES ACCEPTED RESULT =====");
            Console.WriteLine(
                $"RawAcceptedHandEye T=({bestHandEyeMatrix[0, 3]:F3}, {bestHandEyeMatrix[1, 3]:F3}, {bestHandEyeMatrix[2, 3]:F3}) mm");

            if (bestOutputHandEyeMatrix != null)
            {
                Console.WriteLine(
                    $"OutputAcceptedHandEye T=({bestOutputHandEyeMatrix[0, 3]:F3}, {bestOutputHandEyeMatrix[1, 3]:F3}, {bestOutputHandEyeMatrix[2, 3]:F3}) mm, " +
                    $"CoordianteConvert={HandEyeParams.CoordConvertMode}");
            }

            if (HandEyeParams.LastNonlinearAccepted && HandEyeParams.LastTarget2Gripper != null)
            {
                Console.WriteLine(
                    $"Accepted Target2Gripper T=({HandEyeParams.LastTarget2Gripper[0, 3]:F3}, " +
                    $"{HandEyeParams.LastTarget2Gripper[1, 3]:F3}, {HandEyeParams.LastTarget2Gripper[2, 3]:F3}) mm, " +
                    $"ReprojectionRMS={HandEyeParams.LastNonlinearReprojectionRmsPx:F3}px");
            }
            else
            {
                Console.WriteLine("Accepted Target2Gripper가 없습니다. Closed-form/미채택 결과 기준 validation만 저장했습니다.");
            }

            Console.WriteLine($"Validation pose CSV: {validationPath}");
            Console.WriteLine($"Accepted result CSV: {acceptedResultPath}");

            List<UIEndEffectorData> viewerSceneData = hand_Eye_Calibration.BuildViewerSceneData(bestHandEyeMatrix, bestResultMode);
            hand_Eye_Calibration.PrintViewerSceneSummary(viewerSceneData);

            if (viewerSceneData != null && viewerSceneData.Count > 0)
            {
                viewer.UpdateCalibrationData(viewerSceneData, 65.0f);
            }
            else
            {
                viewer.UpdateHandEyeValidation(bestValidationResult);
            }
        }

        private void combobox1_selectedChanged(object sender, EventArgs e)
        {
            EnsureParams();

            string selected = comboBox1.SelectedItem?.ToString();

            // Enum 타입과 문자열이 일치하므로 TryParse로 한 번에 처리 가능합니다.
            if (Enum.TryParse(selected, out EEulerSequence sequence))
            {
                HandEyeParams.RobotEulerSequence = sequence;
            }
            else
            {
                Console.WriteLine("지원하지 않는 Euler Sequence입니다.");
            }
        }
        private void combobox2_selectedChanged(object sender, EventArgs e)
        {
            EnsureParams();

            string selected = comboBox2.SelectedItem?.ToString();

            // Enum 타입과 문자열이 일치하므로 TryParse로 한 번에 처리 가능합니다.
            if (Enum.TryParse(selected, out HandEyeCalibrationMethod method))
            {
                HandEyeParams.CalibrationMethod = method;
            }
            else
            {
                Console.WriteLine("지원하지 않는 Calibration method 입니다.");
            }
        }
        private void combobox3_selectedChanged(object sender, EventArgs e)
        {
            EnsureParams();

            string selected = comboBox3.SelectedItem?.ToString();

            // Enum 타입과 문자열이 일치하므로 TryParse로 한 번에 처리 가능합니다.
            if (Enum.TryParse(selected, out HandEyeMode mode))
            {
                HandEyeParams.mode = mode;
            }
            else
            {
                Console.WriteLine("지원하지 않는 HandEye mode입니다.");
            }
        }
        private void combobox4_selectedChanged(object sender, EventArgs e)
        {
            EnsureParams();

            string selected = comboBox4.SelectedItem?.ToString();

            // Enum 타입과 문자열이 일치하므로 TryParse로 한 번에 처리 가능합니다.
            if (Enum.TryParse(selected, out EndEffectorHandedness handedness))
            {
                HandEyeParams.Handedness = handedness;
            }
            else
            {
                Console.WriteLine("지원하지 않는 EndEffector Handedness 입니다.");
            }
        }
        private void combobox5_selectedChanged(object sender, EventArgs e)
        {
            EnsureParams();

            string selected = comboBox5.SelectedItem?.ToString();

            // Enum 타입과 문자열이 일치하므로 TryParse로 한 번에 처리 가능합니다.
            if (Enum.TryParse(selected, out CoordianteConvert convert))
            {
                HandEyeParams.CoordConvertMode = convert;
            }
            else
            {
                Console.WriteLine("지원하지 않는 Coordiante Convert 입니다.");
            }
        }
        private void combobox6_selectedChanged(object sender, EventArgs e)
        {
            EnsureParams();

            string selected = comboBox6.SelectedItem?.ToString();

            // Enum 타입과 문자열이 일치하므로 TryParse로 한 번에 처리 가능합니다.
            if (Enum.TryParse(selected, out OptimizationMethod method))
            {
                HandEyeParams.Optimization = method;
            }
            else
            {
                Console.WriteLine("지원하지 않는 Optimization Method 입니다.");
            }
        }

        private void scale_valuechanged(object sender, EventArgs e)
        {
            EnsureParams();
            HandEyeParams.scale = (double)nud_scalefactor.Value;
        }

        private void Form1_closing(object sender, FormClosingEventArgs e)
        {
            hand_Eye_Calibration.Dispose();

            viewer.ClearViewer();
            ElementHost.Dispose();
            viewer.Dispose();
        }
    }
}
