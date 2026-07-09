using HelixToolkit.Wpf.SharpDX;
using SharpDX;
using DxColor = SharpDX.Color;
using DxColor4 = SharpDX.Color4;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using System.Windows.Threading;
using WpfPoint = System.Windows.Point;

namespace Calibration_test
{
    /// <summary>
    /// Effector_Viewer.xaml에 대한 상호 작용 논리
    /// </summary>
    public partial class Viewer : UserControl
    {
        public IEffectsManager EffectsManager { get; private set; }
        private readonly Dictionary<int, TextBlock> _poseLabelBlocks = new Dictionary<int, TextBlock>();
        private readonly Dictionary<int, Vector3> _poseOrigins = new Dictionary<int, Vector3>();
        private List<UIEndEffectorData> _currentEffectorData = new List<UIEndEffectorData>();
        private TextBlock _cameraLabelBlock;
        private TextBlock _selectedPoseValueBlock;
        private int? _selectedPoseIndex = null;
        private bool _hasMouseDownPoint = false;
        private WpfPoint _mouseDownPoint;
        private bool _isLeftPanning = false;
        private WpfPoint _lastLeftPanPoint;
        private bool _isRightRotating = false;
        private WpfPoint _lastRightRotatePoint;

        public Viewer()
        {
            EffectsManager = new DefaultEffectsManager();

            InitializeComponent();

            DataContext = this;

            Loaded += Viewer_Loaded;
            Unloaded += Viewer_Unloaded;
            viewPort3D.PreviewMouseLeftButtonDown += ViewPort3D_PreviewMouseLeftButtonDown;
            viewPort3D.PreviewMouseLeftButtonUp += ViewPort3D_PreviewMouseLeftButtonUp;
            viewPort3D.PreviewMouseRightButtonDown += ViewPort3D_PreviewMouseRightButtonDown;
            viewPort3D.PreviewMouseRightButtonUp += ViewPort3D_PreviewMouseRightButtonUp;
            viewPort3D.PreviewMouseMove += ViewPort3D_PreviewMouseMove;
            viewPort3D.SizeChanged += (s, e) => UpdateOverlayPositions();
        }

        public void Dispose()
        {
            EffectsManager.Dispose();
        }


        private void Viewer_Loaded(object sender, RoutedEventArgs e)
        {
            CompositionTarget.Rendering += CompositionTarget_Rendering;
            EnsureStaticOverlayElements();
            UpdateOverlayPositions();
        }

        private void Viewer_Unloaded(object sender, RoutedEventArgs e)
        {
            CompositionTarget.Rendering -= CompositionTarget_Rendering;
        }

        private void CompositionTarget_Rendering(object sender, EventArgs e)
        {
            UpdateOverlayPositions();
        }

        private void EnsureStaticOverlayElements()
        {
            if (_cameraLabelBlock == null)
            {
                _cameraLabelBlock = CreateOverlayLabel("Camera", Brushes.Red, true);
                overlayCanvas.Children.Add(_cameraLabelBlock);
            }

            if (_selectedPoseValueBlock == null)
            {
                _selectedPoseValueBlock = CreateOverlayLabel(string.Empty, Brushes.Cyan, true);
                _selectedPoseValueBlock.Visibility = Visibility.Collapsed;
                overlayCanvas.Children.Add(_selectedPoseValueBlock);
            }
        }

        private TextBlock CreateOverlayLabel(string text, Brush foreground, bool bold = false)
        {
            return new TextBlock
            {
                Text = text,
                Foreground = foreground,
                Background = new SolidColorBrush(System.Windows.Media.Color.FromArgb(140, 0, 0, 0)),
                Padding = new Thickness(4, 1, 4, 1),
                FontSize = 12,
                FontWeight = bold ? FontWeights.Bold : FontWeights.Normal,
                IsHitTestVisible = false
            };
        }

        private void ResetOverlaySelection()
        {
            _selectedPoseIndex = null;
            selectedInfoPanel.Visibility = Visibility.Collapsed;
            if (_selectedPoseValueBlock != null)
                _selectedPoseValueBlock.Visibility = Visibility.Collapsed;
            RefreshLabelStyles();
        }

        private void ClearOverlayLabels()
        {
            _poseOrigins.Clear();
            _currentEffectorData = new List<UIEndEffectorData>();

            foreach (var tb in _poseLabelBlocks.Values)
                overlayCanvas.Children.Remove(tb);

            _poseLabelBlocks.Clear();
            ResetOverlaySelection();
            EnsureStaticOverlayElements();
        }

        private void SetOverlayPoseData(List<UIEndEffectorData> effectorDataList)
        {
            ClearOverlayLabels();
            EnsureStaticOverlayElements();

            if (effectorDataList == null)
                return;

            _currentEffectorData = effectorDataList.Where(d => d != null && d.TransformMatrix4x4 != null).ToList();

            foreach (UIEndEffectorData data in _currentEffectorData)
            {
                string label = string.IsNullOrWhiteSpace(data.Label)
                    ? $"End Effector {data.PoseIndex}"
                    : data.Label;

                TextBlock tb = CreateOverlayLabel(label, Brushes.Yellow, true);
                _poseLabelBlocks[data.PoseIndex] = tb;
                overlayCanvas.Children.Add(tb);

                try
                {
                    _poseOrigins[data.PoseIndex] = TransformPoint(data.TransformMatrix4x4, 0, 0, 0);
                }
                catch
                {
                }
            }

            RefreshLabelStyles();
            UpdateOverlayPositions();
        }

        private void RefreshLabelStyles()
        {
            foreach (var kv in _poseLabelBlocks)
            {
                bool selected = _selectedPoseIndex.HasValue && _selectedPoseIndex.Value == kv.Key;
                kv.Value.Foreground = selected ? Brushes.Black : Brushes.Yellow;
                kv.Value.Background = selected
                    ? new SolidColorBrush(System.Windows.Media.Color.FromArgb(220, 255, 220, 0))
                    : new SolidColorBrush(System.Windows.Media.Color.FromArgb(140, 0, 0, 0));
            }
        }

        private void ViewPort3D_PreviewMouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            _mouseDownPoint = e.GetPosition(overlayCanvas);
            _lastLeftPanPoint = _mouseDownPoint;
            _hasMouseDownPoint = true;
            _isLeftPanning = true;
            viewPort3D.CaptureMouse();

            // 좌클릭 드래그는 2D 이동(Pan) 전용입니다.
            // HelixToolkit 기본 좌클릭 회전을 막기 위해 처리 완료로 표시합니다.
            e.Handled = true;
        }

        private void ViewPort3D_PreviewMouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            WpfPoint upPoint = e.GetPosition(overlayCanvas);

            if (_isLeftPanning)
            {
                _isLeftPanning = false;
                if (viewPort3D.IsMouseCaptured)
                    viewPort3D.ReleaseMouseCapture();
            }

            if (_hasMouseDownPoint)
            {
                _hasMouseDownPoint = false;

                double dx = upPoint.X - _mouseDownPoint.X;
                double dy = upPoint.Y - _mouseDownPoint.Y;
                double move = Math.Sqrt(dx * dx + dy * dy);

                // 거의 움직이지 않은 좌클릭은 pose 선택으로 처리합니다.
                if (move <= 6.0)
                {
                    int? hitPose = FindNearestPose(upPoint, 28.0);

                    if (hitPose.HasValue)
                        SelectPose(hitPose.Value);
                    else
                        ResetOverlaySelection();
                }
            }

            e.Handled = true;
        }

        private void ViewPort3D_PreviewMouseRightButtonDown(object sender, MouseButtonEventArgs e)
        {
            _isRightRotating = true;
            _lastRightRotatePoint = e.GetPosition(overlayCanvas);
            viewPort3D.CaptureMouse();

            // 우클릭 드래그는 회전 전용입니다.
            e.Handled = true;
        }

        private void ViewPort3D_PreviewMouseRightButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (_isRightRotating)
            {
                _isRightRotating = false;
                if (viewPort3D.IsMouseCaptured)
                    viewPort3D.ReleaseMouseCapture();
                e.Handled = true;
            }
        }

        private void ViewPort3D_PreviewMouseMove(object sender, MouseEventArgs e)
        {
            if (_isLeftPanning)
            {
                if (e.LeftButton != MouseButtonState.Pressed)
                {
                    _isLeftPanning = false;
                    if (viewPort3D.IsMouseCaptured)
                        viewPort3D.ReleaseMouseCapture();
                    return;
                }

                WpfPoint current = e.GetPosition(overlayCanvas);
                double dx = current.X - _lastLeftPanPoint.X;
                double dy = current.Y - _lastLeftPanPoint.Y;

                if (Math.Abs(dx) > 0.0 || Math.Abs(dy) > 0.0)
                {
                    PanCameraByScreenDelta(dx, dy);
                    _lastLeftPanPoint = current;
                    UpdateOverlayPositions();
                    viewPort3D.InvalidateRender();
                }

                e.Handled = true;
                return;
            }

            if (_isRightRotating)
            {
                if (e.RightButton != MouseButtonState.Pressed)
                {
                    _isRightRotating = false;
                    if (viewPort3D.IsMouseCaptured)
                        viewPort3D.ReleaseMouseCapture();
                    return;
                }

                WpfPoint current = e.GetPosition(overlayCanvas);
                double dx = current.X - _lastRightRotatePoint.X;
                double dy = current.Y - _lastRightRotatePoint.Y;

                if (Math.Abs(dx) > 0.0 || Math.Abs(dy) > 0.0)
                {
                    RotateCameraByScreenDelta(dx, dy);
                    _lastRightRotatePoint = current;
                    UpdateOverlayPositions();
                    viewPort3D.InvalidateRender();
                }

                e.Handled = true;
            }
        }

        private void PanCameraByScreenDelta(double dx, double dy)
        {
            var camPos = mainCamera.Position;
            Vector3D look = mainCamera.LookDirection;
            Vector3D up = mainCamera.UpDirection;

            double distance = look.Length;
            if (distance < 1e-9)
                distance = 1000.0;

            Vector3D forward = look;
            forward.Normalize();

            Vector3D upN = up;
            if (upN.Length < 1e-9)
                upN = new Vector3D(0, 0, 1);
            upN.Normalize();

            // 화면 기준 오른쪽/위쪽 벡터입니다. TryProjectWorldToScreen과 같은 기준을 사용합니다.
            Vector3D cameraBackward = new Vector3D(-forward.X, -forward.Y, -forward.Z);

            Vector3D screenRight = Vector3D.CrossProduct(upN, cameraBackward);
            if (screenRight.Length < 1e-9)
                screenRight = new Vector3D(1, 0, 0);
            screenRight.Normalize();

            Vector3D screenUp = Vector3D.CrossProduct(cameraBackward, screenRight);
            if (screenUp.Length < 1e-9)
                screenUp = upN;
            screenUp.Normalize();

            double width = Math.Max(overlayCanvas.ActualWidth, 1.0);
            double height = Math.Max(overlayCanvas.ActualHeight, 1.0);

            double fovDeg = mainCamera.FieldOfView;
            if (fovDeg <= 0.1)
                fovDeg = 45.0;

            double fovRad = fovDeg * Math.PI / 180.0;
            double viewHeight = 2.0 * distance * Math.Tan(fovRad / 2.0);
            double viewWidth = viewHeight * Math.Max(width / height, 1e-6);

            double worldPerPixelX = viewWidth / width;
            double worldPerPixelY = viewHeight / height;

            // 좌클릭 드래그 방향으로 화면이 따라오도록 카메라를 반대 방향으로 평행이동합니다.
            Vector3D worldDelta =
                (-dx * worldPerPixelX) * screenRight +
                (dy * worldPerPixelY) * screenUp;

            mainCamera.Position = new Point3D(
                camPos.X + worldDelta.X,
                camPos.Y + worldDelta.Y,
                camPos.Z + worldDelta.Z);

            // LookDirection은 유지합니다. 즉, 2D pan처럼 카메라와 시선 중심이 함께 평행이동한 효과입니다.
        }

        private void RotateCameraByScreenDelta(double dx, double dy)
        {
            var camPos = mainCamera.Position;
            Vector3D look = mainCamera.LookDirection;
            Vector3D up = mainCamera.UpDirection;

            if (look.Length < 1e-9)
                return;

            Point3D target = new Point3D(
                camPos.X + look.X,
                camPos.Y + look.Y,
                camPos.Z + look.Z);

            Vector3D forward = look;
            forward.Normalize();

            Vector3D upN = up;
            if (upN.Length < 1e-9)
                upN = new Vector3D(0, 0, 1);
            upN.Normalize();

            Vector3D cameraBackward = new Vector3D(-forward.X, -forward.Y, -forward.Z);
            Vector3D screenRight = Vector3D.CrossProduct(upN, cameraBackward);
            if (screenRight.Length < 1e-9)
                screenRight = new Vector3D(1, 0, 0);
            screenRight.Normalize();

            double sensitivityDegPerPixel = 0.35;

            System.Windows.Media.Media3D.Quaternion yaw = new System.Windows.Media.Media3D.Quaternion(upN, -dx * sensitivityDegPerPixel);
            System.Windows.Media.Media3D.Quaternion pitch = new System.Windows.Media.Media3D.Quaternion(screenRight, -dy * sensitivityDegPerPixel);

            Matrix3D rot = Matrix3D.Identity;
            rot.RotateAt(yaw, target);
            rot.RotateAt(pitch, target);

            Point3D newPos = rot.Transform(camPos);
            Vector3D newUp = rot.Transform(upN);

            mainCamera.Position = newPos;
            mainCamera.LookDirection = new Vector3D(
                target.X - newPos.X,
                target.Y - newPos.Y,
                target.Z - newPos.Z);

            if (newUp.Length > 1e-9)
            {
                newUp.Normalize();
                mainCamera.UpDirection = newUp;
            }
        }

        private void OverlayCanvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            WpfPoint click = e.GetPosition(overlayCanvas);
            int? hitPose = FindNearestPose(click, 24.0);

            if (hitPose.HasValue)
            {
                SelectPose(hitPose.Value);
            }
            else
            {
                ResetOverlaySelection();
            }
        }

        private int? FindNearestPose(WpfPoint click, double thresholdPx)
        {
            double best = double.MaxValue;
            int? bestPose = null;

            foreach (UIEndEffectorData data in _currentEffectorData)
            {
                if (!_poseOrigins.TryGetValue(data.PoseIndex, out Vector3 origin))
                    continue;

                if (!TryProjectWorldToScreen(origin, out WpfPoint screen))
                    continue;

                double dx = screen.X - click.X;
                double dy = screen.Y - click.Y;
                double dist = Math.Sqrt(dx * dx + dy * dy);
                if (dist < best && dist <= thresholdPx)
                {
                    best = dist;
                    bestPose = data.PoseIndex;
                }
            }

            return bestPose;
        }

        private void SelectPose(int poseIndex)
        {
            UIEndEffectorData data = _currentEffectorData.FirstOrDefault(d => d.PoseIndex == poseIndex);
            if (data == null || data.TransformMatrix4x4 == null)
                return;

            _selectedPoseIndex = poseIndex;
            RefreshLabelStyles();
            selectedInfoPanel.Visibility = Visibility.Visible;

            double[,] m = data.TransformMatrix4x4;

            txtSelectedHeader.Text = data.Label ?? $"End Effector {data.PoseIndex}";
            txtSelectedPosition.Text = $"Pos: ({m[0, 3]:F2}, {m[1, 3]:F2}, {m[2, 3]:F2}) mm";
            txtSelectedXDir.Text = $"Xdir: ({m[0, 0]:F3}, {m[1, 0]:F3}, {m[2, 0]:F3})";
            txtSelectedYDir.Text = $"Ydir: ({m[0, 1]:F3}, {m[1, 1]:F3}, {m[2, 1]:F3})";
            txtSelectedZDir.Text = $"Zdir: ({m[0, 2]:F3}, {m[1, 2]:F3}, {m[2, 2]:F3})";
            txtSelectedExtra.Text = data.InfoText ?? "Selected pose";

            if (_selectedPoseValueBlock != null)
            {
                _selectedPoseValueBlock.Text = $"({m[0, 3]:F2}, {m[1, 3]:F2}, {m[2, 3]:F2})";
                _selectedPoseValueBlock.Visibility = Visibility.Visible;
            }

            UpdateOverlayPositions();
        }

        private void UpdateOverlayPositions()
        {
            EnsureStaticOverlayElements();

            if (_cameraLabelBlock != null)
            {
                if (TryProjectWorldToScreen(new Vector3(0, 0, 0), out WpfPoint camPt))
                {
                    _cameraLabelBlock.Visibility = Visibility.Visible;
                    Canvas.SetLeft(_cameraLabelBlock, camPt.X + 8);
                    Canvas.SetTop(_cameraLabelBlock, camPt.Y - 20);
                }
                else
                {
                    _cameraLabelBlock.Visibility = Visibility.Collapsed;
                }
            }

            foreach (UIEndEffectorData data in _currentEffectorData)
            {
                if (!_poseLabelBlocks.TryGetValue(data.PoseIndex, out TextBlock tb))
                    continue;
                if (!_poseOrigins.TryGetValue(data.PoseIndex, out Vector3 origin))
                    continue;

                if (TryProjectWorldToScreen(origin, out WpfPoint pt))
                {
                    tb.Visibility = Visibility.Visible;
                    Canvas.SetLeft(tb, pt.X + 6);
                    Canvas.SetTop(tb, pt.Y - 10);
                }
                else
                {
                    tb.Visibility = Visibility.Collapsed;
                }
            }

            if (_selectedPoseValueBlock != null && _selectedPoseIndex.HasValue && _poseOrigins.TryGetValue(_selectedPoseIndex.Value, out Vector3 selectedOrigin))
            {
                if (TryProjectWorldToScreen(selectedOrigin, out WpfPoint pt))
                {
                    _selectedPoseValueBlock.Visibility = Visibility.Visible;
                    Canvas.SetLeft(_selectedPoseValueBlock, pt.X + 6);
                    Canvas.SetTop(_selectedPoseValueBlock, pt.Y + 12);
                }
                else
                {
                    _selectedPoseValueBlock.Visibility = Visibility.Collapsed;
                }
            }
        }

        private bool TryProjectWorldToScreen(Vector3 point, out WpfPoint screenPoint)
        {
            screenPoint = new WpfPoint();

            if (ActualWidth <= 1 || ActualHeight <= 1)
                return false;

            var camPos = mainCamera.Position;
            var look = mainCamera.LookDirection;
            var up = mainCamera.UpDirection;

            Vector3D f = look;
            if (f.Length < 1e-9)
                return false;
            f.Normalize();

            Vector3D upN = up;
            if (upN.Length < 1e-9)
                return false;
            upN.Normalize();

            Vector3D zaxis = new Vector3D(-f.X, -f.Y, -f.Z); // camera backward
            Vector3D xaxis = Vector3D.CrossProduct(upN, zaxis);
            if (xaxis.Length < 1e-9)
                return false;
            xaxis.Normalize();
            Vector3D yaxis = Vector3D.CrossProduct(zaxis, xaxis);
            yaxis.Normalize();

            double vx = point.X - camPos.X;
            double vy = point.Y - camPos.Y;
            double vz = point.Z - camPos.Z;

            double cx = vx * xaxis.X + vy * xaxis.Y + vz * xaxis.Z;
            double cy = vx * yaxis.X + vy * yaxis.Y + vz * yaxis.Z;
            double cz = vx * zaxis.X + vy * zaxis.Y + vz * zaxis.Z;
            double zForward = -cz;

            if (zForward <= 1e-6)
                return false;

            double fovDeg = mainCamera.FieldOfView;
            if (fovDeg <= 0.1)
                fovDeg = 45.0;

            double fovRad = fovDeg * Math.PI / 180.0;
            double tanHalf = Math.Tan(fovRad / 2.0);
            double aspect = Math.Max(ActualWidth / ActualHeight, 1e-6);

            double ndcX = cx / (zForward * tanHalf * aspect);
            double ndcY = cy / (zForward * tanHalf);

            double sx = (ndcX + 1.0) * 0.5 * overlayCanvas.ActualWidth;
            double sy = (1.0 - (ndcY + 1.0) * 0.5) * overlayCanvas.ActualHeight;

            if (double.IsNaN(sx) || double.IsNaN(sy) || double.IsInfinity(sx) || double.IsInfinity(sy))
                return false;

            screenPoint = new WpfPoint(sx, sy);
            return true;
        }

        /// <summary>
        /// 산출된 End Effector 데이터 리스트를 받아 3D 공간에 축(X:Red, Y:Green, Z:Blue) 형태로 렌더링합니다.
        /// </summary>
        /// <param name="effectorDataList">HandEyeCalibration에서 계산된 포즈 리스트</param>
        /// <param name="axisLength">화면에 표시될 각 축의 길이 (mm 단위 권장)</param>
        public void UpdateCalibrationData(List<UIEndEffectorData> effectorDataList, float axisLength = 50.0f)
        {
            if (effectorDataList == null || effectorDataList.Count == 0)
            {
                Dispatcher.BeginInvoke(new Action(() =>
                {
                    ClearCameraOriginGeometry();
                    ClearHandEyeValidationGeometry();
                    ClearOverlayLabels();
                    viewPort3D.InvalidateRender();
                }), DispatcherPriority.Render);

                return;
            }

            Vector3 cameraOrigin = new Vector3(0, 0, 0);

            LineBuilder cameraBuilder = new LineBuilder();
            LineBuilder rayBuilder = new LineBuilder();
            LineBuilder bodyBuilder = new LineBuilder();
            LineBuilder xBuilder = new LineBuilder();
            LineBuilder yBuilder = new LineBuilder();
            LineBuilder zBuilder = new LineBuilder();

            AddCameraShape(cameraBuilder, cameraOrigin, 60.0f);

            // Camera local axis도 같은 색상의 삼각형으로 표시합니다.
            AddAxisTriangle(xBuilder, cameraOrigin, new Vector3(axisLength, 0, 0), axisLength * 0.15f);
            AddAxisTriangle(yBuilder, cameraOrigin, new Vector3(0, axisLength, 0), axisLength * 0.15f);
            AddAxisTriangle(zBuilder, cameraOrigin, new Vector3(0, 0, axisLength), axisLength * 0.15f);

            Console.WriteLine("===== Viewer Camera + End Effector Pose View =====");
            Console.WriteLine("Camera = (0.000, 0.000, 0.000)");

            foreach (var data in effectorDataList)
            {
                if (data.TransformMatrix4x4 == null)
                    continue;

                double[,] m = data.TransformMatrix4x4;

                Vector3 origin = TransformPoint(m, 0, 0, 0);
                Vector3 xAxis = TransformPoint(m, axisLength, 0, 0);
                Vector3 yAxis = TransformPoint(m, 0, axisLength, 0);
                Vector3 zAxis = TransformPoint(m, 0, 0, axisLength);

                if (!IsValidPoint(origin) || !IsValidPoint(xAxis) ||
                    !IsValidPoint(yAxis) || !IsValidPoint(zAxis))
                    continue;

                rayBuilder.AddLine(cameraOrigin, origin);

                // 흰색 미니 frustum으로 End Effector 본체 방향을 표시합니다.
                AddMiniFrustum(bodyBuilder, null, m, axisLength * 0.55f);

                // RGB 축을 단순 선분이 아니라 삼각형 arrow/fan 형태로 표시합니다.
                AddAxisTriangle(xBuilder, origin, xAxis, axisLength * 0.18f);
                AddAxisTriangle(yBuilder, origin, yAxis, axisLength * 0.18f);
                AddAxisTriangle(zBuilder, origin, zAxis, axisLength * 0.18f);

                Console.WriteLine(
                    $"{data.Label ?? $"End Effector {data.PoseIndex}"} | " +
                    $"Pos=({origin.X:F3}, {origin.Y:F3}, {origin.Z:F3}) | " +
                    $"Zdir=({m[0, 2]:F3}, {m[1, 2]:F3}, {m[2, 2]:F3})");
            }

            LineGeometry3D cameraGeometry = ToGeometryOrNull(cameraBuilder);
            LineGeometry3D rayGeometry = ToGeometryOrNull(rayBuilder);
            LineGeometry3D bodyGeometry = ToGeometryOrNull(bodyBuilder);
            LineGeometry3D xGeometry = ToGeometryOrNull(xBuilder);
            LineGeometry3D yGeometry = ToGeometryOrNull(yBuilder);
            LineGeometry3D zGeometry = ToGeometryOrNull(zBuilder);

            Dispatcher.BeginInvoke(new Action(() =>
            {
                ClearHandEyeValidationGeometry();

                cameraModel.Geometry = cameraGeometry;
                cameraToEffectorLineModel.Geometry = rayGeometry;

                // 기존 통합 axis model은 사용하지 않습니다. 색상 고정을 위해 축별 model을 분리했습니다.
                endEffectorAxesModel.Geometry = null;
                endEffectorBodyModel.Geometry = bodyGeometry;
                endEffectorXModel.Geometry = xGeometry;
                endEffectorYModel.Geometry = yGeometry;
                endEffectorZModel.Geometry = zGeometry;

                SetOverlayPoseData(effectorDataList);

                viewPort3D.ZoomExtents();
                UpdateOverlayPositions();
                viewPort3D.InvalidateRender();
            }), DispatcherPriority.Render);
        }

        private static void AddCameraShape(LineBuilder builder, Vector3 cameraOrigin, float size)
        {
            /*
             * Viewer의 world 원점은 Camera optical center입니다.
             * Cyan 선분도 이 점에서 각 End Effector로 이어집니다.
             *
             * 이전 버전은 프러스텀 사각면을 +Z 방향에 두어서 화면상 카메라 형상이
             * 선분 시작점 앞쪽으로 튀어나와 보일 수 있었습니다.
             * 여기서는 카메라 형상을 뒤집어 사각면을 -Z 방향에 두고,
             * cameraOrigin 자체가 '끝 모서리/꼭짓점'이 되도록 합니다.
             */
            Vector3 tip = cameraOrigin;

            float zBack = -size * 1.8f;
            float halfW = size * 0.95f;
            float halfH = size * 0.65f;

            Vector3 p1 = tip + new Vector3(-halfW, -halfH, zBack);
            Vector3 p2 = tip + new Vector3(halfW, -halfH, zBack);
            Vector3 p3 = tip + new Vector3(halfW, halfH, zBack);
            Vector3 p4 = tip + new Vector3(-halfW, halfH, zBack);

            builder.AddLine(tip, p1);
            builder.AddLine(tip, p2);
            builder.AddLine(tip, p3);
            builder.AddLine(tip, p4);

            builder.AddLine(p1, p2);
            builder.AddLine(p2, p3);
            builder.AddLine(p3, p4);
            builder.AddLine(p4, p1);

            // 카메라 방향 확인용 local axis
            builder.AddLine(tip, tip + new Vector3(size * 1.3f, 0, 0));
            builder.AddLine(tip, tip + new Vector3(0, size * 1.3f, 0));
            builder.AddLine(tip, tip + new Vector3(0, 0, size * 1.3f));
        }

        public void UpdateHandEyeValidation(
      HandEyeValidationResult result,
      float baseAxisLength = 200.0f,
      float gripperAxisLength = 80.0f,
      float cameraSize = 60.0f,
      float targetMarkerSize = 30.0f,
      float meanMarkerSize = 50.0f)
        {
            if (result == null || result.Poses == null || result.Poses.Count == 0)
            {
                Dispatcher.BeginInvoke(new Action(() =>
                {
                    ClearHandEyeValidationGeometry();
                    ClearOverlayLabels();
                    viewPort3D.InvalidateRender();
                }), DispatcherPriority.Render);

                return;
            }

            bool useResidualView = ShouldUseResidualView(result);

            LineBuilder baseBuilder = new LineBuilder();
            Color4Collection baseColors = new Color4Collection();

            LineBuilder gripperBuilder = new LineBuilder();
            Color4Collection gripperColors = new Color4Collection();

            LineBuilder cameraBuilder = new LineBuilder();
            LineBuilder targetBuilder = new LineBuilder();
            LineBuilder meanBuilder = new LineBuilder();
            LineBuilder errorBuilder = new LineBuilder();

            AddAxis(baseBuilder, baseColors, CreateIdentity4x4(), baseAxisLength);

            if (useResidualView)
            {
                /*
                 * Residual View
                 * Mean Target을 원점으로 놓고,
                 * 각 Target은 ErrorX/Y/Z 위치에 표시합니다.
                 *
                 * 이 모드는 캘리브레이션 결과가 발산했거나,
                 * 임의 로봇 좌표 테스트처럼 절대좌표가 비정상일 때도
                 * 오차 분포를 안정적으로 볼 수 있습니다.
                 */

                double residualScale = GetResidualDisplayScale(result.MaxError);

                Vector3 mean = new Vector3(0, 0, 0);

                AddCrossMarker(meanBuilder, mean, meanMarkerSize);
                AddBoxMarker(meanBuilder, mean, meanMarkerSize * 0.5f);

                foreach (var pose in result.Poses)
                {
                    if (pose == null)
                        continue;

                    Vector3 target = new Vector3(
                        (float)(pose.ErrorX * residualScale),
                        (float)(pose.ErrorY * residualScale),
                        (float)(pose.ErrorZ * residualScale));

                    if (!IsValidPoint(target))
                        continue;

                    AddCrossMarker(targetBuilder, target, targetMarkerSize);

                    // Error Vector: Target Error 위치 → Mean 원점
                    errorBuilder.AddLine(target, mean);
                }

                Console.WriteLine("===== Hand-Eye Validation View =====");
                Console.WriteLine("View Mode = Residual Error View");
                Console.WriteLine($"Residual Display Scale = {residualScale:F3}");
            }
            else
            {
                /*
                 * Raw World View
                 * 정상적인 캘리브레이션 결과일 때 Base 좌표계 기준으로
                 * Gripper, Camera, Target, Mean Target을 함께 표시합니다.
                 */

                Vector3 meanTarget = new Vector3(
                    (float)result.MeanTargetX,
                    (float)result.MeanTargetY,
                    (float)result.MeanTargetZ);

                bool validMeanTarget = IsValidPoint(meanTarget);

                foreach (var pose in result.Poses)
                {
                    if (pose == null)
                        continue;

                    if (IsValidTransform(pose.T_Gripper2Base))
                    {
                        AddAxis(
                            gripperBuilder,
                            gripperColors,
                            pose.T_Gripper2Base,
                            gripperAxisLength);
                    }

                    if (IsValidTransform(pose.T_Cam2Base))
                    {
                        AddCameraFrustum(
                            cameraBuilder,
                            pose.T_Cam2Base,
                            cameraSize);
                    }

                    Vector3 target = new Vector3(
                        (float)pose.TargetX,
                        (float)pose.TargetY,
                        (float)pose.TargetZ);

                    if (IsValidPoint(target))
                    {
                        AddCrossMarker(targetBuilder, target, targetMarkerSize);

                        if (validMeanTarget)
                            errorBuilder.AddLine(target, meanTarget);
                    }
                }

                if (validMeanTarget)
                {
                    AddCrossMarker(meanBuilder, meanTarget, meanMarkerSize);
                    AddBoxMarker(meanBuilder, meanTarget, meanMarkerSize * 0.5f);
                }

                Console.WriteLine("===== Hand-Eye Validation View =====");
                Console.WriteLine("View Mode = Raw World View");
            }

            LineGeometry3D baseGeometry = ToGeometryOrNull(baseBuilder);
            if (baseGeometry != null)
                baseGeometry.Colors = baseColors;

            LineGeometry3D gripperGeometry = ToGeometryOrNull(gripperBuilder);
            if (gripperGeometry != null)
                gripperGeometry.Colors = gripperColors;

            LineGeometry3D cameraGeometry = ToGeometryOrNull(cameraBuilder);
            LineGeometry3D targetGeometry = ToGeometryOrNull(targetBuilder);
            LineGeometry3D meanGeometry = ToGeometryOrNull(meanBuilder);
            LineGeometry3D errorGeometry = ToGeometryOrNull(errorBuilder);

            Dispatcher.BeginInvoke(new Action(() =>
            {
                ClearCameraOriginGeometry();

                baseAxesModel.Geometry = baseGeometry;
                gripperAxesModel.Geometry = gripperGeometry;
                cameraFrustumModel.Geometry = cameraGeometry;
                targetPointModel.Geometry = targetGeometry;
                meanTargetModel.Geometry = meanGeometry;
                errorVectorModel.Geometry = errorGeometry;
                ClearOverlayLabels();

                Console.WriteLine($"Pose Count = {result.Poses.Count}");
                Console.WriteLine($"Mean Target = X:{result.MeanTargetX:F3}, Y:{result.MeanTargetY:F3}, Z:{result.MeanTargetZ:F3}");
                Console.WriteLine($"RMS Error = {result.RmsError:F3} mm");
                Console.WriteLine($"Max Error = {result.MaxError:F3} mm");
                Console.WriteLine($"StdDev X = {result.StdDevX:F3} mm");
                Console.WriteLine($"StdDev Y = {result.StdDevY:F3} mm");
                Console.WriteLine($"StdDev Z = {result.StdDevZ:F3} mm");

                viewPort3D.ZoomExtents();
                viewPort3D.InvalidateRender();
            }), DispatcherPriority.Render);
        }

        private static bool ShouldUseResidualView(HandEyeValidationResult result)
        {
            const double MaxRenderableAbs = 1000000.0; // mm 기준, 1000m 이상이면 Raw 표시 불가로 판단

            if (result == null || result.Poses == null || result.Poses.Count == 0)
                return true;

            if (!IsFinite(result.MeanTargetX) ||
                !IsFinite(result.MeanTargetY) ||
                !IsFinite(result.MeanTargetZ))
                return true;

            if (Math.Abs(result.MeanTargetX) > MaxRenderableAbs ||
                Math.Abs(result.MeanTargetY) > MaxRenderableAbs ||
                Math.Abs(result.MeanTargetZ) > MaxRenderableAbs)
                return true;

            foreach (var p in result.Poses)
            {
                if (p == null)
                    continue;

                if (!IsFinite(p.TargetX) ||
                    !IsFinite(p.TargetY) ||
                    !IsFinite(p.TargetZ))
                    return true;

                if (Math.Abs(p.TargetX) > MaxRenderableAbs ||
                    Math.Abs(p.TargetY) > MaxRenderableAbs ||
                    Math.Abs(p.TargetZ) > MaxRenderableAbs)
                    return true;
            }

            return false;
        }

        private static bool IsFinite(double v)
        {
            return !(double.IsNaN(v) || double.IsInfinity(v));
        }

        private static double GetResidualDisplayScale(double maxError)
        {
            /*
             * Residual View 자동 스케일.
             * Max Error가 너무 작으면 화면에서 안 보이고,
             * 너무 크면 화면 밖으로 나가므로 대략 반경 400 정도에 맞춥니다.
             */

            if (!IsFinite(maxError) || maxError <= 1e-9)
                return 1.0;

            double desiredRadius = 400.0;
            double scale = desiredRadius / maxError;

            // 너무 과도한 확대/축소 방지
            if (scale < 0.01)
                scale = 0.01;

            if (scale > 1000.0)
                scale = 1000.0;

            return scale;
        }

        private static LineGeometry3D ToGeometryOrNull(LineBuilder builder)
        {
            if (builder == null)
                return null;

            LineGeometry3D geometry = builder.ToLineGeometry3D();

            if (geometry == null || geometry.Positions == null || geometry.Positions.Count == 0)
                return null;

            return geometry;
        }

        private void ClearCameraOriginGeometry()
        {
            if (cameraModel != null)
                cameraModel.Geometry = null;

            if (cameraToEffectorLineModel != null)
                cameraToEffectorLineModel.Geometry = null;

            if (endEffectorAxesModel != null)
                endEffectorAxesModel.Geometry = null;

            if (endEffectorBodyModel != null)
                endEffectorBodyModel.Geometry = null;

            if (endEffectorXModel != null)
                endEffectorXModel.Geometry = null;

            if (endEffectorYModel != null)
                endEffectorYModel.Geometry = null;

            if (endEffectorZModel != null)
                endEffectorZModel.Geometry = null;
        }

        private void ClearHandEyeValidationGeometry()
        {
            if (baseAxesModel != null)
                baseAxesModel.Geometry = null;

            if (gripperAxesModel != null)
                gripperAxesModel.Geometry = null;

            if (cameraFrustumModel != null)
                cameraFrustumModel.Geometry = null;

            if (targetPointModel != null)
                targetPointModel.Geometry = null;

            if (meanTargetModel != null)
                meanTargetModel.Geometry = null;

            if (errorVectorModel != null)
                errorVectorModel.Geometry = null;
        }

        private static double[,] CreateIdentity4x4()
        {
            double[,] m = new double[4, 4];

            m[0, 0] = 1.0;
            m[1, 1] = 1.0;
            m[2, 2] = 1.0;
            m[3, 3] = 1.0;

            return m;
        }

        private static Vector3 TransformPoint(double[,] m, float x, float y, float z)
        {
            return new Vector3(
                (float)(m[0, 0] * x + m[0, 1] * y + m[0, 2] * z + m[0, 3]),
                (float)(m[1, 0] * x + m[1, 1] * y + m[1, 2] * z + m[1, 3]),
                (float)(m[2, 0] * x + m[2, 1] * y + m[2, 2] * z + m[2, 3])
            );
        }

        private static Vector3 GetTranslation(double[,] m)
        {
            return new Vector3(
                (float)m[0, 3],
                (float)m[1, 3],
                (float)m[2, 3]);
        }

        private static bool IsValidTransform(double[,] m)
        {
            if (m == null)
                return false;

            if (m.GetLength(0) != 4 || m.GetLength(1) != 4)
                return false;

            Vector3 p = GetTranslation(m);

            return IsValidPoint(p);
        }

        private static bool IsValidPoint(Vector3 p)
        {
            if (float.IsNaN(p.X) || float.IsNaN(p.Y) || float.IsNaN(p.Z))
                return false;

            if (float.IsInfinity(p.X) || float.IsInfinity(p.Y) || float.IsInfinity(p.Z))
                return false;

            // 장비 스케일 방어용. 필요하면 범위 조정.
            if (Math.Abs(p.X) > 10000000 ||
                Math.Abs(p.Y) > 10000000 ||
                Math.Abs(p.Z) > 10000000)
                return false;

            return true;
        }

        private static void AddColoredLine(
            LineBuilder builder,
            Color4Collection colors,
            Vector3 p0,
            Vector3 p1,
            Color4 color)
        {
            if (!IsValidPoint(p0) || !IsValidPoint(p1))
                return;

            builder.AddLine(p0, p1);

            if (colors != null)
            {
                colors.Add(color);
                colors.Add(color);
            }
        }

        private static void AddAxisTriangle(
            LineBuilder builder,
            Vector3 origin,
            Vector3 tip,
            float wingSize)
        {
            if (builder == null)
                return;

            if (!IsValidPoint(origin) || !IsValidPoint(tip))
                return;

            Vector3 dir = tip - origin;
            float len = dir.Length();

            if (len <= 1e-6f)
                return;

            dir.Normalize();

            Vector3 refAxis = Math.Abs(Vector3.Dot(dir, Vector3.UnitZ)) < 0.85f
                ? Vector3.UnitZ
                : Vector3.UnitY;

            Vector3 side = Vector3.Cross(dir, refAxis);

            if (side.Length() <= 1e-6f)
                side = Vector3.UnitX;

            side.Normalize();

            Vector3 baseCenter = origin + dir * (len * 0.70f);
            Vector3 wing1 = baseCenter + side * wingSize;
            Vector3 wing2 = baseCenter - side * wingSize;

            // 중심축 + 삼각형 arrow/fan outline
            builder.AddLine(origin, tip);
            builder.AddLine(tip, wing1);
            builder.AddLine(wing1, wing2);
            builder.AddLine(wing2, tip);
            builder.AddLine(origin, wing1);
            builder.AddLine(origin, wing2);
        }

        private static void AddAxis(
            LineBuilder builder,
            Color4Collection colors,
            double[,] transform,
            float length)
        {
            if (!IsValidTransform(transform))
                return;

            Vector3 origin = TransformPoint(transform, 0, 0, 0);
            Vector3 xAxis = TransformPoint(transform, length, 0, 0);
            Vector3 yAxis = TransformPoint(transform, 0, length, 0);
            Vector3 zAxis = TransformPoint(transform, 0, 0, length);

            AddColoredLine(builder, colors, origin, xAxis, DxColor.Red);
            AddColoredLine(builder, colors, origin, yAxis, DxColor.Green);
            AddColoredLine(builder, colors, origin, zAxis, DxColor.Blue);
        }

        private static void AddCameraFrustum(
            LineBuilder builder,
            double[,] transform,
            float size)
        {
            if (!IsValidTransform(transform))
                return;

            // OpenCV 카메라 좌표계 기준:
            // X: Right, Y: Down, Z: Forward
            Vector3 c = TransformPoint(transform, 0, 0, 0);

            Vector3 p1 = TransformPoint(transform, -size, -size, size * 2.0f);
            Vector3 p2 = TransformPoint(transform, size, -size, size * 2.0f);
            Vector3 p3 = TransformPoint(transform, size, size, size * 2.0f);
            Vector3 p4 = TransformPoint(transform, -size, size, size * 2.0f);

            if (!IsValidPoint(c) ||
                !IsValidPoint(p1) ||
                !IsValidPoint(p2) ||
                !IsValidPoint(p3) ||
                !IsValidPoint(p4))
                return;

            // Camera 중심 -> Frustum 4코너
            builder.AddLine(c, p1);
            builder.AddLine(c, p2);
            builder.AddLine(c, p3);
            builder.AddLine(c, p4);

            // Frustum 사각형
            builder.AddLine(p1, p2);
            builder.AddLine(p2, p3);
            builder.AddLine(p3, p4);
            builder.AddLine(p4, p1);

            // Camera 내부 좌표축을 짧게 표시
            Vector3 x = TransformPoint(transform, size * 1.2f, 0, 0);
            Vector3 y = TransformPoint(transform, 0, size * 1.2f, 0);
            Vector3 z = TransformPoint(transform, 0, 0, size * 1.2f);

            builder.AddLine(c, x);
            builder.AddLine(c, y);
            builder.AddLine(c, z);
        }

        private static void AddMiniFrustum(
            LineBuilder builder,
            Color4Collection colors,
            double[,] transform,
            float size)
        {
            if (!IsValidTransform(transform))
                return;

            Vector3 c = TransformPoint(transform, 0, 0, 0);
            Vector3 p1 = TransformPoint(transform, -size * 0.55f, -size * 0.40f, size);
            Vector3 p2 = TransformPoint(transform, size * 0.55f, -size * 0.40f, size);
            Vector3 p3 = TransformPoint(transform, size * 0.55f, size * 0.40f, size);
            Vector3 p4 = TransformPoint(transform, -size * 0.55f, size * 0.40f, size);

            Color4 bodyColor = new DxColor4(1f, 1f, 1f, 1f);

            AddColoredLine(builder, colors, c, p1, bodyColor);
            AddColoredLine(builder, colors, c, p2, bodyColor);
            AddColoredLine(builder, colors, c, p3, bodyColor);
            AddColoredLine(builder, colors, c, p4, bodyColor);

            AddColoredLine(builder, colors, p1, p2, bodyColor);
            AddColoredLine(builder, colors, p2, p3, bodyColor);
            AddColoredLine(builder, colors, p3, p4, bodyColor);
            AddColoredLine(builder, colors, p4, p1, bodyColor);
        }

        private static void AddCrossMarker(
            LineBuilder builder,
            Vector3 center,
            float size)
        {
            if (!IsValidPoint(center))
                return;

            float h = size * 0.5f;

            Vector3 x0 = new Vector3(center.X - h, center.Y, center.Z);
            Vector3 x1 = new Vector3(center.X + h, center.Y, center.Z);

            Vector3 y0 = new Vector3(center.X, center.Y - h, center.Z);
            Vector3 y1 = new Vector3(center.X, center.Y + h, center.Z);

            Vector3 z0 = new Vector3(center.X, center.Y, center.Z - h);
            Vector3 z1 = new Vector3(center.X, center.Y, center.Z + h);

            builder.AddLine(x0, x1);
            builder.AddLine(y0, y1);
            builder.AddLine(z0, z1);
        }

        private static void AddBoxMarker(
            LineBuilder builder,
            Vector3 center,
            float size)
        {
            if (!IsValidPoint(center))
                return;

            float h = size * 0.5f;

            Vector3 p000 = new Vector3(center.X - h, center.Y - h, center.Z - h);
            Vector3 p001 = new Vector3(center.X - h, center.Y - h, center.Z + h);
            Vector3 p010 = new Vector3(center.X - h, center.Y + h, center.Z - h);
            Vector3 p011 = new Vector3(center.X - h, center.Y + h, center.Z + h);

            Vector3 p100 = new Vector3(center.X + h, center.Y - h, center.Z - h);
            Vector3 p101 = new Vector3(center.X + h, center.Y - h, center.Z + h);
            Vector3 p110 = new Vector3(center.X + h, center.Y + h, center.Z - h);
            Vector3 p111 = new Vector3(center.X + h, center.Y + h, center.Z + h);

            builder.AddLine(p000, p100);
            builder.AddLine(p100, p110);
            builder.AddLine(p110, p010);
            builder.AddLine(p010, p000);

            builder.AddLine(p001, p101);
            builder.AddLine(p101, p111);
            builder.AddLine(p111, p011);
            builder.AddLine(p011, p001);

            builder.AddLine(p000, p001);
            builder.AddLine(p100, p101);
            builder.AddLine(p110, p111);
            builder.AddLine(p010, p011);
        }

        /// <summary>
        /// 뷰어의 화면을 초기 상태로 리셋합니다. (새로운 측정 시작 시 활용)
        /// </summary>
        public void ClearViewer()
        {
            Dispatcher.BeginInvoke(new Action(() =>
            {
                ClearOverlayLabels();
                viewPort3D.InvalidateRender();
            }));
        }
    }
}
