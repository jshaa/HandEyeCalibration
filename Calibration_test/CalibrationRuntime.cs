using System;

namespace Calibration_test
{
    /// <summary>
    /// Composition root that creates Hand-Eye calibration services around one shared HandEyeParams instance.
    /// </summary>
    public sealed class CalibrationRuntime : IDisposable
    {
        /// <summary>
        /// Shared calibration parameter instance used by every service in this runtime.
        /// </summary>
        public HandEyeParams Parameters { get; private set; }

        /// <summary>
        /// Shared math utility for matrix, rotation, and camera parameter conversions.
        /// </summary>
        public MathUtils Math { get; private set; }

        /// <summary>
        /// Shared optimizer service for 12-DoF refinement and robot/camera pose conversion.
        /// </summary>
        public CalibrationOptimizer Optimizer { get; private set; }

        /// <summary>
        /// Main calibration service that runs OpenCV Hand-Eye calibration and validation.
        /// </summary>
        public HandEyeCalibration Calibration { get; private set; }

        /// <summary>
        /// Vision service that detects checkerboards and estimates PnP target poses.
        /// </summary>
        public BoardDetector BoardDetector { get; private set; }

        /// <summary>
        /// Creates the runtime object graph with default parameters.
        /// </summary>
        public CalibrationRuntime()
            : this(new HandEyeParams())
        {
        }

        /// <summary>
        /// Creates the runtime object graph around the supplied shared parameter instance.
        /// </summary>
        /// <param name="parameters">Shared Hand-Eye parameters.</param>
        public CalibrationRuntime(HandEyeParams parameters)
        {
            Parameters = parameters ?? throw new ArgumentNullException(nameof(parameters));
            Math = new MathUtils(Parameters);
            Optimizer = new CalibrationOptimizer(Parameters, Math);
            Calibration = new HandEyeCalibration(Parameters, Math, Optimizer);
            BoardDetector = new BoardDetector(Parameters, Math);
        }

        /// <summary>
        /// Releases accumulated OpenCV Mat pose data owned by the calibration service.
        /// </summary>
        public void Dispose()
        {
            if (Calibration != null)
            {
                Calibration.Dispose();
                Calibration = null;
            }
        }
    }
}
