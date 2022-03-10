package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import frc.robot.limelight.CameraResolution;
import frc.robot.limelight.LimelightConfig;
import frc.robot.limelight.PipelineConfiguration;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.util.InterpolatingDouble;
import libraries.cheesylib.util.InterpolatingTreeMap;
import libraries.cheesylib.vision.GoalTrackerConfig;

public class Constants {
    /* All distance measurements are in inches, unless otherwise noted. */

    public static final String kJuniorName = "Junior";
    public static final String kDeadEyeName = "DeadEye";
    public static final String kRobot2022Name = "Robot2022";

    public static final double kLooperDt = 0.02;

    public static final double kEpsilon = 0.0001;

    public static final boolean kIsUsingCompBot = true;
    public static final boolean kIsUsingTractionWheels = true;

    public static final boolean kDebuggingOutput = false;

    // Physical Robot Dimensions (including bumpers)
    public static final double kRobotWidth = 28.0;
    public static final double kRobotLength = 28.0;
    public static final double kRobotHalfWidth = kRobotWidth / 2.0;
    public static final double kRobotHalfLength = kRobotLength / 2.0;
    public static final double kRobotProbeExtrusion = 4.0;

    public static final double kBallRadius = 6.5;

    // Field Landmarks
    public static final Pose2d closeHatchPosition = new Pose2d(
            new Translation2d(48.0 + 166.57, 27.44 - 10.0 - 162.0), Rotation2d.fromDegrees(-30.0));
    public static final Pose2d rightCloseHatchPosition = new Pose2d(
            new Translation2d(closeHatchPosition.getTranslation().x(),
                    -closeHatchPosition.getTranslation().y()),
            Rotation2d.fromDegrees(30.0));
    public static final Pose2d farHatchPosition = new Pose2d(
            new Translation2d(229.13 + 14.71, 27.44 - 10.0 - 162.0), Rotation2d.fromDegrees(-150.0));
    public static final Pose2d humanLoaderPosition = new Pose2d(new Translation2d(0.0, 25.72 - 162.0),
            Rotation2d.fromDegrees(0.0));
    public static final Pose2d autoBallPosition = new Pose2d(
            new Translation2d(48.0 - 4.0 - kBallRadius, 97.0 - (3.0 * kBallRadius) - 162.0),
            Rotation2d.fromDegrees(-45.0));
    public static final Pose2d rocketPortPosition = new Pose2d(new Translation2d(229.13, 27.44 - 162.0),
            Rotation2d.fromDegrees(-90.0));

    public static final Pose2d closeShipPosition = new Pose2d(new Translation2d(260.8, -28.87),
            Rotation2d.fromDegrees(90.0));
    public static final Pose2d midShipPosition = new Pose2d(new Translation2d(282.55 + 1.0, -28.87),
            Rotation2d.fromDegrees(90.0));
    public static final Pose2d farShipPosition = new Pose2d(new Translation2d(304.3, -28.87),
            Rotation2d.fromDegrees(90.0));

    public static final Pose2d kRobotStartingPose = Pose2d.identity();
    public static final Pose2d kRobotSpecialPose = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180));
    public static final Pose2d kRobotLeftStartingPose = new Pose2d(
            new Translation2d(48.0 + kRobotHalfLength, 97.0 + kRobotHalfWidth - 162.0),
            Rotation2d.fromDegrees(0));
    public static final Pose2d kRobotRightStartingPose = new Pose2d(
            new Translation2d(48.0 + kRobotHalfLength, -(97.0 + kRobotHalfWidth - 162.0)),
            Rotation2d.fromDegrees(0));
    public static final Pose2d kRobotLeftRampExitPose = new Pose2d(
            new Translation2d(95.25 + kRobotHalfLength, 97.0 + kRobotHalfWidth - 162.0),
            Rotation2d.fromDegrees(0));
    public static final Pose2d kRobotRightRampExitPose = new Pose2d(
            new Translation2d(95.25 + kRobotHalfLength, -(97.0 + kRobotHalfWidth - 162.0)),
            Rotation2d.fromDegrees(0));



    // Path following constants
    public static final double kPathLookaheadTime = 0.25; // seconds to look ahead along the path for steering 0.4
    public static final double kPathMinLookaheadDistance = Units.inchesToMeters(6.0); // inches 24.0 (we've been
                                                                                      // using 3.0)
    // NEW SWERVE
    // TODO: use SDS MK4 module configurations file.

    public static final double kMK2_WheelDiameter = 0.1016;
    public static final double kMK2_DriveReduction = (15.0 / 32.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final boolean kMK2_DriveInverted = true;
    public static final double kMK2_SteerReduction = (15.0 / 32.0) * (10.0 / 60.0);
    public static final boolean kMK2_iSteerInverted = true;

    // These settings are for an inverted Mk4 L2. The steering reduction is
    // different:
    // (14.0/50.0) verses (15.0 / 32.0) on a standard Mk4_L2.
    public static final double kMK4_L2iWheelDiameter = 0.10033;
    public static final double kMK4_L2iDriveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final boolean kMK4_L2iDriveInverted = true;
    public static final double kMK4_L2iSteerReduction = (14.0 / 50.0) * (10.0 / 60.0);
    public static final boolean kMK4_L2iSteerInverted = true;

    public static final double kDriveDeadband = 0.05;

    // New Swerve requires SI units
    // NOTE: All Robot specific configuration for Junior, DeadEye, and Robot2022 can
    // be found in config folder.
    // Robot specific Swerve configuration is done in SwerveConfiguration
    // Module specific configuration is done in SwerveDriveConstants

    // Swerve Heading Controller
    public static final double kSwerveHeadingControllerErrorTolerance = 1.0; // degrees

    // END NEW SWERVE

    // LIMELIGHT
    // TODO:  Create Robot specific configurations
    // Goal Tracker
//    public static final double kHorizontalFOV = Math.toRadians(59.6);
//    public static final double kVerticalFOV = Math.toRadians(49.7);

//    public static final double kVPW = 2.0 * Math.tan(kHorizontalFOV / 2);
//    public static final double kVPH = 2.0 * Math.tan(kVerticalFOV / 2);

    public static final boolean kUseTopCorners = false;


    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds

    public static final double kMaxTrackerDistance = Units.feetToMeters(9.0);
    public static final double kMaxGoalTrackAge = 2.5;
    public static final double kMaxGoalTrackSmoothingTime = 0.5;
    public static final double kCameraFrameRate = 90.0; // fps

    public static GoalTrackerConfig kGoalTrackerConfig = new GoalTrackerConfig(
            kMaxTrackerDistance, kMaxGoalTrackAge, kMaxGoalTrackSmoothingTime, kCameraFrameRate);

    public static final double kTrackStabilityWeight = 0.0;
    public static final double kTrackAgeWeight = 10.0;
    public static final double kTrackSwitchingWeight = 100.0;

    public static final double kLimelightLensOffGroundHeight = Units.inchesToMeters(39.8);
    public static final Rotation2d kLimelightHorizontalPlaneToLens = Rotation2d.fromDegrees(38.00);

    // Approximately 8' 8 inches per manual
    public static final double kVisionTargetHeight = Units.inchesToMeters(12.0 * 8.0 + 8.0);

    // Distance between center of shooter and limelight's camera lens
    public static final Pose2d kShooterToLens  = new Pose2d(0.0, -9.373 - (-1.85), Rotation2d.fromDegrees(0));

    // Rim thickness + inner diameter of upper hub.  Reflective tape is at front rim.  Shot should land in center.
    public static final Pose2d kVisionTargetToGoalOffset = new Pose2d(
            Units.inchesToMeters(3.0 + 24.0) , 0, Rotation2d.identity());

    public static final PipelineConfiguration kLowRes1xZoom = new PipelineConfiguration(
            CameraResolution.F_320x240, 1.0);
    public static final PipelineConfiguration kLowRes2xZoom = new PipelineConfiguration(
            CameraResolution.F_320x240, 2.0);

    public static LimelightConfig kLimelight2Config = new LimelightConfig(
            1, // label id
                LimelightConfig.Type.Shooter,
                "Shooter Limelight #1", // name
                "limelight", // table name
                Constants.kLimelightLensOffGroundHeight, // height
                Constants.kShooterToLens, // shooter to lens
                Constants.kLimelightHorizontalPlaneToLens, // horizontalPlaneToLens,
                65.0, //64.03840065743408,
                50.0 //50.34836606499798
            );

    // Goal tracker constants
    public static final double kDefaultCurveDistance = kRobotHalfLength + 36.0;
    public static final double kVisionUpdateDistance = kRobotHalfLength + 75.0;
    public static final double kVisionDistanceStep = 4.0;
    public static final double kClosestVisionDistance = 26.0;// 36.0
    public static final double kDefaultVisionTrackingSpeed = 42.0;
    public static final double kCurvedVisionYOffset = 0.375;// 1.25

    // Vision Speed Constraint Treemap
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kVisionSpeedTreemap = new InterpolatingTreeMap<>();
    static {
        kVisionSpeedTreemap.put(new InterpolatingDouble(-6.0), new InterpolatingDouble(24.0));
        kVisionSpeedTreemap.put(new InterpolatingDouble(kClosestVisionDistance), new InterpolatingDouble(24.0));
        kVisionSpeedTreemap.put(new InterpolatingDouble(60.0), new InterpolatingDouble(48.0));
        kVisionSpeedTreemap.put(new InterpolatingDouble(300.0), new InterpolatingDouble(48.0));
    }
    // END LIMELIGHT


    // Panel Manipulator Constants
    public static final int kPanelManipulatorTalonID = 0; // TODO: temporary value, need adjustments
    public static final int kPanelManipulatorSolenoidID = 0; // TODO: temporary value, need adjustments

    public static final double kPanelManipulatorSpinSpeed = 0.33; // TODO: temporary value, need adjustments

    public static final int kJoystickRumbleFrequency = 1; // TODO: temporary value, need adjustments
    public static final int kJoystickRumbleDuration = 1; // TODO: temporary value, need adjustments

    public static final double kRotationControlLimit = 3.5; // TODO: temporary value, need adjustments
    public static final double kRotationsPerColorChange = 0.125; // TODO: temporary value, need adjustments

    public static final float[] kBlueMNStockReadings = { 1.00f, 0.60f, 0.70f, 0.35f, 0.30f, 0.10f };
    public static final float[] kGreenMNStockReadings = { 0.30f, 0.90f, 1.00f, 0.70f, 0.50f, 0.15f };
    public static final float[] kRedMNStockReadings = { 0.15f, 0.15f, 0.50f, 0.40f, 1.00f, 0.55f };
    public static final float[] kYellowMNStockReadings = { 0.25f, 0.40f, 1.00f, 1.00f, 1.00f, 0.35f };

    public static final float[] kBlueSNStockReadings = { 0.34f, 0.22f, 0.15f, 0.12f, 0.09f, 0.05f };
    public static final float[] kGreenSNStockReadings = { 0.16f, 0.23f, 0.24f, 0.19f, 0.10f, 0.05f };
    public static final float[] kRedSNStockReadings = { 0.07f, 0.05f, 0.09f, 0.09f, 0.42f, 0.20f };
    public static final float[] kYellowSNStockReadings = { 0.05f, 0.10f, 0.26f, 0.22f, 0.25f, 0.08f };

    public static final int kAS7262RefreshRate = 10; // measured in Hz

    // Scrub Factors
    public static final boolean kSimulateReversedCarpet = false;
    public static final double[] kWheelScrubFactors = new double[] { 1.0, 1.0, 1.0, 1.0 };
    public static final double kXScrubFactor = 1.0 / (1.0 - (9549.0 / 293093.0));
    public static final double kYScrubFactor = 1.0 / (1.0 - (4.4736 / 119.9336));

    // Voltage-Velocity equation constants {m, b, x-intercept}
    // First set is the positive direction, second set is negative
    public static final double[][][] kVoltageVelocityEquations = new double[][][] {
            { { 1.70, -4.39, 2.58 }, { 1.83, 5.23, -2.85 } },
            { { 1.59, -3.86, 2.42 }, { 1.43, 3.09, -2.16 } },
            { { 1.53, -3.66, 2.39 }, { 1.66, 4.15, -2.50 } },
            { { 1.84, -4.70, 2.56 }, { 1.85, 5.34, -2.89 } } };

    // LED/Canifier
    public static final int kCanifierId = 0;
    public static final int kCANTimeoutMs = 10; // use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

}
