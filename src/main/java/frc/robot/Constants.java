package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.LimeLights.LimeLight;
import frc.robot.subsystems.SwerveDriveModule;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.util.InterpolatingDouble;
import libraries.cheesylib.util.InterpolatingTreeMap;

public class Constants {
    /* All distance measurements are in inches, unless otherwise noted. */

    public static final String kCetusName = "Cetus";
    public static final String kRobot1Name = "Robot1";
    public static final String kRobot2Name = "Robot2";

    public static final double kLooperDt = 0.02;

    public static final double kEpsilon = 0.0001;

    public static final boolean kIsUsingCompBot = true;
    public static final boolean kIsUsingTractionWheels = true;

    public static final boolean kDebuggingOutput = false;

    //Physical Robot Dimensions (including bumpers)
    public static final double kRobotWidth = 28.0;
    public static final double kRobotLength = 28.0;
    public static final double kRobotHalfWidth = kRobotWidth / 2.0;
    public static final double kRobotHalfLength = kRobotLength / 2.0;
    public static final double kRobotProbeExtrusion = 4.0;

    public static final double kBallRadius = 6.5;

    //Field Landmarks
    public static final Pose2d closeHatchPosition = new Pose2d(new Translation2d(48.0 + 166.57, 27.44 - 10.0 - 162.0), Rotation2d.fromDegrees(-30.0));
    public static final Pose2d rightCloseHatchPosition = new Pose2d(new Translation2d(closeHatchPosition.getTranslation().x(), -closeHatchPosition.getTranslation().y()), Rotation2d.fromDegrees(30.0));
    public static final Pose2d farHatchPosition = new Pose2d(new Translation2d(229.13 + 14.71, 27.44 - 10.0 - 162.0), Rotation2d.fromDegrees(-150.0));
    public static final Pose2d humanLoaderPosition = new Pose2d(new Translation2d(0.0, 25.72 - 162.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d autoBallPosition = new Pose2d(new Translation2d(48.0 - 4.0 - kBallRadius, 97.0 - (3.0*kBallRadius) - 162.0), Rotation2d.fromDegrees(-45.0));
    public static final Pose2d rocketPortPosition = new Pose2d(new Translation2d(229.13, 27.44 - 162.0), Rotation2d.fromDegrees(-90.0));

    public static final Pose2d closeShipPosition = new Pose2d(new Translation2d(260.8, -28.87), Rotation2d.fromDegrees(90.0));
    public static final Pose2d midShipPosition = new Pose2d(new Translation2d(282.55 + 1.0, -28.87), Rotation2d.fromDegrees(90.0));
    public static final Pose2d farShipPosition = new Pose2d(new Translation2d(304.3, -28.87), Rotation2d.fromDegrees(90.0));

    public static final double kOuterTargetHeight = 98.25 - 7.5;
    public static final double kDiskTargetHeight = 28.625;//28.1875
    public static final double kBallTargetHeight = 36.9375;
    public static final List<Rotation2d> kPossibleDiskTargetAngles = Arrays.asList(
            /* Rotation2d.fromDegrees(0.0), */ Rotation2d.fromDegrees(30.0), Rotation2d.fromDegrees(150.0),
            /* Rotation2d.fromDegrees(180.0), */ Rotation2d.fromDegrees(-150.0), Rotation2d.fromDegrees(-30.0));
    public static final List<Rotation2d> kPossibleBallTargetAngles = Arrays.asList(Rotation2d.fromDegrees(90.0),
            Rotation2d.fromDegrees(-90.0));

    public static final Pose2d kRobotStartingPose = new Pose2d();
    public static final Pose2d kRobotSpecialPose = new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(180));
    public static final Pose2d kRobotLeftStartingPose = new Pose2d(
            new Translation2d(48.0 + kRobotHalfLength, 97.0 + kRobotHalfWidth - 162.0), Rotation2d.fromDegrees(0));
    public static final Pose2d kRobotRightStartingPose = new Pose2d(
            new Translation2d(48.0 + kRobotHalfLength, -(97.0 + kRobotHalfWidth - 162.0)), Rotation2d.fromDegrees(0));
    public static final Pose2d kRobotLeftRampExitPose = new Pose2d(
            new Translation2d(95.25 + kRobotHalfLength, 97.0 + kRobotHalfWidth - 162.0), Rotation2d.fromDegrees(0));
    public static final Pose2d kRobotRightRampExitPose = new Pose2d(
            new Translation2d(95.25 + kRobotHalfLength, -(97.0 + kRobotHalfWidth - 162.0)), Rotation2d.fromDegrees(0));
    // public static final Pose2d kRobotLeftRampExitPose = new Pose2d(new
    // Translation2d(48.0 + kRobotHalfLength, -75.25 - kRobotHalfWidth),
    // Rotation2d.fromDegrees(0));
    // public static final Pose2d kRobotRightRampExitPose = new Pose2d(new
    // Translation2d(48.0 + kRobotHalfLength, 75.25 + kRobotHalfWidth),
    // Rotation2d.fromDegrees(0));

    // Swerve Calculations Constants (measurements are in inches)
    public static final double kWheelbaseLength = 21.0;
    public static final double kWheelbaseWidth = 21.0;

    public static double kMaxAngleAimError = 1;
    public static final double kMaxAimTurningVelocity = 0.1;

    // TODO: "fix em up later" -Ram
    // shootwards limelight
    public static final LimeLight.LimelightConstants kShootwardsLimelightConstants = new LimeLight.LimelightConstants();
    static {
        kShootwardsLimelightConstants.kName = "ShootwardsLimelight";
        kShootwardsLimelightConstants.kTableName = "limelight-shooter";
        kShootwardsLimelightConstants.kHeight = 23 /*22.25*/;// cetus: 11  // robot1: 22.25// pinkeye: 23// inches
        kShootwardsLimelightConstants.kSubsystemToLens = new Pose2d(new Translation2d(0, 0.0), Rotation2d.fromDegrees(0.0)); // 0.0 brian // right is positive // -1.5
        kShootwardsLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(20.5); //38 // degrees
        kShootwardsLimelightConstants.kExpectedTargetCount = new double[] {1, 3}; // expect 2 targets (2 top corners)
        kShootwardsLimelightConstants.kPipelineZoom = new int[] {1, 2};
        kShootwardsLimelightConstants.kTargets = new Target[] {
                Target.OUTER_GOAL_MAIN_COUNTOUR,
                Target.OUTER_GOAL_CORNERS,
                Target.OUTER_GOAL_CORNERS
        };
    }

    // shootwards zoom constants

    // when zoomed in (2), if robot closer than 12.5 ft, zoom out (1) will occur.
    // if zoomed out (1), if robot farther than 13.5, zoom in may occur (2).
    // overlap keeps zooms from jumping back and forth
    public static final double kZoomOutDistance = 13.5; // ft
    public static final double kZoomInDistance = 14.5; // ft
    public static final double kZoomInProportion = 0.4; // for zoom 1 - 2, target must be within kZoomConstant * FOV for each x and y
    public static final double kZoomOutProportion = 0.95;
    public static final double kTimeBeforeZoomSwitch = 0.1; // 0.1 sec must pass to zoom in or out

    // collectwards limelight
    public static final LimeLight.LimelightConstants kCollectwardsLimelightConstants = new LimeLight.LimelightConstants();
    static {
        kCollectwardsLimelightConstants.kName = "CollectwardsLimelight";
        kCollectwardsLimelightConstants.kTableName = "limelight-collect"; // TODO: "limelight-collect"
        kCollectwardsLimelightConstants.kHeight = 22;  // deadeye 23, pinkeye 22
        kCollectwardsLimelightConstants.kSubsystemToLens = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
        kCollectwardsLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(-29.0);
        kCollectwardsLimelightConstants.kExpectedTargetCount = new double[] {1, 3};
        kCollectwardsLimelightConstants.kPipelineZoom = new int[] {1};
        kCollectwardsLimelightConstants.kTargets = new Target[] {
                Target.POWER_CELL,
                Target.POWER_CELL,
                Target.POWER_CELL
        };
    }

    // collectwards limelight testing
    // public static final LimelightConstants kCollectwardsLimelightConstants = new LimelightConstants();
    // static {
    // 	kCollectwardsLimelightConstants.kName = "Collectwards Limelight";
    //     kCollectwardsLimelightConstants.kTableName = "limelight-shooter";
    //     kCollectwardsLimelightConstants.kHeight = 25.25/*22.25*/;// cetus: 11  // robot1: 22.25// inches
    //     kCollectwardsLimelightConstants.kSubsystemToLens = new Pose2d(new Translation2d(0, 0.0), Rotation2d.fromDegrees(-0.7)); // right is positive
    // 	kCollectwardsLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(20.5); //38 // degrees
    // 	kCollectwardsLimelightConstants.kExpectedTargetCount = new double[] {1, 3}; // expect 2 targets (2 top corners)
    // 	kCollectwardsLimelightConstants.kPiplineZoom = new int[] {1, 2};
    // }

    //target constants
    public enum Target {
        OUTER_GOAL_MAIN_COUNTOUR,
        OUTER_GOAL_CORNERS,
        POWER_CELL;

        double[] targetHeights = new double[] {
                /*98.25*/ 70 - 7.5,
                /*98.25*/70,
                //raynli
                /*43.5*/ 4.5
        };

        public double getHeight() {
            return targetHeights[this.ordinal()];
        }
    }

    public static final double kGoalTargetHeight = 98.25;
    public static final double kPowerCellTargetHeight = 4.5;
    public static final double kCornerToCornerLength = 39.26;

    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds
    public static final double kHorizontalFOV = Math.toRadians(59.6);
    public static final double kVerticalFOV = Math.toRadians(49.7);

    public static final double kVPW = 2.0 * Math.tan(kHorizontalFOV / 2);
    public static final double kVPH = 2.0 * Math.tan(kVerticalFOV / 2);

    public static final int[] pixelFrameLowRes = new int[] {320, 240};
    public static final int[] pixelFrameHighRes = new int[] {960, 720};


    //Goal tracker constants
    public static double kMaxGoalTrackAge = 0.5;//0.5
    public static double kMaxTrackerDistance = 60.0;//18.0
    public static double kCameraFrameRate = 90.0;
    public static double kTrackReportComparatorStablityWeight = 1.0;
    public static double kTrackReportComparatorAgeWeight = 1.0;
    public static final double kDefaultCurveDistance = kRobotHalfLength + 36.0;
    public static final double kVisionUpdateDistance = kRobotHalfLength + 75.0;
    public static final double kVisionDistanceStep = 4.0;
    public static final double kClosestVisionDistance = 26.0;//36.0
    public static final double kDefaultVisionTrackingSpeed = 42.0;
    public static final double kCurvedVisionYOffset = 0.375;//1.25

    //Vision Speed Constraint Treemap
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kVisionSpeedTreemap = new InterpolatingTreeMap<>();
    static{
        kVisionSpeedTreemap.put(new InterpolatingDouble(-6.0), new InterpolatingDouble(24.0));
        kVisionSpeedTreemap.put(new InterpolatingDouble(kClosestVisionDistance), new InterpolatingDouble(24.0));
        kVisionSpeedTreemap.put(new InterpolatingDouble(60.0), new InterpolatingDouble(48.0));
        kVisionSpeedTreemap.put(new InterpolatingDouble(300.0), new InterpolatingDouble(48.0));
    }

    //Path following constants
    public static final double kPathLookaheadTime = 0.25;  // seconds to look ahead along the path for steering 0.4
    public static final double kPathMinLookaheadDistance = 6.0;  // inches 24.0 (we've been using 3.0)

    //Swerve Odometry Constants
    public static final double kSwerveWheelDiameter = 4.0901; //inches (actual diamter is closer to 3.87, but secondary algorithm prefers 4.0901) 3.76
    public static final double kSwerveRotationMotorTicksPerRotation = 2048.0 * 18.0; // FX encoder ticks per module rotation
    /** The number of rotations the swerve drive encoder undergoes for every rotation of the wheel. */
    public static final double kSwerveDriveTicksPerWheelRev = .85*14178; //brian 1.21 new gear ratio 2048 * 6.923 //SwerveDriveEncoderResolution * kSwerveEncoderToWheelRatio;
    public static final double kSwerveEncUnitsPerInch = kSwerveDriveTicksPerWheelRev / (Math.PI * kSwerveWheelDiameter);

    //Swerve Speed Constants
    //max ticks/100ms; 2048 ticks/shaft rev * 106 max rev/sec=21777.1
    // setting slightly below to assure it is reachable
    public static final double kSwerveDriveMaxSpeed = 21000.0; // drive motor ticks/100ms
    // start with velocity of motor shaft kSwerveDriveMaxSpeed*10 in ticks/sec * 6.923 (gear ratio) * wheel diameter * pi
    public static final double kSwerveMaxSpeedInchesPerSecond = kSwerveDriveMaxSpeed*10.0/(6.923*2048)*kSwerveWheelDiameter*Math.PI;
    //max ticks/100ms; 2048 ticks/shaft rev * 106 max rev/sec=21777.1
    // setting slightly below to assure it is reachable
    public static final double kSwerveRotationMaxSpeed = 21000.0; // rotation motor ticks/100ms
    public static final double kSwerveRotation10VoltMaxSpeed = 1350.0; // TODO: tune this!!!


    // 7/190 = .037
    public static final double kSwerveRotationSpeedScalar = ((1.0 / 0.125) - 1.0) / kSwerveMaxSpeedInchesPerSecond;

    //Swerve Module Wheel Offsets (cancoder Rotation encoder values in degrees when the wheels are facing 0 degrees), gear bevel on inside, cw rotation from bottom is negative
    public static final int kFrontRightCancoderStartingPosDegreesCetus = -112;
    public static final int kFrontLeftCancoderStartingPosDegreesCetus = 92;
    public static final int kRearLeftCancoderStartingPosDegreesCetus = -13;
    public static final int kRearRightCancoderStartingPosDegreesCetus = -130;

    public static final int kFrontRightCancoderStartingPosDegreesR1 = 359; //1; //-slight tuning blockparty day 2 //148;
    public static final int kFrontLeftCancoderStartingPosDegreesR1 = 211; //151; //345;//-wheel was reoriented for repairs (loose falcons) //332;-temp fix //45;
    public static final int kRearLeftCancoderStartingPosDegreesR1 = 235; //125; //5;
    public static final int kRearRightCancoderStartingPosDegreesR1 = 250;

    public static final int kFrontRightCancoderStartingPosDegreesR2 = 0;
    public static final int kFrontLeftCancoderStartingPosDegreesR2 = 0;
    public static final int kRearLeftCancoderStartingPosDegreesR2 = 0;
    public static final int kRearRightCancoderStartingPosDegreesR2 = 0;

    //Swerve Module Positions (relative to the center of the drive base)
    public static final Translation2d kVehicleToModuleZero = new Translation2d(kWheelbaseLength/2, kWheelbaseWidth/2);
    public static final Translation2d kVehicleToModuleOne = new Translation2d(kWheelbaseLength/2, -kWheelbaseWidth/2);
    public static final Translation2d kVehicleToModuleTwo = new Translation2d(-kWheelbaseLength/2, -kWheelbaseWidth/2);
    public static final Translation2d kVehicleToModuleThree = new Translation2d(-kWheelbaseLength/2, kWheelbaseWidth/2);

    public static final List<Translation2d> kModulePositions = Arrays.asList(kVehicleToModuleZero,
            kVehicleToModuleOne, kVehicleToModuleTwo, kVehicleToModuleThree);

    // NEW SWERVE
    // TODO: use SDS MK4 module configurations file.
    // These settings are for an inverted Mk4 L2.  The steering reduction is different:
    // (14.0/50.0) verses (15.0 / 32.0) on a standard Mk4_L2.
    public static final double kMK4_L2iWheelDiameter = 0.10033;
    public static final double kMK4_l2iDriveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final boolean kMK4_l2iDriveInverted = true;
    public static final double kMK4_l2iSteerReduction = (14.0/50.0) * (10.0 / 60.0);
    public static final boolean kMK4_l2iSteerInverted = true;

    // New Swerve requires SI units
    public static final double kWheelbaseLengthInMeters = Units.inchesToMeters(kWheelbaseLength);
    public static final double kWheelbaseWidthInMeters = Units.inchesToMeters(kWheelbaseWidth);

    public static final double kSwerveDiagonal = Math.hypot(kWheelbaseLength, kWheelbaseWidth);
    final static double kRadiusInMeters = Math.hypot(kWheelbaseWidthInMeters / 2, kWheelbaseLengthInMeters / 2);

    // Max translational velocity in m/s
    public static final double kSwerveDriveMaxSpeedInMetersPerSecond = Units.inchesToMeters(kSwerveMaxSpeedInchesPerSecond) ;
    // Maximum angular velocity in rad/s
    public static final double kSwerveRotationMaxSpeedInRadiansPerSecond =
            Math.min(kSwerveDriveMaxSpeedInMetersPerSecond / kRadiusInMeters, Math.toRadians(270));
    public static final double kRotationMotorTicksPerRadian = 2048.0 / (2 * Math.PI);

    // If CW, then right is positive.  If CCW left is positive
    public static final Translation2d kFrontRightModuleLocation = new Translation2d(kWheelbaseLengthInMeters / 2, kWheelbaseWidthInMeters / 2);
    public static final Translation2d kFrontLeftModuleLocation = new Translation2d(kWheelbaseLengthInMeters / 2, -kWheelbaseWidthInMeters / 2);
    public static final Translation2d kBackLeftModuleLocation = new Translation2d(-kWheelbaseLengthInMeters / 2, -kWheelbaseWidthInMeters / 2);
    public static final Translation2d kBackRightModuleLocation = new Translation2d(-kWheelbaseLengthInMeters / 2, kWheelbaseWidthInMeters / 2);

    public static final List<Translation2d> kModuleLocations = Arrays.asList(kFrontRightModuleLocation,
            kFrontLeftModuleLocation, kBackLeftModuleLocation, kBackRightModuleLocation);

    public static final double kDriveDeadband = 0.05;

    // swerve modules
    // zero - bezel to the left
    public static final SwerveDriveModule.SwerveModuleConstants kFrontRightModuleConstants = new SwerveDriveModule.SwerveModuleConstants();

    /*
     * FL 4196.0 - 45.0 * 4096.0 / 360.0 = 3,684 + 2048 //invert
     * FR 1,327 + 45 * 4096.0 / 360.0 = 1839
     * BL 5120.0 - 135 * 4096 / 360 = 3584 + 2048 //invert
     * BR 2060.0 + 135 * 4096 / 360 = 3596
     */

    static {
        kFrontRightModuleConstants.kName = "Front Right";
        kFrontRightModuleConstants.kDriveMotorTalonId = Ports.FRONT_RIGHT_DRIVE;
        kFrontRightModuleConstants.kSteerMotorTalonId = Ports.FRONT_RIGHT_ROTATION;
        kFrontRightModuleConstants.kSteerMotorEncoderHomeOffset = 883.0;
        kFrontRightModuleConstants.kCANCoderId = Ports.FRONT_RIGHT_ENC;
        kFrontRightModuleConstants.kWheelDiameter = kMK4_L2iWheelDiameter;
        kFrontRightModuleConstants.kDriveReduction = kMK4_l2iDriveReduction;
        /* ... */
    }

    public static final SwerveDriveModule.SwerveModuleConstants kFrontLeftModuleConstants = new SwerveDriveModule.SwerveModuleConstants();

    static {
        kFrontLeftModuleConstants.kName = "Front Left";
        kFrontLeftModuleConstants.kDriveMotorTalonId = Ports.FRONT_LEFT_DRIVE;
        kFrontLeftModuleConstants.kSteerMotorTalonId = Ports.FRONT_LEFT_ROTATION;
        kFrontLeftModuleConstants.kSteerMotorEncoderHomeOffset = 1683.0;
        kFrontLeftModuleConstants.kCANCoderId = Ports.FRONT_LEFT_ENC;
        kFrontLeftModuleConstants.kWheelDiameter = kMK4_L2iWheelDiameter;
        kFrontLeftModuleConstants.kDriveReduction = kMK4_l2iDriveReduction;
        /* ... */
    }

    public static final SwerveDriveModule.SwerveModuleConstants kBackLeftModuleConstants = new SwerveDriveModule.SwerveModuleConstants();

    static {
        kBackLeftModuleConstants.kName = "Back Left";
        kBackLeftModuleConstants.kDriveMotorTalonId = Ports.REAR_LEFT_DRIVE;
        kBackLeftModuleConstants.kSteerMotorTalonId = Ports.REAR_LEFT_ROTATION;
        kBackLeftModuleConstants.kSteerMotorEncoderHomeOffset = 3451.0;
        kBackLeftModuleConstants.kCANCoderId = Ports.REAR_LEFT_ENC;
        kBackLeftModuleConstants.kWheelDiameter = kMK4_L2iWheelDiameter;
        kBackLeftModuleConstants.kDriveReduction = kMK4_l2iDriveReduction;
        /* ... */
    }

    public static final SwerveDriveModule.SwerveModuleConstants kBackRightModuleConstants = new SwerveDriveModule.SwerveModuleConstants();

    static {
        kBackRightModuleConstants.kName = "Back Right";
        kBackRightModuleConstants.kDriveMotorTalonId = Ports.REAR_RIGHT_DRIVE;
        kBackRightModuleConstants.kSteerMotorTalonId = Ports.REAR_RIGHT_ROTATION;
        kBackRightModuleConstants.kSteerMotorEncoderHomeOffset = -327.0;
        kBackRightModuleConstants.kCANCoderId = Ports.REAR_RIGHT_ENC;
        kBackRightModuleConstants.kWheelDiameter = kMK4_L2iWheelDiameter;
        kBackRightModuleConstants.kDriveReduction = kMK4_l2iDriveReduction;
        /* ... */
    }

    // Swerve Heading Controller
    public static final double kSwerveHeadingControllerErrorTolerance = 1.0; // degrees

    // good for snapping (dpad)
    public static final double kSnapSwerveHeadingKp = 0.031;
    public static final double kSnapSwerveHeadingKi = 0.0;
    public static final double kSnapSwerveHeadingKd = 0.003;

    // good for maintaining heading
    public static final double kMaintainSwerveHeadingKp = 0.020;
    public static final double kMaintainSwerveHeadingKi = 0.0;
    public static final double kMaintainSwerveHeadingKd = 0.002;


    // END NEW SWERVE

    //Panel Manipulator Constants
    public static final int kPanelManipulatorTalonID    = 0; //TODO: temporary value, need adjustments
    public static final int kPanelManipulatorSolenoidID = 0; //TODO: temporary value, need adjustments

    public static final double kPanelManipulatorSpinSpeed = 0.33; //TODO: temporary value, need adjustments

    public static final int kJoystickRumbleFrequency = 1; //TODO: temporary value, need adjustments
    public static final int kJoystickRumbleDuration  = 1; //TODO: temporary value, need adjustments

    public static final double kRotationControlLimit    = 3.5;   //TODO: temporary value, need adjustments
    public static final double kRotationsPerColorChange = 0.125; //TODO: temporary value, need adjustments

    public static final float[] kBlueMNStockReadings   = {1.00f, 0.60f, 0.70f, 0.35f, 0.30f, 0.10f};
    public static final float[] kGreenMNStockReadings  = {0.30f, 0.90f, 1.00f, 0.70f, 0.50f, 0.15f};
    public static final float[] kRedMNStockReadings    = {0.15f, 0.15f, 0.50f, 0.40f, 1.00f, 0.55f};
    public static final float[] kYellowMNStockReadings = {0.25f, 0.40f, 1.00f, 1.00f, 1.00f, 0.35f};

    public static final float[] kBlueSNStockReadings   = {0.34f, 0.22f, 0.15f, 0.12f, 0.09f, 0.05f};
    public static final float[] kGreenSNStockReadings  = {0.16f, 0.23f, 0.24f, 0.19f, 0.10f, 0.05f};
    public static final float[] kRedSNStockReadings    = {0.07f, 0.05f, 0.09f, 0.09f, 0.42f, 0.20f};
    public static final float[] kYellowSNStockReadings = {0.05f, 0.10f, 0.26f, 0.22f, 0.25f, 0.08f};

    public static final int kAS7262RefreshRate = 10; // measured in Hz


    //Scrub Factors
    public static final boolean kSimulateReversedCarpet = false;
    public static final double[] kWheelScrubFactors = new double[]{1.0, 1.0, 1.0, 1.0};
    public static final double kXScrubFactor = 1.0 / (1.0 - (9549.0 / 293093.0));
    public static final double kYScrubFactor = 1.0 / (1.0 - (4.4736 / 119.9336));

    //Voltage-Velocity equation constants {m, b, x-intercept}
    //First set is the positive direction, second set is negative
    public static final double[][][] kVoltageVelocityEquations = new double[][][]{
            {{1.70, -4.39, 2.58}, {1.83, 5.23, -2.85}},
            {{1.59, -3.86, 2.42}, {1.43, 3.09, -2.16}},
            {{1.53, -3.66, 2.39}, {1.66, 4.15, -2.50}},
            {{1.84, -4.70, 2.56}, {1.85, 5.34, -2.89}}};

    // LED/Canifier
    public static final int kCanifierId = 0;
    public static final int kCANTimeoutMs = 10; //use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; //use for constructors

}
