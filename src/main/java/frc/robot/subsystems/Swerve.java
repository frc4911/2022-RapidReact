package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.planners.DriveMotionPlanner;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Pose2dWithCurvature;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.loops.ILooper;
import libraries.cheesylib.loops.Loop;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.trajectory.TrajectoryIterator;
import libraries.cheesylib.trajectory.timing.TimedState;
import libraries.cyberlib.kinematics.ChassisSpeeds;
import libraries.cyberlib.kinematics.SwerveDriveKinematics;
import libraries.cyberlib.kinematics.SwerveDriveOdometry;
import libraries.cyberlib.kinematics.SwerveModuleState;
import libraries.cyberlib.utils.RobotName;

public class Swerve extends Subsystem {

    public enum ControlState{
        NEUTRAL, MANUAL, DISABLED, TRAJECTORY, VISION_AIM
    }

    private ControlState mControlState = ControlState.NEUTRAL;

    PeriodicIO mPeriodicIO = new PeriodicIO();
    private int mDefaultSchedDelta = 20;

	// Module declaration
	private final List<SwerveDriveModule> mModules = new ArrayList<>();
	private final SwerveDriveModule mFrontRight, mFrontLeft, mBackLeft, mBackRight;

	double lastUpdateTimestamp = 0;

	// Swerve kinematics & odometry
	private final Pigeon mPigeon;
	private Rotation2d mGyroOffset = Rotation2d.identity();

	// TODO Consider moving odometry to RobotStateEstimator
	private final SwerveDriveOdometry mOdometry;
	private final SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(Constants.kModuleLocations);
	private ChassisSpeeds mChassisSpeeds;

    // Updated as part of periodic odometry
    private Pose2d mPose = Pose2d.identity();

    // Trajectory following
    private DriveMotionPlanner mMotionPlanner;
    private boolean mOverrideTrajectory = false;

    RobotState robotState;
    private int mListIndex = -1;

    private static String sClassName;
    private static int sInstanceCount;
    private static Swerve sInstance = null;
    public  static Swerve getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new Swerve(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller) {
        System.out.println("("+caller+") "+"getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Swerve(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        int m0 = 0;
        int m1 = 0;
        int m2 = 0;
        int m3 = 0;

        mPigeon = Pigeon.getInstance();

        if (RobotName.name.equals(Constants.kRobot1Name)) {
            Constants.kFrontLeftModuleConstants.kCANCoderOffsetDegrees = Constants.kFrontLeftCancoderStartingPosDegreesR1;
            Constants.kFrontRightModuleConstants.kCANCoderOffsetDegrees = Constants.kFrontRightCancoderStartingPosDegreesR1;
            Constants.kBackLeftModuleConstants.kCANCoderOffsetDegrees = Constants.kRearLeftCancoderStartingPosDegreesR1;
            Constants.kBackRightModuleConstants.kCANCoderOffsetDegrees = Constants.kRearRightCancoderStartingPosDegreesR1;
        }
        else if (RobotName.name.equals(Constants.kRobot2Name)) {
            Constants.kFrontLeftModuleConstants.kCANCoderOffsetDegrees = Constants.kFrontLeftCancoderStartingPosDegreesR2;
            Constants.kFrontRightModuleConstants.kCANCoderOffsetDegrees = Constants.kFrontRightCancoderStartingPosDegreesR2;
            Constants.kBackLeftModuleConstants.kCANCoderOffsetDegrees = Constants.kRearLeftCancoderStartingPosDegreesR2;
            Constants.kBackRightModuleConstants.kCANCoderOffsetDegrees = Constants.kRearRightCancoderStartingPosDegreesR2;
        }
        else if (RobotName.name.equals(Constants.kCetusName)) {
            Constants.kFrontLeftModuleConstants.kCANCoderOffsetDegrees = Constants.kFrontLeftCancoderStartingPosDegreesCetus;
            Constants.kFrontRightModuleConstants.kCANCoderOffsetDegrees = Constants.kFrontRightCancoderStartingPosDegreesCetus;
            Constants.kBackLeftModuleConstants.kCANCoderOffsetDegrees = Constants.kRearLeftCancoderStartingPosDegreesCetus;
            Constants.kBackRightModuleConstants.kCANCoderOffsetDegrees = Constants.kRearRightCancoderStartingPosDegreesCetus;
        }

		mModules.add(mFrontLeft = new SwerveDriveModule(Constants.kFrontLeftModuleConstants));
		mModules.add(mFrontRight = new SwerveDriveModule(Constants.kFrontRightModuleConstants));
		mModules.add(mBackRight = new SwerveDriveModule(Constants.kBackRightModuleConstants));
		mModules.add(mBackLeft = new SwerveDriveModule(Constants.kBackLeftModuleConstants));

		mOdometry= new SwerveDriveOdometry(mKinematics, mPigeon.getYaw());

        mMotionPlanner = new DriveMotionPlanner();
        robotState = RobotState.getInstance(sClassName);
//        generator = TrajectoryGenerator.getInstance();
    }

    private final Loop loop = new Loop() {

        @Override
        public void onStart(Phase phase) {
            synchronized(Swerve.this) {
                stop();
                lastUpdateTimestamp = Timer.getFPGATimestamp();
                switch (phase) {
                    case DISABLED:
                        mPeriodicIO.schedDeltaDesired = 0; // goto sleep
                        break;
                    default:
                        mPeriodicIO.schedDeltaDesired = mDefaultSchedDelta;
                        break;
                }
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized(Swerve.this) {
                lastUpdateTimestamp = timestamp;

                switch(mControlState) {
                    case MANUAL:
                        handleManual();
                        break;
                    case TRAJECTORY:
                        updatePathFollower();
                        break;
                    case NEUTRAL:
                        stop();
                        break;
                    case DISABLED:
                    default:
                        break;
                }
            }
        }

        /**
         * Handles MANUAL state which corresponds to joy stick inputs.
         * <p>
         * Using the joy stick values in PeriodicIO, calculate and updates the swerve states. The joy stick values
         * are as percent [-1.0, 1.0].  The swerveDriverHelper converts percent inputs to SI units before creating
         * the ChassisSpeeds.
         */
        private void handleManual() {
            // Helper to make driving feel better
            var chassisSpeeds = swerveDriveHelper(
                    mPeriodicIO.forward, mPeriodicIO.strafe, mPeriodicIO.rotation, mPeriodicIO.low_power,
                    mPeriodicIO.field_relative, mPeriodicIO.use_heading_controller);

            // Now calculate the new Swerve Module states using kinematics.
            mPeriodicIO.swerveModuleStates = mKinematics.toSwerveModuleStates(chassisSpeeds);

            // Normalize wheels speeds if any individual speed is above the specified maximum.
            SwerveDriveKinematics.desaturateWheelSpeeds(
                    mPeriodicIO.swerveModuleStates, Constants.kSwerveDriveMaxSpeedInMetersPerSecond);
        }

        @Override
        public void onStop(double timestamp) {
            synchronized(Swerve.this) {
                stop();
            }
        }
    };

    @Override
    public synchronized void stop() {
        setState(ControlState.NEUTRAL);
        mModules.forEach((m) -> m.stop());
    }

    @Override
    public synchronized void zeroSensors() {
        zeroSensors(Constants.kRobotStartingPose);
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        mListIndex = enabledLooper.register(loop);
    }


//    public boolean hasFinishedPath() {
//        return hasFinishedPath;
//    }

//    //Assigns appropriate directions for scrub factors
//    public void setCarpetDirection(boolean standardDirection) {
//        mModules.forEach((m) -> m.setCarpetDirection(standardDirection));
//    }

    public ControlState getState() {
        return mControlState;
    }

    public void setState(ControlState newState) {
        if (mControlState != newState) {
            System.out.println(mControlState + " to " + newState);
            switch (newState) {
                case NEUTRAL:
                case MANUAL:
                case DISABLED:
                    mPeriodicIO.schedDeltaDesired = 100;
                    break;

                case VISION_AIM:
                case TRAJECTORY:
                    mPeriodicIO.schedDeltaDesired = 20;
                    break;
            }
        }

        mControlState = newState;
    }

    /**
     * Returns the angle of the robot as a Rotation2d.
     * <p>
     * @return The angle of the robot (CCW).
     */
    private synchronized Rotation2d getAngle() {
        // Expects CCW.
        return mPigeon.getYaw();
    }

    public Pose2d getPose() {
        return mPose;
    }

    public Rotation2d getHeading() {
        return mPeriodicIO.gyro_heading;
    }


    /**
     * Updates the field relative position of the robot.
     *
     * @param timestamp The current time
     */
    private void updateOdometry(double timestamp) {
        var frontLeft = mFrontLeft.getState();
        var frontRight = mFrontRight.getState();
        var backLeft = mBackLeft.getState();
        var backRight = mBackRight.getState();

        mChassisSpeeds = mKinematics.toChassisSpeeds(frontLeft,frontRight, backLeft, backRight);
        mPose = mOdometry.updateWithTime(timestamp, getAngle(), frontLeft, frontRight, backLeft, backRight);
    }


//    public synchronized void setOpenLoop(double forward, double strafe, double rotation) {
//        if (mControlState != ControlState.MANUAL) {
//            mControlState = ControlState.MANUAL;
//        }
//
//        mPeriodicIO.forward = forward;
//        mPeriodicIO.strafe = strafe;
//        mPeriodicIO.rotation = rotation;
//        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(mPeriodicIO.forward, mPeriodicIO.strafe, mPeriodicIO.rotation);
//    }

    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        if (mMotionPlanner != null) {
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            mControlState = ControlState.TRAJECTORY;
        }
    }

    public boolean isDoneWithTrajectory() {
        if (mMotionPlanner == null || mControlState != ControlState.TRAJECTORY) {
            return false;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }

    public void overrideTrajectory(boolean value) {
        mOverrideTrajectory = value;
    }

    private void updatePathFollower() {
        // TODO: Implement Trajectory Following
        if (mControlState == ControlState.TRAJECTORY) {
            final double now = Timer.getFPGATimestamp();

//            var output = mMotionPlanner.update(now, RobotState.getInstance().getFieldToVehicle(now));
//
//            // DriveSignal signal = new DriveSignal(demand.left_feedforward_voltage / 12.0, demand.right_feedforward_voltage / 12.0);
//
//            mPeriodicIO.error = mMotionPlanner.error();
//            mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();
//
//            if (!mOverrideTrajectory) {
//                setVelocity(new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity), radiansPerSecondToTicksPer100ms(output.right_velocity)),
//                        new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0));
//
//                mPeriodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
//                mPeriodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
//            } else {
//                setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
//                mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
//            }
//        } else {
//            DriverStation.reportError("Drive is not in path following state", false);
        }
    }

    /** Puts all rotation and drive motors into open-loop mode */
    public synchronized void disable() {
        mModules.forEach((m) -> m.disable());
        setState(ControlState.DISABLED);
    }

    /** Zeroes the drive motors, and sets the robot's internal position and heading to match that of the fed pose */
    public synchronized void zeroSensors(Pose2d startingPose) {
        mPigeon.setAngle(startingPose.getRotation().getUnboundedDegrees());
        mModules.forEach((m) -> m.zeroSensors(startingPose));
//        pose = startingPose;
//        distanceTraveled = 0;
    }

    /**
     * Sets inputs from driver in teleop mode.
     *
     * @param forward percent to drive forwards/backwards (as double [-1.0,1.0]).
     * @param strafe percent to drive sideways left/right (as double [-1.0,1.0]).
     * @param rotation percent to rotate chassis (as double [-1.0,1.0]).
     * @param low_power whether to use low or high power.
     * @param field_relative whether operation is robot centric or field relative.
     * @param use_heading_controller whether the heading controller is being used.
     */
    public void setTeleopInputs(double forward, double strafe, double rotation, boolean low_power, boolean field_relative, boolean use_heading_controller) {
        if (mControlState != ControlState.MANUAL) {
            mControlState = ControlState.MANUAL;
        }
        mPeriodicIO.forward = forward;
        mPeriodicIO.strafe = strafe;
        mPeriodicIO.rotation = rotation;
        mPeriodicIO.low_power = low_power;
        mPeriodicIO.field_relative = field_relative;
        mPeriodicIO.use_heading_controller = use_heading_controller;
    }


    private final static double kHighAdjustmentPower = 1.75 + 0.4375;
    private final static double kLowAdjustmentPower = 1.50;
    private final static double kMaxSpeed = 1.0;
    private final static double kHighPowerRotationScalar = 0.8;
    private final static double kLowPowerScalar = 0.5;
    private final static double kRotationExponent = 4.0;
    private final static double kPoleThreshold = 0.0;
    private final static double kRobotRelativePoleThreshold = Math.toRadians(5);
    private final static double kDeadband = 0.25;
    private final static double kRotationDeadband = 0.15;

    // TODO: Consider separate helper class
    /**
     * Based on Team 1323's sendInput method to make driving feel better.
     *
     * @param forwardInput
     * @param strafeInput
     * @param rotationInput
     * @param low_power
     * @param field_relative
     * @param use_heading_controller
     * @return A ChassisSpeeds object for the inputs
     */
    public ChassisSpeeds swerveDriveHelper(double forwardInput, double strafeInput, double rotationInput,
                                   boolean low_power, boolean field_relative, boolean use_heading_controller) {

        Translation2d translationalInput = new Translation2d(forwardInput, strafeInput);
        double inputMagnitude = translationalInput.norm();

        // Snap the translational input to its nearest pole, if it is within a certain
        // threshold of it.

        if (field_relative) {
            if (Math.abs(translationalInput.direction()
                    .distance(translationalInput.direction().nearestPole())) < kPoleThreshold) {
                translationalInput = translationalInput.direction().nearestPole().toTranslation().scale(inputMagnitude);
            }
        } else {
            if (Math.abs(translationalInput.direction()
                    .distance(translationalInput.direction().nearestPole())) < kRobotRelativePoleThreshold) {
                translationalInput = translationalInput.direction().nearestPole().toTranslation().scale(inputMagnitude);
            }
        }

        if (inputMagnitude < kDeadband) {
            translationalInput = new Translation2d();
            inputMagnitude = 0;
        }

        // Scale x and y by applying a power to the magnitude of the vector they create,
        // in order to make the controls less sensitive at the lower end.
        final double power = (low_power) ? kHighAdjustmentPower : kLowAdjustmentPower;
        Rotation2d direction = translationalInput.direction();
        double scaledMagnitude = Math.pow(inputMagnitude, power);
        translationalInput = new Translation2d(direction.cos() * scaledMagnitude, direction.sin() * scaledMagnitude);

        rotationInput = (Math.abs(rotationInput) < kRotationDeadband) ? 0 : rotationInput;
        if (use_heading_controller) { // current constants are tuned to be put to the power of 1.75, and I don't want to retune right now
            rotationInput = Math.pow(Math.abs(rotationInput), 1.75) * Math.signum(rotationInput);
        } else {
            rotationInput = Math.pow(Math.abs(rotationInput), kRotationExponent) * Math.signum(rotationInput);
        }

        translationalInput = translationalInput.scale(kMaxSpeed);
        rotationInput *= kMaxSpeed;

        if (low_power) {
            translationalInput = translationalInput.scale(kLowPowerScalar);
            rotationInput *= kLowPowerScalar;
        } else {
            rotationInput *= kHighPowerRotationScalar;
        }

        // Convert the joystick inputs to SI units.
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                translationalInput.x() * Constants.kSwerveDriveMaxSpeedInMetersPerSecond,
                translationalInput.y() * Constants.kSwerveDriveMaxSpeedInMetersPerSecond,
                rotationInput * Constants.kSwerveRotationMaxSpeedInMetersPerSecond);
        return chassisSpeeds;
    }


    @Override
    public String getLogHeaders() {
        StringBuilder allHeaders = new StringBuilder(256);
        for (SwerveDriveModule m: mModules) {
            if (allHeaders.length() > 0) {
                allHeaders.append(",");
            }
            allHeaders.append(m.getLogHeaders());
        }

        allHeaders.append("," + sClassName+".schedDeltaDesired,"+
                sClassName+".schedDeltaActual,"+
                sClassName+".schedDuration"
        );

        return allHeaders.toString();
    }

    private String generateLogValues(boolean telemetry) {
        String values;
        if (telemetry) {
            values = ""+/*mPeriodicIO.schedDeltaDesired+*/","+
                    /*mPeriodicIO.schedDeltaActual+*/","
            /*mPeriodicIO.schedDuration*/;
        }
        else {
            mPeriodicIO.schedDuration = Timer.getFPGATimestamp() - mPeriodicIO.lastSchedStart;
            values = ""+mPeriodicIO.schedDeltaDesired+","+
                    mPeriodicIO.schedDeltaActual+","+
                    mPeriodicIO.schedDuration;
        }

        return values;
    }


    @Override
    public String getLogValues(boolean telemetry) {
        StringBuilder allValues = new StringBuilder(256);
        for (SwerveDriveModule m: mModules) {
            if (allValues.length() > 0) {
                allValues.append(",");
            }
            allValues.append(m.getLogValues(telemetry));
        }
        allValues.append(","+generateLogValues(telemetry));
        return allValues.toString();
    }

    @Override
    public synchronized void readPeriodicInputs() {
        double now                   = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart   = now;

        // Read odometry every in every loop
        updateOdometry(now);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // Set the module state for each module
        // All modes should use this method of module states.
        for (int i = 0; i < mModules.size(); i++) {
            mModules.get(i).setState(mPeriodicIO.swerveModuleStates[i]);
        }

        mModules.forEach((m) -> m.writePeriodicOutputs());
    }

    @Override
    public int whenRunAgain () {
        return mPeriodicIO.schedDeltaDesired;
    }

    @Override
    public void outputTelemetry() {
        mModules.forEach((m) -> m.outputTelemetry());
        SmartDashboard.putString("Swerve/Swerve State", mControlState.toString());
//        SmartDashboard.putBoolean("Swerve/isOnTarget", isOnTarget());
        if (Constants.kDebuggingOutput) {
            // Get the current pose from odometry state
            Pose2d pose = mOdometry.getPose();
            SmartDashboard.putString("Swerve/pose", pose.toString());
            SmartDashboard.putString("Swerve/State", mControlState.toString());
            SmartDashboard.putNumberArray("Swerve/Pigeon YPR", mPigeon.getYPR());
//            SmartDashboard.putString("Swerve/Heading Controller", mHeadingController.getState().toString());
//            SmartDashboard.putNumber("Swerve/Target Heading", mHeadingController.getTargetHeading());
            SmartDashboard.putNumber("Swerve/Distance from start/last reset", mOdometry.getPose().getTranslation().norm());
            SmartDashboard.putNumber("Swerve/Translational Velocity m/s",
                    Math.hypot(
                            mChassisSpeeds.vxInMetersPerSecond,
                            mChassisSpeeds.vyInMetersPerSecond));
            SmartDashboard.putNumber("Swerve/Translational Velocity ft/s",
                    Math.hypot(
                            Units.metersToFeet(mChassisSpeeds.vxInMetersPerSecond),
                            Units.metersToFeet(mChassisSpeeds.vyInMetersPerSecond)));
            SmartDashboard.putString("Swerve/Chassis Speed", mChassisSpeeds.toString());
//            SmartDashboard.putBoolean("Swerve/Vision Updates Allowed", visionUpdatesAllowed);

            SmartDashboard.putNumberArray("Swerve/Robot Pose", new double[]{pose.getTranslation().x(), pose.getTranslation().y(), pose.getRotation().getUnboundedDegrees()});
            SmartDashboard.putNumber("Swerve/Robot X", pose.getTranslation().x());
            SmartDashboard.putNumber("Swerve/Robot Y", pose.getTranslation().y());
            SmartDashboard.putNumber("Swerve/Robot Heading", pose.getRotation().getUnboundedDegrees());
//            SmartDashboard.putNumber("Swerve/Robot Velocity", currentVelocity);
        }

//        if (!hasFinishedPath() && hasStartedFollowing) {
//            double currentTime = Timer.getFPGATimestamp();
//            SmartDashboard.putNumber("Autopath Timer", currentTime - trajectoryStartTime);
//        }
    }

    public static class PeriodicIO {
        // LOGGING
        public int schedDeltaDesired;
        public double schedDeltaActual;
        public double schedDuration;
        private double lastSchedStart;

        // Inputs
        public Rotation2d gyro_heading = Rotation2d.identity();
        public double forward;
        public double strafe;
        public double rotation;
        public boolean low_power;
        public boolean field_relative;
        public boolean use_heading_controller;

        // OUTPUTS
        public SwerveModuleState[] swerveModuleStates = new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.identity()),
                new SwerveModuleState(0, Rotation2d.identity()),
                new SwerveModuleState(0, Rotation2d.identity()),
                new SwerveModuleState(0, Rotation2d.identity())
        };
    }
}
