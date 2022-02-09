package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
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
import libraries.cyberlib.utils.SwerveDriveHelper;

public class Swerve extends Subsystem {

    public enum ControlState{
        NEUTRAL, MANUAL, DISABLED, PATH_FOLLOWING, VISION_AIM
    }

    private ControlState mControlState = ControlState.NEUTRAL;

    public SwerveConfiguration mSwerveConfiguration;

    PeriodicIO mPeriodicIO = new PeriodicIO();
    private int mDefaultSchedDelta = 20;

	// Module declaration
	private final List<SwerveDriveModule> mModules = new ArrayList<>();
	private SwerveDriveModule mFrontRight=null, mFrontLeft=null, mBackLeft=null, mBackRight=null;

	double lastUpdateTimestamp = 0;

	// Swerve kinematics & odometry
	private final Pigeon mPigeon;
	private Rotation2d mGyroOffset = Rotation2d.identity();

	private final SwerveDriveOdometry mOdometry;
	private final SwerveDriveKinematics mKinematics;

    private SwerveDriveHelper mSwerveDriveHelper;

    // Trajectory following
    private DriveMotionPlanner mMotionPlanner;
    private boolean mOverrideTrajectory = false;

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

        if (RobotName.name.equals(Constants.kJuniorName)) {
            // Constants.kFrontRightModuleConstants.kCANCoderOffsetDegrees = Constants.kFrontRightCancoderStartingPosDegreesR1;
            // Constants.kFrontLeftModuleConstants.kCANCoderOffsetDegrees = Constants.kFrontLeftCancoderStartingPosDegreesR1;
            // Constants.kBackLeftModuleConstants.kCANCoderOffsetDegrees = Constants.kRearLeftCancoderStartingPosDegreesR1;
            // Constants.kBackRightModuleConstants.kCANCoderOffsetDegrees = Constants.kRearRightCancoderStartingPosDegreesR1;
            mSwerveConfiguration = Constants.kSwerveConfigurationJunior;
            mModules.add(mFrontRight = new SwerveDriveModule(Constants.kFrontRightModuleConstantsJunior, mSwerveConfiguration.maxSpeedInMetersPerSecond));
            mModules.add(mFrontLeft = new SwerveDriveModule(Constants.kFrontLeftModuleConstantsJunior, mSwerveConfiguration.maxSpeedInMetersPerSecond));
            mModules.add(mBackLeft = new SwerveDriveModule(Constants.kBackLeftModuleConstantsJunior, mSwerveConfiguration.maxSpeedInMetersPerSecond));
            mModules.add(mBackRight = new SwerveDriveModule(Constants.kBackRightModuleConstantsJunior, mSwerveConfiguration.maxSpeedInMetersPerSecond));
        }
        else if (RobotName.name.equals(Constants.kDeadEyeName)) {
            // Constants.kFrontLeftModuleConstants.kCANCoderOffsetDegrees = Constants.kFrontLeftCancoderStartingPosDegreesR2;
            // Constants.kFrontRightModuleConstants.kCANCoderOffsetDegrees = Constants.kFrontRightCancoderStartingPosDegreesR2;
            // Constants.kBackLeftModuleConstants.kCANCoderOffsetDegrees = Constants.kRearLeftCancoderStartingPosDegreesR2;
            // Constants.kBackRightModuleConstants.kCANCoderOffsetDegrees = Constants.kRearRightCancoderStartingPosDegreesR2;
            mSwerveConfiguration = Constants.kSwerveConfigurationDeadEye;
            mModules.add(mFrontRight = new SwerveDriveModule(Constants.kFrontRightModuleConstantsDeadEye, mSwerveConfiguration.maxSpeedInMetersPerSecond));
            mModules.add(mFrontLeft = new SwerveDriveModule(Constants.kFrontLeftModuleConstantsDeadEye, mSwerveConfiguration.maxSpeedInMetersPerSecond));
            mModules.add(mBackLeft = new SwerveDriveModule(Constants.kBackLeftModuleConstantsDeadEye, mSwerveConfiguration.maxSpeedInMetersPerSecond));
            mModules.add(mBackRight = new SwerveDriveModule(Constants.kBackRightModuleConstantsDeadEye, mSwerveConfiguration.maxSpeedInMetersPerSecond));
        }
        else if (RobotName.name.equals(Constants.kRobot2022Name)) {
            // Constants.kFrontLeftModuleConstants.kCANCoderOffsetDegrees = Constants.kFrontLeftCancoderStartingPosDegreesCetus;
            // Constants.kFrontRightModuleConstants.kCANCoderOffsetDegrees = Constants.kFrontRightCancoderStartingPosDegreesCetus;
            // Constants.kBackLeftModuleConstants.kCANCoderOffsetDegrees = Constants.kRearLeftCancoderStartingPosDegreesCetus;
            // Constants.kBackRightModuleConstants.kCANCoderOffsetDegrees = Constants.kRearRightCancoderStartingPosDegreesCetus;
            mSwerveConfiguration = Constants.kSwerveConfigurationRobot2022;
            mModules.add(mFrontRight = new SwerveDriveModule(Constants.kFrontRightModuleConstantsRobot2022, mSwerveConfiguration.maxSpeedInMetersPerSecond));
            mModules.add(mFrontLeft = new SwerveDriveModule(Constants.kFrontLeftModuleConstantsRobot2022, mSwerveConfiguration.maxSpeedInMetersPerSecond));
            mModules.add(mBackLeft = new SwerveDriveModule(Constants.kBackLeftModuleConstantsRobot2022, mSwerveConfiguration.maxSpeedInMetersPerSecond));
            mModules.add(mBackRight = new SwerveDriveModule(Constants.kBackRightModuleConstantsRobot2022, mSwerveConfiguration.maxSpeedInMetersPerSecond));
        }

//        mSwerveDriveHelper = new SwerveDriveHelper(mSwerveConfiguration.maxSpeedInMetersPerSecond,
//                mSwerveConfiguration.maxSpeedInRadiansPerSecond);

        mKinematics = new SwerveDriveKinematics(mSwerveConfiguration.moduleLocations);
		mOdometry = new SwerveDriveOdometry(mKinematics, mPigeon.getYaw());
        mPeriodicIO.robotPose = mOdometry.getPose();

        mMotionPlanner = new DriveMotionPlanner();

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

                // Update odometry in every loop before any other actions.
                updateOdometry(lastUpdateTimestamp);

                switch (mControlState) {
                    case MANUAL:
                        handleManual();
                        break;
                    case PATH_FOLLOWING:
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

    /**
     * Handles MANUAL state which corresponds to joy stick inputs.
     * <p>
     * Using the joy stick values in PeriodicIO, calculate and updates the swerve states. The joy stick values
     * are as percent [-1.0, 1.0].  They need to be converted to SI units before creating the ChassisSpeeds.
     */
    private void handleManual() {
        // Helper to make driving feel better
        // var chassisSpeeds = mSwerveDriveHelper.calculateChassisSpeeds(
        //         mPeriodicIO.forward, mPeriodicIO.strafe, mPeriodicIO.rotation, mPeriodicIO.low_power,
        //         mPeriodicIO.field_relative, mPeriodicIO.use_heading_controller);

        // Scale joystick values to SI units and relative to maximum.
        var translationInput = new Translation2d(mPeriodicIO.forward, mPeriodicIO.strafe)
                .scale(mSwerveConfiguration.maxSpeedInMetersPerSecond);

        var rotationInput = mPeriodicIO.rotation * mSwerveConfiguration.maxSpeedInRadiansPerSecond;

        if (mPeriodicIO.field_relative) {
            translationInput = translationInput.rotateBy(getPose().getRotation().inverse());
        }

        var chassisSpeeds = new ChassisSpeeds(translationInput.x(), translationInput.y(), rotationInput);

        // Now calculate the new Swerve Module states using inverse kinematics.
        mPeriodicIO.swerveModuleStates = mKinematics.toSwerveModuleStates(chassisSpeeds);
        // brian temp debug
        // System.out.println(mPeriodicIO.swerveModuleStates[0].toString());
        // System.out.println(mPeriodicIO.swerveModuleStates[1].toString());
        // System.out.println(mPeriodicIO.swerveModuleStates[2].toString());
        // System.out.println(mPeriodicIO.swerveModuleStates[3].toString());

        // Normalize wheels speeds if any individual speed is above the specified maximum.
        SwerveDriveKinematics.desaturateWheelSpeeds(
                mPeriodicIO.swerveModuleStates, mSwerveConfiguration.maxSpeedInMetersPerSecond);

        // brian temp debug
        // if(++throttlePrints%printFreq==0){
        //     System.out.println("01 s handleManual (mPeriodicIO.swerveModuleStates[0]) ("+mPeriodicIO.swerveModuleStates[0].toString()+")");
        // }
    }

//    //Assigns appropriate directions for scrub factors
//    public void setCarpetDirection(boolean standardDirection) {
//        mModules.forEach((m) -> m.setCarpetDirection(standardDirection));
//    }

    /**
     * Gets the current control state for the Swerve Drive.
     * <p>
     * @return The current control state.
     */
    public synchronized ControlState getState() {
        return mControlState;
    }

    /**
     * Sets the control state for the Swerve Drive.
     * <p>
     * @param newState The desired state.
     */
     public synchronized void setState(ControlState newState) {
        if (mControlState != newState) {
            System.out.println(mControlState + " to " + newState);
            switch (newState) {
                case NEUTRAL:
                case MANUAL:
                case DISABLED:
                    mPeriodicIO.schedDeltaDesired = 100;
                    break;

                case VISION_AIM:
                case PATH_FOLLOWING:
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
        return mPigeon.getYaw();
    }

    public Pose2d getPose() {
        return mPeriodicIO.robotPose;
    }

    public Rotation2d getHeading() {
        return mPeriodicIO.gyro_heading;
    }

    /**
     * Sets the current robot position on the field.
     * <p>
     * @param pose The (x,y,thetha) position.
     */
    public synchronized void setRobotPosition(Pose2d pose) {
        mOdometry.resetPosition(pose, mPigeon.getYaw());
        mPeriodicIO.robotPose = mOdometry.getPose();
    }

    /**
     * Updates the field relative position of the robot.
     *
     * @param timestamp The current time
     */
    private void updateOdometry(double timestamp) {

        var frontRight = mFrontRight.getState();
        var frontLeft = mFrontLeft.getState();
        var backLeft = mBackLeft.getState();
        var backRight = mBackRight.getState();

        // order is CCW starting with front right.
        mPeriodicIO.chassisSpeeds = mKinematics.toChassisSpeeds(frontRight, frontLeft, backLeft, backRight);
        mPeriodicIO.robotPose = mOdometry.updateWithTime(timestamp, getAngle(), frontRight, frontLeft, backLeft, backRight);
    }

    public boolean isDoneWithTrajectory() {
        if (mMotionPlanner == null || mControlState != mControlState.PATH_FOLLOWING) {
            return false;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
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
            mControlState = ControlState.PATH_FOLLOWING;
        }
    }

    public void overrideTrajectory(boolean value) {
        mOverrideTrajectory = value;
    }

    private void updatePathFollower() {
        // TODO: Implement Trajectory Following
        if (mControlState == ControlState.PATH_FOLLOWING) {
            final double now = Timer.getFPGATimestamp();

//            var output = mMotionPlanner.update(now, RobotState.getInstance(sClassName).getFieldToVehicle(now));
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

    /** Puts all steer and drive motors into open-loop mode */
    public synchronized void disable() {
        mModules.forEach((m) -> m.disable());
        setState(ControlState.DISABLED);
    }

    /** Zeroes the drive motors, and sets the robot's internal position and heading to match that of the fed pose */
    public synchronized void zeroSensors(Pose2d startingPose) {
        setRobotPosition(startingPose);
        mPigeon.setAngle(startingPose.getRotation().getUnboundedDegrees());
        // mModules.forEach((m) -> m.zeroSensors(startingPose));
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
        // brian temp debug
        // if(++throttlePrints%printFreq==0){
        //     System.out.println("00 s setTeleopInputs (forward,strafe,rotation) ("+mPeriodicIO.forward+","+mPeriodicIO.strafe+","+mPeriodicIO.rotation+")");
        // }
    }

    // brian temp debug
    // int throttlePrints;
    // final int printFreq = 10;


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
        mPeriodicIO.gyro_heading     = Rotation2d.fromDegrees(mPigeon.getYaw().getDegrees()).rotateBy(mGyroOffset);

        // read modules
        mModules.forEach((m) -> m.readPeriodicInputs());
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // Set the module state for each module
        // All modes should use this method of module states.
        for (int i = 0; i < mModules.size(); i++) {
            mModules.get(i).setState(mPeriodicIO.swerveModuleStates[i]);
        }
        // System.out.println(mPeriodicIO.swerveModuleStates[0].toString());
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
        SmartDashboard.putString("Swerve/Pose", mPeriodicIO.robotPose.toString());
        SmartDashboard.putString("Swerve/Chassis Speeds", mPeriodicIO.chassisSpeeds.toString());
//        SmartDashboard.putBoolean("Swerve/isOnTarget", isOnTarget());

        if (Constants.kDebuggingOutput) {
            // Get the current pose from odometry state
            SmartDashboard.putNumber("Swerve/Robot X", mPeriodicIO.robotPose.getTranslation().x());
            SmartDashboard.putNumber("Swerve/Robot Y", mPeriodicIO.robotPose.getTranslation().y());

            SmartDashboard.putNumber("Swerve/Translational Velocity m/s",
                    Math.hypot(
                            mPeriodicIO.chassisSpeeds.vxInMetersPerSecond,
                            mPeriodicIO.chassisSpeeds.vyInMetersPerSecond));
            SmartDashboard.putNumber("Swerve/Translational Velocity ft/s",
                    Math.hypot(
                            Units.metersToFeet(mPeriodicIO.chassisSpeeds.vxInMetersPerSecond),
                            Units.metersToFeet(mPeriodicIO.chassisSpeeds.vyInMetersPerSecond)));
            SmartDashboard.putNumber("Swerve/Rotational Velocity rad/s",
                    mPeriodicIO.chassisSpeeds.omegaInRadiansPerSecond);

            SmartDashboard.putNumberArray("Swerve/Pigeon YPR", mPigeon.getYPR());
//            SmartDashboard.putString("Swerve/Heading Controller", mHeadingController.getState().toString());
//            SmartDashboard.putNumber("Swerve/Target Heading", mHeadingController.getTargetHeading());
//            SmartDashboard.putNumber("Swerve/Distance from start/last reset", mOdometry.getPose().getTranslation().norm());

            // SmartDashboard.putBoolean("Swerve/Vision Updates Allowed", visionUpdatesAllowed);

//            SmartDashboard.putNumberArray("Swerve/Robot Pose", new double[]{pose.getTranslation().x(), pose.getTranslation().y(), pose.getRotation().getUnboundedDegrees()});
//            SmartDashboard.putNumber("Swerve/Robot Heading", pose.getRotation().getUnboundedDegrees());
        }

//        if (!hasFinishedPath() && hasStartedFollowing) {
//            double currentTime = Timer.getFPGATimestamp();
//            SmartDashboard.putNumber("Autopath Timer", currentTime - trajectoryStartTime);
//        }   
    // }

        // brian temp debug
        // public void passThru(double x, double y, double z){
        // mModules.forEach((m) -> m.passThru(x,y,z));
    }

    public static class PeriodicIO {
        // LOGGING
        public int schedDeltaDesired;
        public double schedDeltaActual;
        public double schedDuration;
        private double lastSchedStart;

        // Updated as part of periodic odometry
        public Pose2d robotPose = Pose2d.identity();
        public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

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
