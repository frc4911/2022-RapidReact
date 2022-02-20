package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.config.DeadEye;
import frc.robot.config.Junior;
import frc.robot.config.Robot2022;
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
import libraries.cyberlib.utils.HolonomicDriveSignal;

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
	public SwerveDriveModule mFrontRight=null;
	private SwerveDriveModule mFrontLeft=null, mBackLeft=null, mBackRight=null;

	double lastUpdateTimestamp = 0;

	// Swerve kinematics & odometry
	private final PigeonTwo mPigeon;
    private boolean mIsBrakeMode;
	private Rotation2d mGyroOffset = Rotation2d.identity();

	private final SwerveDriveOdometry mOdometry;
	private final SwerveDriveKinematics mKinematics;

    // Trajectory following
    private static DriveMotionPlanner mMotionPlanner;
    private boolean mOverrideTrajectory = false;

    private int mListIndex = -1;

    private static String sClassName;
    private static int sInstanceCount;
    private static Swerve sInstance = null;
    public  static Swerve getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new Swerve(caller);
            mMotionPlanner = new DriveMotionPlanner();
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


        mPigeon = PigeonTwo.getInstance();

        if (RobotName.name.equals(Constants.kJuniorName)) {
            mSwerveConfiguration = Junior.kSwerveConfiguration;
            mModules.add(mFrontRight = new SwerveDriveModule(Junior.kFrontRightModuleConstants, mSwerveConfiguration.maxSpeedInMetersPerSecond));
            mModules.add(mFrontLeft = new SwerveDriveModule(Junior.kFrontLeftModuleConstants, mSwerveConfiguration.maxSpeedInMetersPerSecond));
            mModules.add(mBackLeft = new SwerveDriveModule(Junior.kBackLeftModuleConstants, mSwerveConfiguration.maxSpeedInMetersPerSecond));
            mModules.add(mBackRight = new SwerveDriveModule(Junior.kBackRightModuleConstants, mSwerveConfiguration.maxSpeedInMetersPerSecond));
        }
        else if (RobotName.name.equals(Constants.kDeadEyeName)) {
            mSwerveConfiguration = DeadEye.kSwerveConfiguration;
            mModules.add(mFrontRight = new SwerveDriveModule(DeadEye.kFrontRightModuleConstants, mSwerveConfiguration.maxSpeedInMetersPerSecond));
            mModules.add(mFrontLeft = new SwerveDriveModule(DeadEye.kFrontLeftModuleConstants, mSwerveConfiguration.maxSpeedInMetersPerSecond));
            mModules.add(mBackLeft = new SwerveDriveModule(DeadEye.kBackLeftModuleConstants, mSwerveConfiguration.maxSpeedInMetersPerSecond));
            mModules.add(mBackRight = new SwerveDriveModule(DeadEye.kBackRightModuleConstants, mSwerveConfiguration.maxSpeedInMetersPerSecond));
        }
        else if (RobotName.name.equals(Constants.kRobot2022Name)) {
            mSwerveConfiguration = Robot2022.kSwerveConfiguration;
            mModules.add(mFrontRight = new SwerveDriveModule(Robot2022.kFrontRightModuleConstants, mSwerveConfiguration.maxSpeedInMetersPerSecond));
            mModules.add(mFrontLeft = new SwerveDriveModule(Robot2022.kFrontLeftModuleConstants, mSwerveConfiguration.maxSpeedInMetersPerSecond));
            mModules.add(mBackLeft = new SwerveDriveModule(Robot2022.kBackLeftModuleConstants, mSwerveConfiguration.maxSpeedInMetersPerSecond));
            mModules.add(mBackRight = new SwerveDriveModule(Robot2022.kBackRightModuleConstants, mSwerveConfiguration.maxSpeedInMetersPerSecond));
        }

        mKinematics = new SwerveDriveKinematics(mSwerveConfiguration.moduleLocations);
		mOdometry = new SwerveDriveOdometry(mKinematics, mPigeon.getRotation2dYaw());
        mPeriodicIO.robotPose = mOdometry.getPose();

        // mMotionPlanner = new DriveMotionPlanner();

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
                        updatePathFollower(lastUpdateTimestamp);
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
        HolonomicDriveSignal driveSignal;

        // Helper to make driving feel better
//        driveSignal = SwerveDriveHelper.calculate(
//                mPeriodicIO.forward, mPeriodicIO.strafe, mPeriodicIO.rotation,
//                mPeriodicIO.low_power, mPeriodicIO.field_relative, mPeriodicIO.use_heading_controller);

        driveSignal = new HolonomicDriveSignal(new Translation2d(mPeriodicIO.forward, mPeriodicIO.strafe),
                    mPeriodicIO.rotation, mPeriodicIO.field_relative);

        setOpenLoop(driveSignal);
    }

    private void updateModules(HolonomicDriveSignal driveSignal) {
        ChassisSpeeds chassisSpeeds;

        if (driveSignal == null) {
            chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        } else {
            // Convert to velocities and SI units
            var translationInput = driveSignal.getTranslation().scale(mSwerveConfiguration.maxSpeedInMetersPerSecond);
            var rotationInput = driveSignal.getRotation() * mSwerveConfiguration.maxSpeedInRadiansPerSecond;

            if (driveSignal.isFieldOriented()) {
                // Adjust for robot heading to maintain field relative motion.
                translationInput = translationInput.rotateBy(getPose().getRotation().inverse());
            }

            chassisSpeeds = new ChassisSpeeds(translationInput.x(), translationInput.y(), rotationInput);
        }

        // Now calculate the new Swerve Module states using inverse kinematics.
        mPeriodicIO.swerveModuleStates = mKinematics.toSwerveModuleStates(chassisSpeeds);

        // Normalize wheels speeds if any individual speed is above the specified maximum.
        SwerveDriveKinematics.desaturateWheelSpeeds(
                mPeriodicIO.swerveModuleStates, mSwerveConfiguration.maxSpeedInMetersPerSecond);
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
        return mPigeon.getRotation2dYaw();
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
     * @param pose The (x,y,theta) position.
     */
    public synchronized void setRobotPosition(Pose2d pose) {
        mOdometry.resetPosition(pose, mPigeon.getRotation2dYaw());
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


    /**
     * Gets whether path following is done or not.  Typically, called in autonomous actions.
     * <p>
     * @return true if done; otherwise false.
     */
    public boolean isDoneWithTrajectory() {
        if (mMotionPlanner == null || mControlState != mControlState.PATH_FOLLOWING) {
            return false;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }


    /**
     * Sets a trajectory to follow.
     * <p>
     * @param trajectory The trajectory to follow.
     */
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

    private void updatePathFollower(double now) {
        if (mControlState == ControlState.PATH_FOLLOWING) {
            // Get updated drive signal
            HolonomicDriveSignal driveSignal = null;

            Optional<HolonomicDriveSignal> trajectorySignal = mMotionPlanner.update(now, mPeriodicIO.robotPose, mPeriodicIO.chassisSpeeds);
            mPeriodicIO.error = mMotionPlanner.error();
            mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

            if (!mOverrideTrajectory) {
                if (trajectorySignal.isPresent()) {
                    driveSignal = trajectorySignal.get();
                }
            }

            setPathFollowingVelocity(driveSignal);

        } else {
            DriverStation.reportError("Swerve is not in path following state.", false);
        }
    }

    /**
     * Configure modules for open loop control
     */
    public synchronized void setOpenLoop(HolonomicDriveSignal signal) {
        if (mControlState != ControlState.MANUAL) {
            setBrakeMode(true); // TODO: Consider driving in coast mode
            mControlState = ControlState.MANUAL;
        }
        updateModules(signal);
    }

    /**
     * Configure modules for path following.
     */
    public synchronized void setPathFollowingVelocity(HolonomicDriveSignal signal) {
        if (mControlState != ControlState.PATH_FOLLOWING) {
            setBrakeMode(true);
            mControlState = ControlState.PATH_FOLLOWING;
        }
        updateModules(signal);
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean shouldEnable) {
        if (mIsBrakeMode != shouldEnable) {
            mIsBrakeMode = shouldEnable;
            NeutralMode mode = shouldEnable ? NeutralMode.Brake : NeutralMode.Coast;

            mModules.forEach((m) -> m.setNeutralMode(mode));
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
        mPeriodicIO.gyro_heading     = Rotation2d.fromDegrees(mPigeon.getRotation2dYaw().getDegrees()).rotateBy(mGyroOffset);

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
            SmartDashboard.putString("Swerve/Pose", mPeriodicIO.robotPose.toString());
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
        }
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

        // Updated as part of trajectory following
        public Pose2d error = Pose2d.identity();
        public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());


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
