package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.RobotConfiguration;
import frc.robot.config.SwerveConfiguration;
import frc.robot.constants.Constants;
import frc.robot.planners.DriveMotionPlanner;
import frc.robot.sensors.IMU;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Pose2dWithCurvature;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.trajectory.TrajectoryIterator;
import libraries.cheesylib.trajectory.timing.TimedState;
import libraries.cheesylib.util.SynchronousPIDF;
import libraries.cheesylib.util.Util;
import libraries.cyberlib.control.HeadingController;
import libraries.cyberlib.kinematics.ChassisSpeeds;
import libraries.cyberlib.kinematics.SwerveDriveKinematics;
import libraries.cyberlib.kinematics.SwerveDriveOdometry;
import libraries.cyberlib.kinematics.SwerveModuleState;
import libraries.cyberlib.utils.HolonomicDriveSignal;
import libraries.cyberlib.utils.RobotName;
import libraries.cyberlib.utils.SwerveDriveHelper;

public class Swerve extends Subsystem {

    public enum ControlState {
        NEUTRAL, MANUAL, DISABLED, PATH_FOLLOWING, VISION_AIM
    }

    private ControlState mControlState = ControlState.NEUTRAL;

    public SwerveConfiguration mSwerveConfiguration;
    public RobotConfiguration mRobotConfiguration;

    PeriodicIO mPeriodicIO = new PeriodicIO();
    private int mDefaultSchedDelta = 20;

    // Module declaration
    private final List<SwerveDriveModule> mModules = new ArrayList<>();
    private final SwerveDriveModule mFrontRight;
    private final SwerveDriveModule mFrontLeft;
    private final SwerveDriveModule mBackLeft;
    private final SwerveDriveModule mBackRight;

    private double lastUpdateTimestamp = 0;
    private int driveMode = 3;
    private boolean inAimingDeadzone;
    private final double kDefaultScaler = 2.5; // May need to be increased in Houston
    private double mAimingScaler = kDefaultScaler;

    // Swerve kinematics & odometry
    private final IMU mIMU;
    private boolean mIsBrakeMode;

    // TODO - do we still need this a odometry tracks this already?
    private Rotation2d mGyroOffset = Rotation2d.identity();

    private final SwerveDriveOdometry mOdometry;
    private final SwerveDriveKinematics mKinematics;

    // Aiming Controller to turn in place when using vision system to aim
    private SynchronousPIDF mAimingController = new SynchronousPIDF(
            Constants.kAimingKP, Constants.kAimingKI, Constants.kAimingKD
    );

    private static HeadingController mAimingHeaderController = null;

    private double lastAimTimestamp = -1.0;

    // Trajectory following
    private static DriveMotionPlanner mMotionPlanner;
    private boolean mOverrideTrajectory = false;

    private static String sClassName;
    private static int sInstanceCount;
    private static Swerve sInstance = null;

    public static Swerve getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new Swerve(caller);
            mAimingHeaderController = new HeadingController(
                Constants.kAimingKP, Constants.kAimingKI, Constants.kAimingKD, 0.0);
        
        } else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller) {
        System.out.println("(" + caller + ") " + "getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Swerve(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);

        mRobotConfiguration = RobotConfiguration.getRobotConfiguration(RobotName.name);

        mSwerveConfiguration = mRobotConfiguration.getSwerveConfiguration();
        mModules.add(mFrontRight = new SwerveDriveModule(mRobotConfiguration.getFrontRightModuleConstants(),
                mSwerveConfiguration.maxSpeedInMetersPerSecond));
        mModules.add(mFrontLeft = new SwerveDriveModule(mRobotConfiguration.getFrontLeftModuleConstants(),
                mSwerveConfiguration.maxSpeedInMetersPerSecond));
        mModules.add(mBackLeft = new SwerveDriveModule(mRobotConfiguration.getBackLeftModuleConstants(),
                mSwerveConfiguration.maxSpeedInMetersPerSecond));
        mModules.add(mBackRight = new SwerveDriveModule(mRobotConfiguration.getBackRightModuleConstants(),
                mSwerveConfiguration.maxSpeedInMetersPerSecond));

        // precaution to ensure misconfiguration modules don't run.
        stopSwerveDriveModules();

        mIMU = IMU.createImu(mRobotConfiguration.getImuType());
        mKinematics = new SwerveDriveKinematics(mSwerveConfiguration.moduleLocations);
        mOdometry = new SwerveDriveOdometry(mKinematics, mIMU.getYaw());
        mPeriodicIO.robotPose = mOdometry.getPose();

        mMotionPlanner = new DriveMotionPlanner();

        // rotationPow = SmartDashboard.getNumber("Rotation Power", -1);
        // if(rotationPow == -1) {
        //     SmartDashboard.putNumber("Rotation Power", 0);
        // }
    }

    @Override
    public void onStart(Phase phase) {
        synchronized (Swerve.this) {
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
        synchronized (Swerve.this) {
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
                case VISION_AIM:
                    handleAiming(timestamp);
                    break;
                case NEUTRAL:
                case DISABLED:
                default:
                    break;
            }
        }
    }

    @Override
    public synchronized void stop() {
        // mControlState = ControlState.NEUTRAL;
        setState(ControlState.NEUTRAL);
        stopSwerveDriveModules();
    }

    private void stopSwerveDriveModules() {
        mModules.forEach((m) -> m.stop());
    }

    public void convertCancoderToFX(){
        mModules.forEach((m) -> m.convertCancoderToFX2());
    }

    @Override
    public synchronized void zeroSensors() {
        zeroSensors(Constants.kRobotStartingPose);
    }

    public void toggleThroughDriveModes() {
        driveMode = ++driveMode % 4;
        System.out.println("Shifting Drive Mode***** to " + driveMode);
        switch (driveMode) {
            case 0:
                SmartDashboard.putString("Swerve/DriveMode", driveMode + " SwerveDriveHelper");
                break;
            case 1:
                SmartDashboard.putString("Swerve/DriveMode", driveMode + " Matt's Algorithm");
                break;
            case 2:
                SmartDashboard.putString("Swerve/DriveMode", driveMode + " Squared Inputs");
                break;
            case 3:
                SmartDashboard.putString("Swerve/DriveMode", driveMode + " Raw");
                break;
            default:
                SmartDashboard.putString("Swerve/DriveMode", driveMode + " Unknown");
                break;
        }
    }

    SlewRateLimiter forwardLimiter = new SlewRateLimiter(3.0, 0); // 1.5
    SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0, 0); // 1.5
    SlewRateLimiter rotationLimiter = new SlewRateLimiter(2, 0);

    /**
     * Handles MANUAL state which corresponds to joy stick inputs.
     *
     * <p>Using the joy stick values in PeriodicIO, calculate and updates the swerve
     * states. The joy stick values are as percent [-1.0, 1.0]. They need to be
     * converted to SI units before creating the ChassisSpeeds.</p>
     */
    private void handleManual() {
        HolonomicDriveSignal driveSignal;

        // TODO: Check deadband code in JStick (which reduces range and still returns 0 .s low).
        //  Should always get raw values here and apply deadband code here.
        switch (driveMode) {
            case 0:
                driveSignal = SwerveDriveHelper.calculate(
                        mPeriodicIO.forward, mPeriodicIO.strafe, mPeriodicIO.rotation,
                        mPeriodicIO.low_power, mPeriodicIO.field_relative, mPeriodicIO.use_heading_controller);
                break;
            case 1:
                // Matt's Swerve control
                double driveScalar = 1;
                if (mPeriodicIO.low_power) {
                     driveScalar = 0.25;
                }

                driveSignal = new HolonomicDriveSignal(
                    new Translation2d(mPeriodicIO.forward, mPeriodicIO.strafe).scale(driveScalar * 0.75),
                    mPeriodicIO.rotation * driveScalar * 0.8,
                        mPeriodicIO.field_relative);
                break;
            case 2:
                // Inputs squared
                // rotationPow = SmartDashboard.getNumber("Rotation Power", 1);
                driveSignal = new HolonomicDriveSignal(
                        new Translation2d(
                                Math.copySign(Math.pow(mPeriodicIO.forward, 2), mPeriodicIO.forward),
                                Math.copySign(Math.pow(mPeriodicIO.strafe, 2), mPeriodicIO.strafe)),
                        Math.copySign(Math.pow(mPeriodicIO.rotation, 2), mPeriodicIO.rotation),
                        mPeriodicIO.field_relative);
                break;
            case 3:
                // Slew Rate Limiter
                driveSignal = new HolonomicDriveSignal(
                        new Translation2d(
                        forwardLimiter.calculate(mPeriodicIO.forward),
                        strafeLimiter.calculate(mPeriodicIO.strafe)),
                        // rotationLimiter.calculate(mPeriodicIO.rotation),
                        Math.copySign(Math.pow(mPeriodicIO.rotation, 2), mPeriodicIO.rotation),
                        mPeriodicIO.field_relative);
                break;
            default:
                driveSignal = null;
        }
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

        // Normalize wheels speeds if any individual speed is above the specified
        // maximum.
        SwerveDriveKinematics.desaturateWheelSpeeds(
                mPeriodicIO.swerveModuleStates, mSwerveConfiguration.maxSpeedInMetersPerSecond);
    }

    public void setAimingTwistScaler(double scaler){
        if (Double.isNaN(scaler)){
            mAimingScaler = kDefaultScaler;
        }
        else {
            mAimingScaler = scaler;
        }
    }
    
    private void handleAiming(double timestamp) {
        var dt = timestamp - lastAimTimestamp;
        lastAimTimestamp = timestamp;

        if (dt > Util.kEpsilon) {
            mAimingHeaderController.setGoal(Math.toDegrees(mPeriodicIO.visionSetpointInRadians));
            var rotation = mAimingHeaderController.update();
            
//            mAimingController.setSetpoint(mPeriodicIO.visionSetpointInRadians);
//            double current_angle = getHeading().getDegrees();
//            double current_error = Math.toDegrees(mPeriodicIO.visionSetpointInRadians) - current_angle;
//            if (current_error > 180) {
//                current_angle += 360;
//            } else if (current_error < -180) {
//                current_angle -= 360;
//            }
//
//            var rotation = mAimingController.calculate(Math.toRadians(current_angle), dt);

            // Apply a minimum constant rotation velocity to overcome friction.
            // TODO:  Create constants for these
            // if ((mPeriodicIO.averageWheelVelocity / mSwerveConfiguration.maxSpeedInMetersPerSecond) < 0.2) {
            //     rotation += Math.copySign(0.3 * mSwerveConfiguration.maxSpeedInRadiansPerSecond, rotation);
            // }
            if (Math.abs(rotation)<.1){
                inAimingDeadzone = true;
            } else {
                inAimingDeadzone = false;
            }
            // System.out.println("rot:"+rotation);
            // System.out.println("Swerve.handleAiming() setpoint: "+(((int)(10.0*Math.toDegrees(mPeriodicIO.visionSetpointInRadians)))/10)+ " rotation: "+rotation);
            // Turn in place implies no translational velocity.
            HolonomicDriveSignal driveSignal = new HolonomicDriveSignal(
                    Translation2d.identity(),
                    rotation * mAimingScaler, //2.5,
                    true);

            updateModules(driveSignal);
            //System.out.println("Robot Pose " + mPeriodicIO.robotPose);
        }
    }

    public synchronized void EnableAimingController() {
        mAimingHeaderController.reset();
        mAimingHeaderController.setHeadingControllerState(HeadingController.HeadingControllerState.MAINTAIN);
    }

    public synchronized void DisableAimingController() {
        mAimingHeaderController.setHeadingControllerState(HeadingController.HeadingControllerState.OFF);
    }

    public boolean isInAimingDeadzone() {
        return inAimingDeadzone;
    }

    // //Assigns appropriate directions for scrub factors
    // public void setCarpetDirection(boolean standardDirection) {
    // mModules.forEach((m) -> m.setCarpetDirection(standardDirection));
    // }

    /**
     * Gets the current control state for the Swerve Drive.
     *
     * @return The current control state.
     */
    public synchronized ControlState getState() {
        return mControlState;
    }

    /**
     * Sets the control state for the Swerve Drive.
     *
     * @param newState The desired state.
     */
    public synchronized void setState(ControlState newState) {
        if (mControlState != newState) {
            System.out.println(mControlState + " to " + newState);
            switch (newState) {
                case NEUTRAL:
                    // mPeriodicIO.strafe = 0;
                    // mPeriodicIO.forward = 0;
                    // mPeriodicIO.rotation = 0;
                    // stopSwerveDriveModules();
                    mPeriodicIO.forward= 0.0;
                    mPeriodicIO.strafe = 0.0;
                    mPeriodicIO.rotation = 0.0;
                    mPeriodicIO.visionSetpointInRadians = getHeading().getRadians();
                    mPeriodicIO.schedDeltaDesired = 10; // this is a fast cycle used while testing
                    break;
                case MANUAL:
                case DISABLED:
                    mPeriodicIO.schedDeltaDesired = 10; // this is a fast cycle used while testing
                    break;

                case VISION_AIM:
                case PATH_FOLLOWING:
                    mPeriodicIO.schedDeltaDesired = 20;
                    break;
            }
        }
        mControlState = newState;
    }

    public Pose2d getPose() {
        return mPeriodicIO.robotPose;
    }

    public Rotation2d getHeading() {
        return mPeriodicIO.robotPose.getRotation();
    }

    public ChassisSpeeds getChassisSpeeds() { return mPeriodicIO.chassisSpeeds; }

    /**
     * Sets the current robot position on the field.
     *
     * @param pose The (x,y,theta) position.
     */
    public synchronized void setRobotPosition(Pose2d pose) {
        mOdometry.resetPosition(pose, mIMU.getYaw());
        mPeriodicIO.robotPose = mOdometry.getPose();
        mGyroOffset = pose.getRotation().rotateBy(Rotation2d.fromDegrees(mIMU.getYaw().getDegrees()).inverse());
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

        // Calculate a threshold for use in aiming
        mPeriodicIO.averageWheelVelocity = (frontLeft.speedInMetersPerSecond + frontLeft.speedInMetersPerSecond +
                backLeft.speedInMetersPerSecond + backRight.speedInMetersPerSecond) / 4;

        // order is CCW starting with front right.
        mPeriodicIO.chassisSpeeds = mKinematics.toChassisSpeeds(frontRight, frontLeft, backLeft, backRight);
        mPeriodicIO.robotPose = mOdometry.updateWithTime(
                timestamp, mIMU.getYaw(), frontRight, frontLeft, backLeft, backRight);
    }

    /**
     * Gets whether path following is done or not. Typically, called in autonomous
     * actions.
     *
     * @return true if done; otherwise false.
     */
    public boolean isDoneWithTrajectory() {
        if (mMotionPlanner == null || mControlState != ControlState.PATH_FOLLOWING) {
            return true;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }

    /**
     * Sets a trajectory to follow.
     *
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
        HolonomicDriveSignal driveSignal = null;
        if (mControlState == ControlState.PATH_FOLLOWING) {

            // Get updated drive signal
            var trajectorySignal = mMotionPlanner.update(now, mPeriodicIO.robotPose, mPeriodicIO.chassisSpeeds);
            mPeriodicIO.error = mMotionPlanner.error();
            mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

            if (!mOverrideTrajectory) {
                if (trajectorySignal != null) {
                    driveSignal = trajectorySignal;
                }
            }
            setPathFollowingVelocity(driveSignal);
        } else {
            DriverStation.reportError("Swerve is not in path following state.", false);
        }
    }

    /**
     * Configure modules for open loop control
     * <p>
     * 
     * @param signal The HolonomicDriveSignal to apply
     */
    public synchronized void setOpenLoop(HolonomicDriveSignal signal) {
        if (mControlState != ControlState.MANUAL) {
            setBrakeMode(true);
            mControlState = ControlState.MANUAL;
        }
        updateModules(signal);
    }

    /**
     * Configure modules for path following.
     * <p>
     * 
     * @param signal The HolonomicDriveSignal to apply
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
    // public synchronized void disable() {
    //     mModules.forEach((m) -> m.disable());
    //     mControlState = ControlState.DISABLED;
    // }

    /**
     * Zeroes the drive motors, and sets the robot's internal position and heading
     * to match that of the fed pose
     *
     * DO NOT use this to reset the IMU mid match. Use
     * {@link #setRobotPosition(Pose2d)} for that purpose.
     */
    public synchronized void zeroSensors(Pose2d startingPose) {
        setRobotPosition(startingPose);
        mIMU.setAngle(startingPose.getRotation().getUnboundedDegrees());
        // mModules.forEach((m) -> m.zeroSensors(startingPose));
    }


    /**
     * Set the setpoint used when aiming the robot for auto shooting.
     *
     * @param setPointInRadians Setpoint in radians
     * @param feedforward Feed-forward term
     * @param timestamp Current timestamp
     */
    public synchronized void setAimingSetpoint(double setPointInRadians, double feedforward, double timestamp) {
        if (mControlState != ControlState.VISION_AIM) {
            mControlState = ControlState.VISION_AIM;
            // seed the last timestamp
            lastAimTimestamp = timestamp;
            mAimingController.reset();
        }
        mPeriodicIO.visionSetpointInRadians = setPointInRadians;
        mPeriodicIO.visionFeedForward = feedforward;
    }

    /**
     * Sets inputs from driver in teleop mode.
     * <p>
     * 
     * @param forward                percent to drive forwards/backwards (as double
     *                               [-1.0,1.0]).
     * @param strafe                 percent to drive sideways left/right (as double
     *                               [-1.0,1.0]).
     * @param rotation               percent to rotate chassis (as double
     *                               [-1.0,1.0]).
     * @param low_power              whether to use low or high power.
     * @param field_relative         whether operation is robot centric or field
     *                               relative.
     * @param use_heading_controller whether the heading controller is being used.
     */
    public void setTeleopInputs(double forward, double strafe, double rotation, boolean low_power,
            boolean field_relative, boolean use_heading_controller) {
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
        String headers;

        headers =   sClassName + ".schedDeltaDesired," +
                    sClassName + ".schedDeltaActual," +
                    sClassName + ".schedDuration," +
                    sClassName + ".gyro_heading," +
                    sClassName + ".robotPoseX," +
                    sClassName + ".robotPoseY," +
                    sClassName + ".robotPoseHeading,";

                    headers += mModules.get(0).getLogHeaders()+",";
                    headers += mModules.get(1).getLogHeaders()+",";
                    headers += mModules.get(2).getLogHeaders()+",";
                    headers += mModules.get(3).getLogHeaders();
        return headers;
    }


    @Override
    public String getLogValues(boolean telemetry) {
        String values;
        if (telemetry) {
            values = ",,,";
        } else {
            values = "" + mPeriodicIO.schedDeltaDesired + "," +
                         mPeriodicIO.schedDeltaActual + "," +
                         (Timer.getFPGATimestamp() - mPeriodicIO.lastSchedStart)+",";
        }

        values += mPeriodicIO.gyro_heading.getDegrees()+",";
        values += mPeriodicIO.robotPose.getTranslation().x()+",";
        values += mPeriodicIO.robotPose.getTranslation().y()+",";
        values += mPeriodicIO.robotPose.getRotation().getDegrees()+",";
        values += mModules.get(0).getLogValues(telemetry)+",";
        values += mModules.get(1).getLogValues(telemetry)+",";
        values += mModules.get(2).getLogValues(telemetry)+",";
        values += mModules.get(3).getLogValues(telemetry);

        return values;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        double now = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart = now;
        mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mIMU.getYaw().getDegrees()).rotateBy(mGyroOffset);
        // mPeriodicIO.gyroYaw = mIMU.getYaw();

        // read modules
        mModules.forEach((m) -> m.readPeriodicInputs());
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // Set the module state for each module
        // All modes should use this method of module states.
        if (mControlState != ControlState.NEUTRAL){
            for (int i = 0; i < mModules.size(); i++) {
                mModules.get(i).setState(mPeriodicIO.swerveModuleStates[i]);
            }
        }

        mModules.forEach((m) -> m.writePeriodicOutputs());
    }

    @Override
    public int whenRunAgain() {
        return 20;// mPeriodicIO.schedDeltaDesired; (brian test)
    }

    @Override
    public void outputTelemetry() {
        mModules.forEach((m) -> m.outputTelemetry());
        // SmartDashboard.putString("Swerve/Swerve State", mControlState.toString());
        // SmartDashboard.putString("Swerve/Pose", mPeriodicIO.robotPose.toString());
        // SmartDashboard.putString("Swerve/Chassis Speeds", mPeriodicIO.chassisSpeeds.toString());
        // SmartDashboard.putBoolean("Swerve/isOnTarget", isOnTarget());
        
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

            
            SmartDashboard.putString("Swerve/Pigeon Heading", mPeriodicIO.gyro_heading.toString());
            // SmartDashboard.putString("Swerve/Pigeon Raw Yaw", mPeriodicIO.gyroYaw.toString());
        }
    }

    public static class PeriodicIO {
        // LOGGING
        public int schedDeltaDesired;
        public double schedDeltaActual;
        private double lastSchedStart;

        // Updated as part of periodic odometry
        public Pose2d robotPose = Pose2d.identity();
        public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

        // Updated as part of trajectory following
        public Pose2d error = Pose2d.identity();
        public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<>(Pose2dWithCurvature.identity());

        // Updated as part of vision aiming
        public double visionSetpointInRadians;
        public double visionFeedForward;
        public double averageWheelVelocity;

        // Inputs
        public Rotation2d gyro_heading = Rotation2d.identity();
        // public Rotation2d gyroYaw = Rotation2d.identity();
        public double forward;
        public double strafe;
        public double rotation;
        public boolean low_power;
        public boolean field_relative;
        public boolean use_heading_controller;

        // OUTPUTS
        public SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(0, Rotation2d.identity()),
                new SwerveModuleState(0, Rotation2d.identity()),
                new SwerveModuleState(0, Rotation2d.identity()),
                new SwerveModuleState(0, Rotation2d.identity())
        };
    }
}
