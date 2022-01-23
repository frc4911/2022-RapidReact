package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotState;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Pose2dWithCurvature;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.loops.ILooper;
import libraries.cheesylib.loops.Loop;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.trajectory.Trajectory;
import libraries.cheesylib.trajectory.timing.TimedState;
import libraries.cheesylib.util.SynchronousPIDF;
import libraries.cheesylib.util.Util;
import libraries.cheesylib.vision.AimingParameters;
import libraries.cyberlib.kinematics.ChassisSpeeds;
import libraries.cyberlib.kinematics.SwerveDriveKinematics;
import libraries.cyberlib.kinematics.SwerveDriveOdometry;
import libraries.cyberlib.kinematics.SwerveModuleState;
import libraries.cyberlib.utils.RobotName;
import libraries.madtownlib.util.SwerveHeadingController;
import libraries.madtownlib.util.SwerveInverseKinematics;
import libraries.madtownlib.util.Utils;
import libraries.madtownlib.util.VisionCriteria;
import libraries.madtownlib.vectors.VectorField;

public class Swerve extends Subsystem {

    public enum ControlState{
        NEUTRAL, MANUAL, POSITION, ROTATION, DISABLED, VECTORIZED,
        TRAJECTORY, VELOCITY, VISION, VISION_AIM, CELL_AIM
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


    // Heading controller methods
    SwerveHeadingController headingController = new SwerveHeadingController(Constants.kIsUsingTractionWheels);

    RobotState robotState;
    private int mListIndex = -1;

    // Vision dependencies
    boolean visionUpdatesAllowed = true;
    Translation2d visionTargetPosition = new Translation2d();
    public Translation2d getVisionTargetPosition(){ return visionTargetPosition; }
    int visionUpdateCount = 0;
    int attemptedVisionUpdates = 0;
    int visionVisibleCycles = 0;
    boolean firstVisionCyclePassed = false;
    VisionCriteria visionCriteria = new VisionCriteria();
    double initialVisionDistance = 0.0;
    Optional<AimingParameters> aimingParameters;
    double error = 0;
    // AimingParameters latestAim = new AimingParameters(100.0, new Rotation2d(), 0.0, 0.0, new Rotation2d());
    AimingParameters latestAim = new AimingParameters( new Pose2d(), new Pose2d(), new Rotation2d(), 0.0, 0.0, new Rotation2d(), 1);
    Translation2d latestTargetPosition = new Translation2d();
    Translation2d lastVisionEndTranslation = new Translation2d(-Constants.kRobotProbeExtrusion, 0.0);
    boolean visionUpdateRequested = false;
    boolean robotHasDisk = false;
    boolean useFixedVisionOrientation = false;
    Rotation2d fixedVisionOrientation = Rotation2d.fromDegrees(180.0);
    double visionCutoffDistance = Constants.kClosestVisionDistance;
    double visionTrackingSpeed = Constants.kDefaultVisionTrackingSpeed;
    // private double lastAimTimestamp;
    private boolean mIsOnTarget;
    boolean needsToNotifyDrivers = false;
//    TrajectoryGenerator generator;

    // Odometry variables
    Pose2d pose;
    double distanceTraveled;
    double currentVelocity = 0;

    // Module configuration variables (for beginnning of auto)
    boolean modulesReady = false;
    boolean alwaysConfigureModules = false;
    boolean moduleConfigRequested = false;
    Pose2d startingPose = Constants.kRobotStartingPose;

    // Trajectory variables
//    DriveMotionPlanner motionPlanner;
    double rotationScalar;
    double trajectoryStartTime = 0;
    Translation2d lastTrajectoryVector = new Translation2d();
    boolean hasStartedFollowing = false;
    boolean hasFinishedPath = false;


    private static String sClassName;
    private static int sInstanceCount;
    private static Swerve sInstance = null;
    public  static Swerve getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new Swerve(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
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

//        motionPlanner = new DriveMotionPlanner();
        robotState = RobotState.getInstance(sClassName);
//        generator = TrajectoryGenerator.getInstance();
    }

    private final Loop loop = new Loop() {

        @Override
        public void onStart(Phase phase) {
            synchronized(Swerve.this){
                headingController.temporarilyDisable();
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
            synchronized(Swerve.this){
                updateControlCycle(timestamp);
                lastUpdateTimestamp = timestamp;
            }
        }

        @Override
        public void onStop(double timestamp) {
            synchronized(Swerve.this){
                stop();
            }
        }
    };

    /**
     * Returns the angle of the robot as a Rotation2d.
     *
     * @return The angle of the robot (CCW).
     */
    private Rotation2d getAngle() {
        // Expects CCW.
        return mPigeon.getYaw();
    }

    /**
     * Updates the field relative position of the robot.
     */
    private void updateOdometry(double timestamp) {
        var frontLeft = mFrontLeft.getState();
        var frontRight = mFrontRight.getState();
        var backLeft = mBackLeft.getState();
        var backRight = mBackRight.getState();

        mChassisSpeeds = mKinematics.toChassisSpeeds(frontLeft,frontRight, backLeft, backRight);
        mOdometry.updateWithTime(timestamp, getAngle(), frontLeft,frontRight, backLeft, backRight
        );
    }


    /** Called every cycle to update the swerve based on its control state */
    public synchronized void updateControlCycle(double timestamp) {
        switch(mControlState) {
            case MANUAL:
                // Calculates and updates the swerve states in PeriodicIO
                // The PeriodicIO values are set in JSticks TeleOp method, by calling setTeleopInputs().
                var chassisSpeeds = swerveDriveHelper(mPeriodicIO.forward, mPeriodicIO.strafe, mPeriodicIO.rotation, mPeriodicIO.low_power,
                        mPeriodicIO.field_relative, mPeriodicIO.use_heading_controller);

                // Now command the new Swerve Module states
                mPeriodicIO.swerveModuleStates = mKinematics.toSwerveModuleStates(chassisSpeeds);

                // Make sure they are valid
                SwerveDriveKinematics.desaturateWheelSpeeds(mPeriodicIO.swerveModuleStates, Constants.kSwerveDriveMaxSpeedInMetersPerSecond);
                break;
            case TRAJECTORY:
//                if(!motionPlanner.isDone()){
//                    Translation2d driveVector = motionPlanner.update(timestamp, pose);
//
//                    if(modulesReady){
//                        if(!hasStartedFollowing){
//                            if(moduleConfigRequested){
//                                zeroSensors(startingPose);
//                                System.out.println("Position reset for auto");
//                            }
//                            hasStartedFollowing = true;
//                        }
//                        double rotationInput = Util.deadBand(Util.limit(rotationCorrection*rotationScalar*driveVector.norm(), motionPlanner.getMaxRotationSpeed()), 0.01);
//                        if(Util.epsilonEquals(driveVector.norm(), 0.0, Constants.kEpsilon)){
//                            driveVector = lastTrajectoryVector;
//                            setVelocityDriveOutput(inverseKinematics.updateDriveVectors(driveVector,
//                                    rotationInput, pose, false), 0.0);
//                            // System.out.println("Trajectory Vector set: " + driveVector.toString());
//                        }else{
//                            setVelocityDriveOutput(inverseKinematics.updateDriveVectors(driveVector,
//                                    rotationInput, pose, false));
//                            // System.out.println("Trajectory Vector set: " + driveVector.toString());
//                        }
//                    }else if(!moduleConfigRequested){
//                        //set10VoltRotationMode(true);
//                        setModuleAngles(inverseKinematics.updateDriveVectors(driveVector,
//                                0.0, pose, false));
//                        moduleConfigRequested = true;
//                    }
//
//                    if(moduleAnglesOnTarget() && !modulesReady){
//                        set10VoltRotationMode(false);
//                        modules.forEach((m) -> m.resetLastEncoderReading());
//                        modulesReady = true;
//                        System.out.println("Modules Ready");
//                    }
//
//                    lastTrajectoryVector = driveVector;
//                }else{
//
//                    if(!hasFinishedPath){
//                        System.out.println("Path completed in: " + (timestamp - trajectoryStartTime));
//                        hasFinishedPath = true;
//                        if(alwaysConfigureModules) requireModuleConfiguration();
//                    }
//                }
                break;
            case NEUTRAL:
                stop();
                break;
            case DISABLED:
            default:
                break;
        }
    }

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

    public Pose2d getPose(){
        return pose;
    }

    public boolean hasFinishedPath() {
        return hasFinishedPath;
    }

//    //Assigns appropriate directions for scrub factors
//    public void setCarpetDirection(boolean standardDirection){
//        mModules.forEach((m) -> m.setCarpetDirection(standardDirection));
//    }

    public ControlState getState(){
        return mControlState;
    }

    public void setState(ControlState newState){
        if (mControlState != newState) {
            System.out.println(mControlState + " to " + newState);
            switch (newState){
                case NEUTRAL:
                case MANUAL:
                case DISABLED:
                    mPeriodicIO.schedDeltaDesired = 100;
                    break;
                case ROTATION:
                case POSITION:

                case VISION_AIM:
                case CELL_AIM:
                case VELOCITY:
                case VECTORIZED:
                case TRAJECTORY:
                case VISION:
                    mPeriodicIO.schedDeltaDesired = 20;
                    break;
            }
        }

        mControlState = newState;
    }

    public synchronized void setOpenLoop(double forward, double strafe, double rotation) {
        if (mControlState != ControlState.MANUAL) {
            mControlState = ControlState.MANUAL;
        }

        mPeriodicIO.forward = forward;
        mPeriodicIO.strafe = strafe;
        mPeriodicIO.rotation = rotation;
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(mPeriodicIO.forward, mPeriodicIO.strafe,mPeriodicIO.rotation);
//        mPeriodicIO.swerveModuleStates = mKinematics.toSwerveModuleStates(chassisSpeeds);
    }

    //Various methods to control the heading controller
//    public synchronized void rotate(double goalHeading){
//        if(translationalVector.x() == 0 && translationalVector.y() == 0)
//            rotateInPlace(goalHeading);
//        else
//            headingController.setStabilizationTarget(
//                    Utils.placeInAppropriate0To360Scope(pose.getRotation().getUnboundedDegrees(), goalHeading));
//    }

    public void rotateInPlace(double goalHeading){
        setState(ControlState.ROTATION);
        headingController.setStationaryTarget(
                Utils.placeInAppropriate0To360Scope(pose.getRotation().getUnboundedDegrees(), goalHeading));
    }


    public synchronized boolean isOnTarget() {
        return mIsOnTarget;
    }

    public void setAbsolutePathHeading(double absoluteHeading){
        headingController.setSnapTarget(absoluteHeading);
    }

//    /** Configures each module to match its assigned vector */
//    public void setDriveOutput(List<Translation2d> driveVectors){
//        for(int i=0; i<mModules.size(); i++){
//            if(Utils.shouldReverse(driveVectors.get(i).direction().getDegrees(), modules.get(i).getModuleAngle().getDegrees())){
//                modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
//                modules.get(i).setDriveOpenLoop(-driveVectors.get(i).norm());
//            }else{
//                modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
//                modules.get(i).setDriveOpenLoop(driveVectors.get(i).norm());
//            }
//        }
//    }

    int count = 0;


//    /** Increases each module's rotational power cap for the beginning of auto */
//    public void set10VoltRotationMode(boolean tenVolts){
//        mModules.forEach((m) -> m.set10VoltRotationMode(tenVolts));
//    }

//    /**
//     * @return Whether or not at least one module has reached its MotionMagic setpoint
//     */
//    public boolean positionOnTarget(){
//        boolean onTarget = false;
//        for(SwerveDriveModule m : modules){
//            onTarget |= m.drivePositionOnTarget();
//        }
//        return onTarget;
//    }

    /**
     * @return Whether or not all modules have reached their angle setpoints
     */
//    public boolean moduleAnglesOnTarget(){
//        boolean onTarget = true;
//        for(SwerveDriveModule m : modules){
//            onTarget &= m.angleOnTarget();
//        }
//        return onTarget;
//    }

    /**
     * Sets a trajectory for the robot to follow
     * @param trajectory
     * @param targetHeading Heading that the robot will rotate to during its path following
     * @param rotationScalar Scalar to increase or decrease the robot's rotation speed
     * @param followingCenter The point (relative to the robot) that will follow the trajectory
     */
    public synchronized void setTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, double targetHeading,
                                           double rotationScalar, Translation2d followingCenter) {
//        hasStartedFollowing = false;
//        hasFinishedPath = false;
//        moduleConfigRequested = false;
////        motionPlanner.reset();
////        motionPlanner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(trajectory)));
////        motionPlanner.setFollowingCenter(followingCenter);
//        inverseKinematics.setCenterOfRotation(followingCenter);
//        setAbsolutePathHeading(targetHeading);
//        this.rotationScalar = rotationScalar;
//        trajectoryStartTime = Timer.getFPGATimestamp();
//        setState(ControlState.TRAJECTORY);
    }

    public synchronized void setTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, double targetHeading,
                                           double rotationScalar){
        setTrajectory(trajectory, targetHeading, rotationScalar, Translation2d.identity());
    }

    // *** NEW SWERVE ***
//    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
//        if (mMotionPlanner != null) {
//            mOverrideTrajectory = false;
//            mMotionPlanner.reset();
//            mMotionPlanner.setTrajectory(trajectory);
//            setState(ControlState.TRAJECTORY);
//        }
//    }
    // END NEW WERVE

    public synchronized void setRobotCentricTrajectory(Translation2d relativeEndPos, double targetHeading){
        setRobotCentricTrajectory(relativeEndPos, targetHeading, 45.0);
    }

    public synchronized void setRobotCentricTrajectory(Translation2d relativeEndPos, double targetHeading, double defaultVel){
//        modulesReady = true;
//        Translation2d endPos = pose.transformBy(Pose2d.fromTranslation(relativeEndPos)).getTranslation();
//        Rotation2d startHeading = endPos.translateBy(pose.getTranslation().inverse()).direction();
//        List<Pose2d> waypoints = new ArrayList<>();
//        waypoints.add(new Pose2d(pose.getTranslation(), startHeading));
//        waypoints.add(new Pose2d(pose.transformBy(Pose2d.fromTranslation(relativeEndPos)).getTranslation(), startHeading));
//        Trajectory<TimedState<Pose2dWithCurvature>> trajectory = generator.generateTrajectory(false, waypoints, Arrays.asList(), 96.0, 60.0, 60.0, 9.0, defaultVel, 1);
//        double heading = Utils.placeInAppropriate0To360Scope(pose.getRotation().getUnboundedDegrees(), targetHeading);
//        setTrajectory(trajectory, heading, 1.0);
    }


    /** Puts all rotation and drive motors into open-loop mode */
    public synchronized void disable(){
        mModules.forEach((m) -> m.disable());
        setState(ControlState.DISABLED);
    }

    /** Zeroes the drive motors, and sets the robot's internal position and heading to match that of the fed pose */
    public synchronized void zeroSensors(Pose2d startingPose){
        mPigeon.setAngle(startingPose.getRotation().getUnboundedDegrees());
        mModules.forEach((m) -> m.zeroSensors(startingPose));
//        pose = startingPose;
//        distanceTraveled = 0;
    }

    public synchronized void resetPosition(Pose2d newPose){
        pose = new Pose2d(newPose.getTranslation(), pose.getRotation());
        mModules.forEach((m) -> m.zeroSensors(pose));
//        distanceTraveled = 0;
    }

    /**
     * Sets inputs from driver in teleop mode
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

        // Convert the joystick inputs to SI units for
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                translationalInput.x() * Constants.kSwerveDriveMaxSpeedInMetersPerSecond,
                translationalInput.y() * Constants.kSwerveDriveMaxSpeedInMetersPerSecond,
                rotationInput * Constants.kSwerveRotationSpeedInMetersPerSecond);
        return chassisSpeeds;
    }


    @Override
    public String getLogHeaders() {
        StringBuilder allHeaders = new StringBuilder(256);
        for (SwerveDriveModule m: mModules){
            if (allHeaders.length() > 0){
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

    private String generateLogValues(boolean telemetry){
        String values;
        if (telemetry){
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
        for (SwerveDriveModule m: mModules){
            if (allValues.length() > 0){
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

        //brian
        aimingParameters = robotState.getOuterGoalParameters();
        mIsOnTarget = false;
        if (aimingParameters.isPresent()) {
            //error = -1 * aimingParameters.get().getRobotToGoal().getTranslation().y();
            //radians
            error = Math.atan2(-aimingParameters.get().getRobotToGoal().getTranslation().y(), aimingParameters.get().getRobotToGoal().getTranslation().x());
            if (Math.abs(error) <= Math.toRadians(2.0)){
                if (++count>3){
                    mIsOnTarget = true; //0.5
                }
            }
            else{
                count = 0;
            }
            // System.out.println(count+" "+Math.toDegrees(error));
        }
        // else{
        // 	System.out.println("error not present******************************************");
        // }
        SmartDashboard.putNumber("LL error", Math.toDegrees(error));

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
        SmartDashboard.putBoolean("Swerve/isOnTarget", isOnTarget());
        if(Constants.kDebuggingOutput){
            // Get the current pose from odometry state
            Pose2d pose = mOdometry.getPose();
            SmartDashboard.putString("Swerve/pose", pose.toString());
            SmartDashboard.putString("Swerve/State", mControlState.toString());
            SmartDashboard.putNumberArray("Swerve/Pigeon YPR", mPigeon.getYPR());
            SmartDashboard.putString("Swerve/Heading Controller", headingController.getState().toString());
            SmartDashboard.putNumber("Swerve/Target Heading", headingController.getTargetHeading());
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
            SmartDashboard.putBoolean("Swerve/Vision Updates Allowed", visionUpdatesAllowed);

            SmartDashboard.putNumberArray("Swerve/Robot Pose", new double[]{pose.getTranslation().x(), pose.getTranslation().y(), pose.getRotation().getUnboundedDegrees()});
            SmartDashboard.putNumber("Swerve/Robot X", pose.getTranslation().x());
            SmartDashboard.putNumber("Swerve/Robot Y", pose.getTranslation().y());
            SmartDashboard.putNumber("Swerve/Robot Heading", pose.getRotation().getUnboundedDegrees());
            SmartDashboard.putNumber("Swerve/Robot Velocity", currentVelocity);
        }

        if(!hasFinishedPath() && hasStartedFollowing){
            double currentTime = Timer.getFPGATimestamp();
            SmartDashboard.putNumber("Autopath Timer", currentTime - trajectoryStartTime);
        }
    }

    public static class PeriodicIO {
        // LOGGING
        public int schedDeltaDesired;
        public double schedDeltaActual;
        public double schedDuration;
        private double lastSchedStart;

        // Inputs
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
