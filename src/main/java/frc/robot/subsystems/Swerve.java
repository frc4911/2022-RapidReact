package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Ports;
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
import libraries.cyberlib.utils.RobotName;
import libraries.madtownlib.util.SwerveHeadingController;
import libraries.madtownlib.util.SwerveInverseKinematics;
import libraries.madtownlib.util.Utils;
import libraries.madtownlib.util.VisionCriteria;
import libraries.madtownlib.vectors.VectorField;

public class Swerve extends Subsystem {

    // Module declaration
    public SwerveDriveModule frontRight, frontLeft, rearLeft, rearRight;
    List<SwerveDriveModule> modules;
    List<SwerveDriveModule> positionModules;

    public enum ControlState{
        NEUTRAL, MANUAL, POSITION, ROTATION, DISABLED, VECTORIZED,
        TRAJECTORY, VELOCITY, VISION, VISION_AIM, CELL_AIM
    }

    private ControlState currentState = ControlState.NEUTRAL;

    // Evade maneuver variables
    Translation2d clockwiseCenter = new Translation2d();
    Translation2d counterClockwiseCenter = new Translation2d();
    boolean evading = false;
    boolean evadingToggled = false;

    // Heading controller methods
    Pigeon pigeon;
    SwerveHeadingController headingController = new SwerveHeadingController(Constants.kIsUsingTractionWheels);

    // Vision dependencies
    RobotState robotState;
    Rotation2d visionTargetHeading = new Rotation2d();
    boolean visionUpdatesAllowed = true;
    double visionCurveDistance = Constants.kDefaultCurveDistance;
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
    double lastUpdateTimestamp = 0;

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

    // Experimental
    VectorField vf;

    // Teleop driving variables
    private Translation2d translationalVector = new Translation2d();
    private double rotationalInput = 0;
    private Translation2d lastDriveVector = new Translation2d();
    private final Translation2d rotationalVector = Translation2d.identity();
    private double lowPowerScalar = 0.6;
    private double maxSpeedFactor = 1.0;
    private boolean robotCentric = false;

    // Swerve kinematics (exists in a separate class)
    private SwerveInverseKinematics inverseKinematics = new SwerveInverseKinematics(Constants.kModulePositions);

    // Possible new control method for rotation
    public Rotation2d averagedDirection = Rotation2d.identity();
    public void resetAveragedDirection(){ averagedDirection = pose.getRotation(); }
    public void setAveragedDirection(double degrees){ averagedDirection = Rotation2d.fromDegrees(degrees); }
    public final double rotationDirectionThreshold = Math.toRadians(5.0);
    public final double rotationDivision = 1.0;

    // Aiming PID
    SynchronousPIDF aimingPIDF = new SynchronousPIDF(0.0, 0.0, 0.0);// .4,0,10 //0.75, 0, 5 deadeye brian
    private int aimingParametersCount;
    private int totalAimingCount;

    //Cell PID
    SynchronousPIDF cellPIDF = new SynchronousPIDF(0.30, 0.0, 0.0);

    // PeriodicIO
    PeriodicIO mPeriodicIO = new PeriodicIO();
    private int mDefaultSchedDelta = 20;
    @SuppressWarnings("unused")
    private int mListIndex;

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

        if (RobotName.name.equals(Constants.kRobot1Name)){
            m0 = Constants.kFrontRightCancoderStartingPosDegreesR1;
            m1 = Constants.kFrontLeftCancoderStartingPosDegreesR1;
            m2 = Constants.kRearLeftCancoderStartingPosDegreesR1;
            m3 = Constants.kRearRightCancoderStartingPosDegreesR1;
            // int angOffset = 30;
            // m0 = 214 - angOffset;
            // m1 = 104 - angOffset;
            // m2 = 102 - angOffset;
            // m3 = 259 - angOffset;
        }
        else if (RobotName.name.equals(Constants.kRobot2Name)){
            m0 = Constants.kFrontRightCancoderStartingPosDegreesR2;
            m1 = Constants.kFrontLeftCancoderStartingPosDegreesR2;
            m2 = Constants.kRearLeftCancoderStartingPosDegreesR2;
            m3 = Constants.kRearRightCancoderStartingPosDegreesR2;
        }
        else if (RobotName.name.equals(Constants.kCetusName)){
            m0 = Constants.kFrontRightCancoderStartingPosDegreesCetus;
            m1 = Constants.kFrontLeftCancoderStartingPosDegreesCetus;
            m2 = Constants.kRearLeftCancoderStartingPosDegreesCetus;
            m3 = Constants.kRearRightCancoderStartingPosDegreesCetus;
        }
        frontRight = new SwerveDriveModule(Ports.FRONT_RIGHT_ROTATION, Ports.FRONT_RIGHT_DRIVE,
                Ports.FRONT_RIGHT_CANCODER, 0, m0, Constants.kVehicleToModuleZero);
        frontLeft = new SwerveDriveModule(Ports.FRONT_LEFT_ROTATION, Ports.FRONT_LEFT_DRIVE,
                Ports.FRONT_LEFT_CANCODER, 1, m1, Constants.kVehicleToModuleOne);
        rearLeft = new SwerveDriveModule(Ports.REAR_LEFT_ROTATION, Ports.REAR_LEFT_DRIVE,
                Ports.REAR_LEFT_CANCODER, 2, m2, Constants.kVehicleToModuleTwo);
        rearRight = new SwerveDriveModule(Ports.REAR_RIGHT_ROTATION, Ports.REAR_RIGHT_DRIVE,
                Ports.REAR_RIGHT_CANCODER, 3, m3, Constants.kVehicleToModuleThree);

        modules = Arrays.asList(frontRight, frontLeft, rearLeft, rearRight);
        positionModules = Arrays.asList(frontRight, frontLeft, rearLeft, rearRight);

        //rearLeft.disableDriveEncoder();

        rearLeft.invertDriveMotor(false);
        frontLeft.invertDriveMotor(false);

        modules.forEach((m) -> m.reverseRotationSensor(true));

        pigeon = Pigeon.getInstance();

        pose = new Pose2d();
        distanceTraveled = 0;

//        motionPlanner = new DriveMotionPlanner();

        robotState = RobotState.getInstance(sClassName);

//        generator = TrajectoryGenerator.getInstance();
    }

    private final Loop loop = new Loop() {

        @Override
        public void onStart(Phase phase) {
            synchronized(Swerve.this){
                translationalVector = new Translation2d();
                lastDriveVector = rotationalVector;
                rotationalInput = 0;
                resetAveragedDirection();
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
                if(modulesReady || (getState() != ControlState.TRAJECTORY)) {
                    alternatePoseUpdate();
                }

                updateControlCycle(timestamp);
                lastUpdateTimestamp = timestamp;
            }
        }

        @Override
        public void onStop(double timestamp) {
            synchronized(Swerve.this){
                translationalVector = new Translation2d();
                rotationalInput = 0;
                stop();
            }
        }

    };

    /** Playing around with different methods of odometry. This will require the use of all four modules, however. */
    public synchronized void alternatePoseUpdate() {
        double x = 0.0;
        double y = 0.0;
        Rotation2d heading = pigeon.getYaw();

        double[][] distances = new double[4][2];
        for(SwerveDriveModule m : modules){
            m.updatePose(heading);
            double distance = m.getEstimatedRobotPose().getTranslation().distance(pose.getTranslation());
            distances[m.moduleID][0] = m.moduleID;
            distances[m.moduleID][1] = distance;
        }

        Arrays.sort(distances, new java.util.Comparator<double[]>() {
            public int compare(double[] a, double[] b) {
                return Double.compare(a[1], b[1]);
            }
        });
        List<SwerveDriveModule> modulesToUse = new ArrayList<>();
        double firstDifference = distances[1][1] - distances[0][1];
        double secondDifference = distances[2][1] - distances[1][1];
        double thirdDifference = distances[3][1] - distances[2][1];
        if(secondDifference > (1.5 * firstDifference)){
            modulesToUse.add(modules.get((int)distances[0][0]));
            modulesToUse.add(modules.get((int)distances[1][0]));
        }else if(thirdDifference > (1.5 * firstDifference)){
            modulesToUse.add(modules.get((int)distances[0][0]));
            modulesToUse.add(modules.get((int)distances[1][0]));
            modulesToUse.add(modules.get((int)distances[2][0]));
        }else{
            modulesToUse.add(modules.get((int)distances[0][0]));
            modulesToUse.add(modules.get((int)distances[1][0]));
            modulesToUse.add(modules.get((int)distances[2][0]));
            modulesToUse.add(modules.get((int)distances[3][0]));
        }

        SmartDashboard.putNumber("Modules Used", modulesToUse.size());

        for(SwerveDriveModule m : modulesToUse){
            x += m.getEstimatedRobotPose().getTranslation().x();
            y += m.getEstimatedRobotPose().getTranslation().y();
        }

        Pose2d updatedPose = new Pose2d(new Translation2d(x / modulesToUse.size(), y / modulesToUse.size()), heading);
        double deltaPos = updatedPose.getTranslation().distance(pose.getTranslation());
        distanceTraveled += deltaPos;
        pose = updatedPose;
        modules.forEach((m) -> m.resetPose(pose));
    }

    /** Called every cycle to update the swerve based on its control state */
    public synchronized void updateControlCycle(double timestamp){
        double rotationCorrection = headingController.updateRotationCorrection(pose.getRotation().getUnboundedDegrees(), timestamp);
        if (currentState != ControlState.VISION_AIM && totalAimingCount != 0) {
            System.out.println("aimingParametersCount: " + aimingParametersCount);
            System.out.println("totalAimingCount: " + totalAimingCount);
            aimingParametersCount = 0;
            totalAimingCount = 0;
        }

        switch(currentState){
            case MANUAL:
                if(evading && evadingToggled){
                    determineEvasionWheels();
                    double sign = Math.signum(rotationalInput);
                    if(sign == 1.0){
                        inverseKinematics.setCenterOfRotation(clockwiseCenter);
                    }else if(sign == -1.0){
                        inverseKinematics.setCenterOfRotation(counterClockwiseCenter);
                    }
                    evadingToggled = false;
                }else if(evading){
                    double sign = Math.signum(rotationalInput);
                    if(sign == 1.0){
                        inverseKinematics.setCenterOfRotation(clockwiseCenter);
                    }else if(sign == -1.0){
                        inverseKinematics.setCenterOfRotation(counterClockwiseCenter);
                    }
                }else if(evadingToggled){
                    inverseKinematics.setCenterOfRotation(Translation2d.identity());
                    evadingToggled = false;
                }
                if(translationalVector.equals(Translation2d.identity()) && rotationalInput == 0.0){
                    if(lastDriveVector.equals(rotationalVector)){
                        stop();
                    }else{
                        setDriveOutput(inverseKinematics.updateDriveVectors(lastDriveVector,
                                rotationCorrection, pose, robotCentric), 0.0);
                    }
                }else{
                    setDriveOutput(inverseKinematics.updateDriveVectors(translationalVector,
                            rotationalInput + rotationCorrection, pose, robotCentric));
                }
                break;
            case POSITION:
                if(positionOnTarget())
                    rotate(headingController.getTargetHeading());
                break;

            case ROTATION:

                setDriveOutput(inverseKinematics.updateDriveVectors(new Translation2d(), Util.deadBand(rotationCorrection, 0.1), pose, false));
                break;

            case VECTORIZED:
                Translation2d outputVectorV = vf.getVector(pose.getTranslation()).scale(0.25);
                SmartDashboard.putNumber("Vector Direction", outputVectorV.direction().getDegrees());
                SmartDashboard.putNumber("Vector Magnitude", outputVectorV.norm());
//			System.out.println(outputVector.x()+" "+outputVector.y());
                setDriveOutput(inverseKinematics.updateDriveVectors(outputVectorV, rotationCorrection, getPose(), false));
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
            case VISION:
                break;
            case VELOCITY:
                break;
            case NEUTRAL:
                stop();
                break;
            case DISABLED:
                break;
            case VISION_AIM:
                // aimingParameters = robotState.getOuterGoalParameters();
                // totalAimingCount++;
                // mIsOnTarget = false;
                // if (aimingParameters.isPresent()) {
                // 	//error = -1 * aimingParameters.get().getRobotToGoal().getTranslation().y();
                // 	//radians
                // 	error = Math.atan2(-aimingParameters.get().getRobotToGoal().getTranslation().y(), aimingParameters.get().getRobotToGoal().getTranslation().x());
                // 	if (Math.abs(error) <= Math.toRadians(2.0)){
                // 		if (++count>3){
                // 			mIsOnTarget = true; //0.5
                // 		}
                // 	}
                // 	else{
                // 		count = 0;
                // 	}
                // 	System.out.println(count+" "+Math.toDegrees(error));
                // 	aimingParametersCount++;
                // }
                // else{
                // 	System.out.println("error not present******************************************");
                // }
                // SmartDashboard.putNumber("LL error", Math.toDegrees(error));
                setRotateOutput(Math.toDegrees(error));//aimingPIDF.calculate(error, timestamp)); brian
                // lastAimTimestamp = timestamp;
                break;
            //raynli
            case CELL_AIM:
                aimingParameters = robotState.getPowerCell();
                boolean mIsOnCellTarget = false;
                System.out.println(aimingParameters.isPresent());
                if (aimingParameters.isPresent()) {
                    error = Math.atan2(-aimingParameters.get().getRobotToGoal().getTranslation().y(), aimingParameters.get().getRobotToGoal().getTranslation().x());
                    mIsOnCellTarget = Math.abs(error) <= Math.toRadians(2.0); //0.5
                }
                SmartDashboard.putNumber("LL error", error);
                SmartDashboard.putBoolean("OnTarget", mIsOnCellTarget);
                setStrafeOutput(cellPIDF.calculate(error, timestamp));
                break;
            default:
                break;
        }
    }

    @Override
    public synchronized void stop() {
        setState(ControlState.NEUTRAL);
        modules.forEach((m) -> m.stop());
    }

    @Override
    public synchronized void zeroSensors() {
        zeroSensors(Constants.kRobotStartingPose);
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        mListIndex = enabledLooper.register(loop);
    }

    public void toggleEvade(){
        evading = !evading;
        evadingToggled = true;
    }

    public void temporarilyDisableHeadingController(){
        headingController.temporarilyDisable();
    }

    public double getTargetHeading(){
        return headingController.getTargetHeading();
    }

    public void resetVisionUpdates(){
        visionUpdatesAllowed = true;
        visionUpdateCount = 0;
        attemptedVisionUpdates = 0;
        visionVisibleCycles = 0;
        firstVisionCyclePassed = false;
        visionCriteria.reset();
    }

    public boolean isTracking(){
        return (currentState == ControlState.VISION);
    }

    public boolean needsToNotifyDrivers() {
        if (needsToNotifyDrivers) {
            needsToNotifyDrivers = false;
            return true;
        }

        return false;
    }

    public Pose2d getPose(){
        return pose;
    }

    public void requireModuleConfiguration(){
        modulesReady = false;
    }

    public void alwaysConfigureModules(){
        alwaysConfigureModules = true;
    }

    public void setStartingPose(Pose2d newPose){
        startingPose = newPose;
    }

    public double getRemainingProgress(){
//        if (motionPlanner != null && getState() == ControlState.TRAJECTORY) {
//            return motionPlanner.getRemainingProgress();
//        }

        return 0.0;
    }

    public Translation2d getLastTrajectoryVector() {
        return lastTrajectoryVector;
    }

    public boolean hasFinishedPath() {
        return hasFinishedPath;
    }

    //Assigns appropriate directions for scrub factors
    public void setCarpetDirection(boolean standardDirection){
        modules.forEach((m) -> m.setCarpetDirection(standardDirection));
    }

    public void setLowPowerScalar(double scalar){
        lowPowerScalar = scalar;
    }

    public void setMaxSpeed(double max){
        maxSpeedFactor = max;
    }

    public void setCenterOfRotation(Translation2d center){
        inverseKinematics.setCenterOfRotation(center);
    }

    public ControlState getState(){
        return currentState;
    }

    public void setState(ControlState newState){
        if (currentState != newState) {
            System.out.println(currentState + " to " + newState);
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

        currentState = newState;
    }

    /**
     * Main function used to send manual input during teleop.
     * @param x forward/backward input
     * @param y left/right input
     * @param rotate rotational input
     * @param robotCentric gyro use
     * @param lowPower scaled down output
     */
    public void sendInput(double x, double y, double rotate, boolean robotCentric, boolean lowPower){
        rotate*=.3; // brian
        Translation2d translationalInput = new Translation2d(x, y);
        double inputMagnitude = translationalInput.norm();
		/* Snap the translational input to its nearest pole, if it is within a certain threshold
		  of it. */
        double threshold = Math.toRadians(10.0);
        if(Math.abs(translationalInput.direction().distance(translationalInput.direction().nearestPole())) < threshold){
            translationalInput = translationalInput.direction().nearestPole().toTranslation().scale(inputMagnitude);
        }

        double deadband = 0.0;
        if(inputMagnitude < deadband){
            translationalInput = new Translation2d();
            inputMagnitude = 0;
        }

		/* Scale x and y by applying a power to the magnitude of the vector they create, in order
		 to make the controls less sensitive at the lower end. */
        final double power = (lowPower) ? 1.75 : 1.5;
        Rotation2d direction = translationalInput.direction();
        double scaledMagnitude = Math.pow(inputMagnitude, power);
        translationalInput = new Translation2d(direction.cos() * scaledMagnitude,
                direction.sin() * scaledMagnitude);

        rotate = (Math.abs(rotate) < deadband) ? 0 : rotate;
        // rotate = Math.pow(Math.abs(rotate), 1.75)*Math.signum(rotate);
        rotate = Math.pow(Math.abs(rotate), 1.0)*Math.signum(rotate); // brian

        translationalInput = translationalInput.scale(maxSpeedFactor);
        rotate *= maxSpeedFactor;

        translationalVector = translationalInput;

        if(lowPower){
            translationalVector = translationalVector.scale(lowPowerScalar);
            rotate *= lowPowerScalar;
        }else{
            rotate *= 0.8;
        }

        if(rotate != 0 && rotationalInput == 0){
            headingController.disable();
        }else if(rotate == 0 && rotationalInput != 0){
            headingController.temporarilyDisable();
        }

        rotationalInput = rotate;

        if(translationalInput.norm() != 0){
            if(currentState == ControlState.VISION){
                if(Math.abs(translationalInput.direction().distance(visionTargetHeading)) > Math.toRadians(150.0)){
                    setState(ControlState.MANUAL);
                }
            }else if(currentState != ControlState.MANUAL){
                setState(ControlState.MANUAL);
            }
        }else if(rotationalInput != 0){
            if(currentState != ControlState.MANUAL && currentState != ControlState.VISION && currentState != ControlState.TRAJECTORY){
                setState(ControlState.MANUAL);
            }
        }

        if(inputMagnitude > 0.3)
            lastDriveVector = new Translation2d(x, y);
        else if(translationalVector.x() == 0.0 && translationalVector.y() == 0.0 && rotate != 0.0){
            lastDriveVector = rotationalVector;
        }

        this.robotCentric = robotCentric;
    }

    public synchronized void updateControllerDirection(Translation2d input){
        if(Util.epsilonEquals(input.norm(), 1.0, 0.1)){
            Rotation2d direction = input.direction();
            double roundedDirection = Math.round(direction.getDegrees() / rotationDivision) * rotationDivision;
            averagedDirection = Rotation2d.fromDegrees(roundedDirection);
        }
    }

    //Various methods to control the heading controller
    public synchronized void rotate(double goalHeading){
        if(translationalVector.x() == 0 && translationalVector.y() == 0)
            rotateInPlace(goalHeading);
        else
            headingController.setStabilizationTarget(
                    Utils.placeInAppropriate0To360Scope(pose.getRotation().getUnboundedDegrees(), goalHeading));
    }

    public void rotateInPlace(double goalHeading){
        setState(ControlState.ROTATION);
        headingController.setStationaryTarget(
                Utils.placeInAppropriate0To360Scope(pose.getRotation().getUnboundedDegrees(), goalHeading));
    }

    // Ramiro did this
    public void limeLightAim() {
        setState(ControlState.VISION_AIM);
        // double p = SmartDashboard.getNumber("aim pid P", -1);
        // if (p == -1){
        // 	SmartDashboard.putNumber("aim pid P",0.5);
        // 	p = 0.5;
        // }
        // double d = SmartDashboard.getNumber("aim pid D", -1);
        // if (d == -1){
        // 	SmartDashboard.putNumber("aim pid D",8.0);
        // 	d = 8.0;
        // }
        // aimingPIDF = new SynchronousPIDF(p,0,d);
        // aimingPIDF.reset();
        // lastAimTimestamp = Timer.getFPGATimestamp();
    }

    public void cellAim() {
        setState(ControlState.CELL_AIM);
        cellPIDF.reset();
    }

    public synchronized boolean isOnTarget() {
        return mIsOnTarget;
    }

    public void rotateInPlaceAbsolutely(double absoluteHeading){
        setState(ControlState.ROTATION);
        headingController.setStationaryTarget(absoluteHeading);
    }

    public void setPathHeading(double goalHeading){
        headingController.setSnapTarget(
                Utils.placeInAppropriate0To360Scope(
                        pose.getRotation().getUnboundedDegrees(), goalHeading));
    }

    public void setAbsolutePathHeading(double absoluteHeading){
        headingController.setSnapTarget(absoluteHeading);
    }

    /** Sets MotionMagic targets for the drive motors */
    public void setPositionTarget(double directionDegrees, double magnitudeInches){
        setState(ControlState.POSITION);
        modules.forEach((m) -> m.setModuleAngle(directionDegrees));
        modules.forEach((m) -> m.setDrivePositionTarget(magnitudeInches));
    }

    /** Locks drive motors in place with MotionMagic */
    public void lockDrivePosition(){
        modules.forEach((m) -> m.setDrivePositionTarget(0.0));
    }

    /** Puts drive motors into closed-loop velocity mode */
    public void setVelocity(Rotation2d direction, double velocityInchesPerSecond){
        setState(ControlState.VELOCITY);
        modules.forEach((m) -> m.setModuleAngle(direction.getDegrees()));
        modules.forEach((m) -> m.setVelocitySetpoint(velocityInchesPerSecond));
    }

    /** Configures each module to match its assigned vector */
    public void setDriveOutput(List<Translation2d> driveVectors){
        for(int i=0; i<modules.size(); i++){
            if(Utils.shouldReverse(driveVectors.get(i).direction().getDegrees(), modules.get(i).getModuleAngle().getDegrees())){
                modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
                modules.get(i).setDriveOpenLoop(-driveVectors.get(i).norm());
            }else{
                modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
                modules.get(i).setDriveOpenLoop(driveVectors.get(i).norm());
            }
        }
    }

    int count = 0;

    // Ramiro was here
    public void setRotateOutput(double rotationOutput) {
        // System.out.println(sClassName+".setRotateOutput("+rotationOutput+")");
        if (mIsOnTarget){
            setState(ControlState.NEUTRAL);
            // if (Math.abs(rotationOutput) < 0.5) { //.01 brian
            // 	List<Translation2d> driveVectors = inverseKinematics.updateDriveVectors(new Translation2d(), 1,  pose, false);
            // 	for(int i=0; i<modules.size(); i++){
            // 		if(Utils.shouldReverse(driveVectors.get(i).direction().getDegrees(), modules.get(i).getModuleAngle().getDegrees())){
            // 			modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
            // 			modules.get(i).setDriveOpenLoop(0);
            // 		}else{
            // 			modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
            // 			modules.get(i).setDriveOpenLoop(0);
            // 		}
            // 	}

        } else {
            // add this line (brian)
            rotationOutput = Math.abs(error)*.5;
            rotationOutput = Math.min(rotationOutput,.17)*Math.signum(error)*-1;
            // rotationOutput = error>0 ? -.06:.06;
            setDriveOutput(inverseKinematics.updateDriveVectors(new Translation2d(), rotationOutput, pose, false));
        }
        // SmartDashboard.putNumber("LL rotate output",rotationOutput);
    }

    public void setStrafeOutput(double strafeOutput) {
        setDriveOutput(inverseKinematics.updateDriveVectors(new Translation2d(0, -strafeOutput), 0, pose, false));
    }

    public void setDriveOutput(List<Translation2d> driveVectors, double percentOutputOverride){
        for(int i=0; i<modules.size(); i++){
            if(Utils.shouldReverse(driveVectors.get(i).direction().getDegrees(), modules.get(i).getModuleAngle().getDegrees())){
                modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
                modules.get(i).setDriveOpenLoop(-percentOutputOverride);
            }else{
                modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
                modules.get(i).setDriveOpenLoop(percentOutputOverride);
            }
        }
    }

    /** Configures each module to match its assigned vector, but puts the drive motors into closed-loop velocity mode */
    public void setVelocityDriveOutput(List<Translation2d> driveVectors){
        for(int i=0; i<modules.size(); i++){
            if(Utils.shouldReverse(driveVectors.get(i).direction().getDegrees(), modules.get(i).getModuleAngle().getDegrees())){
                modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
                modules.get(i).setVelocitySetpoint(-driveVectors.get(i).norm() * Constants.kSwerveMaxSpeedInchesPerSecond);
            }else{
                modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
                modules.get(i).setVelocitySetpoint(driveVectors.get(i).norm() * Constants.kSwerveMaxSpeedInchesPerSecond);
            }
        }
    }

    public void setVelocityDriveOutput(List<Translation2d> driveVectors, double velocityOverride){
        for(int i=0; i<modules.size(); i++){
            if(Utils.shouldReverse(driveVectors.get(i).direction().getDegrees(), modules.get(i).getModuleAngle().getDegrees())){
                modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
                modules.get(i).setVelocitySetpoint(-velocityOverride);
            }else{
                modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
                modules.get(i).setVelocitySetpoint(velocityOverride);
            }
        }
    }

    /** Sets only module angles to match their assigned vectors */
    public void setModuleAngles(List<Translation2d> driveVectors){
        for(int i=0; i<modules.size(); i++){
            if(Utils.shouldReverse(driveVectors.get(i).direction().getDegrees(), modules.get(i).getModuleAngle().getDegrees())){
                modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
            }else{
                modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
            }
        }
    }

    /** Increases each module's rotational power cap for the beginning of auto */
    public void set10VoltRotationMode(boolean tenVolts){
        modules.forEach((m) -> m.set10VoltRotationMode(tenVolts));
    }

    /**
     * @return Whether or not at least one module has reached its MotionMagic setpoint
     */
    public boolean positionOnTarget(){
        boolean onTarget = false;
        for(SwerveDriveModule m : modules){
            onTarget |= m.drivePositionOnTarget();
        }
        return onTarget;
    }

    /**
     * @return Whether or not all modules have reached their angle setpoints
     */
    public boolean moduleAnglesOnTarget(){
        boolean onTarget = true;
        for(SwerveDriveModule m : modules){
            onTarget &= m.angleOnTarget();
        }
        return onTarget;
    }

    /**
     * Sets a trajectory for the robot to follow
     * @param trajectory
     * @param targetHeading Heading that the robot will rotate to during its path following
     * @param rotationScalar Scalar to increase or decrease the robot's rotation speed
     * @param followingCenter The point (relative to the robot) that will follow the trajectory
     */
    public synchronized void setTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, double targetHeading,
                                           double rotationScalar, Translation2d followingCenter) {
        hasStartedFollowing = false;
        hasFinishedPath = false;
        moduleConfigRequested = false;
//        motionPlanner.reset();
//        motionPlanner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(trajectory)));
//        motionPlanner.setFollowingCenter(followingCenter);
        inverseKinematics.setCenterOfRotation(followingCenter);
        setAbsolutePathHeading(targetHeading);
        this.rotationScalar = rotationScalar;
        trajectoryStartTime = Timer.getFPGATimestamp();
        setState(ControlState.TRAJECTORY);
    }

    public synchronized void setTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, double targetHeading,
                                           double rotationScalar){
        setTrajectory(trajectory, targetHeading, rotationScalar, Translation2d.identity());
    }

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

    /****************************************************/
    /* Vector Fields */
    public synchronized void setVectorField(VectorField vf_) {
        vf = vf_;
        setState(ControlState.VECTORIZED);
    }

    /** Determines which wheels the robot should rotate about in order to perform an evasive maneuver */
    public synchronized void determineEvasionWheels(){
        Translation2d here = lastDriveVector.rotateBy(pose.getRotation().inverse());
        List<Translation2d> wheels = Constants.kModulePositions;
        clockwiseCenter = wheels.get(0);
        counterClockwiseCenter = wheels.get(wheels.size()-1);
        for(int i = 0; i < wheels.size()-1; i++) {
            Translation2d cw = wheels.get(i);
            Translation2d ccw = wheels.get(i+1);
            if(here.isWithinAngle(cw,ccw)) {
                clockwiseCenter = ccw;
                counterClockwiseCenter = cw;
            }
        }
    }

    public void setNominalDriveOutput(double voltage){
        modules.forEach((m) -> m.setNominalDriveOutput(voltage));
    }

    /** Sets the maximum rotation speed opf the modules, based on the robot's velocity */
    public void setMaxRotationSpeed(){
        double currentDriveSpeed = translationalVector.norm() * Constants.kSwerveMaxSpeedInchesPerSecond;
        double newMaxRotationSpeed = Constants.kSwerveRotationMaxSpeed /
                ((Constants.kSwerveRotationSpeedScalar * currentDriveSpeed) + 1.0);
        modules.forEach((m) -> m.setMaxRotationSpeed(newMaxRotationSpeed));
    }

    /** Puts all rotation and drive motors into open-loop mode */
    public synchronized void disable(){
        modules.forEach((m) -> m.disable());
        setState(ControlState.DISABLED);
    }

    /** Zeroes the drive motors, and sets the robot's internal position and heading to match that of the fed pose */
    public synchronized void zeroSensors(Pose2d startingPose){
        pigeon.setAngle(startingPose.getRotation().getUnboundedDegrees());
        modules.forEach((m) -> m.zeroSensors(startingPose));
        pose = startingPose;
        distanceTraveled = 0;
    }

    public synchronized void resetPosition(Pose2d newPose){
        pose = new Pose2d(newPose.getTranslation(), pose.getRotation());
        modules.forEach((m) -> m.zeroSensors(pose));
        distanceTraveled = 0;
    }

    public synchronized void setXCoordinate(double x){
        pose.getTranslation().setX(x);
        modules.forEach((m) -> m.zeroSensors(pose));
        System.out.println("X coordinate reset to: " + pose.getTranslation().x());
    }

    public synchronized void setYCoordinate(double y){
        pose.getTranslation().setY(y);
        modules.forEach((m) -> m.zeroSensors(pose));
        System.out.println("Y coordinate reset to: " + pose.getTranslation().y());
    }

    @Override
    public String getLogHeaders() {
        StringBuilder allHeaders = new StringBuilder(256);
        for (SwerveDriveModule m: modules){
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
        for (SwerveDriveModule m: modules){
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

        modules.forEach((m) -> m.readPeriodicInputs());
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        modules.forEach((m) -> m.writePeriodicOutputs());
    }

    @Override
    public int whenRunAgain () {
        return mPeriodicIO.schedDeltaDesired;
    }

    @Override
    public void outputTelemetry() {
        modules.forEach((m) -> m.outputTelemetry());
        SmartDashboard.putString("Swerve State", currentState.toString());
        SmartDashboard.putBoolean("isOnTarget", isOnTarget());
        if(Constants.kDebuggingOutput){
            SmartDashboard.putNumberArray("Robot Pose", new double[]{pose.getTranslation().x(), pose.getTranslation().y(), pose.getRotation().getUnboundedDegrees()});
            SmartDashboard.putString("Swerve State", currentState.toString());
            SmartDashboard.putNumber("Robot Heading", pose.getRotation().getUnboundedDegrees()); // Alex
            SmartDashboard.putNumber("Robot X", pose.getTranslation().x());
            SmartDashboard.putNumber("Robot Y", pose.getTranslation().y());
            SmartDashboard.putNumber("Robot Heading", pose.getRotation().getUnboundedDegrees());
            SmartDashboard.putString("Heading Controller", headingController.getState().toString());
            SmartDashboard.putNumber("Target Heading", headingController.getTargetHeading());
            SmartDashboard.putNumber("Distance Traveled", distanceTraveled);
            SmartDashboard.putNumber("Robot Velocity", currentVelocity);
            SmartDashboard.putString("Swerve State", currentState.toString());
            SmartDashboard.putBoolean("Vision Updates Allowed", visionUpdatesAllowed);
            SmartDashboard.putNumberArray("Pigeon YPR", pigeon.getYPR());

        }
        if(!hasFinishedPath() && hasStartedFollowing){
            double currentTime = Timer.getFPGATimestamp();
            SmartDashboard.putNumber("Autopath Timer", currentTime - trajectoryStartTime);
        }
    }

    public static class PeriodicIO {
        // LOGGING
        public  int    schedDeltaDesired;
        public  double schedDeltaActual;
        public  double schedDuration;
        private double lastSchedStart;

    }
}
