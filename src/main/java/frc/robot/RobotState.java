package frc.robot;

import frc.robot.Constants.Target;
import frc.robot.subsystems.LimeLights.LimeLight.LimelightConstants;
import frc.robot.subsystems.Swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.util.InterpolatingDouble;
import libraries.cheesylib.util.InterpolatingTreeMap;
import libraries.cheesylib.vision.AimingParameters;
import libraries.cheesylib.vision.GoalTracker;
import libraries.cheesylib.vision.TargetInfo;

public class RobotState {

    private static String sClassName;
    private static int sInstanceCount;
    private static RobotState sInstance = null;

    public  static RobotState getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new RobotState(caller);
            mSwerve = Swerve.getInstance(sClassName);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+"getInstance " + sClassName + " " + ++sInstanceCount);
    }

    // private static RobotState instance = new RobotState();
    // public static RobotState getInstance(){
    // 	return instance;
    // }
    // private static String sClassName;
    private static Swerve mSwerve;
    private static final int kObservationBufferSize = 100;

    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
    private GoalTracker mGoalTracker;
    private GoalTracker mPowerCellTracker;

    private List<Translation2d> mCameraToTarget = new ArrayList<>();

    private double targetHeight = Constants.kOuterTargetHeight;//Constants.kDiskTargetHeight; // ramiro changed
    private double distance_driven_;

    public final int minimumTargetQuantity = 1; // ramiro changed (we only have one target)
    private final int primaryTargetIndex = 0; // ramiro changed
    private Translation2d lastKnownTargetPosition = new Translation2d();
    public Translation2d lastKnownTargetPosition(){ return lastKnownTargetPosition; }

    public double distanceToTarget(){
        return getLatestFieldToVehicle().getValue().getTranslation().distance(lastKnownTargetPosition);
    }

//    Alliance alliance = SmartDashboardInteractions.STANDARD_CARPET_SIDE;
//    public void setAlliance(Alliance a){
//        alliance = a;
//    }
//    public Alliance getAlliance(){
//        return alliance;
//    }
//    public boolean onStandardCarpet(){
//        return alliance == SmartDashboardInteractions.STANDARD_CARPET_SIDE;
//    }

    private boolean seesTarget = false;
    public boolean seesTarget(){
        return seesTarget;
    }

    private double angleToCube = 0;

    public double getAngleToCube(){
        return angleToCube;
    }
    public void setAngleToCube(double angle){
        angleToCube = angle;
    }


    //ramiro deems thee unclean

    // public Translation2d getCubePosition(){
    // 	List<GoalTracker.TrackReport> reports = goal_tracker_.getTracks();
    // 	if (!reports.isEmpty())
    // 		return goal_tracker_.getTracks().get(0).field_to_target.getTranslation();
    // 	else
    // 		return new Translation2d();
    // }


    private RobotState(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);

        reset(0, new Pose2d());
        // mSwerve = Swerve.getInstance(sClassName);
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        mGoalTracker = new GoalTracker();
        mPowerCellTracker = new GoalTracker();
        distance_driven_ = 0.0;
    }

    public synchronized void resetDistanceDriven() {
        distance_driven_ = 0.0;
    }


    public double getVisionTargetHeight(){
        return targetHeight;
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }


    public void setXTarget(double x, double error){
        mGoalTracker.setXTarget(x, error);
    }

    public void enableXTarget(boolean enable){
        mGoalTracker.enableXTarget(enable);
    }

    public synchronized void resetVision(String limelightName) {
        if (limelightName.equals(Constants.kShootwardsLimelightConstants.kName)) {
            mGoalTracker.reset();
        } else {
            mPowerCellTracker.reset();
        }
    }

    public synchronized void resetVision() {
        mGoalTracker.reset();
        mPowerCellTracker.reset();
    }

    public void addVisionUpdate(double timestamp, List<TargetInfo> observations, LimelightConstants source) {
        mCameraToTarget.clear();
        for (int i = 0; i < observations.size(); i++) { //TargetInfo target : observations
            mCameraToTarget.add(getCameraToVisionTargetPose(observations.get(i), source, source.kTargets[i]));
        }
        updateTracker(timestamp, mCameraToTarget, source);
    }

    private void updateTracker(double timestamp, List<Translation2d> cameraToTargets, LimelightConstants source) {

        if (targetsAreViable(cameraToTargets, source.kExpectedTargetCount)) {
            if (source.kName.equals(Constants.kShootwardsLimelightConstants.kName)) {
                mGoalTracker.update(timestamp, shooterTargetsToTrackerUpdate(timestamp, cameraToTargets, source));
            } else {
                mPowerCellTracker.update(timestamp, collectorTargetsToTrackerUpdate(timestamp, cameraToTargets, source));
            }
        } else {
            if (source.kName.equals(Constants.kShootwardsLimelightConstants.kName)) {
                mGoalTracker.update(timestamp, new ArrayList<>());
            } else {
                mPowerCellTracker.update(timestamp, new ArrayList<>());
            }
        }
    }

    private List<Pose2d> collectorTargetsToTrackerUpdate(double timestamp, List<Translation2d> cameraToTargets, LimelightConstants source) {
        List<Pose2d> cameraToPowerCell = new ArrayList<>();
        for (int i = 0; i < cameraToTargets.size(); i++) {
            cameraToPowerCell.add((getFieldToVisionTarget(timestamp, cameraToTargets.get(i), source)));
        }
        return cameraToPowerCell;
    }

    // uses the corner targets to calculate the outer and inner goal targets to give
    // to goal tracker
    private List<Pose2d> shooterTargetsToTrackerUpdate(double timestamp, List<Translation2d> cameraToTargets, LimelightConstants source) {
        List<Pose2d> targets = new ArrayList<>();
        Translation2d cameraToOuter = cameraToTargets.get(0);
        targets.add(getFieldToVisionTarget(timestamp, cameraToOuter, source));


        // if corners are available, use them to inner goal
        if (cameraToTargets.size() == 3) {
            Translation2d cornerToCorner = cameraToTargets.get(2).translateBy(cameraToTargets.get(1).inverse());

            // if corners are within a certain distance of each other
            if (cornerToCorner.norm() > Constants.kCornerToCornerLength * 0.95 && cornerToCorner.norm() < Constants.kCornerToCornerLength * 1.05) {
                //find inner goal using corners
                Translation2d outerToInner = cornerToCorner.rotateBy(Rotation2d.fromDegrees(90)).normalize().scale(29.25);
                Translation2d cameraToInner = cameraToOuter.translateBy(outerToInner);

                targets.add(getFieldToVisionTarget(timestamp, cameraToInner, source));
            }
        }

        return targets;
    }

    // transforms camera to vision target to a field to target pose
    public Pose2d getFieldToVisionTarget(double timestamp, Translation2d cameraToTarget, LimelightConstants source) {
        return Pose2d.fromTranslation(getFieldToVehicle(timestamp).transformBy(source.kSubsystemToLens).transformBy(Pose2d.fromTranslation(cameraToTarget)).getTranslation());
    }

    // when calculating targets from target information, targets may be lost
    // check to see if there is a usable number of targets for aiming
    private boolean targetsAreViable(List<Translation2d> targets, double[] expectedTargetCount) {
        if (targets == null){
            return false;
        }

        if (!(expectedTargetCount[0] <= targets.size() && targets.size() <= expectedTargetCount[1])) {
            return false;
        }

        boolean noNulls = true;
        for(int i = 0; i < targets.size(); i++) {
            noNulls = ((targets.get(i) != null) && noNulls);
        }

        // seesTarget = noNulls;
        return noNulls;
    }

    // uses target information to calculate a translation2d from the robot to the goal
    // translation2d x axis is perpendicular to camera face
    // translation2d y axis is parrallel to camera face
    private Translation2d getCameraToVisionTargetPose(TargetInfo target, LimelightConstants limelight, Target targetType) {
        // Compensate for camera pitch
        Translation2d xz_plane_translation = new Translation2d(target.getX(), target.getZ()).rotateBy(limelight.kHorizontalPlaneToLens);
        double x = xz_plane_translation.x();
        double y = target.getY();
        double z = xz_plane_translation.y();

        // find intersection with the goal
        double differential_height = limelight.kHeight - targetType.getHeight();
        // System.out.println("z: " + (z < 0.0));
        // System.out.println("diff height: " + (differential_height > 0.0));
        if ((z < 0.0) == (differential_height > 0.0)) {
            double scaling = differential_height / -z;
            double distance = Math.hypot(x, y) * scaling;
            Rotation2d angle = new Rotation2d(x, y, true);
            return new Translation2d(distance * angle.cos(), distance * angle.sin());
        }
        return null;
    }

    // ramiro deems thee unclean
    // public synchronized Optional<AimingParameters> getCachedAimingParameters() {
    //     return cached_shooter_aiming_params_ == null ? Optional.empty() : Optional.of(cached_shooter_aiming_params_);
    // }

    public synchronized Optional<AimingParameters> getOuterGoalParameters() {
        List<GoalTracker.TrackReport> reports = mGoalTracker.getTracks();
        if (reports.size() >= minimumTargetQuantity) {
            GoalTracker.TrackReport report = reports.get(primaryTargetIndex);
            if (report.stability > 0) {
                double timestamp = Timer.getFPGATimestamp();
                lastKnownTargetPosition = report.field_to_target.getTranslation();
                Pose2d vehicleToGoal = getFieldToVehicle(timestamp).inverse().transformBy(report.field_to_target).transformBy(getVisionTargetToGoalOffset());
                AimingParameters params = new AimingParameters(
                        vehicleToGoal ,
                        report.field_to_target,
                        report.field_to_target.getRotation(),
                        report.latest_timestamp,
                        report.stability,
                        new Rotation2d(), //targetOrientation,
                        report.id);

                return Optional.of(params);
            }
        }
        return Optional.empty();

    }



    // public synchronized Optional<AimingParameters> getInnerGoal() {
    //     List<GoalTracker.TrackReport> reports = mGoalTracker.getTracks();
    //     if (reports.size() >= 2) {
    //         GoalTracker.TrackReport report = reports.get(1);
    //         lastKnownTargetPosition = report.field_to_target.getTranslation();

    //         double timestamp = Timer.getFPGATimestamp();
    //         Pose2d vehicleToGoal = getFieldToVehicle(timestamp).inverse().transformBy(report.field_to_target).transformBy(getVisionTargetToGoalOffset());
    //         AimingParameters params = new AimingParameters(
    //                 vehicleToGoal ,
    //                 report.field_to_target,
    //                 report.field_to_target.getRotation(),
    //                 report.latest_timestamp,
    //                 report.stability,
    //                 new Rotation2d(), //targetOrientation,
    //                 report.id);

    //         return Optional.of(params);
    //     } else {
    //         return Optional.empty();
    //     }
    // }

    public synchronized Optional<AimingParameters> getPowerCell() {
        List<GoalTracker.TrackReport> reports = mPowerCellTracker.getTracks();
        //raynli
        // List<AimingParameters> aimingParameters = new ArrayList<>();
        if (reports.size() >= minimumTargetQuantity) {
            // for (TrackReport report : reports) {
            GoalTracker.TrackReport report = reports.get(primaryTargetIndex);
            double timestamp = Timer.getFPGATimestamp();
            lastKnownTargetPosition = report.field_to_target.getTranslation();
            Pose2d vehicleToGoal = getFieldToVehicle(timestamp).inverse().transformBy(report.field_to_target).transformBy(getVisionTargetToGoalOffset());
            AimingParameters params = new AimingParameters(
                    vehicleToGoal ,
                    report.field_to_target,
                    report.field_to_target.getRotation(),
                    report.latest_timestamp,
                    report.stability,
                    new Rotation2d(), //targetOrientation,
                    report.id);
            //aimingParameters.add(params);

            return Optional.of(params);
            // }
        }
        return Optional.empty();
    }


    // degrees
    public synchronized double getErrorAngle(Pose2d vehicleToGoal)  {
        return Math.atan2(vehicleToGoal.getTranslation().y(), vehicleToGoal.getTranslation().x());
    }

    // public synchronized Optional<Pose2d> getRobotScoringPosition(Optional<AimingParameters> aimingParameters, Rotation2d orientation, Translation2d endTranslation){
    //     List<Pose2d> targetPositions = getCaptureTimeFieldToGoal();
    // 	if (targetPositions.size() >= minimumTargetQuantity && aimingParameters.isPresent()){
    //         Translation2d targetPosition = targetPositions.get(primaryTargetIndex).getTranslation();
    //         SmartDashboard.putNumberArray("Path Pose", new double[]{targetPosition.x(), targetPosition.y(), aimingParameters.get().getTargetOrientation().getDegrees(), 0.0});
    // 		Pose2d orientedTargetPosition = new Pose2d(targetPosition, orientation).transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
    //         Pose2d robotScoringPosition = orientedTargetPosition.transformBy(Pose2d.fromTranslation(endTranslation));

    //         return Optional.of(robotScoringPosition);
    //     }
    //     return Optional.empty();
    // }


    // ramiro deems thee unclean

    // public synchronized Optional<Pose2d> getOrientedTargetPosition(Optional<AimingParameters> aimingParameters){
    //     List<Pose2d> targetPositions = getCaptureTimeFieldToGoal();
    // 	if (targetPositions.size() >= minimumTargetQuantity && aimingParameters.isPresent()){
    //         Translation2d targetPosition = targetPositions.get(primaryTargetIndex).getTranslation();
    //         SmartDashboard.putNumberArray("Path Pose", new double[]{targetPosition.x(), targetPosition.y(), aimingParameters.get().getTargetOrientation().getDegrees(), 0.0});
    // 		Pose2d orientedTargetPosition = new Pose2d(targetPosition, aimingParameters.get().getTargetOrientation());

    //         return Optional.of(orientedTargetPosition);
    //     }
    //     return Optional.empty();
    // }

//    public synchronized void resetRobotPosition(Translation2d targetPosition){
//        List<GoalTracker.TrackReport> reports = mGoalTracker.getTracks();
//        if (reports.size() >= minimumTargetQuantity) {
//            GoalTracker.TrackReport report = reports.get(primaryTargetIndex);
//            Translation2d robotFrameToFieldFrame = report.field_to_target.getTranslation().inverse().translateBy(targetPosition);
//            if (robotFrameToFieldFrame.norm() <= 5.0){
//                mSwerve.resetPosition(new Pose2d(mSwerve.getPose().getTranslation().translateBy(robotFrameToFieldFrame), mSwerve.getPose().getRotation()));
//                System.out.println("Coordinates corrected by " + robotFrameToFieldFrame.norm() + " inches");
//            }else{
//                System.out.println("Coordinate correction too large: " + robotFrameToFieldFrame.norm());
//            }
//        }else{
//            System.out.println("Vision did not detect target");
//        }
//    }



    // ramiro deems thee unclean

    // public synchronized List<Pose2d> getCaptureTimeFieldToGoal() {
    //     List<Pose2d> rv = new ArrayList<>();
    //     for (GoalTracker.TrackReport report : goal_tracker_.getTracks()) {
    //         rv.add(new Pose2d(report.field_to_target));
    //     }
    //     return rv;
    // }

    public synchronized void clearGoalTargets(){
        mGoalTracker.clearTracks();
    }

    public synchronized void clearPowerCellTargets(){
        mPowerCellTracker.clearTracks();
    }
    // ramiro deems thee unclean

    // public synchronized void feignVisionTargets(){
    //     List<Pose2d> fakeTargets = Arrays.asList(
    //         new Pose2d(100.0, 50.0, new Rotation2d()),
    //         new Pose2d(100.0, 58.0, new Rotation2d()),
    //         new Pose2d(100.0, 54.0,new Rotation2d())
    //     );
    //     goal_tracker_.update(Timer.getFPGATimestamp(), fakeTargets);
    // }

    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized double getDistanceDriven() {
        return distance_driven_;
    }

    public synchronized Pose2d getVisionTargetToGoalOffset() {
        // if (SuperstructureCommands.isInCargoShipPosition() && EndEffector.getInstance().getObservedGamePiece() == GamePiece.BALL) {
        //     return Pose2d.fromTranslation(new Translation2d(-6.0, 0.0));
        // }

        return Pose2d.identity();
    }

    public void outputToSmartDashboard(){
        SmartDashboard.putBoolean("Sees Target", seesTarget);

        /*List<Pose2d> targets = getCaptureTimeFieldToGoal();
        if (targets.size() >= minimumTargetQuantity){
            Translation2d targetPosition = targets.get(primaryTargetIndex).getTranslation();
            SmartDashboard.putNumberArray("Path Pose", new double[]{targetPosition.x(), targetPosition.y(), 0.0, 0.0});
        }*/

        /*Optional<ShooterAimingParameters> aim = getAimingParameters();
            if (aim.isPresent()) {
                SmartDashboard.putNumber("goal_range", aim.get().getRange());
            } else {
                SmartDashboard.putNumber("goal_range", 0.0);
            }*/

        if (Constants.kDebuggingOutput){

            // ramiro deems theqe unclean

            // List<Pose2d> poses = getCaptureTimeFieldToGoal();
            // for (Pose2d pose : poses) {
            //     // Only output first goal
            //     SmartDashboard.putNumber("goal_pose_x", pose.getTranslation().x());
            //     SmartDashboard.putNumber("goal_pose_y", pose.getTranslation().y());

            //     break;
            // }
            Optional<AimingParameters> aiming_params = /*getCachedAimingParameters();*/getOuterGoalParameters();
            if (aiming_params.isPresent()) {
                SmartDashboard.putNumber("goal_range", aiming_params.get().getRange());
                SmartDashboard.putNumber("goal_theta", aiming_params.get().getRobotToGoal().getRotation().getDegrees());

                // ramiro deems thee unclean
                //getOrientedTargetPosition(aiming_params);
            } else {
                SmartDashboard.putNumber("goal_range", 0.0);
                SmartDashboard.putNumber("goal_theta", 0.0);
            }
        }
    }
}
