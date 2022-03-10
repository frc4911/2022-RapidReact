package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Limelight;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.util.InterpolatingDouble;
import libraries.cheesylib.util.InterpolatingTreeMap;
import libraries.cheesylib.vision.AimingParameters;
import libraries.cheesylib.vision.GoalTracker;
import libraries.cheesylib.vision.GoalTracker.TrackReportComparator;
import libraries.cheesylib.vision.TargetInfo;

/**
 * RobotState keeps track of the poses of various coordinate frames throughout
 * the match. A coordinate frame is simply a point and direction in space that
 * defines an (x,y) coordinate system. Transforms (or poses) keep track of the
 * spatial relationship between different frames.
 *
 * Robot frames of interest (from parent to child):
 *
 * 1. Field frame: origin is where the robot is turned on.
 *
 * 2. Vehicle frame: origin is the center of the robot wheelbase, facing
 * forwards
 *
 * 3. Camera frame: origin is the center of the Limelight relative to the
 * shooter.
 *
 * 4. Target (aka Goal) frame: origin is the center of the vision target, facing outwards
 * along the normal.
 *
 * As a kinematic chain with 3 frames, there are 2 transforms of interest:
 *
 * 1. Field-to-vehicle: This is tracked over time by integrating encoder and
 * gyro measurements. It will inevitably drift, but is usually accurate over
 * short time periods.
 *
 * 2. Vehicle-to-camera: This is a constant (per camera).
 *
 * 3. Camera-to-target: Measured by the vision system.
 */

public class RobotState {

    private static String sClassName;
    private static int sInstanceCount;
    private static RobotState sInstance = null;

    public static RobotState getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new RobotState(caller);
        } else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller) {
        System.out.println("(" + caller + ") " + "getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private RobotState(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        reset(0.0, Pose2d.identity(), Rotation2d.identity());
    }

    private static final int kObservationBufferSize = 100;


    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
    private GoalTracker goal_tracker_ = new GoalTracker(Constants.kGoalTrackerConfig);
    private double distance_driven_;

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(
            double start_time, Pose2d initial_field_to_vehicle, Rotation2d initial_field_to_orientation) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        goal_tracker_ = new GoalTracker(Constants.kGoalTrackerConfig);
        distance_driven_ = 0.0;
    }

    public synchronized void resetDistanceDriven() {
        distance_driven_ = 0.0;
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly
     * interpolates between stored robot positions to fill in the gaps.
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void resetVision() {
        goal_tracker_.reset();
    }

    private Translation2d getCameraToVisionTargetTranslation(TargetInfo target, Limelight source) {
        return getCameraToVisionTargetTranslation(target, source.getLensHeight(), source.getHorizontalPlaneToLens(), Constants.kVisionTargetHeight);
    }

    private static Translation2d getCameraToVisionTargetTranslation(
            TargetInfo target,
            double cameraHeight,
            Rotation2d cameraPitch,
            double targetCornerHeight) {
        // Compensate for camera pitch
        Translation2d xz_plane_translation = new Translation2d(target.getX(), target.getZ()).rotateBy(cameraPitch);
        double x = xz_plane_translation.x();
        double y = target.getY();
        double z = xz_plane_translation.y();

        // find intersection with the goal
        double differential_height = targetCornerHeight - cameraHeight;
        if ((z > 0.0) == (differential_height > 0.0)) {
            double scaling = differential_height / z;
            double distance = Math.hypot(x, y) * scaling;
            Rotation2d angle = new Rotation2d(x, y, true);
            return new Translation2d(distance * angle.cos(), distance * angle.sin());
        }
        return null;
    }

    private void updateGoalTracker(
            double timestamp,
            List<Translation2d> cameraToVisionTargetTranslations,
            GoalTracker tracker,
            Limelight source) {
        if (cameraToVisionTargetTranslations.size() != 2 ||
                cameraToVisionTargetTranslations.get(0) == null ||
                cameraToVisionTargetTranslations.get(1) == null) {
            return;
        }

        Pose2d cameraToVisionTarget = Pose2d.fromTranslation(cameraToVisionTargetTranslations.get(0).interpolate(
                cameraToVisionTargetTranslations.get(1), 0.5));

        Pose2d fieldToVisionTarget = getFieldToVehicle(timestamp).transformBy(source.getShooterToLens()).transformBy(cameraToVisionTarget);

        if (fieldToVisionTarget.getTranslation().direction().cos() < 0.0) {
            return;
        }

        // Goal normal is always oriented at 180 deg.
        tracker.update(timestamp, List.of(new Pose2d(fieldToVisionTarget.getTranslation(), Rotation2d.fromDegrees(180.0))));
    }

    public synchronized void addVisionUpdate(double timestamp, List<TargetInfo> observations, Limelight source) {
        List<Translation2d> cameraToVisionTargetTranslations = new ArrayList<>();

        if (observations == null || observations.isEmpty()) {
            goal_tracker_.maybePruneTracks();
            return;
        }

        for (TargetInfo target : observations) {
            cameraToVisionTargetTranslations.add(getCameraToVisionTargetTranslation(target, source));
        }

        updateGoalTracker(timestamp, cameraToVisionTargetTranslations, goal_tracker_, source);
    }

    public synchronized Pose2d getFieldToVisionTarget() {
        GoalTracker tracker = goal_tracker_;

        if (!tracker.hasTracks()) {
            return null;
        }

        return tracker.getTracks().get(0).field_to_target;
    }

    public synchronized Pose2d getVehicleToVisionTarget(double timestamp) {
        Pose2d fieldToVisionTarget = getFieldToVisionTarget();

        if (fieldToVisionTarget == null) {
            return null;
        }

        return getFieldToVehicle(timestamp).inverse().transformBy(fieldToVisionTarget);
    }

    public synchronized Optional<AimingParameters> getAimingParameters(int prev_track_id, double max_track_age, Pose2d target_to_goal_offset) {
        GoalTracker tracker = goal_tracker_;
        List<GoalTracker.TrackReport> reports = tracker.getTracks();

        if (reports.isEmpty()) {
            return Optional.empty();
        }

        double timestamp = Timer.getFPGATimestamp();

        // Find the best track.
        TrackReportComparator comparator = new GoalTracker.TrackReportComparator(
                Constants.kTrackStabilityWeight,
                Constants.kTrackAgeWeight,
                Constants.kTrackSwitchingWeight,
                prev_track_id, timestamp,
                Constants.kGoalTrackerConfig);
        reports.sort(comparator);

        GoalTracker.TrackReport report = null;
        for (GoalTracker.TrackReport track : reports) {
            if (track.latest_timestamp > timestamp - max_track_age) {
                report = track;
                break;
            }
        }
        if (report == null) {
            return Optional.empty();
        }

        AimingParameters params = new AimingParameters(getFieldToVehicle(timestamp),
                report.field_to_target.transformBy(target_to_goal_offset),
                report.latest_timestamp, report.stability, report.id);
        return Optional.of(params);
    }

    public void outputToSmartDashboard() {
        if (getVehicleToVisionTarget(Timer.getFPGATimestamp()) != null) {
            SmartDashboard.putString("Robot to Vision Target", getVehicleToVisionTarget(Timer.getFPGATimestamp()).toString());
        }

        if (Constants.kDebuggingOutput) {
                SmartDashboard.putNumber("goal_range", 0.0);
                SmartDashboard.putNumber("goal_theta", 0.0);
        }
    }
}
