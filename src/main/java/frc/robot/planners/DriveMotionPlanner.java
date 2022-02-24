package frc.robot.planners;

import frc.robot.config.RobotConfiguration;
import frc.robot.subsystems.SwerveConfiguration;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Pose2dWithCurvature;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.trajectory.*;
import libraries.cheesylib.trajectory.timing.TimedState;
import libraries.cheesylib.trajectory.timing.TimingConstraint;
import libraries.cheesylib.trajectory.timing.TimingUtil;
import libraries.cheesylib.util.CSVWritable;
import libraries.cheesylib.util.Units;
import libraries.cheesylib.util.Util;
import libraries.cyberlib.control.*;
import libraries.cyberlib.kinematics.ChassisSpeeds;
import libraries.cyberlib.utils.HolonomicDriveSignal;
import libraries.cyberlib.utils.RobotName;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class DriveMotionPlanner implements CSVWritable {
    private static final double kMaxDx = Units.inches_to_meters(2.0);
    private static final double kMaxDy = Units.inches_to_meters(0.25);
    private static final double kMaxDTheta = Math.toRadians(5.0);

    TrajectoryFollower follower;
    private HolonomicDriveSignal driveSignal = null;

    private Translation2d followingCenter = Translation2d.identity();

    public enum FollowerType {
        HOLONOMIC,
        PURE_PURSUIT
    }

    FollowerType mFollowerType = FollowerType.HOLONOMIC;

    public void setFollowerType(FollowerType type) {
        mFollowerType = type;
    }


    TrajectoryIterator<TimedState<Pose2dWithCurvature>> mCurrentTrajectory;
    public Trajectory<TimedState<Pose2dWithCurvature>> getTrajectory() {
        return mCurrentTrajectory.trajectory();
    }

    public double getRemainingProgress() {
        if (mCurrentTrajectory != null) {
            return mCurrentTrajectory.getRemainingProgress();
        }
        return 0.0;
    }

    boolean mIsReversed = false;
    double mLastTime = Double.POSITIVE_INFINITY;
    public TimedState<Pose2dWithCurvature> mSetpoint = new TimedState<>(Pose2dWithCurvature.identity());
    Pose2d mError = Pose2d.identity();
    HolonomicDriveSignal mOutput = new HolonomicDriveSignal(Translation2d.identity(), 0.0, true);
    double currentTrajectoryLength = 0.0;

    double mDt = 0.0;

    public SwerveConfiguration mSwerveConfiguration;

    public DriveMotionPlanner() {
        RobotConfiguration mRobotConfiguration = RobotConfiguration.getRobotConfiguration(RobotName.name);
        mSwerveConfiguration =  mRobotConfiguration.getSwerveConfiguration();
        if (mFollowerType == FollowerType.HOLONOMIC) {
            // TODO:  Make these constants
            follower = new HolonomicTrajectoryFollower(
                    new PidGains(0.4, 0.0, 0.025),
                    new PidGains(5.0, 0.0, 0.0),
                    new HolonomicFeedforward(new SwerveDriveFeedforwardGains(
                            0.289, //0.042746,
                            0.0032181,
                            0.30764
                    )));
        } else if (mFollowerType == FollowerType.PURE_PURSUIT) {
           follower = new PurePursuitTrajectoryFollower();
        }
    }

    public void setTrajectory(final TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        mCurrentTrajectory = trajectory;
        mSetpoint = trajectory.getState();
        currentTrajectoryLength = trajectory.trajectory().getLastState().t();
        for (int i = 0; i < trajectory.trajectory().length(); ++i) {
            if (trajectory.trajectory().getState(i).velocity() > Util.kEpsilon) {
                mIsReversed = false;
                break;
            } else if (trajectory.trajectory().getState(i).velocity() < -Util.kEpsilon) {
                mIsReversed = true;
                break;
            }
        }
    }

    public void reset() {
        mError = Pose2d.identity();
        mOutput = new HolonomicDriveSignal(Translation2d.identity(), 0.0, true);
        mLastTime = Double.POSITIVE_INFINITY;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // meters/s
            double max_accel,  // meters/s^2
            double max_decel,
            double max_voltage,
            double default_vel,
            int slowdown_chunks) {
        return generateTrajectory(reversed, waypoints, constraints, 0.0, 0.0, max_vel, max_accel, max_decel, max_voltage,
                default_vel, slowdown_chunks);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,
            double end_vel,
            double max_vel,  // meters/s
            double max_accel,  // meters/s^2
            double max_decel,
            double max_voltage,
            double default_vel,
            int slowdown_chunks) {
        List<Pose2d> waypoints_maybe_flipped = waypoints;
        final Pose2d flip = Pose2d.fromRotation(new Rotation2d(-1, 0, false));
        // TODO re-architect the spline generator to support reverse.
        if (reversed) {
            waypoints_maybe_flipped = new ArrayList<>(waypoints.size());
            for (int i = 0; i < waypoints.size(); ++i) {
                waypoints_maybe_flipped.add(waypoints.get(i).transformBy(flip));
            }
        }

        // Create a trajectory from splines.
        Trajectory<Pose2dWithCurvature> trajectory = TrajectoryUtil.trajectoryFromSplineWaypoints(
                waypoints_maybe_flipped, kMaxDx, kMaxDy, kMaxDTheta);

        if (reversed) {
            List<Pose2dWithCurvature> flipped = new ArrayList<>(trajectory.length());
            for (int i = 0; i < trajectory.length(); ++i) {
                flipped.add(new Pose2dWithCurvature(trajectory.getState(i).getPose().transformBy(flip), -trajectory
                        .getState(i).getCurvature(), trajectory.getState(i).getDCurvatureDs()));
            }
            trajectory = new Trajectory<>(flipped);
        }
        // Create the constraint that the robot must be able to traverse the trajectory without ever applying more
        // than the specified voltage.
        //final CurvatureVelocityConstraint velocity_constraints = new CurvatureVelocityConstraint();
        List<TimingConstraint<Pose2dWithCurvature>> all_constraints = new ArrayList<>();
        //all_constraints.add(velocity_constraints);
        if (constraints != null) {
            all_constraints.addAll(constraints);
        }
        // Generate the timed trajectory.
        Trajectory<TimedState<Pose2dWithCurvature>> timed_trajectory = TimingUtil.timeParameterizeTrajectory
                (reversed, new DistanceView<>(trajectory), kMaxDx, all_constraints,
                        start_vel, end_vel, max_vel, max_accel, max_decel, slowdown_chunks);

        timed_trajectory.setDefaultVelocity(default_vel / mSwerveConfiguration.maxSpeedInMetersPerSecond);
        return timed_trajectory;
    }

    /**
     * @param followingCenter the followingCenter to set (relative to the robot's center)
     */
    public void setFollowingCenter(Translation2d followingCenter) {
//        this.followingCenter = followingCenter;
    }

    @Override
    public String toCSV() {
//        return mOutput.toCSV();
        return "";
    }

     public HolonomicDriveSignal update(double timestamp, Pose2d current_state, ChassisSpeeds chassisSpeeds) {
         HolonomicDriveSignal driveSignal;

        if (mCurrentTrajectory == null) {
            return null;
        }

        if (mCurrentTrajectory.getProgress() == 0.0 && !Double.isFinite(mLastTime)) {
            mLastTime = timestamp;
        }

        mDt = timestamp - mLastTime;
        mLastTime = timestamp;
        TrajectorySamplePoint<TimedState<Pose2dWithCurvature>> sample_point = mCurrentTrajectory.advance(mDt);
        mSetpoint = sample_point.state();

        if (!mCurrentTrajectory.isDone()) {
            mError = current_state.inverse().transformBy(mSetpoint.state().getPose());
            var velocity = new Translation2d(chassisSpeeds.vxInMetersPerSecond, chassisSpeeds.vyInMetersPerSecond);

            Optional<HolonomicDriveSignal> trajectorySignal = follower.update(current_state,
                    velocity,
                    chassisSpeeds.omegaInRadiansPerSecond,
                    mLastTime,
                    mDt);

            if (trajectorySignal.isPresent()) {
                driveSignal = trajectorySignal.get();
                // Scale inputs as Swerve scales them.
                driveSignal = new HolonomicDriveSignal(
                        driveSignal.getTranslation().scale(1 / mSwerveConfiguration.maxSpeedInMetersPerSecond),
                        driveSignal.getRotation() / mSwerveConfiguration.maxSpeedInRadiansPerSecond,
                        driveSignal.isFieldOriented()
                );
            } else {
                driveSignal = this.driveSignal;
            }
            return driveSignal;
        }

        return null;
    }

//    private double distance(Pose2d current_state, double additional_progress) {
//        return mCurrentTrajectory.preview(additional_progress).state().state().getPose().distance(current_state);
//    }

    public boolean isDone() {
        return mCurrentTrajectory != null && mCurrentTrajectory.isDone();
    }

    public Pose2d error() {
        return mError;
    }

    public TimedState<Pose2dWithCurvature> setpoint() {
        return mSetpoint;
    }
}
