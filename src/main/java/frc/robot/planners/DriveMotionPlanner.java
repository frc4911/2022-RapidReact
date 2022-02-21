package frc.robot.planners;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
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
import libraries.cyberlib.kinematics.ChassisSpeeds;
import libraries.cyberlib.utils.Angles;
import libraries.cyberlib.utils.HolonomicDriveSignal;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class DriveMotionPlanner implements CSVWritable {
    private static final double kMaxDx = Units.inches_to_meters(2.0);
    private static final double kMaxDy = Units.inches_to_meters(0.25);
    private static final double kMaxDTheta = Math.toRadians(5.0);
    private Swerve mSwerve;

    private Translation2d followingCenter = Translation2d.identity();

    public enum FollowerType {
        PURE_PURSUIT
    }

    FollowerType mFollowerType = FollowerType.PURE_PURSUIT;

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

    public double getRotationSample() {
        final double kStartPoint = 0.2;
        final double kPivotPoint = 0.5;
        final double kEndPoint = 0.8;
        final double kMaxSpeed = 1.0;
        double normalizedProgress = mCurrentTrajectory.getProgress() / currentTrajectoryLength;
        double scalar = 0.0;
        if (kStartPoint <= normalizedProgress && normalizedProgress <= kEndPoint) {
            if (normalizedProgress <= kPivotPoint) {
                scalar = (normalizedProgress - kStartPoint) / (kPivotPoint - kStartPoint);
            }else{
                scalar = 1.0 - ((normalizedProgress - kPivotPoint) / (kEndPoint - kPivotPoint));
            }
        }

        return kMaxSpeed * scalar;
    }

    public SwerveConfiguration swerveConfiguration;

    public DriveMotionPlanner() {
        mSwerve = Swerve.getInstance("DriveMotionPlanner");
        swerveConfiguration = mSwerve.mSwerveConfiguration;
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

        timed_trajectory.setDefaultVelocity(default_vel / swerveConfiguration.maxSpeedInMetersPerSecond);
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

    protected Optional<HolonomicDriveSignal> updatePurePursuit(Pose2d current_state) {
        double lookahead_time = Constants.kPathLookaheadTime;
        final double kLookaheadSearchDt = 0.01;

        TimedState<Pose2dWithCurvature> lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
        double actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
        while (actual_lookahead_distance < Constants.kPathMinLookaheadDistance &&
                mCurrentTrajectory.getRemainingProgress() > lookahead_time) {
            lookahead_time += kLookaheadSearchDt;
            lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
            actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
        }
        if (actual_lookahead_distance < Constants.kPathMinLookaheadDistance) {
            lookahead_state = new TimedState<>(new Pose2dWithCurvature(lookahead_state.state()
                    .getPose().transformBy(Pose2d.fromTranslation(new Translation2d(
                            (mIsReversed ? -1.0 : 1.0) * (Constants.kPathMinLookaheadDistance -
                                    actual_lookahead_distance), 0.0))), 0.0), lookahead_state.t()
                    , lookahead_state.velocity(), lookahead_state.acceleration());
        }

//        SmartDashboard.putNumber("Path X", lookahead_state.state().getTranslation().x());
//        SmartDashboard.putNumber("Path Y", lookahead_state.state().getTranslation().y());
//        SmartDashboard.putNumber("Path Velocity", lookahead_state.velocity() / Constants.kSwerveMaxSpeedInchesPerSecond);

        // A WCD would have to plot an arc.  Since this is Swerve, use a straight line.
        // Translate to lookahead position in the straight line formed by the two points: current and lookahead position

        var normalizedVelocity = lookahead_state.velocity() / swerveConfiguration.maxSpeedInMetersPerSecond;

        // Now calculate velocities
        Translation2d segmentVelocity = new Translation2d(
                lookahead_state.state().getRotation().cos(),
                lookahead_state.state().getRotation().sin()).scale(normalizedVelocity);

        // Calculate the rotational velocity required to keep rotating  while translating.
        // Get the rotation angle over the trajectory
        // TODO:  Make constants and only calculate once
        var startAngle = mCurrentTrajectory.trajectory().getFirstState().state().getPose().getRotation().getRadians();
        var endAngle = mCurrentTrajectory.trajectory().getLastState().state().getPose().getRotation().getRadians();
        double totalAngle = Angles.shortest_angular_distance(startAngle, endAngle);

        // getMaxRotationSpeed() returns a trapezoidal ramp for rotation speed based on travelled trajectory.
        var theta0 = (startAngle + (totalAngle * getRotationSample()));
        var theta1 = lookahead_state.state().getRotation().getRadians();

        // w = (theta(1) - theta(0)) / dt
        double rotationVelocity = Angles.normalizeAngle(theta1 - theta0) / mDt;

        // Scale it to a percentage of max angular velocity as Swerve will scale it correctly
        rotationVelocity /= mSwerve.mSwerveConfiguration.maxSpeedInRadiansPerSecond;

        var signal = new HolonomicDriveSignal(segmentVelocity, rotationVelocity, true);
        // System.out.println(signal.toString()); // brian leave until fixed
        return Optional.of(signal);
    }

     public Optional<HolonomicDriveSignal> update(double timestamp, Pose2d current_state, ChassisSpeeds chassisSpeeds) {
        if (mCurrentTrajectory == null) {
            return Optional.empty();
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

            if (mFollowerType == FollowerType.PURE_PURSUIT) {
                return updatePurePursuit(current_state);
            }
        }

        return Optional.empty();
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
