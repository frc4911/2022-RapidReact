package libraries.cyberlib.control;

import frc.robot.constants.Constants;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Pose2dWithCurvature;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.trajectory.TrajectoryIterator;
import libraries.cheesylib.trajectory.timing.TimedState;
import libraries.cheesylib.util.Util;
import libraries.cyberlib.utils.Angles;
import libraries.cyberlib.utils.HolonomicDriveSignal;

public class PurePursuitTrajectoryFollower extends TrajectoryFollower<HolonomicDriveSignal> {

    private TimedState<Pose2dWithCurvature> lastState = null;
    private boolean mIsReversed = false;
    private boolean mIsReversedCalulated = false;
    private boolean mFinished = false;

    public PurePursuitTrajectoryFollower() {
    }

    @Override
    protected HolonomicDriveSignal calculateDriveSignal(Pose2d currentPose, Translation2d velocity,
            double rotationalVelocity, TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory, double time,
            double dt) {
        if (trajectory.isDone() || time > trajectory.trajectory().getLastState().t()) {
            mFinished = true;
            return new HolonomicDriveSignal(Translation2d.identity(), 0.0, false);
        }

        lastState = trajectory.advance(dt).state();

        double lookahead_time = Constants.kPathLookaheadTime;
        final double kLookaheadSearchDt = 0.01;

        TimedState<Pose2dWithCurvature> lookahead_state = trajectory.preview(lookahead_time).state();
        double actual_lookahead_distance = lastState.state().distance(lookahead_state.state());
        while (actual_lookahead_distance < Constants.kPathMinLookaheadDistance &&
                trajectory.getRemainingProgress() > lookahead_time) {
            lookahead_time += kLookaheadSearchDt;
            lookahead_state = trajectory.preview(lookahead_time).state();
            actual_lookahead_distance = lastState.state().distance(lookahead_state.state());
        }
        if (actual_lookahead_distance < Constants.kPathMinLookaheadDistance) {
            lookahead_state = new TimedState<>(new Pose2dWithCurvature(lookahead_state.state()
                    .getPose().transformBy(Pose2d.fromTranslation(new Translation2d(
                            (isReversed(trajectory) ? -1.0 : 1.0) * (Constants.kPathMinLookaheadDistance -
                                    actual_lookahead_distance),
                            0.0))),
                    0.0), lookahead_state.t(), lookahead_state.velocity(), lookahead_state.acceleration());
        }

        var normalizedVelocity = lookahead_state.velocity();

        // Now calculate velocities
        Translation2d segmentVelocity = new Translation2d(
                lookahead_state.state().getPose().getRotation().cos(),
                lookahead_state.state().getPose().getRotation().sin()).scale(normalizedVelocity);

        // Calculate the rotational velocity required to keep rotating while
        // translating. Get the rotation angle over the trajectory
        // TODO: Make constants and only calculate once
        var startAngle = trajectory.trajectory().getFirstState().state().getPose().getRotation().getRadians();
        var endAngle = trajectory.trajectory().getLastState().state().getPose().getRotation().getRadians();
        double totalAngle = Angles.shortest_angular_distance(startAngle, endAngle);

        // getMaxRotationSpeed() returns a trapezoidal ramp for rotation speed based on
        // travelled trajectory.
        var theta0 = (startAngle + (totalAngle * getRotationSample(trajectory)));
        var theta1 = lookahead_state.state().getRotation().getRadians();

        // w = (theta(1) - theta(0)) / dt
        double rotationVelocity = Angles.normalizeAngle(theta1 - theta0) / dt;

        var signal = new HolonomicDriveSignal(segmentVelocity, rotationVelocity, true);

        return signal;
    }

    @Override
    protected boolean isFinished() {
        return mFinished;
    }

    @Override
    protected void reset() {
        mFinished = false;
    }

    private boolean isReversed(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        if (!mIsReversedCalulated) {
            for (int i = 0; i < trajectory.trajectory().length(); ++i) {
                if (trajectory.trajectory().getState(i).velocity() > Util.kEpsilon) {
                    mIsReversed = false;
                    break;
                } else if (trajectory.trajectory().getState(i).velocity() < -Util.kEpsilon) {
                    mIsReversed = true;
                    break;
                }
            }
            mIsReversedCalulated = true;
        }
        return mIsReversed;
    }

    public double getRotationSample(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        final double kStartPoint = 0.2;
        final double kPivotPoint = 0.5;
        final double kEndPoint = 0.8;
        final double kMaxSpeed = 1.0;
        double normalizedProgress = trajectory.getProgress() / trajectory.trajectory().length();
        double scalar = 0.0;
        if (kStartPoint <= normalizedProgress && normalizedProgress <= kEndPoint) {
            if (normalizedProgress <= kPivotPoint) {
                scalar = (normalizedProgress - kStartPoint) / (kPivotPoint - kStartPoint);
            } else {
                scalar = 1.0 - ((normalizedProgress - kPivotPoint) / (kEndPoint - kPivotPoint));
            }
        }

        return kMaxSpeed * scalar;
    }
}
