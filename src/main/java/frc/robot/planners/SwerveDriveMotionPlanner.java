package frc.robot.planners;

import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Pose2dWithCurvature;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.trajectory.TrajectoryIterator;
import libraries.cheesylib.trajectory.TrajectorySamplePoint;
import libraries.cheesylib.trajectory.timing.TimedState;
import libraries.cheesylib.util.Util;
import libraries.cyberlib.control.*;
import libraries.cyberlib.kinematics.ChassisSpeeds;
import libraries.cyberlib.utils.HolonomicDriveSignal;

import java.util.Optional;

public class SwerveDriveMotionPlanner {

    TrajectoryIterator<TimedState<Pose2dWithCurvature>> mCurrentTrajectory;
    boolean mIsReversed = false;
    double mLastTime = Double.POSITIVE_INFINITY;
    public TimedState<Pose2dWithCurvature> mSetpoint = new TimedState<>(Pose2dWithCurvature.identity());
    Pose2d mError = Pose2d.identity();

    double mDt = 0.0;

    public enum FollowerType {
        JACKS_HOLONOMIC,
        PURE_PURSUIT,
        PID,
    }

    FollowerType mFollowerType = FollowerType.JACKS_HOLONOMIC;

    public void setFollowerType(FollowerType type) {
        mFollowerType = type;
    }

    TrajectoryFollower follower;


    public SwerveDriveMotionPlanner() {
        // TODO:  Make these constants
        follower = new HolonomicTrajectoryFollower(
                new PidGains(0.4, 0.0, 0.025),
                new PidGains(5.0, 0.0, 0.0),
                new HolonomicFeedforward(new SwerveDriveFeedforwardGains(
                        0.042746,
                        0.0032181,
                        0.30764
                        )));
    }

    public void setTrajectory(final TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        mCurrentTrajectory = trajectory;
        mSetpoint = trajectory.getState();
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

    // TODO:  Add generateTrajectory methods with kinodynamic constraints.
    // Remember, the HolonomicTrajectoryFollower interprets the Pose2dWithCurvature as follows:
    //  - Translation2d : the x and y coordinates for any point in the path
    //  - Rotation2d : the orientation (or angular rotation) of the robot at any point in the path


    public void reset() {
        mError = Pose2d.identity();
//        mOutput = new DriveOutput();
        mLastTime = Double.POSITIVE_INFINITY;
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

            var velocity = new Translation2d(chassisSpeeds.vxInMetersPerSecond, chassisSpeeds.vyInMetersPerSecond);

            if (mFollowerType == FollowerType.JACKS_HOLONOMIC) {
                return follower.update(
                        current_state,
                        velocity,
                        0.0,
                        timestamp,
                        mDt);
            }

        }

        return Optional.empty();
    }

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