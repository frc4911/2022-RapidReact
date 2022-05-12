package frc.robot.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.subsystems.Swerve;
import libraries.cheesylib.autos.actions.Action;
import libraries.cheesylib.geometry.Pose2dWithCurvature;
import libraries.cheesylib.trajectory.TimedView;
import libraries.cheesylib.trajectory.Trajectory;
import libraries.cheesylib.trajectory.TrajectoryIterator;
import libraries.cheesylib.trajectory.timing.TimedState;

public class DriveTrajectoryAction implements Action {
    private static final Swerve mDrive = Swerve.getInstance("DriveTrajectoryAction");

    private final TrajectoryIterator<TimedState<Pose2dWithCurvature>> mTrajectory;
    private final boolean mResetPose;

    public DriveTrajectoryAction(Trajectory<TimedState<Pose2dWithCurvature>> trajectory) {
        this(trajectory, false);
    }

    public DriveTrajectoryAction(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, boolean resetPose) {
        mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
        mResetPose = resetPose;
    }

    @Override
    public void start() {
        System.out.println("Starting trajectory! (length=" + mTrajectory.getRemainingProgress() + ")");
        if (mResetPose) {
            var startPose = mTrajectory.getState().state().getPose();
            var orientation = mTrajectory.getState().state().getPose().getRotation();
            // TODO - when using Swerve enabled trajectpries get the orientation forom the state
//            var orientation = Rotation2d.fromDegrees(Math.toRadians(mTrajectory.getState().state().orientation));
            RobotState.getInstance("DriveTrajectoryAction").reset(Timer.getFPGATimestamp(), startPose, orientation);
            mDrive.setRobotPosition(startPose);
        }
        mDrive.setTrajectory(mTrajectory);
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        if (mDrive.isDoneWithTrajectory()) {
            System.out.println("Trajectory finished");
            return true;
        }
        return false;
    }

    @Override
    public void done() {
    }
}
