package frc.robot.actions;

import frc.robot.subsystems.Swerve;
import libraries.cheesylib.autos.actions.Action;
import libraries.cheesylib.geometry.Pose2d;

public class SetPoseAction implements Action {

	private final Swerve mSwerve;
	private final Pose2d pose;
	private final boolean useHeading;

	public SetPoseAction(Pose2d pose, boolean useHeading) {
		String sClassName = this.getClass().getSimpleName();
		mSwerve = Swerve.getInstance(sClassName);
		this.pose = pose;
		this.useHeading = useHeading;
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void start() {
		if (useHeading) {
			mSwerve.setRobotPosition(new Pose2d(
					mSwerve.getPose().getTranslation().plus(pose.getTranslation()),
					mSwerve.getPose().getRotation().rotateBy(pose.getRotation())));
		} else {
			mSwerve.setRobotPosition(pose);
		}
	}

	@Override
	public void update() {

	}

	@Override
	public void done() {

	}
}