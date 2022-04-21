package frc.robot.actions;

import frc.robot.subsystems.Swerve;
import libraries.cheesylib.autos.actions.Action;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Rotation2d;

public class SetEndOfAutoModePoseAction implements Action {

	private final Swerve mSwerve;
	private final double trueInitHeading;
    private final double setInitHeading;

	public SetEndOfAutoModePoseAction(double trueInitHeading, double setInitHeading) {
		String sClassName = this.getClass().getSimpleName();
		mSwerve = Swerve.getInstance(sClassName);
		this.trueInitHeading = trueInitHeading;
		this.setInitHeading = setInitHeading;
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void start() {
        double currentHeading = mSwerve.getHeading().getDegrees();
        double actualHeading = currentHeading-setInitHeading+trueInitHeading;

        mSwerve.setRobotPosition(new Pose2d(0,0,new Rotation2d(actualHeading)));
    }

	@Override
	public void update() {

	}

	@Override
	public void done() {

	}
}