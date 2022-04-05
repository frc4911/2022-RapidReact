package frc.robot.actions;

import frc.robot.subsystems.Swerve;
import libraries.cheesylib.autos.actions.Action;

public class SwerveTwistAction implements Action {

	private String sClassName;
	private Swerve mSwerve;
	private double heading;
	private boolean direction; // CW is true, CCW is false
	private double target;
	private double speed;

	public SwerveTwistAction(double heading, boolean direction) {
		sClassName = this.getClass().getSimpleName();
		mSwerve = Swerve.getInstance(sClassName);
		this.heading = Math.abs(heading);
		this.direction = direction;
	}

	@Override
	public boolean isFinished() {
		if (Math.abs(target - mSwerve.getHeading().getDegrees()) <= 10.0) {
			return true;
		}
		return false;
	}

	@Override
	public void start() {
		if (direction) {
			target = mSwerve.getHeading().getDegrees() - heading;
			speed = -0.75;
		} else {
			target = mSwerve.getHeading().getDegrees() + heading;
			speed = 0.75;
		}
	}

	@Override
	public void update() {
        mSwerve.setTeleopInputs(0, 0, speed, false, false, false);
	}

	@Override
	public void done() {
		mSwerve.setTeleopInputs(0, 0, 0, false, false, false);
	}
}