package frc.robot.actions;

import frc.robot.subsystems.Swerve;
import libraries.cheesylib.autos.actions.Action;

public class TwistAction implements Action {

	private String sClassName;
	private Swerve mSwerve;
	private double targetHeading; // Assume the start heading is 0 - var is the desired heading 0-360
	private boolean relative;
	private double speed;
		
	public TwistAction(double heading, boolean relative) { 
		this(heading, relative, .7);
	}

	/**
	 * Independent action to twist without translation
	 * @param heading Desired angle to twist to
	 * @param relative Relative to the robot's current heading or field heading
	 * @param speed Percent output
	 */
	public TwistAction(double heading, boolean relative, double speed) {
		sClassName = this.getClass().getSimpleName();
		mSwerve = Swerve.getInstance(sClassName);
		this.targetHeading = heading;
		this.speed = Math.abs(speed);
		this.relative = relative;
	}

	@Override
	public boolean isFinished() {
		if (Math.abs(targetHeading - mSwerve.getHeading().getDegrees()) <= 10) {
			return true;
		}
		return false;
	}

	@Override
	public void start() {
		if (relative){
			targetHeading = mSwerve.getHeading().getDegrees()+targetHeading;
		}
	}

	@Override
	public void update() {
		double delta = targetHeading - mSwerve.getHeading().getDegrees();
		double tmpSpeed = delta>=0 ? speed : -speed;

        mSwerve.setTeleopInputs(0, 0, tmpSpeed, false, false, false);
	}

	@Override
	public void done() {
		mSwerve.setTeleopInputs(0, 0, 0, false, false, false);
	}
}