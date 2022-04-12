package frc.robot.actions;

import frc.robot.subsystems.Swerve;
import libraries.cheesylib.autos.actions.Action;

public class TwistAction implements Action {

	private String sClassName;
	private Swerve mSwerve;
	private double targetHeading; // Assume the start heading is 0 - var is the desired heading 0-360
	private boolean relative; // CCW is true, CW is false
	private double speed;

	/**
	 *  Independent twist action for swerve during auto. Allows the robot to rotate (in place)
	 *  until a certain heading is achieved. Can rotate either clockwise or counterclockwise.
	 *  Normalizes angles to 0-360, where positive 90 is counterclockwise, directly to the left
	 *  @param heading The desired heading to twist to
	 *  @param relative The direction of the twist (true/counterclockwise, false/clockwise)
	 */
		
	public TwistAction(double heading, boolean relative) { 
		this(heading, relative, .7);
	}

	public TwistAction(double heading, boolean relative, double speed) {
		sClassName = this.getClass().getSimpleName();
		mSwerve = Swerve.getInstance(sClassName);
		// heading %= 360;
		// if (heading < 0) {
		// 	heading += 360;
		// }
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