package frc.robot.actions;

import frc.robot.subsystems.Swerve;
import libraries.cheesylib.autos.actions.Action;

public class TwistAction implements Action {

	private String sClassName;
	private Swerve mSwerve;
	private double heading; // Assume the start heading is 0 - var is the desired heading 0-360
	private boolean direction; // CCW is true, CW is false
	private double speed;

	/**
	 *  Independent twist action for swerve during auto. Allows the robot to rotate (in place)
	 *  until a certain heading is achieved. Can rotate either clockwise or counterclockwise.
	 *  Normalizes angles to 0-360, where positive 90 is counterclockwise, directly to the left
	 *  @param heading The desired heading to twist to
	 *  @param direction The direction of the twist (true/counterclockwise, false/clockwise)
	 */
		
	public TwistAction(double heading, boolean direction) { //If direction changes within one automode it will cause the heading variable vs. actual heading to be off-caleb
		sClassName = this.getClass().getSimpleName();
		mSwerve = Swerve.getInstance(sClassName);
		heading %= 360;
		if (heading < 0) {
			heading += 360;
		}
		this.heading = heading;
		this.direction = direction;
	}

	@Override
	public boolean isFinished() {
		double normalizedHeading = (mSwerve.getHeading().getDegrees() % 360);
		if (normalizedHeading < 0) {
			normalizedHeading += 360;
		}
		if (Math.abs(heading - normalizedHeading) <= 10) {
			return true;
		}
		return false;
	}

	@Override
	public void start() {
		if (direction) {
			speed = 0.7;
		} else {
			speed = -0.7;
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