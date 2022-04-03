package frc.robot.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Swerve;
import libraries.cheesylib.autos.actions.Action;

public class SwerveTwistAction implements Action {

	private String sClassName;
	private Swerve mSwerve;
	private double heading;
	private double error;
	private double speed;

	public SwerveTwistAction(double heading) {
		sClassName = this.getClass().getSimpleName();
		mSwerve = Swerve.getInstance(sClassName);
		this.heading = heading;
	}

	@Override
	public boolean isFinished() {
		if (Math.abs(error) <= 1.5){
			return true;
		}
		return false;
	}

	@Override
	public void start() {
		error = heading - mSwerve.getHeading().getDegrees();
		speed = Math.copySign(0.7, error);
	}

	@Override
	public void update() {
		error = heading - mSwerve.getHeading().getDegrees();
        mSwerve.setTeleopInputs(0, 0, speed, false, false, false);
	}

	@Override
	public void done() {
		mSwerve.setTeleopInputs(0, 0, 0, false, false, false);
	}
}