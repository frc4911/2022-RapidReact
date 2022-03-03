package frc.robot.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Swerve;
import libraries.cheesylib.autos.actions.Action;

public class SwerveTwistAction implements Action {

	private String sClassName;
	private Swerve mSwerve;
    private double timeout;
	private double duration;
	private double speed;

	public SwerveTwistAction(double speed, double duration) {
		sClassName = this.getClass().getSimpleName();
		mSwerve = Swerve.getInstance(sClassName);
		this.speed = speed;
		this.duration = duration;
	}

	@Override
	public boolean isFinished() {
		if (Timer.getFPGATimestamp() > timeout){
			return true;
		}
		return false;
	}

	@Override
	public void start() {
		timeout = Timer.getFPGATimestamp()+duration;
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