package frc.robot.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Swerve;
import libraries.cheesylib.autos.actions.Action;

public class EndTrajectoryAction implements Action {

	private double mDuration;
	private double timeout;

	public EndTrajectoryAction(double duration) {
		mDuration = duration;
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
		timeout = Timer.getFPGATimestamp()+mDuration;
	}

	@Override
	public void update() {
		if (Timer.getFPGATimestamp() > timeout){
			done();
		}
	}

	@Override
	public void done() {
        Swerve.getInstance("OverrideTrajectory").overrideTrajectory(true);
	}

}