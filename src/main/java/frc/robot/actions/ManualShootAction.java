package frc.robot.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Superstructure;
import libraries.cheesylib.autos.actions.Action;

public class ManualShootAction implements Action {

	private Superstructure mSuperstructure = Superstructure.getInstance("ManualShootAction");
	private double target = 0.0;
	private double shotDistance;
	private double shotTimeout;
	private String sClassName = "ManualShootAction";

	public ManualShootAction(double distance, double duration) {
		shotDistance = distance;
		shotTimeout = duration;
	}

	@Override
	public boolean isFinished() {
		//
		return /* mIndexer.getBallCount() == 0 */ Timer.getFPGATimestamp() >= target;
	}

	@Override
	public void start() {
		target = Timer.getFPGATimestamp() + shotTimeout;
		mSuperstructure.setManualShootDistance(shotDistance);
		mSuperstructure.setWantedState(Superstructure.WantedState.MANUAL_SHOOT, sClassName);
	}

	@Override
	public void update() {

	}

	@Override
	public void done() {
		mSuperstructure.setWantedState(Superstructure.WantedState.HOLD, sClassName);
	}

}