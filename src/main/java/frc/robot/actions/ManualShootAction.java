package frc.robot.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Superstructure;
import libraries.cheesylib.autos.actions.Action;

public class ManualShootAction implements Action {
	
	private Superstructure mSuperstructure = Superstructure.getInstance("ManualShootAction");
    private Indexer mIndexer = Indexer.getInstance("ManualShootAction");
	private double target = 0.0;

	public ManualShootAction(double duration) {
		target = Timer.getFPGATimestamp() + duration;
	}
	
	@Override
	public boolean isFinished() {
		return Timer.getFPGATimestamp() >= target;// || mIndexer.ball;
	}
	
	@Override
	public void start() {
		mSuperstructure.setWantedState(Superstructure.WantedState.MANUAL_SHOOT);
	}
	
	@Override
	public void update() {

	}
	
	@Override
	public void done() {
		mSuperstructure.setWantedState(Superstructure.WantedState.HOLD);
	}
	
}