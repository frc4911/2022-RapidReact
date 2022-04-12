package frc.robot.actions;

import frc.robot.subsystems.Superstructure;
import libraries.cheesylib.autos.actions.Action;

public class CollectAction implements Action {

	private String sClassName;
	private Superstructure mSuperstructure;
	private boolean mTurnOn;
	private int counter = 10;

	public CollectAction(boolean turnOn) {
		sClassName = this.getClass().getSimpleName();
		mSuperstructure = Superstructure.getInstance(sClassName);
		mTurnOn = turnOn;
	}

	@Override
	public boolean isFinished() {
		return counter--<=0 ? true : false;
	}

	@Override
	public void start() {
		if (mTurnOn) {
			mSuperstructure.setWantedState(Superstructure.WantedState.COLLECT, sClassName);
		} else {
			mSuperstructure.setWantedState(Superstructure.WantedState.HOLD, sClassName);
		}
		counter = 10;
	}

	@Override
	public void update() {

	}

	@Override
	public void done() {

	}

}