package frc.robot.actions;

import frc.robot.subsystems.Superstructure;
import libraries.cheesylib.autos.actions.Action;

public class CollectAction implements Action {

    private String sClassName;
    private Superstructure mSuperstructure;
    private boolean mTurnOn;

    public CollectAction(boolean turnOn) {
        sClassName = this.getClass().getSimpleName();
        mSuperstructure = Superstructure.getInstance(sClassName);
        mTurnOn = turnOn;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void start() {
        if (mTurnOn) {
            mSuperstructure.setWantedState(Superstructure.WantedState.COLLECT);
        } else {
            mSuperstructure.setWantedState(Superstructure.WantedState.HOLD);
        }
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

}