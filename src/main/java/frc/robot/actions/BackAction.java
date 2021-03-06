package frc.robot.actions;

import frc.robot.subsystems.Superstructure;
import libraries.cheesylib.autos.actions.Action;

public class BackAction implements Action{

    private String sClassName;
    private Superstructure mSuperstructure;
    private boolean mTurnOn;

    public BackAction(boolean turnOn) {
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
            mSuperstructure.setWantedState(Superstructure.WantedState.BACK, sClassName);
        } else {
            mSuperstructure.setWantedState(Superstructure.WantedState.HOLD, sClassName);
        }
    }

    @Override
    public void update() {
        
    }

    @Override
    public void done() {
        
    }
    
}
