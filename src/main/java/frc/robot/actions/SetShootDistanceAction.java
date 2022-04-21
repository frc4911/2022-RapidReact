package frc.robot.actions;

import frc.robot.subsystems.Superstructure;
import libraries.cheesylib.autos.actions.Action;

public class SetShootDistanceAction implements Action{

    private String sClassName;
    private Superstructure mSuperstructure;
    private double mShootDistance;

    public SetShootDistanceAction(double shootDistance) {
        sClassName = this.getClass().getSimpleName();
        mSuperstructure = Superstructure.getInstance(sClassName);
        mShootDistance = shootDistance;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void start(){
        mSuperstructure.setShootDistance(mShootDistance);
    }

    @Override
    public void update() {
        
    }

    @Override
    public void done() {
        
    }
    
}

