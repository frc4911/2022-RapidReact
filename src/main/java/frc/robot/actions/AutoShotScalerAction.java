package frc.robot.actions;

import frc.robot.subsystems.Swerve;
import libraries.cheesylib.autos.actions.Action;

public class AutoShotScalerAction implements Action {
    private String sClassName;
    private Swerve mSwerve;
    private double mScaler;

    public AutoShotScalerAction(double scaler) {
        sClassName = this.getClass().getSimpleName();
        mSwerve = Swerve.getInstance(sClassName);
        mScaler = scaler;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void start() {
        mSwerve.setAimingTwistScaler(mScaler);
    }

    @Override
    public void update() {
        
    }

    @Override
    public void done() {
        
    }

}
