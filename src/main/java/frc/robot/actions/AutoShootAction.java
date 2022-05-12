package frc.robot.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;
import libraries.cheesylib.autos.actions.Action;
import libraries.cyberlib.control.SwerveHeadingController;

public class AutoShootAction implements Action{

    private String sClassName = "AutoShootAction";
    private Superstructure mSuperstructure = Superstructure.getInstance(sClassName);
    private Swerve mSwerve = Swerve.getInstance(sClassName);
    private SwerveHeadingController mHeadingController = SwerveHeadingController.getInstance();
	private double target = 0.0;
	private double shotTimeout;

	public AutoShootAction(double duration) {
		shotTimeout = duration;
	}

	@Override
	public boolean isFinished() {
		return mSuperstructure.autoShootingComplete() || Timer.getFPGATimestamp() >= target;
	}

	@Override
	public void start() {
		target = Timer.getFPGATimestamp() + shotTimeout;
		mHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.OFF);
        mSuperstructure.setWantedState(Superstructure.WantedState.AUTO_SHOOT, sClassName);
        mSwerve.EnableAimingController();
	}

	@Override
	public void update() {

	}

	@Override
	public void done() {
        mSwerve.DisableAimingController();
        mSuperstructure.setWantedState(Superstructure.WantedState.HOLD, sClassName);
        mHeadingController.setGoal(mSwerve.getHeading().getDegrees());
	}
    
}
