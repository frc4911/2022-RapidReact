package frc.robot.actions;

//import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Swerve;
import libraries.cheesylib.autos.actions.Action;
import libraries.cheesylib.geometry.Pose2d;

public class SetPoseAction implements Action {

	private String sClassName;
	private Swerve mSwerve;
	private Pose2d pose;


	public SetPoseAction(Pose2d pose) {
		sClassName = this.getClass().getSimpleName();
		mSwerve = Swerve.getInstance(sClassName);
		this.pose = pose;
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void start() {
		mSwerve.setRobotPosition(pose);
	}

	@Override
	public void update() {

	}

	@Override
	public void done() {

	}

}