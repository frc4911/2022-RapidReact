package frc.robot.actions;

import frc.robot.subsystems.Swerve;
import libraries.cheesylib.autos.actions.Action;
import libraries.cheesylib.geometry.Pose2d;

public class SetStartPoseAction implements Action{

    private String sClassName = "SetStartPoseAction";
    private Swerve mSwerve;
    private Pose2d startingPose;

    public SetStartPoseAction(Pose2d pose) {
        mSwerve = Swerve.getInstance(sClassName);
        startingPose = pose;
    }

    @Override
    public void start() {
        mSwerve.setRobotPosition(startingPose);
    }

    @Override
    public void update() {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {
        
    }
    
}
