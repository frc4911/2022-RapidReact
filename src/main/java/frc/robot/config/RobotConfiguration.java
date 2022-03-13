package frc.robot.config;

import frc.robot.constants.Constants;
import frc.robot.limelight.LimelightConfiguration;
import frc.robot.sensors.IMU.ImuType;

public interface RobotConfiguration {

    public SwerveConfiguration getSwerveConfiguration();

    public SwerveModuleConfiguration getFrontRightModuleConstants();

    public SwerveModuleConfiguration getFrontLeftModuleConstants();

    public SwerveModuleConfiguration getBackLeftModuleConstants();

    public SwerveModuleConfiguration getBackRightModuleConstants();

    public ImuType getImuType();

    public LimelightConfiguration getLimelightConfiguration();

    public static RobotConfiguration getRobotConfiguration(String robotName) {
        switch (robotName) {
            case Constants.kJuniorName:
                return new Junior();
            case Constants.kDeadEyeName:
                return new DeadEye();
            case Constants.kRobot2022Name:
            default:
                return new Robot2022();
        }
    }
}
