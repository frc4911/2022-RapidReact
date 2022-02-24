package frc.robot.config;

import frc.robot.Constants;
import frc.robot.sensors.IMU.ImuType;
import frc.robot.subsystems.SwerveConfiguration;
import frc.robot.subsystems.SwerveDriveModule.SwerveModuleConstants;

public interface RobotConfiguration {

    public SwerveConfiguration getSwerveConfiguration();

    public SwerveModuleConstants getFrontRightModuleConstants();

    public SwerveModuleConstants getFrontLeftModuleConstants();

    public SwerveModuleConstants getBackLeftModuleConstants();

    public SwerveModuleConstants getBackRightModuleConstants();

    public ImuType getImuType();

    public static RobotConfiguration getRobotConfiguration(String robotName) {
        switch(robotName) {
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
