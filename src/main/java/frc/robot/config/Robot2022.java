package frc.robot.config;

import edu.wpi.first.math.util.Units;
import frc.robot.Ports;
import frc.robot.subsystems.SwerveConfiguration;
import frc.robot.subsystems.SwerveDriveModule.SwerveModuleConstants;

import static frc.robot.Constants.*;

// New Swerve requires SI units

public class Robot2022 implements RobotConfiguration {

    @Override
    public SwerveConfiguration getSwerveConfiguration() {
        return new SwerveConfiguration(
                Units.inchesToMeters(23.75),
                Units.inchesToMeters(20.75),
                Units.feetToMeters(1.0), // Max speed in feet per second: Theoretical max is 17.065 ft per second
                Math.toRadians(270), // Max change in degrees per second
                .018, 0, 0, 0 // kSwerveHeadingKp,kSwerveHeadingKi,kSwerveHeadingKp,kSwerveHeadingKf
        );
    }

    @Override
    public SwerveModuleConstants getFrontRightModuleConstants() {
        SwerveModuleConstants frontRightModuleConstants = new SwerveModuleConstants();

        frontRightModuleConstants.kName = "Front Right";
        frontRightModuleConstants.kDriveMotorTalonId = Ports.ROBOT_2022_FRONT_RIGHT_DRIVE;
        frontRightModuleConstants.kSteerMotorTalonId = Ports.ROBOT_2022_FRONT_RIGHT_STEER;
        frontRightModuleConstants.kSteerMotorEncoderHomeOffset = 883.0;
        frontRightModuleConstants.kCANCoderId = Ports.FRONT_RIGHT_CANCODER;
        frontRightModuleConstants.kCANCoderOffsetDegrees = 358;
        frontRightModuleConstants.kWheelDiameter = kMK4_L2iWheelDiameter;
        frontRightModuleConstants.kDriveReduction = kMK4_L2iDriveReduction;
        frontRightModuleConstants.kSteerReduction = kMK4_L2iSteerReduction;
        frontRightModuleConstants.kInvertDrive = true;
        frontRightModuleConstants.kInvertSteerMotor = true;
        frontRightModuleConstants.kSteerMotorSlot0Kp = 0.25;
        frontRightModuleConstants.kSteerMotorSlot0Ki = 0.0;
        frontRightModuleConstants.kSteerMotorSlot0Kd = 0.0;
        frontRightModuleConstants.kSteerMotorSlot0Kf = 0.0;

        return frontRightModuleConstants;
    }

    @Override
    public SwerveModuleConstants getFrontLeftModuleConstants() {
        SwerveModuleConstants frontLeftModuleConstants = new SwerveModuleConstants();

        frontLeftModuleConstants.kName = "Front Left";
        frontLeftModuleConstants.kDriveMotorTalonId = Ports.ROBOT_2022_FRONT_LEFT_DRIVE;
        frontLeftModuleConstants.kSteerMotorTalonId = Ports.ROBOT_2022_FRONT_LEFT_STEER;
        frontLeftModuleConstants.kSteerMotorEncoderHomeOffset = 1683.0;
        frontLeftModuleConstants.kCANCoderId = Ports.FRONT_LEFT_CANCODER;
        frontLeftModuleConstants.kCANCoderOffsetDegrees = 210;
        frontLeftModuleConstants.kWheelDiameter = kMK4_L2iWheelDiameter;
        frontLeftModuleConstants.kDriveReduction = kMK4_L2iDriveReduction;
        frontLeftModuleConstants.kSteerReduction = kMK4_L2iSteerReduction;
        frontLeftModuleConstants.kInvertDrive = true;
        frontLeftModuleConstants.kInvertSteerMotor = true;
        frontLeftModuleConstants.kSteerMotorSlot0Kp = 0.25;
        frontLeftModuleConstants.kSteerMotorSlot0Ki = 0.0;
        frontLeftModuleConstants.kSteerMotorSlot0Kd = 0.0;
        frontLeftModuleConstants.kSteerMotorSlot0Kf = 0.0;

        return frontLeftModuleConstants;
    }

    @Override
    public SwerveModuleConstants getBackLeftModuleConstants() {
        SwerveModuleConstants backLeftModuleConstants = new SwerveModuleConstants();

        backLeftModuleConstants.kName = "Back Left";
        backLeftModuleConstants.kDriveMotorTalonId = Ports.ROBOT_2022_BACK_LEFT_DRIVE;
        backLeftModuleConstants.kSteerMotorTalonId = Ports.ROBOT_2022_BACK_LEFT_STEER;
        backLeftModuleConstants.kSteerMotorEncoderHomeOffset = 3451.0;
        backLeftModuleConstants.kCANCoderId = Ports.BACK_LEFT_CANCODER;
        backLeftModuleConstants.kCANCoderOffsetDegrees = 278;
        backLeftModuleConstants.kWheelDiameter = kMK4_L2iWheelDiameter;
        backLeftModuleConstants.kDriveReduction = kMK4_L2iDriveReduction;
        backLeftModuleConstants.kSteerReduction = kMK4_L2iSteerReduction;
        backLeftModuleConstants.kInvertDrive = true;
        backLeftModuleConstants.kInvertSteerMotor = true;
        backLeftModuleConstants.kSteerMotorSlot0Kp = 0.25;
        backLeftModuleConstants.kSteerMotorSlot0Ki = 0.0;
        backLeftModuleConstants.kSteerMotorSlot0Kd = 0.0;
        backLeftModuleConstants.kSteerMotorSlot0Kf = 0.0;

        return backLeftModuleConstants;
    }

    @Override
    public SwerveModuleConstants getBackRightModuleConstants() {
        SwerveModuleConstants backRightModuleConstants = new SwerveModuleConstants();

        backRightModuleConstants.kName = "Back Right";
        backRightModuleConstants.kDriveMotorTalonId = Ports.ROBOT_2022_BACK_RIGHT_DRIVE;
        backRightModuleConstants.kSteerMotorTalonId = Ports.ROBOT_2022_BACK_RIGHT_STEER;
        backRightModuleConstants.kSteerMotorEncoderHomeOffset = -327.0;
        backRightModuleConstants.kCANCoderId = Ports.BACK_RIGHT_CANCODER;
        backRightModuleConstants.kCANCoderOffsetDegrees = 116;
        backRightModuleConstants.kWheelDiameter = kMK4_L2iWheelDiameter;
        backRightModuleConstants.kDriveReduction = kMK4_L2iDriveReduction;
        backRightModuleConstants.kSteerReduction = kMK4_L2iSteerReduction;
        backRightModuleConstants.kInvertDrive = true;
        backRightModuleConstants.kInvertSteerMotor = true;
        backRightModuleConstants.kSteerMotorSlot0Kp = 0.25;
        backRightModuleConstants.kSteerMotorSlot0Ki = 0.0;
        backRightModuleConstants.kSteerMotorSlot0Kd = 0.0;
        backRightModuleConstants.kSteerMotorSlot0Kf = 0.0;

        return backRightModuleConstants;
    }
}
