package frc.robot.config;

import edu.wpi.first.math.util.Units;
import frc.robot.Ports;
import frc.robot.subsystems.SwerveConfiguration;
import frc.robot.subsystems.SwerveDriveModule.SwerveModuleConstants;

import static frc.robot.Constants.kMK2_SteerReduction;
import static frc.robot.Constants.kMK2_WheelDiameter;

// New Swerve requires SI units

public class Junior implements RobotConfiguration {

    @Override
    public SwerveConfiguration getSwerveConfiguration() {
        return new SwerveConfiguration(
                Units.inchesToMeters(14.5),
                Units.inchesToMeters(14.5),
                Units.feetToMeters(14.2),
                Math.toRadians(270),
                .018, 0, 0, 0 // kSwerveHeadingKp,kSwerveHeadingKi,kSwerveHeadingKp,kSwerveHeadingKf
        );
    }

    @Override
    public SwerveModuleConstants getFrontRightModuleConstants() {
        SwerveModuleConstants frontRightModuleConstants = new SwerveModuleConstants();

        frontRightModuleConstants.kName = "Front Right";
        frontRightModuleConstants.kModuleId = 0;
        frontRightModuleConstants.kDriveMotorTalonId = Ports.FRONT_RIGHT_DRIVE;
        frontRightModuleConstants.kSteerMotorTalonId = Ports.FRONT_RIGHT_STEER;
        frontRightModuleConstants.kSteerMotorEncoderHomeOffset = 883.0;
        frontRightModuleConstants.kCANCoderOffsetDegrees = 80;
        frontRightModuleConstants.kCANCoderId = Ports.FRONT_RIGHT_CANCODER;
        frontRightModuleConstants.kWheelDiameter = kMK2_WheelDiameter;
        frontRightModuleConstants.kDriveReduction = (18.0 / 38.0) * (26.0 / 18.0) * (15.0 / 60.0);
        frontRightModuleConstants.kSteerReduction = kMK2_SteerReduction;
        frontRightModuleConstants.kSteerMotorSlot0Kp = 0.4;
        frontRightModuleConstants.kSteerMotorSlot0Ki = 0.0;
        frontRightModuleConstants.kSteerMotorSlot0Kd = 0.0;
        frontRightModuleConstants.kSteerMotorSlot0Kf = 0.0;

        return frontRightModuleConstants;
    }

    @Override
    public SwerveModuleConstants getFrontLeftModuleConstants() {
        SwerveModuleConstants frontLeftModuleConstants = new SwerveModuleConstants();

        frontLeftModuleConstants.kName = "Front Left";
        frontLeftModuleConstants.kModuleId = 1;
        frontLeftModuleConstants.kDriveMotorTalonId = Ports.FRONT_LEFT_DRIVE;
        frontLeftModuleConstants.kSteerMotorTalonId = Ports.FRONT_LEFT_STEER;
        frontLeftModuleConstants.kSteerMotorEncoderHomeOffset = 1683.0;
        frontLeftModuleConstants.kCANCoderOffsetDegrees = -28;
        frontLeftModuleConstants.kCANCoderId = Ports.FRONT_LEFT_CANCODER;
        frontLeftModuleConstants.kWheelDiameter = kMK2_WheelDiameter;
        frontLeftModuleConstants.kDriveReduction = (18.0 / 38.0) * (26.0 / 18.0) * (15.0 / 60.0);
        frontLeftModuleConstants.kSteerReduction = kMK2_SteerReduction;
        frontLeftModuleConstants.kSteerMotorSlot0Kp = 0.4;
        frontLeftModuleConstants.kSteerMotorSlot0Ki = 0.0;
        frontLeftModuleConstants.kSteerMotorSlot0Kd = 0.0;
        frontLeftModuleConstants.kSteerMotorSlot0Kf = 0.0;

        return frontLeftModuleConstants;
    }

    @Override
    public SwerveModuleConstants getBackLeftModuleConstants() {
        SwerveModuleConstants backLeftModuleConstants = new SwerveModuleConstants();

        backLeftModuleConstants.kName = "Back Left";
        backLeftModuleConstants.kModuleId = 2;
        backLeftModuleConstants.kDriveMotorTalonId = Ports.BACK_LEFT_DRIVE;
        backLeftModuleConstants.kSteerMotorTalonId = Ports.BACK_LEFT_STEER;
        backLeftModuleConstants.kSteerMotorEncoderHomeOffset = 3451.0;
        backLeftModuleConstants.kCANCoderOffsetDegrees = 146;
        backLeftModuleConstants.kCANCoderId = Ports.BACK_LEFT_CANCODER;
        backLeftModuleConstants.kWheelDiameter = kMK2_WheelDiameter;
        backLeftModuleConstants.kDriveReduction = (18.0 / 38.0) * (26.0 / 18.0) * (15.0 / 60.0);
        backLeftModuleConstants.kSteerReduction = kMK2_SteerReduction;
        backLeftModuleConstants.kSteerMotorSlot0Kp = 0.4;
        backLeftModuleConstants.kSteerMotorSlot0Ki = 0.0;
        backLeftModuleConstants.kSteerMotorSlot0Kd = 0.0;
        backLeftModuleConstants.kSteerMotorSlot0Kf = 0.0;

        return backLeftModuleConstants;
    }

    @Override
    public SwerveModuleConstants getBackRightModuleConstants() {
        SwerveModuleConstants backRightModuleConstants = new SwerveModuleConstants();

        backRightModuleConstants.kName = "Back Right";
        backRightModuleConstants.kModuleId = 3;
        backRightModuleConstants.kDriveMotorTalonId = Ports.BACK_RIGHT_DRIVE;
        backRightModuleConstants.kSteerMotorTalonId = Ports.BACK_RIGHT_STEER;
        backRightModuleConstants.kSteerMotorEncoderHomeOffset = -327.0;
        backRightModuleConstants.kCANCoderId = Ports.BACK_RIGHT_CANCODER;
        backRightModuleConstants.kCANCoderOffsetDegrees = -24;
        backRightModuleConstants.kWheelDiameter = kMK2_WheelDiameter;
        backRightModuleConstants.kDriveReduction = (18.0 / 38.0) * (26.0 / 18.0) * (15.0 / 60.0);
        backRightModuleConstants.kSteerReduction = kMK2_SteerReduction;
        backRightModuleConstants.kSteerMotorSlot0Kp = 0.4;
        backRightModuleConstants.kSteerMotorSlot0Ki = 0.0;
        backRightModuleConstants.kSteerMotorSlot0Kd = 0.0;
        backRightModuleConstants.kSteerMotorSlot0Kf = 0.0;

        return backRightModuleConstants;
    }
}
