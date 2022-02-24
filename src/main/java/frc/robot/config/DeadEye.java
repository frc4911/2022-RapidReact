package frc.robot.config;

import edu.wpi.first.math.util.Units;
import frc.robot.Ports;
import frc.robot.sensors.IMU.ImuType;
import frc.robot.subsystems.SwerveConfiguration;
import frc.robot.subsystems.SwerveDriveModule.SwerveModuleConstants;

import static frc.robot.Constants.kMK2_SteerReduction;
import static frc.robot.Constants.kMK2_WheelDiameter;

// New Swerve requires SI units
public class DeadEye implements RobotConfiguration {

    private static final double FRONT_RIGHT_STEER_MOTOR_ENCODER_HOME_OFFSET = 883.0;
    private static final double FRONT_LEFT_STEER_MOTOR_ENCODER_HOME_OFFSET = 1683.0;
    private static final double BACK_LEFT_STEER_MOTOR_ENCODER_HOME_OFFSET = 3451.0;
    private static final double BACK_RIGHT_STEER_MOTOR_ENCODER_HOME_OFFSET = -327.0;

    private static final int FRONT_RIGHT_CANCODER_OFFSET_DEGREES = 0;
    private static final int FRONT_LEFT_CANCODER_OFFSET_DEGREES = 0;
    private static final int BACK_LEFT_CANCODER_OFFSET_DEGREES = 0;
    private static final int BACK_RIGHT_CANCODER_OFFSET_DEGREES = 0;

    @Override
    public SwerveConfiguration getSwerveConfiguration() {
        return new SwerveConfiguration(
                Units.inchesToMeters(21.0),
                Units.inchesToMeters(21.0),
                Units.feetToMeters(14.2),
                Math.toRadians(270),
                .018, 0, 0, 0 // kSwerveHeadingKp,kSwerveHeadingKi,kSwerveHeadingKp,kSwerveHeadingKf
        );
    }

    @Override
    public SwerveModuleConstants getFrontRightModuleConstants() {
        SwerveModuleConstants frontRightModuleConstants = new SwerveModuleConstants();

        frontRightModuleConstants.kName = "Front Right";
        frontRightModuleConstants.kDriveMotorTalonId = Ports.FRONT_RIGHT_DRIVE;
        frontRightModuleConstants.kSteerMotorTalonId = Ports.FRONT_RIGHT_STEER;
        frontRightModuleConstants.kSteerMotorEncoderHomeOffset = FRONT_RIGHT_STEER_MOTOR_ENCODER_HOME_OFFSET;
        frontRightModuleConstants.kCANCoderId = Ports.FRONT_RIGHT_CANCODER;
        frontRightModuleConstants.kCANCoderOffsetDegrees = FRONT_RIGHT_CANCODER_OFFSET_DEGREES;
        frontRightModuleConstants.kWheelDiameter = kMK2_WheelDiameter;
        frontRightModuleConstants.kDriveReduction = (16.0 / 40.0) * (26.0 / 18.0) * (15.0 / 60.0);
        frontRightModuleConstants.kSteerReduction = kMK2_SteerReduction;

        return frontRightModuleConstants;
    }

    @Override
    public SwerveModuleConstants getFrontLeftModuleConstants() {
        SwerveModuleConstants frontLeftModuleConstants = new SwerveModuleConstants();

        frontLeftModuleConstants.kName = "Front Left";
        frontLeftModuleConstants.kDriveMotorTalonId = Ports.FRONT_LEFT_DRIVE;
        frontLeftModuleConstants.kSteerMotorTalonId = Ports.FRONT_LEFT_STEER;
        frontLeftModuleConstants.kSteerMotorEncoderHomeOffset = FRONT_LEFT_STEER_MOTOR_ENCODER_HOME_OFFSET;
        frontLeftModuleConstants.kCANCoderId = Ports.FRONT_LEFT_CANCODER;
        frontLeftModuleConstants.kCANCoderOffsetDegrees = FRONT_LEFT_CANCODER_OFFSET_DEGREES;
        frontLeftModuleConstants.kWheelDiameter = kMK2_WheelDiameter;
        frontLeftModuleConstants.kDriveReduction = (16.0 / 40.0) * (26.0 / 18.0) * (15.0 / 60.0);
        frontLeftModuleConstants.kSteerReduction = kMK2_SteerReduction;

        return frontLeftModuleConstants;
    }

    @Override
    public SwerveModuleConstants getBackLeftModuleConstants() {
        SwerveModuleConstants backLeftModuleConstants = new SwerveModuleConstants();

        backLeftModuleConstants.kName = "Back Left";
        backLeftModuleConstants.kDriveMotorTalonId = Ports.BACK_LEFT_DRIVE;
        backLeftModuleConstants.kSteerMotorTalonId = Ports.BACK_LEFT_STEER;
        backLeftModuleConstants.kSteerMotorEncoderHomeOffset = BACK_LEFT_STEER_MOTOR_ENCODER_HOME_OFFSET;
        backLeftModuleConstants.kCANCoderId = Ports.BACK_LEFT_CANCODER;
        backLeftModuleConstants.kCANCoderOffsetDegrees = BACK_LEFT_CANCODER_OFFSET_DEGREES;
        backLeftModuleConstants.kWheelDiameter = kMK2_WheelDiameter;
        backLeftModuleConstants.kDriveReduction = (16.0 / 40.0) * (26.0 / 18.0) * (15.0 / 60.0);
        backLeftModuleConstants.kSteerReduction = kMK2_SteerReduction;

        return backLeftModuleConstants;
    }

    @Override
    public SwerveModuleConstants getBackRightModuleConstants() {
        SwerveModuleConstants backRightModuleConstants = new SwerveModuleConstants();

        backRightModuleConstants.kName = "Back Right";
        backRightModuleConstants.kDriveMotorTalonId = Ports.BACK_RIGHT_DRIVE;
        backRightModuleConstants.kSteerMotorTalonId = Ports.BACK_RIGHT_STEER;
        backRightModuleConstants.kSteerMotorEncoderHomeOffset = BACK_RIGHT_STEER_MOTOR_ENCODER_HOME_OFFSET;
        backRightModuleConstants.kCANCoderId = Ports.BACK_RIGHT_CANCODER;
        backRightModuleConstants.kCANCoderOffsetDegrees = BACK_RIGHT_CANCODER_OFFSET_DEGREES;
        backRightModuleConstants.kWheelDiameter = kMK2_WheelDiameter;
        backRightModuleConstants.kDriveReduction = (16.0 / 40.0) * (26.0 / 18.0) * (15.0 / 60.0);
        backRightModuleConstants.kSteerReduction = kMK2_SteerReduction;

        return backRightModuleConstants;
    }

    @Override
    public ImuType getImuType() {
        return ImuType.PIGEON;
    }
}
