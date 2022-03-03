package frc.robot.config;

import static frc.robot.constants.Constants.kMK2_SteerReduction;
import static frc.robot.constants.Constants.kMK2_WheelDiameter;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.Ports;
import frc.robot.sensors.IMU.ImuType;

// New Swerve requires SI units
public class Junior implements RobotConfiguration {

    private static final double FRONT_RIGHT_STEER_MOTOR_ENCODER_HOME_OFFSET = 883.0;
    private static final double FRONT_LEFT_STEER_MOTOR_ENCODER_HOME_OFFSET = 1683.0;
    private static final double BACK_LEFT_STEER_MOTOR_ENCODER_HOME_OFFSET = 3451.0;
    private static final double BACK_RIGHT_STEER_MOTOR_ENCODER_HOME_OFFSET = -327.0;

    private static final int FRONT_RIGHT_CANCODER_OFFSET_DEGREES = 80;
    private static final int FRONT_LEFT_CANCODER_OFFSET_DEGREES = -28;
    private static final int BACK_LEFT_CANCODER_OFFSET_DEGREES = 146;
    private static final int BACK_RIGHT_CANCODER_OFFSET_DEGREES = -24;

    private static final double STEER_MOTOR_KP = 0.4;
    private static final double STEER_MOTOR_KI = 0.0;
    private static final double STEER_MOTOR_KD = 0.0;
    private static final double STEER_MOTOR_KF = 0.0;

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
    public SwerveModuleConfiguration getFrontRightModuleConstants() {
        SwerveModuleConfiguration frontRightModuleConstants = new SwerveModuleConfiguration();

        frontRightModuleConstants.kName = "Front Right";
        frontRightModuleConstants.kModuleId = 0;
        frontRightModuleConstants.kDriveMotorTalonId = Ports.FRONT_RIGHT_DRIVE;
        frontRightModuleConstants.kSteerMotorTalonId = Ports.FRONT_RIGHT_STEER;
        frontRightModuleConstants.kSteerMotorEncoderHomeOffset = FRONT_RIGHT_STEER_MOTOR_ENCODER_HOME_OFFSET;
        frontRightModuleConstants.kCANCoderOffsetDegrees = FRONT_RIGHT_CANCODER_OFFSET_DEGREES;
        frontRightModuleConstants.kCANCoderId = Ports.FRONT_RIGHT_CANCODER;
        frontRightModuleConstants.kWheelDiameter = kMK2_WheelDiameter;
        frontRightModuleConstants.kDriveReduction = (18.0 / 38.0) * (26.0 / 18.0) * (15.0 / 60.0);
        frontRightModuleConstants.kSteerReduction = kMK2_SteerReduction;
        frontRightModuleConstants.kSteerMotorSlot0Kp = STEER_MOTOR_KP;
        frontRightModuleConstants.kSteerMotorSlot0Ki = STEER_MOTOR_KI;
        frontRightModuleConstants.kSteerMotorSlot0Kd = STEER_MOTOR_KD;
        frontRightModuleConstants.kSteerMotorSlot0Kf = STEER_MOTOR_KF;

        return frontRightModuleConstants;
    }

    @Override
    public SwerveModuleConfiguration getFrontLeftModuleConstants() {
        SwerveModuleConfiguration frontLeftModuleConstants = new SwerveModuleConfiguration();

        frontLeftModuleConstants.kName = "Front Left";
        frontLeftModuleConstants.kModuleId = 1;
        frontLeftModuleConstants.kDriveMotorTalonId = Ports.FRONT_LEFT_DRIVE;
        frontLeftModuleConstants.kSteerMotorTalonId = Ports.FRONT_LEFT_STEER;
        frontLeftModuleConstants.kSteerMotorEncoderHomeOffset = FRONT_LEFT_STEER_MOTOR_ENCODER_HOME_OFFSET;
        frontLeftModuleConstants.kCANCoderOffsetDegrees = FRONT_LEFT_CANCODER_OFFSET_DEGREES;
        frontLeftModuleConstants.kCANCoderId = Ports.FRONT_LEFT_CANCODER;
        frontLeftModuleConstants.kWheelDiameter = kMK2_WheelDiameter;
        frontLeftModuleConstants.kDriveReduction = (18.0 / 38.0) * (26.0 / 18.0) * (15.0 / 60.0);
        frontLeftModuleConstants.kSteerReduction = kMK2_SteerReduction;
        frontLeftModuleConstants.kSteerMotorSlot0Kp = STEER_MOTOR_KP;
        frontLeftModuleConstants.kSteerMotorSlot0Ki = STEER_MOTOR_KI;
        frontLeftModuleConstants.kSteerMotorSlot0Kd = STEER_MOTOR_KD;
        frontLeftModuleConstants.kSteerMotorSlot0Kf = STEER_MOTOR_KF;

        return frontLeftModuleConstants;
    }

    @Override
    public SwerveModuleConfiguration getBackLeftModuleConstants() {
        SwerveModuleConfiguration backLeftModuleConstants = new SwerveModuleConfiguration();

        backLeftModuleConstants.kName = "Back Left";
        backLeftModuleConstants.kModuleId = 2;
        backLeftModuleConstants.kDriveMotorTalonId = Ports.BACK_LEFT_DRIVE;
        backLeftModuleConstants.kSteerMotorTalonId = Ports.BACK_LEFT_STEER;
        backLeftModuleConstants.kSteerMotorEncoderHomeOffset = BACK_LEFT_STEER_MOTOR_ENCODER_HOME_OFFSET;
        backLeftModuleConstants.kCANCoderOffsetDegrees = BACK_LEFT_CANCODER_OFFSET_DEGREES;
        backLeftModuleConstants.kCANCoderId = Ports.BACK_LEFT_CANCODER;
        backLeftModuleConstants.kWheelDiameter = kMK2_WheelDiameter;
        backLeftModuleConstants.kDriveReduction = (18.0 / 38.0) * (26.0 / 18.0) * (15.0 / 60.0);
        backLeftModuleConstants.kSteerReduction = kMK2_SteerReduction;
        backLeftModuleConstants.kSteerMotorSlot0Kp = STEER_MOTOR_KP;
        backLeftModuleConstants.kSteerMotorSlot0Ki = STEER_MOTOR_KI;
        backLeftModuleConstants.kSteerMotorSlot0Kd = STEER_MOTOR_KD;
        backLeftModuleConstants.kSteerMotorSlot0Kf = STEER_MOTOR_KF;

        return backLeftModuleConstants;
    }

    @Override
    public SwerveModuleConfiguration getBackRightModuleConstants() {
        SwerveModuleConfiguration backRightModuleConstants = new SwerveModuleConfiguration();

        backRightModuleConstants.kName = "Back Right";
        backRightModuleConstants.kModuleId = 3;
        backRightModuleConstants.kDriveMotorTalonId = Ports.BACK_RIGHT_DRIVE;
        backRightModuleConstants.kSteerMotorTalonId = Ports.BACK_RIGHT_STEER;
        backRightModuleConstants.kSteerMotorEncoderHomeOffset = BACK_RIGHT_STEER_MOTOR_ENCODER_HOME_OFFSET;
        backRightModuleConstants.kCANCoderId = Ports.BACK_RIGHT_CANCODER;
        backRightModuleConstants.kCANCoderOffsetDegrees = BACK_RIGHT_CANCODER_OFFSET_DEGREES;
        backRightModuleConstants.kWheelDiameter = kMK2_WheelDiameter;
        backRightModuleConstants.kDriveReduction = (18.0 / 38.0) * (26.0 / 18.0) * (15.0 / 60.0);
        backRightModuleConstants.kSteerReduction = kMK2_SteerReduction;
        backRightModuleConstants.kSteerMotorSlot0Kp = STEER_MOTOR_KP;
        backRightModuleConstants.kSteerMotorSlot0Ki = STEER_MOTOR_KI;
        backRightModuleConstants.kSteerMotorSlot0Kd = STEER_MOTOR_KD;
        backRightModuleConstants.kSteerMotorSlot0Kf = STEER_MOTOR_KF;

        return backRightModuleConstants;
    }

    @Override
    public ImuType getImuType() {
        return ImuType.PIGEON;
    }
}
