package frc.robot.config;

import static frc.robot.constants.Constants.kMK4_L2iDriveReduction;
import static frc.robot.constants.Constants.kMK4_L2iSteerReduction;
import static frc.robot.constants.Constants.kMK4_L2iWheelDiameter;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.Ports;
import frc.robot.sensors.IMU.ImuType;
import frc.robot.subsystems.SwerveDriveModule.SwerveModuleConstants;

// New Swerve requires SI units
public class Robot2022 implements RobotConfiguration {

    private static final double FRONT_RIGHT_STEER_MOTOR_ENCODER_HOME_OFFSET = 883.0;
    private static final double FRONT_LEFT_STEER_MOTOR_ENCODER_HOME_OFFSET = 1683.0;
    private static final double BACK_LEFT_STEER_MOTOR_ENCODER_HOME_OFFSET = 3451.0;
    private static final double BACK_RIGHT_STEER_MOTOR_ENCODER_HOME_OFFSET = -327.0;

    private static final int FRONT_RIGHT_CANCODER_OFFSET_DEGREES = 358;
    private static final int FRONT_LEFT_CANCODER_OFFSET_DEGREES = 210;
    private static final int BACK_LEFT_CANCODER_OFFSET_DEGREES = 278;
    private static final int BACK_RIGHT_CANCODER_OFFSET_DEGREES = 116;

    private static final double STEER_MOTOR_KP = 0.25;
    private static final double STEER_MOTOR_KI = 0.0;
    private static final double STEER_MOTOR_KD = 0.0;
    private static final double STEER_MOTOR_KF = 0.0;

    @Override
    public SwerveConfiguration getSwerveConfiguration() {
        return new SwerveConfiguration(
                Units.inchesToMeters(23.75),
                Units.inchesToMeters(20.75),
                Units.feetToMeters(14.2), // Max speed in feet per second: Theoretical max is 17.065 ft per second
                Math.toRadians(270), // Max change in degrees per second
                0.01, 0, 0, 0 // kSwerveHeadingKp,kSwerveHeadingKi,kSwerveHeadingKp,kSwerveHeadingKf
        );
    }

    @Override
    public SwerveModuleConstants getFrontRightModuleConstants() {
        SwerveModuleConstants frontRightModuleConstants = new SwerveModuleConstants();

        frontRightModuleConstants.kName = "Front Right";
        frontRightModuleConstants.kDriveMotorTalonId = Ports.ROBOT_2022_FRONT_RIGHT_DRIVE;
        frontRightModuleConstants.kSteerMotorTalonId = Ports.ROBOT_2022_FRONT_RIGHT_STEER;
        frontRightModuleConstants.kSteerMotorEncoderHomeOffset = FRONT_RIGHT_STEER_MOTOR_ENCODER_HOME_OFFSET;
        frontRightModuleConstants.kCANCoderId = Ports.FRONT_RIGHT_CANCODER;
        frontRightModuleConstants.kCANCoderOffsetDegrees = FRONT_RIGHT_CANCODER_OFFSET_DEGREES;
        frontRightModuleConstants.kWheelDiameter = kMK4_L2iWheelDiameter;
        frontRightModuleConstants.kDriveReduction = kMK4_L2iDriveReduction;
        frontRightModuleConstants.kSteerReduction = kMK4_L2iSteerReduction;
        frontRightModuleConstants.kInvertDrive = true;
        frontRightModuleConstants.kInvertSteerMotor = true;
        frontRightModuleConstants.kSteerMotorSlot0Kp = STEER_MOTOR_KP;
        frontRightModuleConstants.kSteerMotorSlot0Ki = STEER_MOTOR_KI;
        frontRightModuleConstants.kSteerMotorSlot0Kd = STEER_MOTOR_KD;
        frontRightModuleConstants.kSteerMotorSlot0Kf = STEER_MOTOR_KF;

        return frontRightModuleConstants;
    }

    @Override
    public SwerveModuleConstants getFrontLeftModuleConstants() {
        SwerveModuleConstants frontLeftModuleConstants = new SwerveModuleConstants();

        frontLeftModuleConstants.kName = "Front Left";
        frontLeftModuleConstants.kDriveMotorTalonId = Ports.ROBOT_2022_FRONT_LEFT_DRIVE;
        frontLeftModuleConstants.kSteerMotorTalonId = Ports.ROBOT_2022_FRONT_LEFT_STEER;
        frontLeftModuleConstants.kSteerMotorEncoderHomeOffset = FRONT_LEFT_STEER_MOTOR_ENCODER_HOME_OFFSET;
        frontLeftModuleConstants.kCANCoderId = Ports.FRONT_LEFT_CANCODER;
        frontLeftModuleConstants.kCANCoderOffsetDegrees = FRONT_LEFT_CANCODER_OFFSET_DEGREES;
        frontLeftModuleConstants.kWheelDiameter = kMK4_L2iWheelDiameter;
        frontLeftModuleConstants.kDriveReduction = kMK4_L2iDriveReduction;
        frontLeftModuleConstants.kSteerReduction = kMK4_L2iSteerReduction;
        frontLeftModuleConstants.kInvertDrive = true;
        frontLeftModuleConstants.kInvertSteerMotor = true;
        frontLeftModuleConstants.kSteerMotorSlot0Kp = STEER_MOTOR_KP;
        frontLeftModuleConstants.kSteerMotorSlot0Ki = STEER_MOTOR_KI;
        frontLeftModuleConstants.kSteerMotorSlot0Kd = STEER_MOTOR_KD;
        frontLeftModuleConstants.kSteerMotorSlot0Kf = STEER_MOTOR_KF;

        return frontLeftModuleConstants;
    }

    @Override
    public SwerveModuleConstants getBackLeftModuleConstants() {
        SwerveModuleConstants backLeftModuleConstants = new SwerveModuleConstants();

        backLeftModuleConstants.kName = "Back Left";
        backLeftModuleConstants.kDriveMotorTalonId = Ports.ROBOT_2022_BACK_LEFT_DRIVE;
        backLeftModuleConstants.kSteerMotorTalonId = Ports.ROBOT_2022_BACK_LEFT_STEER;
        backLeftModuleConstants.kSteerMotorEncoderHomeOffset = BACK_LEFT_STEER_MOTOR_ENCODER_HOME_OFFSET;
        backLeftModuleConstants.kCANCoderId = Ports.BACK_LEFT_CANCODER;
        backLeftModuleConstants.kCANCoderOffsetDegrees = BACK_LEFT_CANCODER_OFFSET_DEGREES;
        backLeftModuleConstants.kWheelDiameter = kMK4_L2iWheelDiameter;
        backLeftModuleConstants.kDriveReduction = kMK4_L2iDriveReduction;
        backLeftModuleConstants.kSteerReduction = kMK4_L2iSteerReduction;
        backLeftModuleConstants.kInvertDrive = true;
        backLeftModuleConstants.kInvertSteerMotor = true;
        backLeftModuleConstants.kSteerMotorSlot0Kp = STEER_MOTOR_KP;
        backLeftModuleConstants.kSteerMotorSlot0Ki = STEER_MOTOR_KI;
        backLeftModuleConstants.kSteerMotorSlot0Kd = STEER_MOTOR_KD;
        backLeftModuleConstants.kSteerMotorSlot0Kf = STEER_MOTOR_KF;

        return backLeftModuleConstants;
    }

    @Override
    public SwerveModuleConstants getBackRightModuleConstants() {
        SwerveModuleConstants backRightModuleConstants = new SwerveModuleConstants();

        backRightModuleConstants.kName = "Back Right";
        backRightModuleConstants.kDriveMotorTalonId = Ports.ROBOT_2022_BACK_RIGHT_DRIVE;
        backRightModuleConstants.kSteerMotorTalonId = Ports.ROBOT_2022_BACK_RIGHT_STEER;
        backRightModuleConstants.kSteerMotorEncoderHomeOffset = BACK_RIGHT_STEER_MOTOR_ENCODER_HOME_OFFSET;
        backRightModuleConstants.kCANCoderId = Ports.BACK_RIGHT_CANCODER;
        backRightModuleConstants.kCANCoderOffsetDegrees = BACK_RIGHT_CANCODER_OFFSET_DEGREES;
        backRightModuleConstants.kWheelDiameter = kMK4_L2iWheelDiameter;
        backRightModuleConstants.kDriveReduction = kMK4_L2iDriveReduction;
        backRightModuleConstants.kSteerReduction = kMK4_L2iSteerReduction;
        backRightModuleConstants.kInvertDrive = true;
        backRightModuleConstants.kInvertSteerMotor = true;
        backRightModuleConstants.kSteerMotorSlot0Kp = STEER_MOTOR_KP;
        backRightModuleConstants.kSteerMotorSlot0Ki = STEER_MOTOR_KI;
        backRightModuleConstants.kSteerMotorSlot0Kd = STEER_MOTOR_KD;
        backRightModuleConstants.kSteerMotorSlot0Kf = STEER_MOTOR_KF;

        return backRightModuleConstants;
    }

    @Override
    public ImuType getImuType() {
        return ImuType.PIGEON2;
    }
}
