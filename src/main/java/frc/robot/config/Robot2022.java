package frc.robot.config;

import edu.wpi.first.math.util.Units;
import frc.robot.Ports;
import frc.robot.subsystems.SwerveConfiguration;
import frc.robot.subsystems.SwerveDriveModule;

import static frc.robot.Constants.*;

// New Swerve requires SI units

public class Robot2022 {
    public static final SwerveConfiguration kSwerveConfiguration = new SwerveConfiguration(
            Units.inchesToMeters(23.75),
            Units.inchesToMeters(20.75),
            Units.feetToMeters(1.0), //Max speed in feet per second: Theoretical max is 17.065 ft per second
            Math.toRadians(270), //Max change in degrees per second
            .018,0,0,0 //kSwerveHeadingKp,kSwerveHeadingKi,kSwerveHeadingKp,kSwerveHeadingKf
    );

    public static final SwerveDriveModule.SwerveModuleConstants kFrontRightModuleConstants = new SwerveDriveModule.SwerveModuleConstants();

    static {
        kFrontRightModuleConstants.kName = "Front Right";
        kFrontRightModuleConstants.kDriveMotorTalonId = Ports.ROBOT_2022_FRONT_RIGHT_DRIVE;
        kFrontRightModuleConstants.kSteerMotorTalonId = Ports.ROBOT_2022_FRONT_RIGHT_STEER;
        kFrontRightModuleConstants.kSteerMotorEncoderHomeOffset = 883.0;
        kFrontRightModuleConstants.kCANCoderId = Ports.FRONT_RIGHT_CANCODER;
        kFrontRightModuleConstants.kCANCoderOffsetDegrees = 358;
        kFrontRightModuleConstants.kWheelDiameter = kMK4_L2iWheelDiameter;
        kFrontRightModuleConstants.kDriveReduction = kMK4_L2iDriveReduction;
        kFrontRightModuleConstants.kSteerReduction = kMK4_L2iSteerReduction;
        kFrontRightModuleConstants.kInvertDrive = true;
        kFrontRightModuleConstants.kInvertSteerMotor = true;
        kFrontRightModuleConstants.kSteerMotorSlot0Kp = 0.25;
        kFrontRightModuleConstants.kSteerMotorSlot0Ki = 0.0;
        kFrontRightModuleConstants.kSteerMotorSlot0Kd = 0.0;
        kFrontRightModuleConstants.kSteerMotorSlot0Kf = 0.0;
        /* ... */
    }

    public static final SwerveDriveModule.SwerveModuleConstants kFrontLeftModuleConstants = new SwerveDriveModule.SwerveModuleConstants();

    static {
        kFrontLeftModuleConstants.kName = "Front Left";
        kFrontLeftModuleConstants.kDriveMotorTalonId = Ports.ROBOT_2022_FRONT_LEFT_DRIVE;
        kFrontLeftModuleConstants.kSteerMotorTalonId = Ports.ROBOT_2022_FRONT_LEFT_STEER;
        kFrontLeftModuleConstants.kSteerMotorEncoderHomeOffset = 1683.0;
        kFrontLeftModuleConstants.kCANCoderId = Ports.FRONT_LEFT_CANCODER;
        kFrontLeftModuleConstants.kCANCoderOffsetDegrees = 210;
        kFrontLeftModuleConstants.kWheelDiameter = kMK4_L2iWheelDiameter;
        kFrontLeftModuleConstants.kDriveReduction = kMK4_L2iDriveReduction;
        kFrontLeftModuleConstants.kSteerReduction = kMK4_L2iSteerReduction;
        kFrontLeftModuleConstants.kInvertDrive = true;
        kFrontLeftModuleConstants.kInvertSteerMotor = true;
        kFrontLeftModuleConstants.kSteerMotorSlot0Kp = 0.25;
        kFrontLeftModuleConstants.kSteerMotorSlot0Ki = 0.0;
        kFrontLeftModuleConstants.kSteerMotorSlot0Kd = 0.0;
        kFrontLeftModuleConstants.kSteerMotorSlot0Kf = 0.0;
        /* ... */
    }

    public static final SwerveDriveModule.SwerveModuleConstants kBackLeftModuleConstants = new SwerveDriveModule.SwerveModuleConstants();

    static {
        kBackLeftModuleConstants.kName = "Back Left";
        kBackLeftModuleConstants.kDriveMotorTalonId = Ports.ROBOT_2022_BACK_LEFT_DRIVE;
        kBackLeftModuleConstants.kSteerMotorTalonId = Ports.ROBOT_2022_BACK_LEFT_STEER;
        kBackLeftModuleConstants.kSteerMotorEncoderHomeOffset = 3451.0;
        kBackLeftModuleConstants.kCANCoderId = Ports.BACK_LEFT_CANCODER;
        kBackLeftModuleConstants.kCANCoderOffsetDegrees = 278;
        kBackLeftModuleConstants.kWheelDiameter = kMK4_L2iWheelDiameter;
        kBackLeftModuleConstants.kDriveReduction = kMK4_L2iDriveReduction;
        kBackLeftModuleConstants.kSteerReduction = kMK4_L2iSteerReduction;
        kBackLeftModuleConstants.kInvertDrive = true;
        kBackLeftModuleConstants.kInvertSteerMotor = true;
        kBackLeftModuleConstants.kSteerMotorSlot0Kp = 0.25;
        kBackLeftModuleConstants.kSteerMotorSlot0Ki = 0.0;
        kBackLeftModuleConstants.kSteerMotorSlot0Kd = 0.0;
        kBackLeftModuleConstants.kSteerMotorSlot0Kf = 0.0;
        /* ... */
    }

    public static final SwerveDriveModule.SwerveModuleConstants kBackRightModuleConstants = new SwerveDriveModule.SwerveModuleConstants();

    static {
        kBackRightModuleConstants.kName = "Back Right";
        kBackRightModuleConstants.kDriveMotorTalonId = Ports.ROBOT_2022_BACK_RIGHT_DRIVE;
        kBackRightModuleConstants.kSteerMotorTalonId = Ports.ROBOT_2022_BACK_RIGHT_STEER;
        kBackRightModuleConstants.kSteerMotorEncoderHomeOffset = -327.0;
        kBackRightModuleConstants.kCANCoderId = Ports.BACK_RIGHT_CANCODER;
        kBackRightModuleConstants.kCANCoderOffsetDegrees = 116;
        kBackRightModuleConstants.kWheelDiameter = kMK4_L2iWheelDiameter;
        kBackRightModuleConstants.kDriveReduction = kMK4_L2iDriveReduction;
        kBackRightModuleConstants.kSteerReduction = kMK4_L2iSteerReduction;
        kBackRightModuleConstants.kInvertDrive = true;
        kBackRightModuleConstants.kInvertSteerMotor = true;
        kBackRightModuleConstants.kSteerMotorSlot0Kp = 0.25;
        kBackRightModuleConstants.kSteerMotorSlot0Ki = 0.0;
        kBackRightModuleConstants.kSteerMotorSlot0Kd = 0.0;
        kBackRightModuleConstants.kSteerMotorSlot0Kf = 0.0;
        /* ... */
    }
}
