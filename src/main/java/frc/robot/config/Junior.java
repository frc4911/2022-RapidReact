package frc.robot.config;

import edu.wpi.first.math.util.Units;
import frc.robot.Ports;
import frc.robot.subsystems.SwerveConfiguration;
import frc.robot.subsystems.SwerveDriveModule;

import static frc.robot.Constants.kMK2_SteerReduction;
import static frc.robot.Constants.kMK2_WheelDiameter;

// New Swerve requires SI units

public class Junior {
    public static final SwerveConfiguration kSwerveConfiguration = new SwerveConfiguration(
            Units.inchesToMeters(14.5),
            Units.inchesToMeters(14.5),
            Units.feetToMeters(14.2),
            Math.toRadians(270),
            .018,0,0,0 //kSwerveHeadingKp,kSwerveHeadingKi,kSwerveHeadingKp,kSwerveHeadingKf

    );

    public static final SwerveDriveModule.SwerveModuleConstants kFrontRightModuleConstants = new SwerveDriveModule.SwerveModuleConstants();

    static {
        kFrontRightModuleConstants.kName = "Front Right";
        kFrontRightModuleConstants.kModuleId = 0;
        kFrontRightModuleConstants.kDriveMotorTalonId = Ports.FRONT_RIGHT_DRIVE;
        kFrontRightModuleConstants.kSteerMotorTalonId = Ports.FRONT_RIGHT_STEER;
        kFrontRightModuleConstants.kSteerMotorEncoderHomeOffset = 883.0;
        kFrontRightModuleConstants.kCANCoderOffsetDegrees = 80;
        kFrontRightModuleConstants.kCANCoderId = Ports.FRONT_RIGHT_CANCODER;
        kFrontRightModuleConstants.kWheelDiameter = kMK2_WheelDiameter;
        kFrontRightModuleConstants.kDriveReduction = (18.0 / 38.0) * (26.0 / 18.0) * (15.0 / 60.0);
        kFrontRightModuleConstants.kSteerReduction = kMK2_SteerReduction;
        kFrontRightModuleConstants.kSteerMotorSlot0Kp = 0.4;
        kFrontRightModuleConstants.kSteerMotorSlot0Ki = 0.0;
        kFrontRightModuleConstants.kSteerMotorSlot0Kd = 0.0;
        kFrontRightModuleConstants.kSteerMotorSlot0Kf = 0.0;

        /* ... */
    }

    public static final SwerveDriveModule.SwerveModuleConstants kFrontLeftModuleConstants = new SwerveDriveModule.SwerveModuleConstants();

    static {
        kFrontLeftModuleConstants.kName = "Front Left";
        kFrontLeftModuleConstants.kModuleId = 1;
        kFrontLeftModuleConstants.kDriveMotorTalonId = Ports.FRONT_LEFT_DRIVE;
        kFrontLeftModuleConstants.kSteerMotorTalonId = Ports.FRONT_LEFT_STEER;
        kFrontLeftModuleConstants.kSteerMotorEncoderHomeOffset = 1683.0;
        kFrontLeftModuleConstants.kCANCoderOffsetDegrees = -28;
        kFrontLeftModuleConstants.kCANCoderId = Ports.FRONT_LEFT_CANCODER;
        kFrontLeftModuleConstants.kWheelDiameter = kMK2_WheelDiameter;
        kFrontLeftModuleConstants.kDriveReduction = (18.0 / 38.0) * (26.0 / 18.0) * (15.0 / 60.0);
        kFrontLeftModuleConstants.kSteerReduction = kMK2_SteerReduction;
        kFrontLeftModuleConstants.kSteerMotorSlot0Kp = 0.4;
        kFrontLeftModuleConstants.kSteerMotorSlot0Ki = 0.0;
        kFrontLeftModuleConstants.kSteerMotorSlot0Kd = 0.0;
        kFrontLeftModuleConstants.kSteerMotorSlot0Kf = 0.0;
        /* ... */
    }

    public static final SwerveDriveModule.SwerveModuleConstants kBackLeftModuleConstants = new SwerveDriveModule.SwerveModuleConstants();

    static {
        kBackLeftModuleConstants.kName = "Back Left";
        kBackLeftModuleConstants.kModuleId = 2;
        kBackLeftModuleConstants.kDriveMotorTalonId = Ports.BACK_LEFT_DRIVE;
        kBackLeftModuleConstants.kSteerMotorTalonId = Ports.BACK_LEFT_STEER;
        kBackLeftModuleConstants.kSteerMotorEncoderHomeOffset = 3451.0;
        kBackLeftModuleConstants.kCANCoderOffsetDegrees = 146;
        kBackLeftModuleConstants.kCANCoderId = Ports.BACK_LEFT_CANCODER;
        kBackLeftModuleConstants.kWheelDiameter = kMK2_WheelDiameter;
        kBackLeftModuleConstants.kDriveReduction = (18.0 / 38.0) * (26.0 / 18.0) * (15.0 / 60.0);
        kBackLeftModuleConstants.kSteerReduction = kMK2_SteerReduction;
        kBackLeftModuleConstants.kSteerMotorSlot0Kp = 0.4;
        kBackLeftModuleConstants.kSteerMotorSlot0Ki = 0.0;
        kBackLeftModuleConstants.kSteerMotorSlot0Kd = 0.0;
        kBackLeftModuleConstants.kSteerMotorSlot0Kf = 0.0;
        /* ... */
    }

    public static final SwerveDriveModule.SwerveModuleConstants kBackRightModuleConstants = new SwerveDriveModule.SwerveModuleConstants();

    static {
        kBackRightModuleConstants.kName = "Back Right";
        kBackRightModuleConstants.kModuleId = 3;
        kBackRightModuleConstants.kDriveMotorTalonId = Ports.BACK_RIGHT_DRIVE;
        kBackRightModuleConstants.kSteerMotorTalonId = Ports.BACK_RIGHT_STEER;
        kBackRightModuleConstants.kSteerMotorEncoderHomeOffset = -327.0;
        kBackRightModuleConstants.kCANCoderId = Ports.BACK_RIGHT_CANCODER;
        kBackRightModuleConstants.kCANCoderOffsetDegrees = -24;
        kBackRightModuleConstants.kWheelDiameter = kMK2_WheelDiameter;
        kBackRightModuleConstants.kDriveReduction = (18.0 / 38.0) * (26.0 / 18.0) * (15.0 / 60.0);
        kBackRightModuleConstants.kSteerReduction = kMK2_SteerReduction;
        kBackRightModuleConstants.kSteerMotorSlot0Kp = 0.4;
        kBackRightModuleConstants.kSteerMotorSlot0Ki = 0.0;
        kBackRightModuleConstants.kSteerMotorSlot0Kd = 0.0;
        kBackRightModuleConstants.kSteerMotorSlot0Kf = 0.0;
        /* ... */
    }
}
