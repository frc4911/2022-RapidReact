package frc.robot.config;

import edu.wpi.first.math.util.Units;
import frc.robot.Ports;
import frc.robot.subsystems.SwerveConfiguration;
import frc.robot.subsystems.SwerveDriveModule;

import static frc.robot.Constants.kMK2_SteerReduction;
import static frc.robot.Constants.kMK2_WheelDiameter;

// New Swerve requires SI units

public class DeadEye {
    public static final SwerveConfiguration kSwerveConfiguration = new SwerveConfiguration(
            Units.inchesToMeters(21.0),
            Units.inchesToMeters(21.0),
            Units.feetToMeters(14.2),
            Math.toRadians(270),
            .018,0,0,0 //kSwerveHeadingKp,kSwerveHeadingKi,kSwerveHeadingKp,kSwerveHeadingKf
    );

    public static final SwerveDriveModule.SwerveModuleConstants kFrontRightModuleConstants = new SwerveDriveModule.SwerveModuleConstants();

    static {
        kFrontRightModuleConstants.kName = "Front Right";
        kFrontRightModuleConstants.kDriveMotorTalonId = Ports.FRONT_RIGHT_DRIVE;
        kFrontRightModuleConstants.kSteerMotorTalonId = Ports.FRONT_RIGHT_STEER;
        kFrontRightModuleConstants.kSteerMotorEncoderHomeOffset = 883.0;
        kFrontRightModuleConstants.kCANCoderId = Ports.FRONT_RIGHT_CANCODER;
        kFrontRightModuleConstants.kCANCoderOffsetDegrees = 0;
        kFrontRightModuleConstants.kWheelDiameter = kMK2_WheelDiameter;
        kFrontRightModuleConstants.kDriveReduction = (16.0 / 40.0) * (26.0 / 18.0) * (15.0 / 60.0);
        kFrontRightModuleConstants.kSteerReduction = kMK2_SteerReduction;
        /* ... */
    }

    public static final SwerveDriveModule.SwerveModuleConstants kFrontLeftModuleConstants = new SwerveDriveModule.SwerveModuleConstants();

    static {
        kFrontLeftModuleConstants.kName = "Front Left";
        kFrontLeftModuleConstants.kDriveMotorTalonId = Ports.FRONT_LEFT_DRIVE;
        kFrontLeftModuleConstants.kSteerMotorTalonId = Ports.FRONT_LEFT_STEER;
        kFrontLeftModuleConstants.kSteerMotorEncoderHomeOffset = 1683.0;
        kFrontLeftModuleConstants.kCANCoderId = Ports.FRONT_LEFT_CANCODER;
        kFrontLeftModuleConstants.kCANCoderOffsetDegrees = 0;
        kFrontLeftModuleConstants.kWheelDiameter = kMK2_WheelDiameter;
        kFrontLeftModuleConstants.kDriveReduction = (16.0 / 40.0) * (26.0 / 18.0) * (15.0 / 60.0);
        kFrontLeftModuleConstants.kSteerReduction = kMK2_SteerReduction;
        /* ... */
    }

    public static final SwerveDriveModule.SwerveModuleConstants kBackLeftModuleConstants = new SwerveDriveModule.SwerveModuleConstants();

    static {
        kBackLeftModuleConstants.kName = "Back Left";
        kBackLeftModuleConstants.kDriveMotorTalonId = Ports.BACK_LEFT_DRIVE;
        kBackLeftModuleConstants.kSteerMotorTalonId = Ports.BACK_LEFT_STEER;
        kBackLeftModuleConstants.kSteerMotorEncoderHomeOffset = 3451.0;
        kBackLeftModuleConstants.kCANCoderId = Ports.BACK_LEFT_CANCODER;
        kBackLeftModuleConstants.kCANCoderOffsetDegrees = 0;
        kBackLeftModuleConstants.kWheelDiameter = kMK2_WheelDiameter;
        kBackLeftModuleConstants.kDriveReduction = (16.0 / 40.0) * (26.0 / 18.0) * (15.0 / 60.0);
        kBackLeftModuleConstants.kSteerReduction = kMK2_SteerReduction;
        /* ... */
    }

    public static final SwerveDriveModule.SwerveModuleConstants kBackRightModuleConstants = new SwerveDriveModule.SwerveModuleConstants();

    static {
        kBackRightModuleConstants.kName = "Back Right";
        kBackRightModuleConstants.kDriveMotorTalonId = Ports.BACK_RIGHT_DRIVE;
        kBackRightModuleConstants.kSteerMotorTalonId = Ports.BACK_RIGHT_STEER;
        kBackRightModuleConstants.kSteerMotorEncoderHomeOffset = -327.0;
        kBackRightModuleConstants.kCANCoderId = Ports.BACK_RIGHT_CANCODER;
        kBackRightModuleConstants.kCANCoderOffsetDegrees = 0;
        kBackRightModuleConstants.kWheelDiameter = kMK2_WheelDiameter;
        kBackRightModuleConstants.kDriveReduction = (16.0 / 40.0) * (26.0 / 18.0) * (15.0 / 60.0);
        kBackRightModuleConstants.kSteerReduction = kMK2_SteerReduction;
        /* ... */
    }
}
