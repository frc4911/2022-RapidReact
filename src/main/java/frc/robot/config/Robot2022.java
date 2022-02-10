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
        kFrontRightModuleConstants.kWheelDiameter = kMK4_L2iWheelDiameter;
        kFrontRightModuleConstants.kDriveReduction = kMK4_L2iDriveReduction;
        kFrontRightModuleConstants.kSteerReduction = kMK4_L2iSteerReduction;
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
        kFrontLeftModuleConstants.kWheelDiameter = kMK4_L2iWheelDiameter;
        kFrontLeftModuleConstants.kDriveReduction = kMK4_L2iDriveReduction;
        kFrontLeftModuleConstants.kSteerReduction = kMK4_L2iSteerReduction;
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
        kBackLeftModuleConstants.kWheelDiameter = kMK4_L2iWheelDiameter;
        kBackLeftModuleConstants.kDriveReduction = kMK4_L2iDriveReduction;
        kBackLeftModuleConstants.kSteerReduction = kMK4_L2iSteerReduction;
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
        kBackRightModuleConstants.kWheelDiameter = kMK4_L2iWheelDiameter;
        kBackRightModuleConstants.kDriveReduction = kMK4_L2iDriveReduction;
        kBackRightModuleConstants.kSteerReduction = kMK4_L2iSteerReduction;
        /* ... */
    }
}
