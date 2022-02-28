package frc.robot.config;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public class SwerveModuleConfiguration {
    public String kName = "Name";
    public int kModuleId = -1;
    public int kDriveMotorTalonId = -1;
    public int kSteerMotorTalonId = -1;

    // Default steer reduction is Mk4_L2i value
    public double kSteerReduction = (14.0 / 50.0) * (10.0 / 60.0); // 1/21.43
    public double kSteerTicksPerUnitDistance = (1.0 / 2048.0) * kSteerReduction * (2.0 * Math.PI);
    public double kSteerTicksPerUnitVelocity = kSteerTicksPerUnitDistance * 10; // Motor controller unit is ticks per
                                                                                // 100 ms

    // general Steer Motor
    public boolean kInvertSteerMotor = false;
    public boolean kInvertSteerMotorSensorPhase = true;
    public NeutralMode kSteerMotorInitNeutralMode = NeutralMode.Coast; // neutral mode could change
    public double kSteerMotorTicksPerRadian = (2048.0 / kSteerReduction) / (2.0 * Math.PI); // for steer motor
    public double kSteerMotorTicksPerRadianPerSecond = kSteerMotorTicksPerRadian / 10; // for steer motor
    public double kSteerMotorEncoderHomeOffset = 0;

    // Steer CANCoder
    public int kCANCoderId = -1;
    public SensorInitializationStrategy kCANCoderSensorInitializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    public int kCANCoderStatusFramePeriodVbatAndFaults = 255;
    public int kCANCoderStatusFramePeriodSensorData = 255;
    public double kCANCoderOffsetDegrees = 0.0;

    // Steer Motor motion
    public double kSteerMotorSlot0Kp = 0.4;
    public double kSteerMotorSlot0Ki = 0.0;
    public double kSteerMotorSlot0Kd = 0.0;
    public double kSteerMotorSlot0Kf = 0.0;
    public int kSteerMotorSlot0IZone = 25;
    public int kSteerMotorSlot0CruiseVelocity = 1698;
    public int kSteerMotorSlot0Acceleration = 20379; // 12 * kSteerMotorCruiseVelocity
    public int kSteerMotorClosedLoopAllowableError = 5;

    // Steer Motor current/voltage
    public int kSteerMotorContinuousCurrentLimit = 20; // amps
    public int kSteerMotorPeakCurrentLimit = 60; // amps
    public int kSteerMotorPeakCurrentDuration = 200; // ms
    public boolean kSteerMotorEnableCurrentLimit = false;
    public double kSteerMotorMaxVoltage = 7.0; // volts
    public boolean kSteerMotorEnableVoltageCompensation = false;
    public int kSteerMotorVoltageMeasurementFilter = 8; // # of samples in rolling average

    // Steer Motor measurement
    public int kSteerMotorStatusFrame2UpdateRate = 10; // feedback for selected sensor, ms
    public int kSteerMotorStatusFrame10UpdateRate = 10; // motion magic, ms
    // dt for velocity measurements, ms
    public SensorVelocityMeasPeriod kSteerMotorVelocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms; 
    public int kSteerMotorVelocityMeasurementWindow = 64; // # of samples in rolling average

    // general drive
    public boolean kInvertDrive = true;
    public boolean kInvertDriveSensorPhase = false;
    public NeutralMode kDriveInitNeutralMode = NeutralMode.Brake; // neutral mode could change
    // Default wheel diameter and drive reduction to Mk4_L2i values which are in SI
    // units
    public double kWheelDiameter = 0.10033; // Probably should tune for each individual wheel maybe
    public double kDriveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public double kDriveTicksPerUnitDistance = (1.0 / 2048.0) * kDriveReduction * (Math.PI * kWheelDiameter);
    public double kDriveTicksPerUnitVelocity = kDriveTicksPerUnitDistance * 10; // Motor controller unit is ticks per
                                                                                // 100 ms
    public double kDriveDeadband = 0.01;

    // drive current/voltage
    public int kDriveContinuousCurrentLimit = 30; // amps
    public int kDrivePeakCurrentLimit = 50; // amps
    public int kDrivePeakCurrentDuration = 200; // ms
    public boolean kDriveEnableCurrentLimit = false;
    public double kDriveMaxVoltage = 12.0; // 10 //volts
    public double kDriveNominalVoltage = 0.0; // volts
    public int kDriveVoltageMeasurementFilter = 8; // # of samples in rolling average

    // drive measurement
    public int kDriveStatusFrame2UpdateRate = 15; // feedback for selected sensor, ms
    public int kDriveStatusFrame10UpdateRate = 200; // motion magic, ms
    // dt for velocity measurements, ms
    public SensorVelocityMeasPeriod kDriveMotorVelocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
    public int kDriveVelocityMeasurementWindow = 32; // # of samples in rolling average
}