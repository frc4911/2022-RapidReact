package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.util.Units;
import libraries.cheesylib.util.Util;
import libraries.cyberlib.kinematics.SwerveModuleState;
import libraries.cyberlib.utils.CheckFaults;

import frc.robot.Constants;
import libraries.cheesylib.drivers.TalonFXFactory;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents a swerve module consisting of a drive motor that controls the speed of the wheel and
 * a steer motor that controls the angle the wheel is pointing towards relative to the chassis frame.
 */
public class SwerveDriveModule extends Subsystem {
    // Hardware
    TalonFX mSteerMotor, mDriveMotor;
    CANCoder mCANCoder;

	public enum ControlState {
		/**
		 * No motor outputs.  This state is used robot is powered up and stopped by Driver Station.
		 */
		NEUTRAL,
		/**
		 * Runs motors in response to inputs.
		 */
		OPEN_LOOP
	}

    String mModuleName;
    private final CheckFaults mCheckFaults = new CheckFaults();

	public static class SwerveModuleConstants {
		public String kName = "Name";
        public int kModuleId = -1;
		public int kDriveMotorTalonId = -1;
		public int kSteerMotorTalonId = -1;

		// general Steer Motor
		public boolean kInvertSteerMotor = true;
		public boolean kInvertSteerMotorSensorPhase = true;
		public NeutralMode kSteerMotorInitNeutralMode = NeutralMode.Brake; // neutral mode could change
        public double kSteerMotorTicksPerRadian = 2048.0 / (2 * Math.PI); // for steer motor
        public double kSteerMotorTicksPerRadianPerSecond = kSteerMotorTicksPerRadian / 10; // for steer motor
		public double kSteerMotorEncoderHomeOffset = 0;

		// Steer CANCoder
		public int kCANCoderId = -1;
		public SensorInitializationStrategy kCANCoderSensorInitializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
		public int kCANCoderStatusFramePeriodVbatAndFaults = 255;
		public int kCANCoderStatusFramePeriodSensorData = 255;
		public double kCANCoderOffsetDegrees = 0.0;

		// CANCoder Motor motion
		public double kCANCoderSteerMotorKp = 0.1;
		public double kCANCoderSteerMotorKi = 0.0;
		public double kCANCoderSteerMotorKd = 0.84;
		public double kCANCoderSteerMotorKf = 0.05;
		public int kCANCoderSteerMotorIZone = 25;
		public int kCANCoderSteerMotorCruiseVelocity = 1698;
		public int kCANCoderSteerMotorAcceleration = 20379; // 12 * kSteerMotorCruiseVelocity
		public int kCANCoderSteerMotorClosedLoopAllowableError = 5;

		// Steer Motor motion
		public double kSteerMotorSlot0Kp = 0.07;
		public double kSteerMotorSlot0Ki = 0.0;
		public double kSteerMotorSlot0Kd = 0.84;
		public double kSteerMotorSlot0Kf = 0.025;
		public int kSteerMotorSlot0IZone = 25;
		public int kSteerMotorSlot0CruiseVelocity = 1698;
		public int kSteerMotorSlot0Acceleration = 20379; // 12 * kSteerMotorCruiseVelocity
		public int kSteerMotorClosedLoopAllowableError = 5;

		public double kSteerMotorSlot1Kp = 0.07;
		public double kSteerMotorSlot1Ki = 0.0;
		public double kSteerMotorSlot1Kd = 0.84;
		public double kSteerMotorSlot1Kf = 0.025;
		public int kSteerMotorSlot1IZone = 25;
		public int kSteerMotorSlot1CruiseVelocity = 1698;
		public int kSteerMotorSlot1Acceleration = 20379; // 12 * kSteerMotorCruiseVelocity

		// Steer Motor current/voltage
		public int kSteerMotorContinuousCurrentLimit = 30; // amps
		public int kSteerMotorPeakCurrentLimit = 60; // amps
		public int kSteerMotorPeakCurrentDuration = 200; // ms
		public boolean kSteerMotorEnableCurrentLimit = true;
		public double kSteerMotorMaxVoltage = 7.0; // volts
		public boolean kSteerMotorEnableVoltageCompensation = false;
		public int kSteerMotorVoltageMeasurementFilter = 8; // # of samples in rolling average

		// Steer Motor measurement
		public int kSteerMotorStatusFrame2UpdateRate = 10; // feedback for selected sensor, ms
		public int kSteerMotorStatusFrame10UpdateRate = 10; // motion magic, ms
//		public VelocityMeasPeriod kSteerMotorVelocityMeasurementPeriod = VelocityMeasPeriod.Period_100Ms; // dt for velocity measurements, ms
		public int kSteerMotorVelocityMeasurementWindow = 64; // # of samples in rolling average

		// general drive
		public boolean kInvertDrive = true;
		public boolean kInvertDriveSensorPhase = false;
		public NeutralMode kDriveInitNeutralMode = NeutralMode.Brake; // neutral mode could change
        // Default wheel diameter and drive reduction to Mk4_L2i values which are in SI units
        public double kWheelDiameter = 0.10033; // Probably should tune for each individual wheel maybe
        public double kDriveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
		public double kDriveTicksPerUnitDistance = (1.0 / 2048.0) * kDriveReduction * (Math.PI * kWheelDiameter);
        public double kDriveTicksPerUnitVelocity = kDriveTicksPerUnitDistance * 10;  // Motor controller unit is ticks per 100 ms
		public double kDriveDeadband = 0.01;

		// drive current/voltage
		public int kDriveContinuousCurrentLimit = 30; // amps
		public int kDrivePeakCurrentLimit = 50; // amps
		public int kDrivePeakCurrentDuration = 200; // ms
		public boolean kDriveEnableCurrentLimit = true;
		public double kDriveMaxVoltage = 12.0; // 10 //volts
		public double kDriveNominalVoltage = 0.0; //volts
		public int kDriveVoltageMeasurementFilter = 8; // # of samples in rolling average

		// drive measurement
		public int kDriveStatusFrame2UpdateRate = 15; // feedback for selected sensor, ms
		public int kDriveStatusFrame10UpdateRate = 200; // motion magic, ms
//		public VelocityMeasPeriod kDriveVelocityMeasurementPeriod = VelocityMeasPeriod.Period_10Ms; // dt for velocity measurements, ms
		public int kDriveVelocityMeasurementWindow = 32; // # of samples in rolling average
	}

	private final boolean mLoggingEnabled = true;    // used to disable logging for this subsystem only
	private final CheckFaults mFaultChecker = new CheckFaults();

//	boolean tenVoltSteerMode = false;
//	private boolean standardCarpetDirection = true;

	private final PeriodicIO mPeriodicIO = new PeriodicIO();
	private ControlState mControlState = ControlState.NEUTRAL;
	private final SwerveModuleConstants mConstants;

    public SwerveDriveModule(SwerveModuleConstants constants) {
        mConstants = constants;
        mModuleName = String.format("%s %d", mConstants.kName, mConstants.kModuleId);
        mPeriodicIO.moduleID = mConstants.kModuleId;

        mDriveMotor = TalonFXFactory.createDefaultTalon(constants.kDriveMotorTalonId);
        mSteerMotor = TalonFXFactory.createDefaultTalon(constants.kSteerMotorTalonId);

        mCANCoder = new CANCoder(constants.kCANCoderId);
        mCANCoder.configSensorInitializationStrategy(constants.kCANCoderSensorInitializationStrategy, Constants.kLongCANTimeoutMs);
        mCANCoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, constants.kCANCoderStatusFramePeriodVbatAndFaults);
        mCANCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, constants.kCANCoderStatusFramePeriodSensorData);

        configureMotors();
    }

    /**
     * Configure motors based on current SwerveModuleConstants.
     */
    private void configureMotors() {
        mSteerMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mSteerMotor.setSensorPhase(mConstants.kInvertSteerMotorSensorPhase);
        mSteerMotor.setInverted(mConstants.kInvertSteerMotor);
        mSteerMotor.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mSteerMotor.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        // multiple (usually 2) sets were needed to set new encoder value
        double fxCurrentEnc = mSteerMotor.getSelectedSensorPosition(0);
        System.out.println(mConstants.kName + " current fx encoder ticks: " + fxCurrentEnc);
        double cancoderDegrees = mCANCoder.getAbsolutePosition();
        System.out.println(mConstants.kName + " cancoder degrees-offset: " + cancoderDegrees +
                "-" + mConstants.kCANCoderOffsetDegrees + "=" + (cancoderDegrees - mConstants.kCANCoderOffsetDegrees));
        int startEncoderValue = degreesToEncUnits(cancoderDegrees - mConstants.kCANCoderOffsetDegrees);

        int count = 0;
        while (Math.abs(fxCurrentEnc-startEncoderValue) > 10 && count < 5) {
            count++;
            mSteerMotor.setSelectedSensorPosition(startEncoderValue, 0, Constants.kLongCANTimeoutMs);
            Timer.delay(.1);
            fxCurrentEnc = mSteerMotor.getSelectedSensorPosition();
        }

        System.out.println(mConstants.kName + " loops needed to set fx encoder: " + count);

        mSteerMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, mConstants.kSteerMotorStatusFrame2UpdateRate, Constants.kLongCANTimeoutMs);
        mSteerMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, mConstants.kSteerMotorStatusFrame10UpdateRate, Constants.kLongCANTimeoutMs);
        mSteerMotor.setNeutralMode(mConstants.kSteerMotorInitNeutralMode);
        mSteerMotor.configNominalOutputForward(mConstants.kDriveNominalVoltage, Constants.kLongCANTimeoutMs);
        mSteerMotor.configNominalOutputReverse(0.0, Constants.kLongCANTimeoutMs);
        mSteerMotor.configVoltageCompSaturation(0.0, Constants.kLongCANTimeoutMs);
        mSteerMotor.enableVoltageCompensation(mConstants.kSteerMotorEnableVoltageCompensation);
        mSteerMotor.configAllowableClosedloopError(0, mConstants.kSteerMotorClosedLoopAllowableError, Constants.kLongCANTimeoutMs);

        // TODO: Set these correctly
        mSteerMotor.configMotionAcceleration(Constants.kSwerveRotationMaxSpeed * 12.5, Constants.kLongCANTimeoutMs);
        mSteerMotor.configMotionCruiseVelocity(Constants.kSwerveRotationMaxSpeed, Constants.kLongCANTimeoutMs);
//        mSteerMotor.configVelocityMeasurementPeriod(mConstants.kSteerMotorVelocityMeasurementPeriod, Constants.kLongCANTimeoutMs);
        mSteerMotor.configVelocityMeasurementWindow(mConstants.kSteerMotorVelocityMeasurementWindow, Constants.kLongCANTimeoutMs);
        mSteerMotor.selectProfileSlot(0, 0);

        //Slot 1 is for normal use (tuned for fx integrated encoder)
        mSteerMotor.config_kP(0, mConstants.kSteerMotorSlot0Kp, Constants.kLongCANTimeoutMs);
        mSteerMotor.config_kI(0, mConstants.kSteerMotorSlot0Ki, Constants.kLongCANTimeoutMs);
        mSteerMotor.config_kD(0, mConstants.kSteerMotorSlot0Kd, Constants.kLongCANTimeoutMs);
        mSteerMotor.config_kF(0, mConstants.kSteerMotorSlot0Kf, Constants.kLongCANTimeoutMs);
        mSteerMotor.config_IntegralZone(0, mConstants.kSteerMotorSlot0IZone, Constants.kLongCANTimeoutMs);

        //Slot 2 is reserved for the beginning of auto (tuned for cancoders needs retune)
        // mSteerMotor.config_kP(1, 0.07, 10);
        // mSteerMotor.config_kI(1, 0.0, 10);
        // mSteerMotor.config_kD(1, 0.84, 10);
        // mSteerMotor.config_kF(1, 0.05, 10);
        mSteerMotor.config_kP(1, mConstants.kSteerMotorSlot1Kp, Constants.kLongCANTimeoutMs); // TODO: test this
        mSteerMotor.config_kI(1, mConstants.kSteerMotorSlot1Ki, Constants.kLongCANTimeoutMs);
        mSteerMotor.config_kD(1, mConstants.kSteerMotorSlot1Kd, Constants.kLongCANTimeoutMs);
        mSteerMotor.config_kF(1, mConstants.kSteerMotorSlot1Kf, Constants.kLongCANTimeoutMs);
        mSteerMotor.config_IntegralZone(1, mConstants.kSteerMotorSlot1IZone, Constants.kLongCANTimeoutMs);
        mSteerMotor.set(ControlMode.MotionMagic, mSteerMotor.getSelectedSensorPosition(0));

        mDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mDriveMotor.setSelectedSensorPosition(0, 0, Constants.kLongCANTimeoutMs);
        mDriveMotor.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mDriveMotor.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mDriveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, mConstants.kDriveStatusFrame2UpdateRate, Constants.kLongCANTimeoutMs);
//        mDriveMotor.configVelocityMeasurementPeriod(mConstants.kDriveVelocityMeasurementPeriod, Constants.kLongCANTimeoutMs);
        mDriveMotor.configVelocityMeasurementWindow(mConstants.kDriveVelocityMeasurementWindow, Constants.kLongCANTimeoutMs);
        mDriveMotor.configNominalOutputForward(0.0, Constants.kLongCANTimeoutMs);
        mDriveMotor.configNominalOutputReverse(0.0, Constants.kLongCANTimeoutMs);
        mDriveMotor.configVoltageCompSaturation(mConstants.kDriveMaxVoltage, Constants.kLongCANTimeoutMs);
        mDriveMotor.enableVoltageCompensation(true);
        mDriveMotor.configOpenloopRamp(0.25, Constants.kLongCANTimeoutMs);
        mDriveMotor.configClosedloopRamp(0.0);
        mDriveMotor.configAllowableClosedloopError(0, 0, Constants.kLongCANTimeoutMs);
        mDriveMotor.setInverted(mConstants.kInvertDrive);
        mDriveMotor.setSensorPhase(mConstants.kInvertDriveSensorPhase);
        mDriveMotor.setNeutralMode(mConstants.kDriveInitNeutralMode);

        // Slot 0 is reserved for MotionMagic
        mDriveMotor.selectProfileSlot(0, 0);
        mDriveMotor.config_kP(0, 2.0, Constants.kLongCANTimeoutMs);
        mDriveMotor.config_kI(0, 0.0, Constants.kLongCANTimeoutMs);
        mDriveMotor.config_kD(0, 24.0, Constants.kLongCANTimeoutMs);
        mDriveMotor.config_kF(0, 1023.0/Constants.kSwerveDriveMaxSpeed, Constants.kLongCANTimeoutMs);

        // TODO: Set these correctly
        mDriveMotor.configMotionCruiseVelocity(Constants.kSwerveDriveMaxSpeed*0.9, Constants.kLongCANTimeoutMs);
        mDriveMotor.configMotionAcceleration(Constants.kSwerveDriveMaxSpeed, Constants.kLongCANTimeoutMs);

        // Slot 1 corresponds to velocity mode (USED FOR AUTO)
        mDriveMotor.config_kP(1, 0.03, Constants.kLongCANTimeoutMs);
        mDriveMotor.config_kI(1, 0.0, Constants.kLongCANTimeoutMs);
        mDriveMotor.config_kD(1, 0.0, Constants.kLongCANTimeoutMs);
        mDriveMotor.config_kF(1, 0.05, Constants.kLongCANTimeoutMs);//0.3

        // if (!isDriveSensorConnected()) {
        // 	DriverStation.reportError(mConstants.kName + "drive encoder not detected!", false);
        // 	hasEmergency = true;
        // }
    }

	/**
	 * Gets the current state of the module.
	 * <p>
	 * @return The current state of the module.
	 */
	public synchronized SwerveModuleState getState() {
        // Convert to encoder readings to SI units.
		return new SwerveModuleState(
                encVelocityToMetersPerSecond(mPeriodicIO.driveDemand),
				new Rotation2d(encUnitsToDegrees(mPeriodicIO.steerPosition)));
	}

	/**
	 * Sets the state for the module.
	 * <p>
	 * @param swreveModuleState Desired state for the module.
	 */
	public synchronized void setState(SwerveModuleState swreveModuleState) {
        // Converts the velocity in SI units (meters per second) to a
        // voltage (as a percentage) for the motor controllers.
		setDriveOpenLoop(metersPerSecondToEncVelocity(swreveModuleState.speedInMetersPerSecond));

        // TODO:  Consider using SwerveModule.optimize() here instead of setReferenceAngle()
        setReferenceAngle(swreveModuleState.angle.getRadians());
	}

    /**
     * Sets the reference angle for the steer motor
     * <p>
     * @param referenceAngleRadians goal angle in radians
     */
    private void setReferenceAngle(double referenceAngleRadians) {
        double currentAngleRadians = encoderUnitsToRadians(mPeriodicIO.steerPosition);

        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Falcon's encoder can go above that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        mPeriodicIO.steerControlMode = ControlMode.MotionMagic;
        mPeriodicIO.steerDemand = radiansToEncoderUnits(adjustedReferenceAngleRadians);
    }

    /**
     * Sets the motor controller settings and values for the steer motor.
     * <p>
     * @param angularVelocityInRadiansPerSecond Normalized value
     */
    private void setSteerOpenLoop(double angularVelocityInRadiansPerSecond) {
        mPeriodicIO.steerControlMode = ControlMode.PercentOutput;
        mPeriodicIO.steerDemand = angularVelocityInRadiansPerSecond;
    }

    /**
     * Sets the motor controller settings and values for the Drive motor.
     * <p>
     * @param velocity Normalized value
     */
    private void setDriveOpenLoop(double velocity) {
        if (mControlState != ControlState.OPEN_LOOP) {
            mControlState = ControlState.OPEN_LOOP;
        }

        mPeriodicIO.driveControlMode = ControlMode.PercentOutput;
        mPeriodicIO.driveDemand = velocity;
    }

    // Steer motor
    private double encoderUnitsToRadians(double ticks) {
        return ticks / mConstants.kSteerMotorTicksPerRadian;
    }

    private double radiansToEncoderUnits(double radians) {
        return radians * mConstants.kSteerMotorTicksPerRadian;
    }

//    private int degreesToEncUnits(double degrees) {
//        return (int) radiansToEncoderUnits(Math.toRadians(degrees));
//    }
//
//    private double encUnitsToDegrees(double encUnits) {
//        return Math.toDegrees(encoderUnitsToRadians(encUnits));
//    }

    // Drive motor
    private double encoderUnitsToDistance(double ticks) {
        return ticks * mConstants.kDriveTicksPerUnitDistance;
    }

    private double encoderUnitsToVelocity(double ticks) {
        return ticks * mConstants.kDriveTicksPerUnitVelocity;
    }

    private double distanceToEncoderUnits(double distanceInMeters) {
        return distanceInMeters / mConstants.kDriveTicksPerUnitDistance;
    }

//    private double encUnitsToInches(double encUnits) {
//        return Units.metersToInches(encoderUnitsToDistance(encUnits));
//    }
//
//    private double inchesToEncUnits(double inches) {
//        return distanceToEncoderUnits(Units.inchesToMeters(inches));
//    }

    private double metersPerSecondToEncVelocity(double metersPerSecond) {
        return metersPerSecond / mConstants.kDriveTicksPerUnitVelocity;
    }

    private double encVelocityToMetersPerSecond(double encUnitsPer100ms) {
        return encUnitsPer100ms * mConstants.kDriveTicksPerUnitVelocity;
    }

    public synchronized void disable() {
        setDriveOpenLoop(0.0);
        setSteerOpenLoop(0.0);
    }

    @Override
    public synchronized void stop() {
        if (mControlState != ControlState.NEUTRAL) {
            mControlState = ControlState.NEUTRAL;
        }

        mDriveMotor.set(ControlMode.PercentOutput, 0.0);
        mSteerMotor.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public String getLogHeaders() {
        if (mLoggingEnabled) {
            mCheckFaults.clearFaults(mDriveMotor);
            mCheckFaults.clearFaults(mSteerMotor);
            String shortName = mModuleName;

            return  shortName+".steerPosition,"+
                    shortName+".drivePosition,"+
                    shortName+".steerControlMode,"+
                    shortName+".driveControlMode,"+
                    shortName+".steerDemand,"+
                    shortName+".driveDemand,"+
                    shortName+".steerCurrent,"+
                    shortName+".driveCurrent,"+
                    shortName+".steerFaults,"+
                    shortName+".driveFaults";
        }
        return null;
    }

    private String generateLogValues(boolean telemetry) {
        String values;

        if (telemetry) {
            mPeriodicIO.steerCurrent = mSteerMotor.getStatorCurrent();
            mPeriodicIO.driveCurrent = mDriveMotor.getStatorCurrent();
            mPeriodicIO.steerFaults = mCheckFaults.getFaults(mSteerMotor);
            mPeriodicIO.driveFaults = mCheckFaults.getFaults(mDriveMotor);

            values = ""+ mPeriodicIO.steerPosition +","+
                    mPeriodicIO.drivePosition+","+
                    mPeriodicIO.steerControlMode +","+
                    mPeriodicIO.driveControlMode+","+
                    mPeriodicIO.steerDemand +","+
                    mPeriodicIO.driveDemand+","+
                    mPeriodicIO.steerCurrent +","+
                    mPeriodicIO.driveCurrent+","+
                    mPeriodicIO.steerFaults +","+
                    mPeriodicIO.driveFaults;
        }
        else{
            values = ""+ mPeriodicIO.steerPosition +","+
                    mPeriodicIO.drivePosition+","+
                    mPeriodicIO.steerControlMode +","+
                    mPeriodicIO.driveControlMode+","+
                    mPeriodicIO.steerDemand +","+
                    mPeriodicIO.driveDemand+","+
                    /*periodicIO.steerCurrent+*/","+
                    /*periodicIO.driveCurrent+*/","+
                    /*mCheckFaults.getFXFaults(steerMotor)+*/","
                    /*mCheckFaults.getFXFaults(driveMotor)*/;
        }
        return values;
    }

    @Override
    public String getLogValues(boolean telemetry) {
        if (mLoggingEnabled) {
            return generateLogValues(telemetry);
        }
        return null;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.steerPosition = (int) mSteerMotor.getSelectedSensorPosition(0);
        mPeriodicIO.drivePosition = (int) mDriveMotor.getSelectedSensorPosition(0);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        switch(mControlState) {
            case OPEN_LOOP:
                // don't move if throttle is 0
                if (Util.epsilonEquals(mPeriodicIO.driveDemand, 0.0, mConstants.kDriveDeadband)) {
                    stop();
                } else {
                    mSteerMotor.set(mPeriodicIO.steerControlMode, mPeriodicIO.steerDemand);
                    mDriveMotor.set(mPeriodicIO.driveControlMode, mPeriodicIO.driveDemand);
                }
                break;
            case NEUTRAL:
            default:
                break;
        }
    }

    @Override
    public void outputTelemetry() {
        // SmartDashboard.putNumber(mModuleName + "Angle", getModuleAngle().getDegrees());
        // SmartDashboard.putNumber(mModuleName + "Inches Driven", getDriveDistanceInches());
        // SmartDashboard.putNumber(mModuleName + "Steer", periodicIO.steerPosition);
        // SmartDashboard.putNumber(mModuleName + "Steer velocity", mSteerMotor.getSelectedSensorVelocity(0));
        // SmartDashboard.putNumber(mModuleName + "Steer (cancoder)", enc.getAbsolutePosition()-cancoderOffsetDegrees);
        SmartDashboard.putNumber(mModuleName + "Steer (cancoder)", mCANCoder.getAbsolutePosition());
        // SmartDashboard.putNumber(mModuleName + "Steer", periodicIO.drivePosition);
        // SmartDashboard.putNumber(mModuleName + "Velocity", mDriveMotor.getSelectedSensorVelocity(0));
        //SmartDashboard.putNumber(mModuleName + "Velocity", encVelocityToInchesPerSecond(periodicIO.velocity));
        if(Constants.kDebuggingOutput) {
            SmartDashboard.putNumber(mModuleName + "Pulse Width", mSteerMotor.getSelectedSensorPosition(0));
            if(mSteerMotor.getControlMode() == ControlMode.MotionMagic)
                SmartDashboard.putNumber(mModuleName + "Error", encUnitsToDegrees(mSteerMotor.getClosedLoopError(0)));
            //SmartDashboard.putNumber(mModuleName + "X", position.x());
            //SmartDashboard.putNumber(mModuleName + "Y", position.y());
            SmartDashboard.putNumber(mModuleName + "Steer Speed", mSteerMotor.getSelectedSensorVelocity(0));
        }
    }

    public static class PeriodicIO {
        //Inputs
        public int moduleID;
        public int steerPosition;
        public int drivePosition;
        public double steerCurrent;
        public double driveCurrent;
        public String steerFaults;
        public String driveFaults;

        //Outputs are in units for the motor controller.
        public ControlMode steerControlMode = ControlMode.PercentOutput;
        public ControlMode driveControlMode = ControlMode.PercentOutput;
        public double steerDemand;
        public double driveDemand;
    }
}
