package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.sensors.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import libraries.cheesylib.util.Util;
import libraries.cyberlib.kinematics.SwerveModuleState;
import libraries.cyberlib.utils.Angles;
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

        // Default steer reduction is Mk4_L2i value
        public double kSteerReduction = (14.0 / 50.0) * (10.0 / 60.0); // 1/21.43
        public double kSteerTicksPerUnitDistance = (1.0 / 2048.0) * kSteerReduction * (2.0 * Math.PI);
        public double kSteerTicksPerUnitVelocity = kSteerTicksPerUnitDistance * 10;  // Motor controller unit is ticks per 100 ms

		// general Steer Motor
		public boolean kInvertSteerMotor = false;
		public boolean kInvertSteerMotorSensorPhase = true;
		public NeutralMode kSteerMotorInitNeutralMode = NeutralMode.Coast; // neutral mode could change
        public double kSteerMotorTicksPerRadian = (2048.0 / kSteerReduction)/(2.0 * Math.PI); // for steer motor
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
		public boolean kSteerMotorEnableCurrentLimit = true;
		public double kSteerMotorMaxVoltage = 7.0; // volts
		public boolean kSteerMotorEnableVoltageCompensation = false;
		public int kSteerMotorVoltageMeasurementFilter = 8; // # of samples in rolling average

		// Steer Motor measurement
		public int kSteerMotorStatusFrame2UpdateRate = 10; // feedback for selected sensor, ms
		public int kSteerMotorStatusFrame10UpdateRate = 10; // motion magic, ms
		public SensorVelocityMeasPeriod kSteerMotorVelocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms; // dt for velocity measurements, ms
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
        public SensorVelocityMeasPeriod kDriveMotorVelocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms; // dt for velocity measurements, ms
		public int kDriveVelocityMeasurementWindow = 32; // # of samples in rolling average
    }

	private final boolean mLoggingEnabled = true;    // used to disable logging for this subsystem only
	private final CheckFaults mFaultChecker = new CheckFaults();

//	boolean tenVoltSteerMode = false;
//	private boolean standardCarpetDirection = true;

	private final PeriodicIO mPeriodicIO = new PeriodicIO();
	private ControlState mControlState = ControlState.NEUTRAL;
	public final SwerveModuleConstants mConstants;

    private double mMaxSpeedInMetersPerSecond = 1.0;

    public SwerveDriveModule(SwerveModuleConstants constants, double maxSpeedInMetersPerSecond) {
        mConstants = constants;
        mMaxSpeedInMetersPerSecond = maxSpeedInMetersPerSecond;
        mModuleName = String.format("%s %d", mConstants.kName, mConstants.kModuleId);
        mPeriodicIO.moduleID = mConstants.kModuleId;

        System.out.println("SwerveDriveModule "+mModuleName+","+mConstants.kSteerMotorSlot0Kp);

        mDriveMotor = TalonFXFactory.createDefaultTalon(mConstants.kDriveMotorTalonId);
        mSteerMotor = TalonFXFactory.createDefaultTalon(mConstants.kSteerMotorTalonId);

        CANCoderConfiguration config = new CANCoderConfiguration();
        config.initializationStrategy = mConstants.kCANCoderSensorInitializationStrategy;
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.magnetOffsetDegrees = constants.kCANCoderOffsetDegrees;
        config.sensorDirection = false; //TODO - Make cancoder direction configurable through robot config files

        mCANCoder = new CANCoder(constants.kCANCoderId);
        mCANCoder.configAllSettings(config, Constants.kLongCANTimeoutMs);

        mCANCoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, mConstants.kCANCoderStatusFramePeriodVbatAndFaults);
        mCANCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, mConstants.kCANCoderStatusFramePeriodSensorData);
        
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
        double fxTicksBefore = mSteerMotor.getSelectedSensorPosition();
        double cancoderDegrees = mCANCoder.getAbsolutePosition();
        double fxTicksTarget = degreesToEncUnits(cancoderDegrees);
        double fxTicksNow = fxTicksBefore;
        int loops = 0;
        final double acceptableTickErr = 10;

        while (Math.abs(fxTicksNow-fxTicksTarget) > acceptableTickErr && loops < 5) {
            mSteerMotor.setSelectedSensorPosition(fxTicksTarget, 0, 0);
            Timer.delay(.1);
            fxTicksNow = mSteerMotor.getSelectedSensorPosition();
            loops++;
        }

        System.out.println(mConstants.kName+" cancoder degrees: "+cancoderDegrees+
                        ",  fx encoder ticks (before, target, adjusted): (" +fxTicksBefore+","+fxTicksTarget+","+fxTicksNow+") loops:"+loops);

        mSteerMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, mConstants.kSteerMotorStatusFrame2UpdateRate, Constants.kLongCANTimeoutMs);
        mSteerMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, mConstants.kSteerMotorStatusFrame10UpdateRate, Constants.kLongCANTimeoutMs);
        mSteerMotor.setNeutralMode(mConstants.kSteerMotorInitNeutralMode);
        mSteerMotor.configNominalOutputForward(mConstants.kDriveNominalVoltage, Constants.kLongCANTimeoutMs);
        mSteerMotor.configNominalOutputReverse(0.0, Constants.kLongCANTimeoutMs);
        mSteerMotor.configVoltageCompSaturation(0.0, Constants.kLongCANTimeoutMs);
        mSteerMotor.enableVoltageCompensation(mConstants.kSteerMotorEnableVoltageCompensation);
        mSteerMotor.configAllowableClosedloopError(0, mConstants.kSteerMotorClosedLoopAllowableError, Constants.kLongCANTimeoutMs);

        if (mConstants.kSteerMotorEnableCurrentLimit) {
            var supplyCurrLimit = new SupplyCurrentLimitConfiguration();
            supplyCurrLimit.currentLimit = mConstants.kSteerMotorContinuousCurrentLimit;
            supplyCurrLimit.triggerThresholdCurrent = mConstants.kSteerMotorPeakCurrentLimit;
            supplyCurrLimit.triggerThresholdTime = mConstants.kSteerMotorPeakCurrentDuration;
            supplyCurrLimit.enable = true;

            mSteerMotor.configGetSupplyCurrentLimit(supplyCurrLimit, Constants.kLongCANTimeoutMs);
        }

        // TODO: Set these correctly
        mSteerMotor.configMotionAcceleration(0.9 * mConstants.kSteerTicksPerUnitVelocity * 0.25, Constants.kLongCANTimeoutMs);
        mSteerMotor.configMotionCruiseVelocity(0.9 * mConstants.kSteerTicksPerUnitVelocity, Constants.kLongCANTimeoutMs);
        mSteerMotor.configVelocityMeasurementPeriod(mConstants.kSteerMotorVelocityMeasurementPeriod, Constants.kLongCANTimeoutMs);
        mSteerMotor.configVelocityMeasurementWindow(mConstants.kSteerMotorVelocityMeasurementWindow, Constants.kLongCANTimeoutMs);
        mSteerMotor.selectProfileSlot(0, 0);

        //Slot 0 is for normal use (tuned for fx integrated encoder)
        mSteerMotor.config_kP(0, mConstants.kSteerMotorSlot0Kp, Constants.kLongCANTimeoutMs);
        mSteerMotor.config_kI(0, mConstants.kSteerMotorSlot0Ki, Constants.kLongCANTimeoutMs);
        mSteerMotor.config_kD(0, mConstants.kSteerMotorSlot0Kd, Constants.kLongCANTimeoutMs);
        mSteerMotor.config_kF(0, mConstants.kSteerMotorSlot0Kf, Constants.kLongCANTimeoutMs);
        mSteerMotor.config_IntegralZone(0, mConstants.kSteerMotorSlot0IZone, Constants.kLongCANTimeoutMs);

        // TODO:  Pass in and tune these Motion Magic settings.
//        mSteerMotor.configMotionCruiseVelocity(Constants.kSwerveDriveMaxSpeed*0.9, Constants.kLongCANTimeoutMs);
//        mSteerMotor.configMotionAcceleration(Constants.kSwerveDriveMaxSpeed, Constants.kLongCANTimeoutMs);

        //Slot 2 is reserved for the beginning of auto (tuned for cancoders needs retune)
        // mSteerMotor.config_kP(1, 0.07, 10);
        // mSteerMotor.config_kI(1, 0.0, 10);
        // mSteerMotor.config_kD(1, 0.84, 10);
        // mSteerMotor.config_kF(1, 0.05, 10);
//        mSteerMotor.config_kP(1, mConstants.kSteerMotorSlot1Kp, Constants.kLongCANTimeoutMs); // TODO: test this
//        mSteerMotor.config_kI(1, mConstants.kSteerMotorSlot1Ki, Constants.kLongCANTimeoutMs);
//        mSteerMotor.config_kD(1, mConstants.kSteerMotorSlot1Kd, Constants.kLongCANTimeoutMs);
//        mSteerMotor.config_kF(1, mConstants.kSteerMotorSlot1Kf, Constants.kLongCANTimeoutMs);
//        mSteerMotor.config_IntegralZone(1, mConstants.kSteerMotorSlot1IZone, Constants.kLongCANTimeoutMs);

        // TODO:  Do we want to do command motor to "move" here (even though it should be stationary)?
        // mSteerMotor.set(ControlMode.MotionMagic, mSteerMotor.getSelectedSensorPosition(0));

        // Configure Drive motor
        mDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mDriveMotor.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mDriveMotor.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mDriveMotor.configVelocityMeasurementPeriod(mConstants.kDriveMotorVelocityMeasurementPeriod, Constants.kLongCANTimeoutMs);
        mDriveMotor.configVelocityMeasurementWindow(mConstants.kDriveVelocityMeasurementWindow, Constants.kLongCANTimeoutMs);
        mDriveMotor.configNominalOutputForward(0.0, Constants.kLongCANTimeoutMs);
        mDriveMotor.configNominalOutputReverse(0.0, Constants.kLongCANTimeoutMs);
        mDriveMotor.configVoltageCompSaturation(mConstants.kDriveMaxVoltage, Constants.kLongCANTimeoutMs);
        mDriveMotor.enableVoltageCompensation(true);
        mDriveMotor.setControlFramePeriod(ControlFrame.Control_3_General, 18); // Need this to prevent clicking sounds
        mDriveMotor.setInverted(mConstants.kInvertDrive);
        mDriveMotor.setNeutralMode(mConstants.kDriveInitNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0, 0, Constants.kLongCANTimeoutMs);
        mDriveMotor.setSensorPhase(mConstants.kInvertDriveSensorPhase);
        mDriveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, mConstants.kDriveStatusFrame2UpdateRate, Constants.kLongCANTimeoutMs);
        mDriveMotor.configOpenloopRamp(0.15, Constants.kLongCANTimeoutMs); //Increase if swerve acceleration is too fast
        mDriveMotor.configClosedloopRamp(0.0);
        mDriveMotor.configAllowableClosedloopError(0, 0, Constants.kLongCANTimeoutMs);

        if (mConstants.kDriveEnableCurrentLimit) {
            var supplyCurrLimit = new SupplyCurrentLimitConfiguration();
            supplyCurrLimit.currentLimit = mConstants.kDriveContinuousCurrentLimit;
            supplyCurrLimit.triggerThresholdCurrent = mConstants.kDrivePeakCurrentLimit;
            supplyCurrLimit.triggerThresholdTime = mConstants.kDrivePeakCurrentDuration;
            supplyCurrLimit.enable = true;

            mDriveMotor.configGetSupplyCurrentLimit(supplyCurrLimit, Constants.kLongCANTimeoutMs);
        }

//        // Slot 0 is reserved for MotionMagic
//        mDriveMotor.selectProfileSlot(0, 0);
//        mDriveMotor.config_kP(0, 2.0, Constants.kLongCANTimeoutMs);
//        mDriveMotor.config_kI(0, 0.0, Constants.kLongCANTimeoutMs);
//        mDriveMotor.config_kD(0, 24.0, Constants.kLongCANTimeoutMs);
//        mDriveMotor.config_kF(0, 1023.0/Constants.kSwerveDriveMaxSpeed, Constants.kLongCANTimeoutMs);
//
//        // TODO: Set these correctly
//        mDriveMotor.configMotionCruiseVelocity(Constants.kSwerveDriveMaxSpeed*0.9, Constants.kLongCANTimeoutMs);
//        mDriveMotor.configMotionAcceleration(Constants.kSwerveDriveMaxSpeed, Constants.kLongCANTimeoutMs);
//
//        // Slot 1 corresponds to velocity mode (USED FOR AUTO)
//        mDriveMotor.config_kP(1, 0.03, Constants.kLongCANTimeoutMs);
//        mDriveMotor.config_kI(1, 0.0, Constants.kLongCANTimeoutMs);
//        mDriveMotor.config_kD(1, 0.0, Constants.kLongCANTimeoutMs);
//        mDriveMotor.config_kF(1, 0.05, Constants.kLongCANTimeoutMs);//0.3

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
        // Note that Falcon is contiguous, so it can be larger than 2pi.  Convert to encoder readings
        // to SI units and let  Rotation2d normalizes the angle between 0 and 2pi.
        Rotation2d currentAngle = Rotation2d.fromRadians(encoderUnitsToRadians(mPeriodicIO.steerPosition));

		return new SwerveModuleState(encVelocityToMetersPerSecond(mPeriodicIO.drivePosition), currentAngle);
	}

    /**
     * Sets the state for the module.
     * <p>
     * @param desiredState Desired state for the module.
     */
    public synchronized void setState(SwerveModuleState desiredState) {
        // Note that Falcon is contiguous, so it can be larger than 2pi.  Convert to encoder readings
        // to SI units and let  Rotation2d normalizes the angle between 0 and 2pi.
        Rotation2d currentAngle = Rotation2d.fromRadians(encoderUnitsToRadians(mPeriodicIO.steerPosition));

        // Minimize the change in heading the desired swerve module state would require by potentially
        // reversing the direction the wheel spins. Odometry will still be accurate as both steer angle
        // and wheel speeds will have their signs "flipped."
        var state = SwerveModuleState.optimize(desiredState, currentAngle);

        // Converts the velocity in SI units (meters per second) to a
        // voltage (as a percentage) for the motor controllers.
        setDriveOpenLoop(state.speedInMetersPerSecond / mMaxSpeedInMetersPerSecond);

        setReferenceAngle(state.angle.getRadians());
    }

    /**
     * Sets the reference angle for the steer motor
     * <p>
     * @param referenceAngleRadians goal angle in radians
     */
    private void setReferenceAngle(double referenceAngleRadians) {
        // Note that Falcon is contiguous, so it can be larger then 2pi.
        double currentAngleRadians = encoderUnitsToRadians(mPeriodicIO.steerPosition);

        // Map onto (0, 2pi)
        double currentAngleRadiansMod = Angles.normalizeAngle(currentAngleRadians);
        referenceAngleRadians = Angles.normalizeAngle(referenceAngleRadians);

        // Get the shortest angular distance between current and reference angles.
        double shortestDistance = Angles.shortest_angular_distance(currentAngleRadiansMod, referenceAngleRadians);

        // Adjust by adding the shortest distance to current angle (which can be in  multiples of 2pi)
        double adjustedReferenceAngleRadians = currentAngleRadians + shortestDistance;

        // mPeriodicIO.steerControlMode = ControlMode.MotionMagic;
        mPeriodicIO.steerControlMode = ControlMode.Position;
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
    private double encoderUnitsToRadians(double encUnits) {
        return encUnits / mConstants.kSteerMotorTicksPerRadian;
    }

    private double radiansToEncoderUnits(double radians) {
        return radians * mConstants.kSteerMotorTicksPerRadian;
    }

    private int degreesToEncUnits(double degrees) {
        return (int) radiansToEncoderUnits(Math.toRadians(degrees));
    }

    private double encUnitsToDegrees(double encUnits) {
        return Math.toDegrees(encoderUnitsToRadians(encUnits));
    }

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


    /**
     * Sets the mode of operation during neutral throttle output.
     * <p>
     * @param neutralMode  The desired mode of operation when the Talon FX
     *                     Controller output throttle is neutral (ie brake/coast)
     **/
    public synchronized void setNeutralMode(NeutralMode neutralMode) {
        mDriveMotor.setNeutralMode(neutralMode);
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
        // SmartDashboard.putNumber(mModuleName + " cancoder", mCANCoder.getAbsolutePosition());
        // SmartDashboard.putNumber(mModuleName + " steerDemand", mPeriodicIO.steerDemand);
        // SmartDashboard.putNumber(mModuleName + " steerPosition", mPeriodicIO.steerPosition);
        // SmartDashboard.putNumber(mModuleName + " driveDemand", mPeriodicIO.driveDemand);
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
