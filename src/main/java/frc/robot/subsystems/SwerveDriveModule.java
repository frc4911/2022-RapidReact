package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
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
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.loops.ILooper;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.madtownlib.util.Utils;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents a swerve module consisting of a drive motor that controls the speed of the wheel and
 * a rotation motor (aka. azimuth) that controls the angle the wheel is pointing towards relative to
 * the chassis frame.
 */
public class SwerveDriveModule extends Subsystem {

    // Hardware
    TalonFX mRotationMotor, mDriveMotor;
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
    private CheckFaults cf = new CheckFaults();

	public static class SwerveModuleConstants {
		public String kName = "Name";
        public int kModuleId = -1;
		public int kDriveMotorTalonId = -1;
		public int kRotationMotorTalonId = -1;

		// general Rotation Motor
		public boolean kInvertRotationMotor = true;
		public boolean kInvertRotationMotorSensorPhase = true;
		public NeutralMode kRotationMotorInitNeutralMode = NeutralMode.Brake; // neutral mode could change
		public double kRotationMotorTicksPerRadian = 4096.0 / (2 * Math.PI); // for rotation motor
		public double kRotationMotorEncoderHomeOffset = 0;

		// Rotation CANCoder
		public int kCANCoderId = -1;
		public SensorInitializationStrategy kCANCoderSensorInitializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
		public int kCANCoderStatusFramePeriodVbatAndFaults = 255;
		public int kCANCoderStatusFramePeriodSensorData = 255;
		public double kCANCoderOffsetDegrees = 0.0;

		// CANCoder Motor motion
		public double kCANCoderRotationMotorKp = 0.1;
		public double kCANCoderRotationMotorKi = 0.0;
		public double kCANCoderRotationMotorKd = 0.84;
		public double kCANCoderRotationMotorKf = 0.05;
		public int kCANCoderRotationMotorIZone = 25;
		public int kCANCoderRotationMotorCruiseVelocity = 1698;
		public int kCANCoderRotationMotorAcceleration = 20379; // 12 * kRotationMotorCruiseVelocity
		public int kCANCoderRotationMotorClosedLoopAllowableError = 5;

		// Rotation Motor motion
		public double kRotationMotorSlot0Kp = 0.07;
		public double kRotationMotorSlot0Ki = 0.0;
		public double kRotationMotorSlot0Kd = 0.84;
		public double kRotationMotorSlot0Kf = 0.025;
		public int kRotationMotorSlot0IZone = 25;
		public int kRotationMotorSlot0CruiseVelocity = 1698;
		public int kRotationMotorSlot0Acceleration = 20379; // 12 * kRotationMotorCruiseVelocity
		public int kRotationMotorClosedLoopAllowableError = 5;

		public double kRotationMotorSlot1Kp = 0.07;
		public double kRotationMotorSlot1Ki = 0.0;
		public double kRotationMotorSlot1Kd = 0.84;
		public double kRotationMotorSlot1Kf = 0.025;
		public int kRotationMotorSlot1IZone = 25;
		public int kRotationMotorSlot1CruiseVelocity = 1698;
		public int kRotationMotorSlot1Acceleration = 20379; // 12 * kRotationMotorCruiseVelocity

		// Rotation Motor current/voltage
		public int kRotationMotorContinuousCurrentLimit = 30; // amps
		public int kRotationMotorPeakCurrentLimit = 60; // amps
		public int kRotationMotorPeakCurrentDuration = 200; // ms
		public boolean kRotationMotorEnableCurrentLimit = true;
		public double kRotationMotorMaxVoltage = 7.0; // volts
		public boolean kRotationMotorEnableVoltageCompensation = false;
		public int kRotationMotorVoltageMeasurementFilter = 8; // # of samples in rolling average

		// Rotation Motor measurement
		public int kRotationMotorStatusFrame2UpdateRate = 10; // feedback for selected sensor, ms
		public int kRotationMotorStatusFrame10UpdateRate = 10; // motion magic, ms
		public VelocityMeasPeriod kRotationMotorVelocityMeasurementPeriod = VelocityMeasPeriod.Period_100Ms; // dt for velocity measurements, ms
		public int kRotationMotorVelocityMeasurementWindow = 64; // # of samples in rolling average

		// general drive
		public boolean kInvertDrive = true;
		public boolean kInvertDriveSensorPhase = false;
		public NeutralMode kDriveInitNeutralMode = NeutralMode.Brake; // neutral mode could change
		public double kWheelDiameter = 4.0; // Probably should tune for each individual wheel maybe
		public double kDriveTicksPerUnitDistance = (1.0 / 4096.0) * (18.0 / 28.0 * 15.0 / 45.0)
				* (Math.PI * kWheelDiameter);
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

	boolean tenVoltRotationMode = false;
	private Translation2d position;
	private Translation2d startingPosition;
	private boolean standardCarpetDirection = true;

	private final PeriodicIO mPeriodicIO = new PeriodicIO();
	private ControlState mControlState = ControlState.NEUTRAL;
	private final SwerveModuleConstants mConstants;

    public SwerveDriveModule(SwerveModuleConstants constants) {
        mConstants = constants;
        mModuleName = String.format("%s %d", mConstants.kName, mConstants.kModuleId);
        mPeriodicIO.moduleID = mConstants.kModuleId;

        mDriveMotor = TalonFXFactory.createDefaultTalon(constants.kDriveMotorTalonId);
        mRotationMotor = TalonFXFactory.createDefaultTalon(constants.kRotationMotorTalonId);

        mCANCoder = new CANCoder(constants.kCANCoderId);
        mCANCoder.configSensorInitializationStrategy(constants.kCANCoderSensorInitializationStrategy, Constants.kLongCANTimeoutMs);
        mCANCoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, constants.kCANCoderStatusFramePeriodVbatAndFaults);
        mCANCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, constants.kCANCoderStatusFramePeriodSensorData);

        configureMotors();
        getRawAngle();
    }

    /**
     * Configure motors based on current SwerveModuleConstants.
     */
    private void configureMotors() {
        mRotationMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mRotationMotor.setSensorPhase(mConstants.kInvertRotationMotorSensorPhase);
        mRotationMotor.setInverted(mConstants.kInvertRotationMotor);
        mRotationMotor.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mRotationMotor.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        // multiple (usually 2) sets were needed to set new encoder value
        double fxCurrentEnc = mRotationMotor.getSelectedSensorPosition(0);
        System.out.println(mConstants.kName + " current fx encoder ticks: " + fxCurrentEnc);
        double cancoderDegrees = mCANCoder.getAbsolutePosition();
        System.out.println(mConstants.kName + " cancoder degrees-offset: " + cancoderDegrees +
                "-" + mConstants.kCANCoderOffsetDegrees + "=" + (cancoderDegrees - mConstants.kCANCoderOffsetDegrees));
        int startEncoderValue = (int)degreesToEncUnits(cancoderDegrees - mConstants.kCANCoderOffsetDegrees);

        int count = 0;
        while (Math.abs(fxCurrentEnc-startEncoderValue) > 10 && count < 5) {
            count++;
            mRotationMotor.setSelectedSensorPosition(startEncoderValue, 0, Constants.kLongCANTimeoutMs);
            Timer.delay(.1);
            fxCurrentEnc = mRotationMotor.getSelectedSensorPosition();
        }

        System.out.println(mConstants.kName + " loops needed to set fx encoder: " + count);

        mRotationMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, mConstants.kRotationMotorStatusFrame2UpdateRate, Constants.kLongCANTimeoutMs);
        mRotationMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, mConstants.kRotationMotorStatusFrame10UpdateRate, Constants.kLongCANTimeoutMs);
        mRotationMotor.setNeutralMode(mConstants.kRotationMotorInitNeutralMode);
        mRotationMotor.configNominalOutputForward(mConstants.kDriveNominalVoltage, Constants.kLongCANTimeoutMs);
        mRotationMotor.configNominalOutputReverse(0.0, Constants.kLongCANTimeoutMs);
        mRotationMotor.configVoltageCompSaturation(0.0, Constants.kLongCANTimeoutMs);
        mRotationMotor.enableVoltageCompensation(mConstants.kRotationMotorEnableVoltageCompensation);
        mRotationMotor.configAllowableClosedloopError(0, mConstants.kRotationMotorClosedLoopAllowableError, Constants.kLongCANTimeoutMs);

        mRotationMotor.configMotionAcceleration(Constants.kSwerveRotationMaxSpeed * 12.5, Constants.kLongCANTimeoutMs);
        mRotationMotor.configMotionCruiseVelocity(Constants.kSwerveRotationMaxSpeed, Constants.kLongCANTimeoutMs);
//        mRotationMotor.configVelocityMeasurementPeriod(mConstants.kRotationMotorVelocityMeasurementPeriod, Constants.kLongCANTimeoutMs);
        mRotationMotor.configVelocityMeasurementWindow(mConstants.kRotationMotorVelocityMeasurementWindow, Constants.kLongCANTimeoutMs);
        mRotationMotor.selectProfileSlot(0, 0);

        //Slot 1 is for normal use (tuned for fx integrated encoder)
        mRotationMotor.config_kP(0, mConstants.kRotationMotorSlot0Kp, Constants.kLongCANTimeoutMs);
        mRotationMotor.config_kI(0, mConstants.kRotationMotorSlot0Ki, Constants.kLongCANTimeoutMs);
        mRotationMotor.config_kD(0, mConstants.kRotationMotorSlot0Kd, Constants.kLongCANTimeoutMs);
        mRotationMotor.config_kF(0, mConstants.kRotationMotorSlot0Kf, Constants.kLongCANTimeoutMs);
        mRotationMotor.config_IntegralZone(0, mConstants.kRotationMotorSlot0IZone, Constants.kLongCANTimeoutMs);

        //Slot 2 is reserved for the beginning of auto (tuned for cancoders needs retune)
        // rotationMotor.config_kP(1, 0.07, 10);
        // rotationMotor.config_kI(1, 0.0, 10);
        // rotationMotor.config_kD(1, 0.84, 10);
        // rotationMotor.config_kF(1, 0.05, 10);
        mRotationMotor.config_kP(1, mConstants.kRotationMotorSlot1Kp, Constants.kLongCANTimeoutMs); // TODO: test this
        mRotationMotor.config_kI(1, mConstants.kRotationMotorSlot1Ki, Constants.kLongCANTimeoutMs);
        mRotationMotor.config_kD(1, mConstants.kRotationMotorSlot1Kd, Constants.kLongCANTimeoutMs);
        mRotationMotor.config_kF(1, mConstants.kRotationMotorSlot1Kf, Constants.kLongCANTimeoutMs);
        mRotationMotor.config_IntegralZone(1, mConstants.kRotationMotorSlot1IZone, Constants.kLongCANTimeoutMs);
        mRotationMotor.set(ControlMode.MotionMagic, mRotationMotor.getSelectedSensorPosition(0));

        // if (!isRotationSensorConnected()) {
        // 	DriverStation.reportError(mConstants.kName + "rotation encoder not detected!", false);
        // 	hasEmergency = true;
        // }

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
	 * Returns the current state of the module.
	 *
	 * @return The current state of the module.
	 */
	public SwerveModuleState getState() {
		return new SwerveModuleState(
                // Convert to SI units to keep everything consistent.
                encVelocityToMetersPerSecond(mPeriodicIO.driveDemand),
				new Rotation2d(encUnitsToDegrees(mPeriodicIO.rotationPosition)));
	}

	/**
	 * Sets the state for the module.
	 * It converts the velocity in SI units (meters per second) to a voltage (as a percentage) for the motor controllers.
	 *
	 * @param state Desired state with speed and angle.
	 */
	public void setState(SwerveModuleState state) {
		// Convert SwerveModuleState speed (m/s) into percent output
		setDriveOpenLoop(metersPerSecondToEncVelocity(state.speedInMetersPerSecond));
		setModuleAngle(state.angle.getDegrees());
	}
	
	@Override
	public synchronized void stop() {
		if (mControlState != ControlState.NEUTRAL) {
			mControlState = ControlState.NEUTRAL;
		}
		mDriveMotor.set(ControlMode.PercentOutput, 0.0);
		mRotationMotor.set(ControlMode.PercentOutput, 0.0);
	}

    @Override
    public synchronized void zeroSensors() {
        zeroSensors(new Pose2d());
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
    }

	private double getRawAngle() {
		return encUnitsToDegrees(mPeriodicIO.rotationPosition);
	}

    private void setModuleAngle(double goalAngle){
		double angle = Utils.placeInAppropriate0To360Scope(getRawAngle(), goalAngle);
		int setpoint = degreesToEncUnits(angle);
        mPeriodicIO.rotationControlMode = ControlMode.MotionMagic;
        mPeriodicIO.rotationDemand = setpoint;
    }

    public void setRotationOpenLoop(double power) {
        mPeriodicIO.rotationControlMode = ControlMode.PercentOutput;
        mPeriodicIO.rotationDemand = power;
    }

    /**
     * @param velocity Normalized value
     */
    public void setDriveOpenLoop(double velocity) {
        if (mControlState != ControlState.OPEN_LOOP) {
            mControlState = ControlState.OPEN_LOOP;
        }

        mPeriodicIO.driveControlMode = ControlMode.PercentOutput;
        mPeriodicIO.driveDemand = velocity;
    }

    public double encUnitsToInches(double encUnits) {
        return encUnits/Constants.kSwerveEncUnitsPerInch;
    }

    public double inchesToEncUnits(double inches) {
        return inches * Constants.kSwerveEncUnitsPerInch;
    }

    public double metersPerSecondToEncVelocity(double metersPerSecond) {
        return inchesToEncUnits(Units.metersToInches(metersPerSecond) / 10.0);
    }

    public double encVelocityToInchesPerSecond(double encUnitsPer100ms){
        return encUnitsToInches(encUnitsPer100ms) * 10;
    }

    public double encVelocityToMetersPerSecond(double encUnitsPer100ms) {
        return Units.inchesToMeters(encVelocityToInchesPerSecond(encUnitsPer100ms));
    }

    public double inchesPerSecondToEncVelocity(double inchesPerSecond){
        return inchesToEncUnits(inchesPerSecond / 10.0);
    }

    public int degreesToEncUnits(double degrees) {
        return (int) (degrees / 360.0 * Constants.kSwerveRotationMotorTicksPerRotation);
    }

    public double encUnitsToDegrees(double encUnits) {
        return encUnits / Constants.kSwerveRotationMotorTicksPerRotation * 360.0;
    }

    public Translation2d getPosition() {
        return position;
    }

    public synchronized void resetPose(Pose2d robotPose) {
        Translation2d modulePosition = robotPose.transformBy(Pose2d.fromTranslation(startingPosition)).getTranslation();
        position = modulePosition;
    }

    public synchronized void zeroSensors(Pose2d robotPose) {
        //driveMotor.setSelectedSensorPosition(0, 0, 100); TODO check if this is necessary
        resetPose(robotPose);
    }

    public synchronized void disable() {
        setDriveOpenLoop(0.0);
        setRotationOpenLoop(0.0);
    }

    @Override
    public String getLogHeaders() {
        if (mLoggingEnabled){
            cf.clearFaults(mDriveMotor);
            cf.clearFaults(mRotationMotor);
            String shortName = mModuleName;

            return  shortName+".rotationPosition,"+
                    shortName+".drivePosition,"+
                    shortName+".rotationControlMode,"+
                    shortName+".driveControlMode,"+
                    shortName+".rotationDemand,"+
                    shortName+".driveDemand,"+
                    shortName+".rotationCurrent,"+
                    shortName+".driveCurrent,"+
                    shortName+".rotationFaults,"+
                    shortName+".driveFaults";
        }
        return null;
    }

    private String generateLogValues(boolean telemetry){
        String values;

        if (telemetry){
            mPeriodicIO.rotationCurrent = mRotationMotor.getStatorCurrent();
            mPeriodicIO.driveCurrent    = mDriveMotor.getStatorCurrent();
            mPeriodicIO.rotationFaults  = cf.getFaults(mRotationMotor);
            mPeriodicIO.driveFaults     = cf.getFaults(mDriveMotor);

            values = ""+ mPeriodicIO.rotationPosition+","+
                    mPeriodicIO.drivePosition+","+
                    mPeriodicIO.rotationControlMode+","+
                    mPeriodicIO.driveControlMode+","+
                    mPeriodicIO.rotationDemand+","+
                    mPeriodicIO.driveDemand+","+
                    mPeriodicIO.rotationCurrent+","+
                    mPeriodicIO.driveCurrent+","+
                    mPeriodicIO.rotationFaults+","+
                    mPeriodicIO.driveFaults;
        }
        else{
            values = ""+ mPeriodicIO.rotationPosition+","+
                    mPeriodicIO.drivePosition+","+
                    mPeriodicIO.rotationControlMode+","+
                    mPeriodicIO.driveControlMode+","+
                    mPeriodicIO.rotationDemand+","+
                    mPeriodicIO.driveDemand+","+
                    /*periodicIO.rotationCurrent+*/","+
                    /*periodicIO.driveCurrent+*/","+
                    /*cf.getFXFaults(rotationMotor)+*/","
            /*cf.getFXFaults(driveMotor)*/;
        }
        return values;
    }

    @Override
    public String getLogValues(boolean telemetry) {
        if (mLoggingEnabled){
            return generateLogValues(telemetry);
        }
        return null;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.rotationPosition = (int) mRotationMotor.getSelectedSensorPosition(0);
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
                    mRotationMotor.set(mPeriodicIO.rotationControlMode, mPeriodicIO.rotationDemand);
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
        // SmartDashboard.putNumber(name + "Angle", getModuleAngle().getDegrees());
        // SmartDashboard.putNumber(name + "Inches Driven", getDriveDistanceInches());
        // SmartDashboard.putNumber(name + "Rotation", periodicIO.rotationPosition); // Alex
        // SmartDashboard.putNumber(name + "Rotation velocity", rotationMotor.getSelectedSensorVelocity(0)); // Alex
        // SmartDashboard.putNumber(name + "Rotation (cancoder)", enc.getAbsolutePosition()-cancoderOffsetDegrees); // Alex
        SmartDashboard.putNumber(mModuleName + "Rotation (cancoder)", mCANCoder.getAbsolutePosition()); // Alex
        // SmartDashboard.putNumber(name + "Position", periodicIO.drivePosition); // Alex
        // SmartDashboard.putNumber(name + "Velocity", driveMotor.getSelectedSensorVelocity(0)); // Alex
        //SmartDashboard.putNumber(name + "Velocity", encVelocityToInchesPerSecond(periodicIO.velocity));
        if(Constants.kDebuggingOutput){
            SmartDashboard.putNumber(mModuleName + "Pulse Width", mRotationMotor.getSelectedSensorPosition(0));
            if(mRotationMotor.getControlMode() == ControlMode.MotionMagic)
                SmartDashboard.putNumber(mModuleName + "Error", encUnitsToDegrees(mRotationMotor.getClosedLoopError(0)));
            //SmartDashboard.putNumber(name + "X", position.x());
            //SmartDashboard.putNumber(name + "Y", position.y());
            SmartDashboard.putNumber(mModuleName + "Rotation Speed", mRotationMotor.getSelectedSensorVelocity(0));
        }
    }

    public static class PeriodicIO {
        //Inputs
        public int moduleID;
        public int rotationPosition;
        public int drivePosition;
        public double rotationCurrent;
        public double driveCurrent;
        public String rotationFaults;
        public String driveFaults;


        //Outputs
        public ControlMode rotationControlMode = ControlMode.PercentOutput;
        public ControlMode driveControlMode = ControlMode.PercentOutput;
        public double rotationDemand;
        public double driveDemand;
    }
}
