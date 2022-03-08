package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.SwerveModuleConfiguration;
import frc.robot.constants.Constants;
import libraries.cheesylib.drivers.TalonFXFactory;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.util.Util;
import libraries.cyberlib.kinematics.SwerveModuleState;
import libraries.cyberlib.utils.Angles;

/**
 * Represents a swerve module consisting of a drive motor that controls the
 * speed of the wheel and
 * a steer motor that controls the angle the wheel is pointing towards relative
 * to the chassis frame.
 */
public class SwerveDriveModule extends Subsystem {
    // Hardware
    TalonFX mSteerMotor, mDriveMotor;
    CANCoder mCANCoder;

    public enum ControlState {
        /**
         * No motor outputs. This state is used robot is powered up and stopped by
         * Driver Station.
         */
        NEUTRAL,
        /**
         * Runs motors in response to inputs.
         */
        OPEN_LOOP
    }

    String mModuleName;

    private final boolean mLoggingEnabled = true; // used to disable logging for this subsystem only

    // boolean tenVoltSteerMode = false;
    // private boolean standardCarpetDirection = true;

    private final PeriodicIO mPeriodicIO = new PeriodicIO();
    private ControlState mControlState = ControlState.NEUTRAL;
    public final SwerveModuleConfiguration mConfig;

    private double mMaxSpeedInMetersPerSecond = 1.0;

    public SwerveDriveModule(SwerveModuleConfiguration constants, double maxSpeedInMetersPerSecond) {
        mConfig = constants;
        mMaxSpeedInMetersPerSecond = maxSpeedInMetersPerSecond;
        mModuleName = String.format("%s %d", mConfig.kName, mConfig.kModuleId);
        mPeriodicIO.moduleID = mConfig.kModuleId;

        System.out.println("SwerveDriveModule " + mModuleName + "," + mConfig.kSteerMotorSlot0Kp);

        mDriveMotor = TalonFXFactory.createDefaultTalon(mConfig.kDriveMotorTalonId);
        mSteerMotor = TalonFXFactory.createDefaultTalon(mConfig.kSteerMotorTalonId);

        CANCoderConfiguration config = new CANCoderConfiguration();
        config.initializationStrategy = mConfig.kCANCoderSensorInitializationStrategy;
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.magnetOffsetDegrees = constants.kCANCoderOffsetDegrees;
        config.sensorDirection = false; // TODO - Make cancoder direction configurable through robot config files

        mCANCoder = new CANCoder(constants.kCANCoderId);
        mCANCoder.configAllSettings(config, Constants.kLongCANTimeoutMs);

        mCANCoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults,
                mConfig.kCANCoderStatusFramePeriodVbatAndFaults);
        mCANCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, mConfig.kCANCoderStatusFramePeriodSensorData);

        configureMotors();
    }

    /**
     * Configure motors based on current SwerveModuleConstants.
     */
    private void configureMotors() {
        mSteerMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mSteerMotor.setSensorPhase(mConfig.kInvertSteerMotorSensorPhase);
        mSteerMotor.setInverted(mConfig.kInvertSteerMotor);
        mSteerMotor.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mSteerMotor.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        convertCancoderToFX();
        convertCancoderToFX();

        mSteerMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, mConfig.kSteerMotorStatusFrame2UpdateRate,
                Constants.kLongCANTimeoutMs);
        mSteerMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic,
                mConfig.kSteerMotorStatusFrame10UpdateRate, Constants.kLongCANTimeoutMs);
        mSteerMotor.setNeutralMode(mConfig.kSteerMotorInitNeutralMode);
        mSteerMotor.configNominalOutputForward(mConfig.kDriveNominalVoltage, Constants.kLongCANTimeoutMs);
        mSteerMotor.configNominalOutputReverse(0.0, Constants.kLongCANTimeoutMs);
        mSteerMotor.configVoltageCompSaturation(0.0, Constants.kLongCANTimeoutMs);
        mSteerMotor.enableVoltageCompensation(mConfig.kSteerMotorEnableVoltageCompensation);
        mSteerMotor.configAllowableClosedloopError(0, mConfig.kSteerMotorClosedLoopAllowableError,
                Constants.kLongCANTimeoutMs);

        if (mConfig.kSteerMotorEnableCurrentLimit) {
            var supplyCurrLimit = new SupplyCurrentLimitConfiguration();
            supplyCurrLimit.currentLimit = mConfig.kSteerMotorContinuousCurrentLimit;
            supplyCurrLimit.triggerThresholdCurrent = mConfig.kSteerMotorPeakCurrentLimit;
            supplyCurrLimit.triggerThresholdTime = mConfig.kSteerMotorPeakCurrentDuration;
            supplyCurrLimit.enable = true;

            mSteerMotor.configGetSupplyCurrentLimit(supplyCurrLimit, Constants.kLongCANTimeoutMs);
        }
        // mSteerMotor.configStatorCurrentLimit(new
        // StatorCurrentLimitConfiguration(true, 30, 30, 0));

        // TODO: Set these correctly
        mSteerMotor.configMotionAcceleration(0.9 * mConfig.kSteerTicksPerUnitVelocity * 0.25,
                Constants.kLongCANTimeoutMs);
        mSteerMotor.configMotionCruiseVelocity(0.9 * mConfig.kSteerTicksPerUnitVelocity,
                Constants.kLongCANTimeoutMs);
        mSteerMotor.configVelocityMeasurementPeriod(mConfig.kSteerMotorVelocityMeasurementPeriod,
                Constants.kLongCANTimeoutMs);
        mSteerMotor.configVelocityMeasurementWindow(mConfig.kSteerMotorVelocityMeasurementWindow,
                Constants.kLongCANTimeoutMs);
        mSteerMotor.selectProfileSlot(0, 0);

        // Slot 0 is for normal use (tuned for fx integrated encoder)
        mSteerMotor.config_kP(0, mConfig.kSteerMotorSlot0Kp, Constants.kLongCANTimeoutMs);
        mSteerMotor.config_kI(0, mConfig.kSteerMotorSlot0Ki, Constants.kLongCANTimeoutMs);
        mSteerMotor.config_kD(0, mConfig.kSteerMotorSlot0Kd, Constants.kLongCANTimeoutMs);
        mSteerMotor.config_kF(0, mConfig.kSteerMotorSlot0Kf, Constants.kLongCANTimeoutMs);
        mSteerMotor.config_IntegralZone(0, mConfig.kSteerMotorSlot0IZone, Constants.kLongCANTimeoutMs);

        // TODO: Pass in and tune these Motion Magic settings.
        // mSteerMotor.configMotionCruiseVelocity(Constants.kSwerveDriveMaxSpeed*0.9,
        // Constants.kLongCANTimeoutMs);
        // mSteerMotor.configMotionAcceleration(Constants.kSwerveDriveMaxSpeed,
        // Constants.kLongCANTimeoutMs);

        // Slot 2 is reserved for the beginning of auto (tuned for cancoders needs
        // retune)
        // mSteerMotor.config_kP(1, 0.07, 10);
        // mSteerMotor.config_kI(1, 0.0, 10);
        // mSteerMotor.config_kD(1, 0.84, 10);
        // mSteerMotor.config_kF(1, 0.05, 10);
        // mSteerMotor.config_kP(1, mConstants.kSteerMotorSlot1Kp,
        // Constants.kLongCANTimeoutMs); // TODO: test this
        // mSteerMotor.config_kI(1, mConstants.kSteerMotorSlot1Ki,
        // Constants.kLongCANTimeoutMs);
        // mSteerMotor.config_kD(1, mConstants.kSteerMotorSlot1Kd,
        // Constants.kLongCANTimeoutMs);
        // mSteerMotor.config_kF(1, mConstants.kSteerMotorSlot1Kf,
        // Constants.kLongCANTimeoutMs);
        // mSteerMotor.config_IntegralZone(1, mConstants.kSteerMotorSlot1IZone,
        // Constants.kLongCANTimeoutMs);

        // TODO: Do we want to do command motor to "move" here (even though it should be
        // stationary)?
        // mSteerMotor.set(ControlMode.MotionMagic,
        // mSteerMotor.getSelectedSensorPosition(0));

        // Configure Drive motor
        mDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mDriveMotor.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mDriveMotor.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mDriveMotor.configVelocityMeasurementPeriod(mConfig.kDriveMotorVelocityMeasurementPeriod,
                Constants.kLongCANTimeoutMs);
        mDriveMotor.configVelocityMeasurementWindow(mConfig.kDriveVelocityMeasurementWindow,
                Constants.kLongCANTimeoutMs);
        mDriveMotor.configNominalOutputForward(0.0, Constants.kLongCANTimeoutMs);
        mDriveMotor.configNominalOutputReverse(0.0, Constants.kLongCANTimeoutMs);
        mDriveMotor.configVoltageCompSaturation(mConfig.kDriveMaxVoltage, Constants.kLongCANTimeoutMs);
        mDriveMotor.enableVoltageCompensation(true);
        mDriveMotor.setControlFramePeriod(ControlFrame.Control_3_General, 18); // Need this to prevent clicking sounds
        mDriveMotor.setInverted(mConfig.kInvertDrive);
        mDriveMotor.setNeutralMode(mConfig.kDriveInitNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0, 0, Constants.kLongCANTimeoutMs);
        mDriveMotor.setSensorPhase(mConfig.kInvertDriveSensorPhase);
        mDriveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, mConfig.kDriveStatusFrame2UpdateRate,
                Constants.kLongCANTimeoutMs);
        mDriveMotor.configOpenloopRamp(0.3, Constants.kLongCANTimeoutMs); // Increase if swerve acceleration is too fast
        mDriveMotor.configClosedloopRamp(0.0);
        mDriveMotor.configAllowableClosedloopError(0, 0, Constants.kLongCANTimeoutMs);

        if (mConfig.kDriveEnableCurrentLimit) {
            var supplyCurrLimit = new SupplyCurrentLimitConfiguration();
            supplyCurrLimit.currentLimit = mConfig.kDriveContinuousCurrentLimit;
            supplyCurrLimit.triggerThresholdCurrent = mConfig.kDrivePeakCurrentLimit;
            supplyCurrLimit.triggerThresholdTime = mConfig.kDrivePeakCurrentDuration;
            supplyCurrLimit.enable = true;

            mDriveMotor.configGetSupplyCurrentLimit(supplyCurrLimit, Constants.kLongCANTimeoutMs);
        }

        mDriveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 90, 90, 0));

        // // Slot 0 is reserved for MotionMagic
        // mDriveMotor.selectProfileSlot(0, 0);
        // mDriveMotor.config_kP(0, 2.0, Constants.kLongCANTimeoutMs);
        // mDriveMotor.config_kI(0, 0.0, Constants.kLongCANTimeoutMs);
        // mDriveMotor.config_kD(0, 24.0, Constants.kLongCANTimeoutMs);
        // mDriveMotor.config_kF(0, 1023.0/Constants.kSwerveDriveMaxSpeed,
        // Constants.kLongCANTimeoutMs);
        //
        // // TODO: Set these correctly
        // mDriveMotor.configMotionCruiseVelocity(Constants.kSwerveDriveMaxSpeed*0.9,
        // Constants.kLongCANTimeoutMs);
        // mDriveMotor.configMotionAcceleration(Constants.kSwerveDriveMaxSpeed,
        // Constants.kLongCANTimeoutMs);
        //
        // // Slot 1 corresponds to velocity mode (USED FOR AUTO)
        // mDriveMotor.config_kP(1, 0.03, Constants.kLongCANTimeoutMs);
        // mDriveMotor.config_kI(1, 0.0, Constants.kLongCANTimeoutMs);
        // mDriveMotor.config_kD(1, 0.0, Constants.kLongCANTimeoutMs);
        // mDriveMotor.config_kF(1, 0.05, Constants.kLongCANTimeoutMs);//0.3

        // if (!isDriveSensorConnected()) {
        // DriverStation.reportError(mConstants.kName + "drive encoder not detected!",
        // false);
        // hasEmergency = true;
        // }
    }

    protected void convertCancoderToFX(){
        // multiple (usually 2) sets were needed to set new encoder value
        double fxTicksBefore = mSteerMotor.getSelectedSensorPosition();
        double cancoderDegrees = mCANCoder.getAbsolutePosition();
        int limit = 5;
        do{
            if (mCANCoder.getLastError() != ErrorCode.OK) {
                System.out.println("error reading cancoder. Trying again!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! value read was "
                        + cancoderDegrees);
                Timer.delay(.1);
                cancoderDegrees = mCANCoder.getAbsolutePosition();
            }
            else{
                break;
            }
        }while (limit-- > 0);

        double fxTicksTarget = degreesToEncUnits(cancoderDegrees);
        double fxTicksNow = fxTicksBefore;
        int loops = 0;
        final double acceptableTickErr = 10;

        while (Math.abs(fxTicksNow - fxTicksTarget) > acceptableTickErr && loops < 5) {
            mSteerMotor.setSelectedSensorPosition(fxTicksTarget, 0, 0);
            Timer.delay(.1);
            fxTicksNow = mSteerMotor.getSelectedSensorPosition();
            loops++;
        }

        System.out.println(mConfig.kName + " cancoder degrees: " + cancoderDegrees +
                ",  fx encoder ticks (before, target, adjusted): (" + fxTicksBefore + "," + fxTicksTarget + ","
                + fxTicksNow + ") loops:" + loops);
    }

    /**
     * Gets the current state of the module.
     * <p>
     *
     * @return The current state of the module.
     */
    public synchronized SwerveModuleState getState() {
        // Note that Falcon is contiguous, so it can be larger than 2pi. Convert to
        // encoder readings
        // to SI units and let Rotation2d normalizes the angle between 0 and 2pi.
        Rotation2d currentAngle = Rotation2d.fromRadians(encoderUnitsToRadians(mPeriodicIO.steerPosition));

        return new SwerveModuleState(encVelocityToMetersPerSecond(mPeriodicIO.driveVelocity), currentAngle);
    }

    /**
     * Sets the state for the module.
     * <p>
     *
     * @param desiredState Desired state for the module.
     */
    public synchronized void setState(SwerveModuleState desiredState) {
        // Note that Falcon is contiguous, so it can be larger than 2pi. Convert to
        // encoder readings
        // to SI units and let Rotation2d normalizes the angle between 0 and 2pi.
        Rotation2d currentAngle = Rotation2d.fromRadians(encoderUnitsToRadians(mPeriodicIO.steerPosition));

        // Minimizing the change in heading for the swerve module potentially requires
        // reversing the
        // direction the wheel spins. Odometry will still be accurate as both steer
        // angle and wheel
        // speeds will have their signs "flipped."
        var state = SwerveModuleState.optimize(desiredState, currentAngle);

        // Converts the velocity in SI units (meters per second) to a voltage (as a
        // percentage)
        // for the motor controllers.
        setDriveOpenLoop(state.speedInMetersPerSecond / mMaxSpeedInMetersPerSecond);

        setReferenceAngle(state.angle.getRadians());
    }

    /**
     * Sets the reference angle for the steer motor
     * <p>
     *
     * @param referenceAngleRadians goal angle in radians
     */
    private void setReferenceAngle(double referenceAngleRadians) {
        // Note that Falcon is contiguous, so it can be larger than 2pi.
        double currentAngleRadians = encoderUnitsToRadians(mPeriodicIO.steerPosition);

        // Map onto (0, 2pi)
        double currentAngleRadiansMod = Angles.normalizeAngle(currentAngleRadians);
        referenceAngleRadians = Angles.normalizeAngle(referenceAngleRadians);

        // Get the shortest angular distance between current and reference angles.
        double shortestDistance = Angles.shortest_angular_distance(currentAngleRadiansMod, referenceAngleRadians);

        // Adjust by adding the shortest distance to current angle (which can be in
        // multiples of 2pi)
        double adjustedReferenceAngleRadians = currentAngleRadians + shortestDistance;

        // mPeriodicIO.steerControlMode = ControlMode.MotionMagic;
        mPeriodicIO.steerControlMode = ControlMode.Position;
        mPeriodicIO.steerDemand = radiansToEncoderUnits(adjustedReferenceAngleRadians);
    }

    /**
     * Sets the motor controller settings and values for the steer motor.
     * <p>
     *
     * @param angularVelocityInRadiansPerSecond Normalized value
     */
    private void setSteerOpenLoop(double angularVelocityInRadiansPerSecond) {
        mPeriodicIO.steerControlMode = ControlMode.PercentOutput;
        mPeriodicIO.steerDemand = angularVelocityInRadiansPerSecond;
    }

    /**
     * Sets the motor controller settings and values for the Drive motor.
     * <p>
     *
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
        return encUnits / mConfig.kSteerMotorTicksPerRadian;
    }

    private double radiansToEncoderUnits(double radians) {
        return radians * mConfig.kSteerMotorTicksPerRadian;
    }

    private int degreesToEncUnits(double degrees) {
        return (int) radiansToEncoderUnits(Math.toRadians(degrees));
    }

    private double encUnitsToDegrees(double encUnits) {
        return Math.toDegrees(encoderUnitsToRadians(encUnits));
    }

    // Drive motor
    private double encoderUnitsToDistance(double ticks) {
        return ticks * mConfig.kDriveTicksPerUnitDistance;
    }

    private double encoderUnitsToVelocity(double ticks) {
        return ticks * mConfig.kDriveTicksPerUnitVelocity;
    }

    private double distanceToEncoderUnits(double distanceInMeters) {
        return distanceInMeters / mConfig.kDriveTicksPerUnitDistance;
    }

    // private double encUnitsToInches(double encUnits) {
    // return Units.metersToInches(encoderUnitsToDistance(encUnits));
    // }
    //
    // private double inchesToEncUnits(double inches) {
    // return distanceToEncoderUnits(Units.inchesToMeters(inches));
    // }

    private double metersPerSecondToEncVelocity(double metersPerSecond) {
        return metersPerSecond / mConfig.kDriveTicksPerUnitVelocity;
    }

    private double encVelocityToMetersPerSecond(double encUnitsPer100ms) {
        return encUnitsPer100ms * mConfig.kDriveTicksPerUnitVelocity;
    }

    /**
     * Sets the mode of operation during neutral throttle output.
     * <p>
     *
     * @param neutralMode The desired mode of operation when the Talon FX
     *                    Controller output throttle is neutral (ie brake/coast)
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
            String shortName = mModuleName;

            return shortName + ".steerPosition," +
                    shortName + ".drivePosition," +
                    shortName + ".steerControlMode," +
                    shortName + ".driveControlMode," +
                    shortName + ".steerDemand," +
                    shortName + ".driveDemand," +
                    shortName + ".steerCurrent," +
                    shortName + ".driveCurrent";
        }
        return null;
    }

    private String generateLogValues(boolean telemetry) {
        String values;

        if (telemetry) {
            values = "" + mPeriodicIO.steerPosition + "," +
                    mPeriodicIO.drivePosition + "," +
                    mPeriodicIO.steerControlMode + "," +
                    mPeriodicIO.driveControlMode + "," +
                    mPeriodicIO.steerDemand + "," +
                    mPeriodicIO.driveDemand + "," +
                    mPeriodicIO.steerCurrent + "," +
                    mPeriodicIO.driveCurrent;
        } else {
            values = "" + mPeriodicIO.steerPosition + "," +
                    mPeriodicIO.drivePosition + "," +
                    mPeriodicIO.steerControlMode + "," +
                    mPeriodicIO.driveControlMode + "," +
                    mPeriodicIO.steerDemand + "," +
                    mPeriodicIO.driveDemand + "," +
                    /* periodicIO.steerCurrent+ */"," +
                    /* periodicIO.driveCurrent+ */",";
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
        mPeriodicIO.driveVelocity = mDriveMotor.getSelectedSensorVelocity(0);
        mPeriodicIO.steerVelocity = mSteerMotor.getSelectedSensorVelocity(0);
        mPeriodicIO.steerError = mSteerMotor.getClosedLoopError(0);

        mPeriodicIO.steerCurrent = mSteerMotor.getStatorCurrent();
        mPeriodicIO.driveCurrent = mDriveMotor.getStatorCurrent();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        switch (mControlState) {
            case OPEN_LOOP:
                // don't move if throttle is 0
                if (Util.epsilonEquals(mPeriodicIO.driveDemand, 0.0, mConfig.kDriveDeadband)) {
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
        // SmartDashboard.putNumber(mModuleName + "Steer velocity",
        // mSteerMotor.getSelectedSensorVelocity(0));
        // SmartDashboard.putNumber(mModuleName + "Steer (cancoder)",
        // enc.getAbsolutePosition()-cancoderOffsetDegrees);
        // SmartDashboard.putNumber(mModuleName + " cancoder",
        // mCANCoder.getAbsolutePosition());
        // SmartDashboard.putNumber(mModuleName + " steerDemand",
        // mPeriodicIO.steerDemand);
        // SmartDashboard.putNumber(mModuleName + " steerPosition",
        // mPeriodicIO.steerPosition);
        // SmartDashboard.putNumber(mModuleName + " driveDemand",
        // mPeriodicIO.driveDemand);
        // SmartDashboard.putNumber(mModuleName + "Steer", periodicIO.drivePosition);
        // SmartDashboard.putNumber(mModuleName + "Velocity",
        // mDriveMotor.getSelectedSensorVelocity(0));
        // SmartDashboard.putNumber(mModuleName + "Velocity",
        // encVelocityToInchesPerSecond(periodicIO.velocity));
        // SmartDashboard.putNumber(mModuleName + ": Current",
        // mSteerMotor.getStatorCurrent());
        if (Constants.kDebuggingOutput) {
            SmartDashboard.putNumber(mModuleName + "Pulse Width", mPeriodicIO.steerPosition);
            if (mPeriodicIO.steerControlMode == ControlMode.MotionMagic)
                SmartDashboard.putNumber(mModuleName + "Error", encUnitsToDegrees(mPeriodicIO.steerError));
            // SmartDashboard.putNumber(mModuleName + "X", position.x());
            // SmartDashboard.putNumber(mModuleName + "Y", position.y());
            SmartDashboard.putNumber(mModuleName + "Steer Speed", mPeriodicIO.steerVelocity);
        }
    }

    public static class PeriodicIO {
        // Inputs
        public int moduleID;
        public int steerPosition;
        public int drivePosition;
        public double steerVelocity;
        public double driveVelocity;
        public double steerCurrent;
        public double driveCurrent;
        public double steerError;

        // Outputs are in units for the motor controller.
        public ControlMode steerControlMode = ControlMode.PercentOutput;
        public ControlMode driveControlMode = ControlMode.PercentOutput;
        public double steerDemand;
        public double driveDemand;
    }
}
