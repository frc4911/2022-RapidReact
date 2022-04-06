package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.SwerveModuleConfiguration;
import frc.robot.constants.Constants;
import libraries.cheesylib.drivers.TalonFXFactory;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.util.Util;
import libraries.cyberlib.control.FramePeriodSwitch;
import libraries.cyberlib.kinematics.SwerveModuleState;
import libraries.cyberlib.utils.Angles;

import java.util.Arrays;

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

        mDriveMotor = TalonFXFactory.createDefaultTalon(mConfig.kDriveMotorTalonId, Constants.kCanivoreName);
        mSteerMotor = TalonFXFactory.createDefaultTalon(mConfig.kSteerMotorTalonId, Constants.kCanivoreName);
        mCANCoder = new CANCoder(constants.kCANCoderId, Constants.kCanivoreName);
        configCancoder();
        configureMotors();

        System.out.println("Be sure to reset convertCancoderToFX2() call before DCMP's");
        convertCancoderToFX2(true);
    }

    private void configCancoder(){
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.initializationStrategy = mConfig.kCANCoderSensorInitializationStrategy;
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.magnetOffsetDegrees = mConfig.kCANCoderOffsetDegrees;
        config.sensorDirection = false; // TODO - Make cancoder direction configurable through robot config files

        mCANCoder.configAllSettings(config, Constants.kLongCANTimeoutMs);

        // mCANCoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults,
        //         mConfig.kCANCoderStatusFramePeriodVbatAndFaults);
        // mCANCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, mConfig.kCANCoderStatusFramePeriodSensorData);
    }
    /**
     * Configure motors based on current SwerveModuleConstants.
     */
    private void configureMotors() {

        mCANCoder.configMagnetOffset(mConfig.kCANCoderOffsetDegrees, 200);

        commonMotorConfig(mDriveMotor, "Drive");
        commonMotorConfig(mSteerMotor, "Steer");

        mSteerMotor.setInverted(mConfig.kInvertSteerMotor);
        mSteerMotor.configMotionAcceleration(0.9 * mConfig.kSteerTicksPerUnitVelocity * 0.25, Constants.kLongCANTimeoutMs);
        mSteerMotor.configMotionCruiseVelocity(0.9 * mConfig.kSteerTicksPerUnitVelocity,Constants.kLongCANTimeoutMs);
        mSteerMotor.configVelocityMeasurementPeriod(mConfig.kSteerMotorVelocityMeasurementPeriod, Constants.kLongCANTimeoutMs);
        mSteerMotor.configVelocityMeasurementWindow(mConfig.kSteerMotorVelocityMeasurementWindow, Constants.kLongCANTimeoutMs);
        mSteerMotor.selectProfileSlot(0, 0);

        // Slot 0 is for normal use (tuned for fx integrated encoder)
        mSteerMotor.config_kP(0, mConfig.kSteerMotorSlot0Kp, Constants.kLongCANTimeoutMs);
        mSteerMotor.config_kI(0, mConfig.kSteerMotorSlot0Ki, Constants.kLongCANTimeoutMs);
        mSteerMotor.config_kD(0, mConfig.kSteerMotorSlot0Kd, Constants.kLongCANTimeoutMs);
        mSteerMotor.config_kF(0, mConfig.kSteerMotorSlot0Kf, Constants.kLongCANTimeoutMs);
        mSteerMotor.config_IntegralZone(0, mConfig.kSteerMotorSlot0IZone, Constants.kLongCANTimeoutMs);

        mDriveMotor.setInverted(mConfig.kInvertDrive);
        mDriveMotor.configOpenloopRamp(0.3, Constants.kLongCANTimeoutMs); // Increase if swerve acceleration is too fast

        FramePeriodSwitch.configStatorCurrentLimitPermanent(mDriveMotor, new StatorCurrentLimitConfiguration(true, 90, 90, 0));
    }

    private void commonMotorConfig(TalonFX motor, String motorName){
        System.out.println("configuring "+motorName+" motor");

        // The following commands are stored in nonVolatile ram in the motor
        // They are repeated on boot incase a motor needs to replaced quickly
        FramePeriodSwitch.configFactoryDefaultPermanent(motor);

        // the following commands are stored in nonVolatile ram but they are
        // no longer deemed necessary. Keeping around for a while in case they
        // need to be brought back
        // motor.configNeutralDeadband(.04, 100);
        // motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, 100);
        // motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, 100);

        // the following are volatile settings and must be run every power cycle
        FramePeriodSwitch.setFramePeriodsVolatile(motor); // set frame periods

        FramePeriodSwitch.setNeutralModeVolatile(motor, NeutralMode.Brake);
    }

    protected void convertCancoderToFX2(){
        convertCancoderToFX2(true);
    }
    protected void convertCancoderToFX2(boolean useCancoders){
        int limit = 500;
        // mCANCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 50);
        double lastFrameTimestamp = -1;
        double frameTimestamp;
        double position;
        int index = 0;
        double[] cancoderPositions = {-10000,-1000,-100,-1,100}; // just need to be different numbers
        boolean allDone = false;
        double cancoderDegrees=0;

        if (useCancoders){
            do{
                frameTimestamp = mCANCoder.getLastTimestamp();
                if (frameTimestamp != lastFrameTimestamp){
                    position = mCANCoder.getAbsolutePosition();
                    cancoderPositions[(++index)%cancoderPositions.length]=position;
                    allDone = true;
                    for (int kk=0; kk<cancoderPositions.length;kk++){
                        if ( Math.abs(cancoderPositions[kk]-cancoderPositions[(kk+1)%cancoderPositions.length]) >1 ){
                            allDone=false;
                            break;
                        }
                    }
                    // System.out.println(limit+" ("+mModuleName+")"+": CANCoder last frame timestamp = "+ frameTimestamp + 
                    //                     " current time = "+Timer.getFPGATimestamp() +" pos="+position);
                    lastFrameTimestamp = frameTimestamp;
                }
                Timer.delay(.1);

            } while (!allDone && (limit-- > 0));
            
            // System.out.println(mModuleName+": allDone "+Arrays.toString(cancoderPositions));
            cancoderDegrees = cancoderPositions[0];
        }
        else{
            System.out.println(mModuleName+" assuming wheels aligned to 0 degrees, not using CANCoders");
            cancoderDegrees = 0;
        }
        double fxTicksBefore = mSteerMotor.getSelectedSensorPosition();
        double fxTicksTarget = degreesToEncUnits(cancoderDegrees);
        double fxTicksNow = fxTicksBefore;
        int loops = 0;
        final double acceptableTickErr = 10;

        while ((Math.abs(fxTicksNow - fxTicksTarget) > acceptableTickErr) && (loops < 5)) {
            mSteerMotor.setSelectedSensorPosition(fxTicksTarget, 0, 0);
            Timer.delay(.1);
            fxTicksNow = mSteerMotor.getSelectedSensorPosition();
            loops++;
        }

        System.out.println(mConfig.kName + " cancoder degrees: " + cancoderDegrees +
                ",  fx encoder ticks (before, target, adjusted): (" + fxTicksBefore + "," + fxTicksTarget + ","
                + fxTicksNow + ") loops:" + loops);
    }

    // protected void convertCancoderToFX(){
    //     // multiple (usually 2) sets were needed to set new encoder value
    //     double fxTicksBefore = mSteerMotor.getSelectedSensorPosition();
    //     double cancoderDegrees = mCANCoder.getAbsolutePosition();
        
    //     int limit = 5;
    //     do{
    //         if (mCANCoder.getLastError() != ErrorCode.OK) {
    //             System.out.println("error reading cancoder. Trying again!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! value read was "
    //                     + cancoderDegrees);
    //             Timer.delay(.1);
    //             cancoderDegrees = mCANCoder.getAbsolutePosition();
    //         }
    //         else{
    //             break;
    //         }
    //     }while (limit-- > 0);

    //     double fxTicksTarget = degreesToEncUnits(cancoderDegrees);
    //     double fxTicksNow = fxTicksBefore;
    //     int loops = 0;
    //     final double acceptableTickErr = 10;

    //     while (Math.abs(fxTicksNow - fxTicksTarget) > acceptableTickErr && loops < 5) {
    //         mSteerMotor.setSelectedSensorPosition(fxTicksTarget, 0, 0);
    //         Timer.delay(.1);
    //         fxTicksNow = mSteerMotor.getSelectedSensorPosition();
    //         loops++;
    //     }

    //     System.out.println(mConfig.kName + " cancoder degrees: " + cancoderDegrees +
    //             ",  fx encoder ticks (before, target, adjusted): (" + fxTicksBefore + "," + fxTicksTarget + ","
    //             + fxTicksNow + ") loops:" + loops);
    // }

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

    @Override
    public synchronized void stop() {
        if (mControlState != ControlState.NEUTRAL) {
            mControlState = ControlState.NEUTRAL;
        }

        mDriveMotor.set(ControlMode.PercentOutput, 0.0);
        mSteerMotor.set(ControlMode.PercentOutput, 0.0);
        mPeriodicIO.driveDemand = 0;
        mPeriodicIO.steerDemand = 0;
        mPeriodicIO.driveControlMode = ControlMode.PercentOutput;
        mPeriodicIO.steerControlMode = ControlMode.PercentOutput;
    }

    @Override
    public String getLogHeaders() {
        return  mModuleName + ".driveDemand," +
                mModuleName + ".drivePosition," +
                mModuleName + ".steerDemand," +
                mModuleName + ".steerPosition," +
                mModuleName + ".driveCurrent,"+
                mModuleName + ".steerCurrent";
    }

    @Override
    public String getLogValues(boolean telemetry) {
        return  mPeriodicIO.driveDemand + "," +
                mPeriodicIO.drivePosition + "," +
                mPeriodicIO.steerDemand + "," +
                mPeriodicIO.steerPosition + "," +
                mPeriodicIO.driveCurrent + "," +
                mPeriodicIO.steerCurrent;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.steerPosition = (int) mSteerMotor.getSelectedSensorPosition();
        mPeriodicIO.drivePosition = (int) mDriveMotor.getSelectedSensorPosition();
        mPeriodicIO.driveVelocity = mDriveMotor.getSelectedSensorVelocity();
        mPeriodicIO.steerVelocity = mSteerMotor.getSelectedSensorVelocity();
        // mPeriodicIO.steerError = mSteerMotor.getClosedLoopError(0);
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
                mSteerMotor.set(mPeriodicIO.steerControlMode, mPeriodicIO.steerDemand);
                mDriveMotor.set(mPeriodicIO.driveControlMode, mPeriodicIO.driveDemand);
                break;
        }
    }

    @Override
    public void outputTelemetry() {
        mPeriodicIO.steerCurrent = FramePeriodSwitch.getStatorCurrent(mSteerMotor);
        mPeriodicIO.driveCurrent = FramePeriodSwitch.getStatorCurrent(mDriveMotor);
        // SmartDashboard.putNumber(mModuleName + "Steer velocity",
        // mSteerMotor.getSelectedSensorVelocity(0));
        // SmartDashboard.putNumber(mModuleName + "Steer (cancoder)",enc.getAbsolutePosition()-cancoderOffsetDegrees);
        SmartDashboard.putNumber(mModuleName + " cancoder abs",mCANCoder.getAbsolutePosition());
        SmartDashboard.putNumber(mModuleName + " cancoder reg",mCANCoder.getPosition());
        // SmartDashboard.putNumber(mModuleName + " steerDemand",
        // mPeriodicIO.steerDemand);
        SmartDashboard.putNumber(mModuleName + " steerPosition", mPeriodicIO.steerPosition);
        SmartDashboard.putNumber(mModuleName + " drivePosition", mPeriodicIO.drivePosition);
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
                // SmartDashboard.putNumber(mModuleName + "Error", encUnitsToDegrees(mPeriodicIO.steerError));
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
