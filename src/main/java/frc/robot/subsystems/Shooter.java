package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Ports;
import libraries.cheesylib.drivers.TalonFXFactory;
import libraries.cheesylib.loops.ILooper;
import libraries.cheesylib.loops.Loop;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.subsystems.SubsystemManager;

public class Shooter extends Subsystem{

    //Hardware
    private final TalonFX mFXLeftFlyWheel, mFXRightFlyWheel;
    private final TalonFX mFXHood; // TODO: Decide if hood adjustment will be controlled by SHOOTING state or by superstructure

    //Subsystem Constants
    private final double kMinShootDistance = 0; // Fender shot is 0
    private final double kMaxShootDistance = 146; // Approximate distance from fender to launch pad (the shooter's location in inches)
    private final double kMinShootSpeed = 12000; // Ticks per 100Ms
    private final double kMaxShootSpeed = 20500;
    private final double kFlywheelSlope = (kMaxShootSpeed - kMinShootSpeed) / (kMaxShootDistance - kMinShootDistance);

    private final double kMinHoodPosition = 0; // Hood at lower hard stop
    private final double kMaxHoodPosition = 27800; // Hood at max hard stop
    private final double kHoodSlope = (kMaxHoodPosition - kMinHoodPosition) / (kMaxShootDistance - kMinShootDistance);

    //Configuration Constants
    private final double kFlywheelKp = 0.1;
    private final double kFlywheelKi = 0.0;
    private final double kFlywheelKd = 0.0;
    private final double kFlywheelKf = 0.05;
    private final double kClosedRamp = 0.5;
    private final double kClosedError = 50.0;

    private final double kFlywheelCurrentLimit = 40;
    private final double kHoodCurrentLimit = 5;

    //Subsystem States
    public enum SystemState {
        HOLDING,
        SHOOTING,
    }

    public enum WantedState {
        HOLD,
        SHOOT,
    }

    private SystemState mSystemState;
    private WantedState mWantedState;
    private boolean mStateChanged;
    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private double mDistance;
    private double mHoldSpeed;

    //Other
    private SubsystemManager mSubsystemManager;
    private int              mListIndex;
    
    //Subsystem Creation
    private static String sClassName;
    private static int sInstanceCount;
    private static Shooter sInstance = null;
    public static Shooter getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new Shooter(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+" getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Shooter(String caller){
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mFXLeftFlyWheel = TalonFXFactory.createDefaultTalon(Ports.LEFT_FLYWHEEL);
        mFXRightFlyWheel = TalonFXFactory.createDefaultTalon(Ports.RIGHT_FLYWHEEL);
        mFXHood = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_HOOD);
        mSubsystemManager = SubsystemManager.getInstance(sClassName);
        configMotors();
    }

    private void configMotors(){
        mFXLeftFlyWheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mFXRightFlyWheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mFXHood.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        // only one encoder is needed
        mFXLeftFlyWheel.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, Constants.kLongCANTimeoutMs);
        mFXHood.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, Constants.kLongCANTimeoutMs);

        // If flywheel makes clicking sound, test config line for both motors
        mFXLeftFlyWheel.setControlFramePeriod(ControlFrame.Control_3_General,18);
        mFXRightFlyWheel.setControlFramePeriod(ControlFrame.Control_3_General,18);
        mFXHood.setControlFramePeriod(ControlFrame.Control_3_General,18);
    
        mFXLeftFlyWheel.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mFXRightFlyWheel.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mFXHood.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mFXLeftFlyWheel.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mFXRightFlyWheel.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mFXHood.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mFXLeftFlyWheel.setInverted(true);
        mFXRightFlyWheel.setInverted(false);
        mFXHood.setInverted(true);

        mFXLeftFlyWheel.setNeutralMode(NeutralMode.Coast);
        mFXRightFlyWheel.setNeutralMode(NeutralMode.Coast);  
        mFXHood.setNeutralMode(NeutralMode.Coast);

        // parameters are enable, current limit after triggering, trigger limit, time allowed to exceed trigger limit before triggering
        mFXLeftFlyWheel.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kFlywheelCurrentLimit, kFlywheelCurrentLimit, 0));
        mFXRightFlyWheel.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kFlywheelCurrentLimit, kFlywheelCurrentLimit, 0));
        mFXHood.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kHoodCurrentLimit, kHoodCurrentLimit, 0));

        mFXRightFlyWheel.selectProfileSlot(0, 0);
        mFXLeftFlyWheel.selectProfileSlot(0, 0);
        mFXHood.selectProfileSlot(0, 0);

        mFXLeftFlyWheel.config_kP(0, kFlywheelKp, Constants.kLongCANTimeoutMs);
        mFXLeftFlyWheel.config_kI(0, kFlywheelKi, Constants.kLongCANTimeoutMs);
        mFXLeftFlyWheel.config_kD(0, kFlywheelKd, Constants.kLongCANTimeoutMs);
        mFXLeftFlyWheel.config_kF(0, kFlywheelKf, Constants.kLongCANTimeoutMs);
        mFXLeftFlyWheel.config_IntegralZone(0, 0, Constants.kLongCANTimeoutMs);
        mFXLeftFlyWheel.configClosedloopRamp(kClosedRamp, Constants.kLongCANTimeoutMs);
        mFXLeftFlyWheel.configAllowableClosedloopError(0, kClosedError, Constants.kLongCANTimeoutMs);

        mFXRightFlyWheel.config_kP(0, kFlywheelKp, Constants.kLongCANTimeoutMs);
        mFXRightFlyWheel.config_kI(0, kFlywheelKi, Constants.kLongCANTimeoutMs);
        mFXRightFlyWheel.config_kD(0, kFlywheelKd, Constants.kLongCANTimeoutMs);
        mFXRightFlyWheel.config_kF(0, kFlywheelKf, Constants.kLongCANTimeoutMs);
        mFXRightFlyWheel.config_IntegralZone(0, 0, Constants.kLongCANTimeoutMs);
        mFXRightFlyWheel.configClosedloopRamp(kClosedRamp, Constants.kLongCANTimeoutMs);
        mFXRightFlyWheel.configAllowableClosedloopError(0, kClosedError, Constants.kLongCANTimeoutMs);

        mFXHood.config_kP(0, 0.5, Constants.kLongCANTimeoutMs); //May need to tune more?
        mFXHood.config_kI(0, 0, Constants.kLongCANTimeoutMs);
        mFXHood.config_kD(0, 0, Constants.kLongCANTimeoutMs);
        mFXHood.config_kF(0, 0, Constants.kLongCANTimeoutMs);
        mFXHood.config_IntegralZone(0, 0, Constants.kLongCANTimeoutMs);
        mFXHood.configClosedloopRamp(kClosedRamp, Constants.kLongCANTimeoutMs);
        mFXHood.configAllowableClosedloopError(0, kClosedError, Constants.kLongCANTimeoutMs);

    }

    private Loop mLoop = new Loop() {

        @Override
        public void onStart(Phase phase){
            synchronized (Shooter.this) {
                mSystemState = SystemState.HOLDING;
                mWantedState = WantedState.HOLD;
                mStateChanged = true;
                System.out.println(sClassName + " state " + mSystemState);
                // this subsystem is "on demand" so
                mPeriodicIO.schedDeltaDesired = 0;
                mFXHood.setSelectedSensorPosition(0.0); // TODO: Create homing sequence instead
                stop(); // put into a known state
            }
        }

        @Override
        public void onLoop(double timestamp){
            synchronized (Shooter.this) {
                SystemState newState;
                switch (mSystemState) {
                case SHOOTING:
                    newState = handleShooting();
                    break;
                case HOLDING:
                default:
                    newState = handleHolding();
                    break;
                }

                if (newState != mSystemState) {
                    System.out.println(sClassName + " state " + mSystemState + " to " + newState + " (" + timestamp + ")");
                    mSystemState = newState;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }
        }

        @Override
        public void onStop(double timestamp){
            stop();
        }

    };

    public synchronized void setWantedState(WantedState state) {
        if (state != mWantedState) {
            mSubsystemManager.scheduleMe(mListIndex, 1, false);
            System.out.println("waking " + sClassName);
        }

        mWantedState = state;
    }

    private SystemState handleHolding() {
        mPeriodicIO.hoodDemand = mPeriodicIO.hoodPosition;

        return defaultStateTransfer();
    }
    
    private SystemState handleShooting() {
        if(mStateChanged){
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
        }
        mPeriodicIO.flywheelVelocityDemand = distanceToTicksPer100Ms(mDistance);
        mPeriodicIO.hoodDemand = distanceToHoodPos(mDistance);

        return defaultStateTransfer();
    }

    private SystemState defaultStateTransfer(){
        switch(mWantedState){
            case SHOOT:
                return SystemState.SHOOTING;
            case HOLD:
            default:
                return SystemState.HOLDING;
        }
    }

    public synchronized boolean readyToShoot() {
        return mPeriodicIO.reachedDesiredSpeed && mPeriodicIO.reachedDesiredHoodPosition;
    }

    public synchronized void setShootDistance(double distance) {
        if(distance != mDistance){
            mPeriodicIO.reachedDesiredSpeed = false;
            mPeriodicIO.reachedDesiredHoodPosition = false;
            mDistance = Math.max(kMinShootDistance, Math.min(distance, kMaxShootDistance));
        }
        
    }

    // TODO: Confirm if distance to flywheel velocity relation is linear
    private double distanceToTicksPer100Ms(double distance){
        return (kFlywheelSlope * distance) + kMinShootSpeed;
    }

    // Hood range is from 0 to 27800 ticks - TODO: Verify range
    // Hood range is from 15.82 degrees to 35.82 degrees 
    // 1390 falcon ticks per degree
    private double distanceToHoodPos(double distance){
        return (kHoodSlope * distance) + kMinHoodPosition;
    }

    // Motor to flywheel pulley ratio is 60/16 (15/4 reduced)
    // Multiply by 2048 (falcon ticks per revolution) to get 7567.5
    // private double ticksPer100MsToRPM(double speed) {
    //     return speed / 7567.5 * 1000.0 / 100.0 * 60.0;
    // }

    @Override
    public void readPeriodicInputs() {
        double now       = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart   = now;
        
        mPeriodicIO.flywheelVelocity = mFXLeftFlyWheel.getSelectedSensorVelocity();
        mPeriodicIO.reachedDesiredSpeed = mFXLeftFlyWheel.getClosedLoopError() <= (kClosedError * 2); // mPeriodicIO.flywheelVelocity >= mPeriodicIO.flywheelVelocityDemand;
        
        mPeriodicIO.hoodPosition = mFXHood.getSelectedSensorPosition();
        mPeriodicIO.reachedDesiredHoodPosition = mFXHood.getClosedLoopError() <= (kClosedError * 2);
    }

    @Override
    public void writePeriodicOutputs() {
        if(mSystemState == SystemState.SHOOTING){
            System.out.println(mPeriodicIO.flywheelVelocityDemand);
            mFXLeftFlyWheel.set(ControlMode.Velocity, mPeriodicIO.flywheelVelocityDemand);  
            mFXRightFlyWheel.set(ControlMode.Velocity, mPeriodicIO.flywheelVelocityDemand);
        }
        mFXHood.set(ControlMode.Position, mPeriodicIO.hoodDemand);
    }


    @Override
    public void stop() {
        mFXLeftFlyWheel.set(ControlMode.PercentOutput, 0.0);
        mFXRightFlyWheel.set(ControlMode.PercentOutput, 0.0);
        mFXHood.set(ControlMode.PercentOutput, 0.0);

        mPeriodicIO.flywheelVelocityDemand = 0.0;
        mPeriodicIO.hoodDemand = mPeriodicIO.hoodPosition;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        mListIndex = enabledLooper.register(mLoop);
    }

    @Override
    public int whenRunAgain () {
        if (mStateChanged && mPeriodicIO.schedDeltaDesired == 0){
            return 1; // one more loop before going to sleep
        }

        return mPeriodicIO.schedDeltaDesired;
    }

    @Override
    public String getLogHeaders() {
        // TODO Auto-generated method stub
        return "Shooter";
    }

    @Override
    public String getLogValues(boolean telemetry) {
        // TODO Auto-generated method stub
        return "Shooter.Values";
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Hood Position", mPeriodicIO.hoodPosition);
        SmartDashboard.putNumber("Flywheel Speed", mPeriodicIO.flywheelVelocity);
        SmartDashboard.putNumber("Right Stator Current", mFXRightFlyWheel.getStatorCurrent());
        SmartDashboard.putNumber("Left Stator Current", mFXLeftFlyWheel.getStatorCurrent());        
    }

    public static class PeriodicIO{
        //Logging
        private final int mDefaultSchedDelta = 20; // axis updated every 20 msec
        private int    schedDeltaDesired;
        public  double schedDeltaActual;
        public  double schedDuration;
        private double lastSchedStart;

        //Inputs
        private boolean reachedDesiredSpeed;
        private boolean reachedDesiredHoodPosition;
        private double flywheelVelocity;
        private double hoodPosition;

        //Outputs
        private double flywheelVelocityDemand;
        private double hoodDemand;
    }

}