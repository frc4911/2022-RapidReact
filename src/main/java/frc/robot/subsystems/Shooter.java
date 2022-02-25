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
import frc.robot.subsystems.Climber.WantedState;
import libraries.cheesylib.drivers.TalonFXFactory;
import libraries.cheesylib.loops.ILooper;
import libraries.cheesylib.loops.Loop;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.subsystems.SubsystemManager;
import libraries.cheesylib.util.LatchedBoolean;

public class Shooter extends Subsystem{

    //Hardware
    private final TalonFX mFXLeftFlyWheel, mFXRightFlyWheel;
    private final TalonFX mFXHood;

    //Subsystem Constants
    private final double kMinShootDistance = 0; // Fender shot is 0
    private final double kMaxShootDistance = 146; // Approximate distance from fender to launch pad (the shooter's location in inches)
    private double kMinShootSpeed = 10400; // Ticks per 100Ms
    private final double kMaxShootSpeed = 20500;
    private double kFlywheelSlope = (kMaxShootSpeed - kMinShootSpeed) / (kMaxShootDistance - kMinShootDistance);

    private double kMinHoodPosition = 2000; // Hood at lower hard stop
    private final double kMaxHoodPosition = 27800; // Hood at max hard stop
    private double kHoodSlope = (kMaxHoodPosition - kMinHoodPosition) / (kMaxShootDistance - kMinShootDistance);

    //Configuration Constants
    private final double kFlywheelKp = 0.1;
    private final double kFlywheelKi = 0.0;
    private final double kFlywheelKd = 0.0;
    private final double kFlywheelKf = 0.05;
    private final double kClosedRamp = 0.5;
    private final double kClosedError = 25.0;

    private final double kFlywheelCurrentLimit = 40;
    private final double kHoodCurrentLimit = 5;

    //Subsystem States
    public enum SystemState {
        DISABLING,
        HOMINGHOOD,
        HOLDING,
        SHOOTING
    }

    public enum WantedState {
        DISABLE,
        HOMEHOOD,
        HOLD,
        SHOOT,
    }

    private SystemState mSystemState;
    private WantedState mWantedState;
    private boolean mStateChanged;
    private LatchedBoolean mSystemStateChange = new LatchedBoolean();
    private PeriodicIO mPeriodicIO = new PeriodicIO();

    // hood homing state variables
    // homing is done by sending the hood to a negative position
    // while watching for the hood encoder to stop changing for a sufficient amount of time
    private boolean hoodHomed; // global flag
    private final double hoodMovementThreshhold = 5; // encoder movements below this threshhold are considered stopped
    private final double hoodNonMovementDuration = .25; // reading below threshhold encoder reads for this long is considered stopped
    private double hoodNonMovementTimeout; // timestamp of when low readings are sufficient
    private WantedState wantedStateAfterHoming = WantedState.HOLD; // state to transition to after homed
    private final double hoodHomingDemand = -2 * kMaxHoodPosition; // a number negative enough to drive past 0 regardless of where started

    double minSpeed;
    double minHood;

    private double mDistance = -1;

    //Other
    private SubsystemManager mSubsystemManager;
    private int              mListIndex;
    private boolean alreadyScheduled = false;    
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
        // double minSpeed = SmartDashboard.getNumber("Set Shoot Speed", -1.0);
        // double minHood = SmartDashboard.getNumber("Set Hood Angle", -1.0);
        // if(minSpeed == -1.0){
        //     SmartDashboard.putNumber("Set Shoot Speed", 0.0);
        // }
        // if(minHood == -1.0){
        //     SmartDashboard.putNumber("Set Hood Angle", 0.0);
        // }
        configMotors();
    }

    private void configMotors(){
        mFXLeftFlyWheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mFXRightFlyWheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mFXHood.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        // only one encoder is needed
        mFXLeftFlyWheel.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, Constants.kLongCANTimeoutMs);
        mFXHood.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, Constants.kLongCANTimeoutMs);

        // Prevents flywheel clicking sound
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
        mFXHood.configClosedloopRamp(0/*kClosedRamp*/, Constants.kLongCANTimeoutMs);
        mFXHood.configAllowableClosedloopError(0, kClosedError, Constants.kLongCANTimeoutMs);

    }

    @Override
    public void onStart(Phase phase){
        synchronized (Shooter.this) {
            mStateChanged = true;
            switch (phase) {
                case DISABLED:
                case TEST:
                    mSystemState = SystemState.DISABLING;
                    mWantedState = WantedState.DISABLE;
                    mPeriodicIO.schedDeltaDesired = 0; // goto sleep
                    break;
                case AUTONOMOUS:
                case TELEOP:
                    mSystemState = SystemState.HOLDING;
                    mWantedState = WantedState.HOLD;
                    hoodHomed = false;
                    setShootDistance(0);
                    mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
                    break;
            }
            System.out.println(sClassName + " state " + mSystemState);
        }
    }

    @Override
    public void onLoop(double timestamp){
        synchronized (Shooter.this) {
            int loopCount = 0;
            do{
                SystemState newState;
                switch (mSystemState) {
                case DISABLING:
                    newState = handleDisabling();
                    break;
                case HOMINGHOOD:
                    newState = handleHomingHood();
                    break;
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
                loopCount++;
            } while(mSystemStateChange.update(mStateChanged));
            if (loopCount>1){
                System.out.println(sClassName+" onLoop loops "+loopCount);
            }
        }
    }

    public synchronized void setWantedState(WantedState state) {
        if (state != mWantedState) {
            mWantedState = state;
            mSubsystemManager.scheduleMe(mListIndex, 12, true);
            System.out.println("waking " + sClassName + " to "+state);
        }
    }

    private SystemState handleDisabling() {
        if(mStateChanged){
            stop();
        }

        return defaultStateTransfer();
    }
    
    private SystemState handleHomingHood() {
        double now = Timer.getFPGATimestamp();
        if(mStateChanged){
            hoodHomed = false;
            hoodNonMovementTimeout = now+hoodNonMovementDuration;
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
        }

        double distance = Math.abs(mPeriodicIO.hoodPosition - mPeriodicIO.lastHoodPosition);
        if (distance > hoodMovementThreshhold){
            hoodNonMovementTimeout = now+hoodNonMovementDuration;
        }

        if (now > hoodNonMovementTimeout){
            mFXHood.setSelectedSensorPosition(0);
            hoodHomed = true;
            mWantedState = wantedStateAfterHoming;
        }

        return defaultStateTransfer();
    }
    
    private SystemState handleHolding() {
        if(mStateChanged){
            if (!hoodHomed){
                wantedStateAfterHoming = WantedState.HOLD;
                mWantedState = WantedState.HOMEHOOD;
            }
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
        }
        updateShooterStatus();

        return defaultStateTransfer();
    }
    
    private SystemState handleShooting() {
        if(mStateChanged){
            if (!hoodHomed){
                wantedStateAfterHoming = WantedState.SHOOT;
                mWantedState = WantedState.HOMEHOOD;
            }
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
        }
        updateShooterStatus();

        return defaultStateTransfer();
    }

    private SystemState defaultStateTransfer(){
        switch(mWantedState){
            case SHOOT:
                return SystemState.SHOOTING;
            case DISABLE:
                return SystemState.DISABLING;
            case HOMEHOOD:
                return SystemState.HOMINGHOOD;
            case HOLD:
            default:
                return SystemState.HOLDING;
        }
    }

    public synchronized boolean readyToShoot() {
        return mSystemState == SystemState.SHOOTING && (mPeriodicIO.reachedDesiredSpeed && mPeriodicIO.reachedDesiredHoodPosition);
    }

    public synchronized void setShootDistance(double distance) {
        if(distance != mDistance){
            mDistance = Math.max(kMinShootDistance, Math.min(distance, kMaxShootDistance));
            updateShooterStatus();
        }
    }

    private void updateShooterStatus() {
        mPeriodicIO.flywheelVelocityDemand = distanceToTicksPer100Ms(mDistance);
        mPeriodicIO.reachedDesiredSpeed = Math.abs(mPeriodicIO.flywheelVelocity - mPeriodicIO.flywheelVelocityDemand) <= 300;

        if (hoodHomed){
            mPeriodicIO.hoodDemand = distanceToHoodPos(mDistance);
            mPeriodicIO.reachedDesiredHoodPosition = Math.abs(mPeriodicIO.hoodPosition - mPeriodicIO.hoodDemand) <= 100;
        }
        else {
            mPeriodicIO.hoodDemand = hoodHomingDemand;
            mPeriodicIO.reachedDesiredHoodPosition = false;
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

    @Override
    public void readPeriodicInputs() {
        double now       = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart   = now;

        mPeriodicIO.flywheelVelocity = mFXLeftFlyWheel.getSelectedSensorVelocity();
        mPeriodicIO.lastHoodPosition = mPeriodicIO.hoodPosition;      
        mPeriodicIO.hoodPosition = mFXHood.getSelectedSensorPosition();

        if (!hoodHomed){
            System.out.println("readIO  currentPos: "+mPeriodicIO.hoodPosition+" now: "+now+" index: "+mListIndex);
        }
        updateShooterStatus();
    }

    @Override
    public void writePeriodicOutputs() {
        setWheels(mPeriodicIO.flywheelVelocityDemand, mPeriodicIO.hoodDemand);
    }

    private void setWheels(double flyDemand, double hoodDemand){
        mFXLeftFlyWheel.set(ControlMode.Velocity, flyDemand);  
        mFXRightFlyWheel.set(ControlMode.Velocity, flyDemand);
        mFXHood.set(ControlMode.Position, hoodDemand);
    }

    @Override
    public void stop() {
        System.out.println(sClassName + " stop()");
        mPeriodicIO.flywheelVelocityDemand = 0;
        mPeriodicIO.hoodDemand = mPeriodicIO.hoodPosition;
        setWheels(mPeriodicIO.flywheelVelocityDemand, mPeriodicIO.hoodDemand);
    }
    
    @Override
    public void passInIndex(int listIndex) {
        mListIndex = listIndex;
        System.out.println(sClassName+" listIndex = "+mListIndex);
    }

    @Override
    public int whenRunAgain () {
        if (alreadyScheduled){
            alreadyScheduled = false;
            return 0;
        }
        else{
            return mPeriodicIO.schedDeltaDesired;
        }
    }

    @Override
    public String getLogHeaders() {
        return "Shooter";
    }

    @Override
    public String getLogValues(boolean telemetry) {
        return "Shooter.Values";
    }

    @Override
    public void outputTelemetry() {
        mPeriodicIO.hoodCurrent = mFXHood.getStatorCurrent();
        mPeriodicIO.flyLeftCurrent = mFXLeftFlyWheel.getStatorCurrent();
        mPeriodicIO.flyRightCurrent = mFXRightFlyWheel.getStatorCurrent();

        SmartDashboard.putNumber("Hood Position", mPeriodicIO.hoodPosition);
        SmartDashboard.putNumber("Flywheel Speed", mPeriodicIO.flywheelVelocity);
        SmartDashboard.putBoolean("Reached Desired Speed", mPeriodicIO.reachedDesiredSpeed);
        SmartDashboard.putBoolean("Reached Desired Hood", mPeriodicIO.reachedDesiredHoodPosition);
        SmartDashboard.putBoolean("Ready To Shoot", readyToShoot());
        SmartDashboard.putNumber("Flywheel Right Current", mPeriodicIO.flyRightCurrent);
        SmartDashboard.putNumber("Flywheel Left Current", mPeriodicIO.flyLeftCurrent);  
        SmartDashboard.putNumber("Hood Current", mPeriodicIO.hoodCurrent);    
    }

    public static class PeriodicIO{
        //Logging
        private final int mDefaultSchedDelta = 60;//20; // loop run every 20 msec
        private int    schedDeltaDesired;
        public  double schedDeltaActual;
        public  double schedDuration;
        private double lastSchedStart;

        //Inputs
        private double flywheelVelocity;
        private double hoodPosition;
        private double hoodCurrent;
        private double flyRightCurrent;
        private double flyLeftCurrent;

        //Outputs
        private double flywheelVelocityDemand;
        private double hoodDemand;

        // Other
        private double lastHoodPosition;
        private boolean reachedDesiredSpeed;
        private boolean reachedDesiredHoodPosition;
    }

}