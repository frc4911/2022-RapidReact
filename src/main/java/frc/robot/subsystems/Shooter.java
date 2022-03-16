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
import frc.robot.constants.Constants;
import frc.robot.constants.Ports;
import libraries.cheesylib.drivers.TalonFXFactory;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.subsystems.SubsystemManager;
import libraries.cheesylib.util.LatchedBoolean;

public class Shooter extends Subsystem {

    // Hardware
    private final TalonFX mFXLeftFlyWheel, mFXRightFlyWheel;
    private final TalonFX mFXHood;

    // Subsystem Constants
    private final double kMinShootDistance = 0; // Fender shot is 0
    private final double kMaxShootDistance = 102; // Approximate distance of tarmac shot to fender (max wanted shot)
                                                  // (the shooter's location in inches)
    private final double kMinShootSpeed = 10920; // Ticks per 100Ms // TODO: Tune pid so actual flywheel is closer to
                                                 // this speed
    private final double kMaxShootSpeed = 12500;
    private final double kFlywheelSlope = (kMaxShootSpeed - kMinShootSpeed) / (kMaxShootDistance - kMinShootDistance);

    private final double kMinHoodPosition = 3000; // Hood at wanted angle for fender shot
    private final double kMaxHoodPosition = 22000; // Hood at wanted angle for tarmac shot
    private final double kHoodSlope = (kMaxHoodPosition - kMinHoodPosition) / (kMaxShootDistance - kMinShootDistance);

    // Configuration Constants
    private final double kFlywheelKp = 0.1;
    private final double kFlywheelKi = 0.0;
    private final double kFlywheelKd = 0.0;
    private final double kFlywheelKf = 0.05;
    private final double kClosedRamp = 0.5;
    private final double kClosedError = 25.0;

    private final double kFlywheelCurrentLimit = 40;
    private final double kHoodCurrentLimitLow = 5;
    private final double kHoodCurrentLimitHigh = 15;

    // Subsystem States
    public enum SystemState {
        DISABLING,
        HOMINGHOOD,
        HOLDING,
        SHOOTING,
        TESTING
    }

    public enum WantedState {
        DISABLE,
        HOMEHOOD,
        HOLD,
        SHOOT,
        TEST
    }

    private SystemState mSystemState;
    private WantedState mWantedState;
    private boolean mStateChanged;
    private LatchedBoolean mSystemStateChange = new LatchedBoolean();
    private PeriodicIO mPeriodicIO = new PeriodicIO();

    // hood homing state variables
    // homing is done by sending the hood to a negative position
    // while watching for the hood encoder to stop changing for a sufficient amount
    // of time
    private boolean hoodHomed; // global flag
    private final double hoodNonMovementThreshhold = 5; // encoder movements below this threshhold are considered
                                                        // stopped
    private final double hoodNonMovementDuration = .25; // reading below threshhold encoder reads for this long is
                                                        // considered stopped
    private final double hoodHomingDemand = -2 * kMaxHoodPosition; // a number negative enough to drive past 0
                                                                   // regardless of where started
    private double hoodNonMovementTimeout; // timestamp of when low readings are sufficient
    private WantedState wantedStateAfterHoming = WantedState.HOLD; // state to transition to after homed
    private double hoodEncoderOffset = 0; // used after homing to avoid resetting encoder position

    double minSpeed;
    double minHood;
    private double hoodTestDemand;

    private double mDistance = -1;
    private boolean turnOffFlywheel = false;

    // Other
    private SubsystemManager mSubsystemManager;
    // Subsystem Creation
    private static String sClassName;
    private static int sInstanceCount;
    private static Shooter sInstance = null;

    public static Shooter getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new Shooter(caller);
        } else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller) {
        System.out.println("(" + caller + ") " + " getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Shooter(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mFXLeftFlyWheel = TalonFXFactory.createDefaultTalon(Ports.LEFT_FLYWHEEL);
        mFXRightFlyWheel = TalonFXFactory.createDefaultTalon(Ports.RIGHT_FLYWHEEL);
        mFXHood = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_HOOD);
        mSubsystemManager = SubsystemManager.getInstance(sClassName);
        configMotors();
    }

    private void configMotors() {
        mFXLeftFlyWheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mFXRightFlyWheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mFXHood.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        // only one encoder is needed
        mFXLeftFlyWheel.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, Constants.kLongCANTimeoutMs);
        mFXHood.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, Constants.kLongCANTimeoutMs);

        // Prevents flywheel clicking sound
        mFXLeftFlyWheel.setControlFramePeriod(ControlFrame.Control_3_General, 18);
        mFXRightFlyWheel.setControlFramePeriod(ControlFrame.Control_3_General, 18);
        mFXHood.setControlFramePeriod(ControlFrame.Control_3_General, 18);

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

        // parameters are enable, current limit after triggering, trigger limit, time
        // allowed to exceed trigger limit before triggering
         mFXLeftFlyWheel.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kFlywheelCurrentLimit, kFlywheelCurrentLimit, 0));
        mFXRightFlyWheel.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kFlywheelCurrentLimit, kFlywheelCurrentLimit, 0));

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

        mFXHood.config_kP(0, 0.7, Constants.kLongCANTimeoutMs); // May need to tune more? .7,.1,15
        mFXHood.config_kI(0, 0.04, Constants.kLongCANTimeoutMs);
        mFXHood.config_kD(0, 10.0, Constants.kLongCANTimeoutMs);
        mFXHood.config_kF(0, 0, Constants.kLongCANTimeoutMs);
        mFXHood.config_IntegralZone(0, 200, Constants.kLongCANTimeoutMs);
        mFXHood.configClosedloopRamp(0/* kClosedRamp */, Constants.kLongCANTimeoutMs);
        mFXHood.configAllowableClosedloopError(0, 15 /*kClosedError*/, Constants.kLongCANTimeoutMs);

        mFXHood.configMotionCruiseVelocity(10000); // ticks/100ms // measured 8800 ave ticks/second while traveling full path
        mFXHood.configMotionAcceleration(5400); // ticks/100mse/sec // 2500 ticks/(sec^2)
        mFXHood.configMotionSCurveStrength(0); // trapezoidal curve

    }

    @Override
    public void onStart(Phase phase) {
        synchronized (Shooter.this) {
            mStateChanged = true;
            switch (phase) {
                case TEST:
                    mSystemState = SystemState.TESTING;
                    mWantedState = WantedState.TEST;
                    mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
                    hoodTestDemand = 0;
                    break;
                case DISABLED:
                    mSystemState = SystemState.DISABLING;
                    mWantedState = WantedState.DISABLE;
                    mPeriodicIO.schedDeltaDesired = 0; // goto sleep
                    break;
                case AUTONOMOUS:
                case TELEOP:
                    mSystemState = SystemState.HOLDING;
                    mWantedState = WantedState.HOLD;
                    setShootDistance(0);
                    mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
                    break;
            }
            stop();
            System.out.println(sClassName + " state " + mSystemState);
            if (!hoodHomed){
                mFXHood.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kHoodCurrentLimitLow, kHoodCurrentLimitLow, 0));    
            }
            else{
                mFXHood.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kHoodCurrentLimitHigh, kHoodCurrentLimitHigh, 0));
            }
        }
    }

    @Override
    public void onLoop(double timestamp) {
        synchronized (Shooter.this) {
            do {
                SystemState newState;
                switch (mSystemState) {
                    case DISABLING:
                        newState = handleDisabling();
                        break;
                    case TESTING:
                        newState = handleTesting();
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
                    System.out.println(
                            sClassName + " state " + mSystemState + " to " + newState + " (" + timestamp + ")");
                    mSystemState = newState;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
                // this do/while does a second run through so state changes take place in one
                // onLoop call
            } while (mSystemStateChange.update(mStateChanged));
        }
    }

    // this method should only be used by external subsystems.
    // if you want to change your own wantedState then simply set
    // it directly
    public synchronized void setWantedState(WantedState state, String who) {
        if (state != mWantedState) {
            mWantedState = state;
            mSubsystemManager.scheduleMe(mListIndex, 1, true);
            System.out.println(who + " is setting wanted state of " + sClassName + " to " + state);
        } else {
            System.out.println(who + " is setting wanted state of " + sClassName + " to " + state + " again!!!");
        }
    }

    public synchronized WantedState getWantedState() {
        return mWantedState;
    }
  

    private SystemState handleDisabling() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = 0;
            stop();
        }

        return defaultStateTransfer();
    }

    private SystemState handleHomingHood() {
        double now = Timer.getFPGATimestamp();
        if (mStateChanged) {
            mFXHood.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kHoodCurrentLimitLow, kHoodCurrentLimitLow, 0));    
            hoodHomed = false;
            hoodEncoderOffset = 0;
            hoodNonMovementTimeout = now + hoodNonMovementDuration;
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
        }

        double distance = Math.abs(mPeriodicIO.hoodPosition - mPeriodicIO.lastHoodPosition);
        if (distance > hoodNonMovementThreshhold) {
            hoodNonMovementTimeout = now + hoodNonMovementDuration;
        }

        if (now > hoodNonMovementTimeout) {
            // instead of resetting the sensor the code remembers the offset
            hoodEncoderOffset = mPeriodicIO.hoodPosition;
            hoodHomed = true;
            mFXHood.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kHoodCurrentLimitHigh, kHoodCurrentLimitHigh, 0));
            mWantedState = wantedStateAfterHoming;
            System.out.println("homing is complete");
        }

        return defaultStateTransfer();
    }

    private SystemState handleHolding() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
            if (!hoodHomed) {
                wantedStateAfterHoming = WantedState.HOLD;
                mWantedState = WantedState.HOMEHOOD;
                // if updatestatus is called it will start a magicmotion
                // move that cannot be aborted
                return defaultStateTransfer();
            }
        }
        updateShooterStatus();

        return holdingStateTransfer();
    }

    private SystemState handleTesting() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
            turnOffFlywheel = true; // only testing hood right now
            // test code runs open loop so don't care if it is homed and current limit must be low
            mFXHood.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kHoodCurrentLimitHigh, kHoodCurrentLimitHigh, 0));
        }
        mPeriodicIO.hoodDemand = hoodTestDemand;

        return defaultStateTransfer();
    }

    private SystemState handleShooting() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
            if (!hoodHomed) {
                wantedStateAfterHoming = WantedState.SHOOT;
                mWantedState = WantedState.HOMEHOOD;
                // if updatestatus is called it will start a magicmotion
                // move that cannot be aborted
                return defaultStateTransfer();
            }
        }
        updateShooterStatus();

        return defaultStateTransfer();
    }

    private SystemState holdingStateTransfer() {
        if (mWantedState != WantedState.HOLD) {
            turnOffFlywheel = false;
        }
        return defaultStateTransfer();
    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case SHOOT:
                return SystemState.SHOOTING;
            case TEST:
                return SystemState.TESTING;
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
        return mSystemState == SystemState.SHOOTING
                && (mPeriodicIO.reachedDesiredSpeed && mPeriodicIO.reachedDesiredHoodPosition);
    }

    public synchronized void setShootDistance(double distance) {
        if (distance != mDistance) {
            mDistance = Math.max(kMinShootDistance, Math.min(distance, kMaxShootDistance));
            updateShooterStatus();
        }
    }

    public synchronized void setHoodTestDemand(double hoodTestDemand) {
        this.hoodTestDemand = hoodTestDemand;
    }

    private void updateShooterStatus() {
        mPeriodicIO.flywheelVelocityDemand = distanceToTicksPer100Ms(mDistance);
        mPeriodicIO.reachedDesiredSpeed = Math
                .abs(mPeriodicIO.flywheelVelocity - mPeriodicIO.flywheelVelocityDemand) <= 300;

        if (hoodHomed) {
            mPeriodicIO.hoodDemand = distanceToHoodPos(mDistance);
            mPeriodicIO.reachedDesiredHoodPosition = Math.abs(mPeriodicIO.hoodPosition - mPeriodicIO.hoodDemand) <= 100;
        } else {
            mPeriodicIO.hoodDemand = hoodHomingDemand;
            mPeriodicIO.reachedDesiredHoodPosition = false;
        }
    }

    public void stopFlywheel() {
        if (mSystemState == SystemState.HOLDING) {
            turnOffFlywheel = true;
        }
    }

    private double distanceToTicksPer100Ms(double distance) {
        return (kFlywheelSlope * distance) + kMinShootSpeed;
    }

    // Hood range is from 0 to 27800 ticks
    // Hood range is from 15.82 degrees to 35.82 degrees
    // 1390 falcon ticks per degree
    private double distanceToHoodPos(double distance) {
        return (kHoodSlope * distance) + kMinHoodPosition;
    }

    @Override
    public void readPeriodicInputs() {
        double now = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart = now;

        mPeriodicIO.flywheelVelocity = mFXLeftFlyWheel.getSelectedSensorVelocity();
        mPeriodicIO.lastHoodPosition = mPeriodicIO.hoodPosition;
        mPeriodicIO.hoodPosition = adjustHoodEncoderPosition(mFXHood.getSelectedSensorPosition());

        updateShooterStatus();
    }

    // these next two methods are used in liew of resetting the encoder.
    // resetting the encoder can take a few milliseconds which could
    // cause unexpected behavior if the encoder values jump while PID'ing
    private double adjustHoodEncoderPosition(double rawPosition) {
        return rawPosition - hoodEncoderOffset;
    }

    private double unadjustHoodEncoderPosition(double adjustedPosition) {
        return adjustedPosition + hoodEncoderOffset;
    }

    @Override
    public void writePeriodicOutputs() {
        double velocity = turnOffFlywheel ? 0.0 : mPeriodicIO.flywheelVelocityDemand;
        setMotors(velocity, mPeriodicIO.hoodDemand);

        // SmartDashboard.putNumber("Hood Position", mPeriodicIO.hoodPosition);
        // SmartDashboard.putNumber("Flywheel Speed", mPeriodicIO.flywheelVelocity);
        // SmartDashboard.putNumber("Hood Demand", mPeriodicIO.hoodDemand);
        // SmartDashboard.putNumber("Flywheel demand", mPeriodicIO.flywheelVelocityDemand);

    }

    private void setMotors(double flyDemand, double hoodDemand) {
        mFXLeftFlyWheel.set(ControlMode.Velocity, flyDemand);
        mFXRightFlyWheel.set(ControlMode.Velocity, flyDemand);
        // TODO: see if there is a better way to "flush" a magicmotion path
        // w/o this the hood sits at the bottom ofter homing for 2 to 6 seconds
        // until moving up
        if (mSystemState == SystemState.HOMINGHOOD) {
            mFXHood.set(ControlMode.Position, unadjustHoodEncoderPosition(hoodDemand));
        }
        else if (mSystemState == SystemState.TESTING){
            mFXHood.set(ControlMode.PercentOutput, hoodDemand);
        }
        else {
            mFXHood.set(ControlMode.MotionMagic, unadjustHoodEncoderPosition(hoodDemand));
        }
    }

    @Override
    public void stop() {
        System.out.println(sClassName + " stop()");
        mFXLeftFlyWheel.set(ControlMode.PercentOutput, 0);
        mFXRightFlyWheel.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public int whenRunAgain() {
        return mPeriodicIO.schedDeltaDesired;
    }

    @Override
    public String getLogHeaders() {
        return  sClassName+".schedDeltaDesired,"+
                sClassName+".schedDeltaActual,"+
                sClassName+".schedDuration,"+
                sClassName+".mSystemState,"+
                sClassName+".mWantedState,"+
                sClassName+".flywheelVelocity,"+
                sClassName+".hoodPosition,"+
                sClassName+".flywheelVelocityDemand,"+
                sClassName+".hoodDemand,"+
                sClassName+".reachedDesiredSpeed,"+
                sClassName+".reachedDesiredHoodPosition,"+
                sClassName+".mDistance,"+
                sClassName+".hoodHomed";
    }

    @Override
    public String getLogValues(boolean telemetry) {
        String start;
        if (telemetry){
            start = ",,,";
        }
        else{
            start = mPeriodicIO.schedDeltaDesired+","+
                    mPeriodicIO.schedDeltaActual+","+
                    (Timer.getFPGATimestamp()-mPeriodicIO.lastSchedStart)+",";
        }
        return  start+
        mSystemState+","+
        mWantedState+","+
        mPeriodicIO.flywheelVelocity+","+
        mPeriodicIO.hoodPosition+","+
        mPeriodicIO.flywheelVelocityDemand+","+
        mPeriodicIO.hoodDemand+","+
        mPeriodicIO.reachedDesiredSpeed+","+
        mPeriodicIO.reachedDesiredHoodPosition+","+
        mDistance+","+
        hoodHomed;
    }

    @Override
    public void outputTelemetry() {
        // SmartDashboard.putNumber("Hood Position", mPeriodicIO.hoodPosition);
        // SmartDashboard.putNumber("Flywheel Speed", mPeriodicIO.flywheelVelocity);
        // SmartDashboard.putBoolean("Reached Desired Speed", mPeriodicIO.reachedDesiredSpeed);
        // SmartDashboard.putBoolean("Reached Desired Hood", mPeriodicIO.reachedDesiredHoodPosition);
        // SmartDashboard.putBoolean("Ready To Shoot", readyToShoot());
        // // these next values are bypassing readPeriodicInputs to reduce the ctre errors
        // // in
        // // riolog
        // SmartDashboard.putNumber("FlyStatorRight", mFXRightFlyWheel.getStatorCurrent());
        // SmartDashboard.putNumber("FlyStatorLeft", mFXLeftFlyWheel.getStatorCurrent());
        // SmartDashboard.putNumber("HoodStator", mFXHood.getStatorCurrent());
        // SmartDashboard.putNumber("FlySupplyRight", mFXRightFlyWheel.getSupplyCurrent());
        // SmartDashboard.putNumber("FlySupplyLeft", mFXLeftFlyWheel.getSupplyCurrent());
        // SmartDashboard.putNumber("HoodSupply", mFXHood.getSupplyCurrent());
    }

    public static class PeriodicIO {
        // Logging
        public final int mDefaultSchedDelta = 20; // loop run every 20 msec
        public int schedDeltaDesired;
        public double schedDeltaActual;
        public double lastSchedStart;

        // Inputs
        public double flywheelVelocity;
        public double hoodPosition;

        // Outputs
        public double flywheelVelocityDemand;
        public double hoodDemand;

        // Other
        public double lastHoodPosition;
        public boolean reachedDesiredSpeed;
        public boolean reachedDesiredHoodPosition;
    }

}
