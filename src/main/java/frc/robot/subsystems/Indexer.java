package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.constants.Ports;
import libraries.cheesylib.drivers.TalonFXFactory;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.subsystems.SubsystemManager;
import libraries.cheesylib.util.LatchedBoolean;

public class Indexer extends Subsystem {

    // Hardware
    private final TalonFX mFXIndexer;
    private final AnalogInput mAIEnterBeamBreak;
    private final AnalogInput mAIExitBeamBreak;

    // Subsystem Constants
    private final double kBackingSpeed = -.3;
    private final double kFeedingSpeed = .9;
    private final double kLoadingSpeed = .3;

    private boolean hasBallOnLoadingStart;
    private boolean loadingCompleted;
    private boolean feedingCompleted;    

    private double motorPositionTarget;
    private final double kIndexerLengthTicks = 35000;
    private final double kBeamBreakThreshold = 3.0;
    private final double kIndexerCurrentLimit = 50;

    // Subsystem States
    public enum SystemState {
        HOLDING,
        LOADING,
        FEEDING,
        BACKING
    }

    public enum WantedState {
        HOLD,
        LOAD,
        FEED,
        BACK
    }

    private SystemState mSystemState;
    private WantedState mWantedState;
    private boolean mStateChanged;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private LatchedBoolean mSystemStateChange = new LatchedBoolean();

    // Other
    private SubsystemManager mSubsystemManager;

    // Subsystem Creation
    private static String sClassName;
    private static int sInstanceCount;
    private static Indexer sInstance = null;

    public static Indexer getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new Indexer(caller);
        } else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller) {
        System.out.println("(" + caller + ") " + " getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Indexer(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mFXIndexer = TalonFXFactory.createDefaultTalon(Ports.INDEXER);
        mAIEnterBeamBreak = new AnalogInput(Ports.ENTRANCE_BEAM_BREAK);
        mAIExitBeamBreak = new AnalogInput(Ports.EXIT_BEAM_BREAK);
        mSubsystemManager = SubsystemManager.getInstance(sClassName);
        configMotors();
    }

    private void configMotors() {
        mFXIndexer.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        // TODO verify if this and/or others are needed
        mFXIndexer.setControlFramePeriod(ControlFrame.Control_3_General, 18);

        mFXIndexer.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mFXIndexer.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mFXIndexer.setInverted(true);
        // mFXIndexer.setSensorPhase(false); TODO: verify this is not needed

        mFXIndexer.setNeutralMode(NeutralMode.Brake);

        mFXIndexer.configStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, kIndexerCurrentLimit, kIndexerCurrentLimit, 0));

    }

    @Override
    public void onStart(Phase phase) {
        synchronized (Indexer.this) {
            mSystemState = SystemState.HOLDING;
            mWantedState = WantedState.HOLD;
            mStateChanged = true;
            System.out.println(sClassName + " state " + mSystemState);
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
            feedingCompleted = false;
            loadingCompleted = false;
            stop(); // stop motors just in case they were left running
        }
    }

    @Override
    public void onLoop(double timestamp) {
        synchronized (Indexer.this) {
            do {
                SystemState newState;
                switch (mSystemState) {
                    case LOADING:
                        newState = handleLoading();
                        break;
                    case FEEDING:
                        newState = handleFeeding();
                        break;
                    case BACKING:
                        newState = handleBacking();
                        break;
                    case HOLDING:
                        newState = handleHolding();
                        break;
                    default:
                        newState = null;
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

    private SystemState handleHolding() {
        if (mStateChanged) {
            mPeriodicIO.indexerDemand = 0;
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mSleepCycle; // goto sleep until woken by others
        }

        return defaultStateTransfer();
    }

    // this will run until stopped by call to setWantedState
    private SystemState handleLoading() {

        if (mStateChanged) {
            if (mPeriodicIO.exitBeamBlocked){
                hasBallOnLoadingStart = true;
                mPeriodicIO.indexerDemand = 0;
                mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
            }
            else{
                hasBallOnLoadingStart = false;
                mPeriodicIO.indexerDemand = kLoadingSpeed;
                mPeriodicIO.schedDeltaDesired = mPeriodicIO.mFastCycle;
            }
        }

        // loading first ball
        if (!hasBallOnLoadingStart){
            if (mPeriodicIO.exitBeamBlocked){
                mPeriodicIO.indexerDemand = 0; // stop indexer when sensor trips
                loadingCompleted = true;
            }
        }
        else{ // loading 2nd ball
            if (mPeriodicIO.enterBeamBlocked){
                loadingCompleted = true;
            }
        }

        if (mWantedState != WantedState.LOAD) {
            loadingCompleted = false;
        }

        return defaultStateTransfer();
    }

    // used by automode
    public boolean loadingCompleted(){
        return loadingCompleted;
    }

    private SystemState handleFeeding() {
        if (mStateChanged) {
            mPeriodicIO.indexerDemand = kFeedingSpeed;
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
            motorPositionTarget = mPeriodicIO.motorPosition + kIndexerLengthTicks;
            feedingCompleted = false;
        }

        if (mPeriodicIO.motorPosition>motorPositionTarget){
            feedingCompleted = true;
        }

        if (mWantedState != WantedState.FEED) {
            feedingCompleted = false;
        }

        return defaultStateTransfer();
    }

    // used by automode
    public boolean feedingComplete(){
        return feedingCompleted;
    }

    private SystemState handleBacking() {
        if (mStateChanged) {
            mPeriodicIO.indexerDemand = kBackingSpeed;
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
            motorPositionTarget = mPeriodicIO.motorPosition - kIndexerLengthTicks;
        }

        return defaultStateTransfer();
    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case LOAD:
                return SystemState.LOADING;
            case FEED:
                return SystemState.FEEDING;
            case BACK:
                return SystemState.BACKING;
            case HOLD:
            default:
                return SystemState.HOLDING;
        }
    }

    @Override
    public void readPeriodicInputs() {
        double now = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart = now;

        mPeriodicIO.motorPosition = mFXIndexer.getSelectedSensorPosition();
        // true if beam is blocked
        mPeriodicIO.exitBeamBlocked = mAIExitBeamBreak.getVoltage()<kBeamBreakThreshold;
        mPeriodicIO.enterBeamBlocked = mAIEnterBeamBreak.getVoltage()<kBeamBreakThreshold;
    }

    @Override
    public void writePeriodicOutputs() {
        mFXIndexer.set(ControlMode.PercentOutput, mPeriodicIO.indexerDemand);
    }

    @Override
    public void stop() {
        System.out.println(sClassName + " stop()");
        mFXIndexer.set(ControlMode.PercentOutput, 0);
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
                sClassName+".exitBeamBlocked,"+
                sClassName+".enterBeamBlocked,"+
                sClassName+".indexerDemand,"+
                sClassName+".loadingCompleted,"+
                sClassName+".feedingCompleted";    
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
                mPeriodicIO.exitBeamBlocked+","+
                mPeriodicIO.enterBeamBlocked+","+
                mPeriodicIO.indexerDemand+","+
                loadingCompleted+","+
                feedingCompleted;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Indexer Enter Beam Blocked", mPeriodicIO.enterBeamBlocked);
        SmartDashboard.putBoolean("Indexer Exit Beam Blocked", mPeriodicIO.exitBeamBlocked);
        SmartDashboard.putNumber("Indexer Position", mPeriodicIO.motorPosition);

        SmartDashboard.putNumber("Indexer Current", mFXIndexer.getStatorCurrent());
    }

    public static class PeriodicIO {
        // Logging
        public final int mDefaultSchedDelta = 20; // axis updated every 20 msec TODO: slow down after testing
        public final int mFastCycle = 20;
        public final int mSleepCycle = 0;
        public int schedDeltaDesired;
        public double schedDeltaActual;
        public double lastSchedStart;

        // Inputs
        public boolean exitBeamBlocked;
        public boolean enterBeamBlocked;
        public double motorPosition;

        // Outputs
        public double indexerDemand;
    }
}
