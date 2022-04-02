package frc.robot.subsystems;

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
import libraries.cyberlib.control.FramePeriodSwitch;

public class Indexer extends Subsystem {

    // Hardware
    private final TalonFX mFXMotor;
    private final AnalogInput mAIEnterBeamBreak;
    private final AnalogInput mAIExitBeamBreak;

    // Subsystem Constants
    private final double kBackingSpeed = -.3;
    private final double kFeedingSpeed = .2;
    private final double kLoadingSpeed = .3;

    private boolean mHasBallOnLoadingStart;
    private boolean mLoadingCompleted;
    private boolean mFeedingCompleted;    

    private double motorPositionTarget;
    private final double kIndexerLengthTicks = 35000;
    private final double kBeamBreakThreshold = 3.0;
    private final double kIndexerCurrentLimit = 50;
    private final double kAssessingMotorDemand = kFeedingSpeed;
    private final int kSchedDeltaActive = 20;
    private final int kSchedDeltaDormant = 100;
    private final double kMinAssessmentMovement = 100; // ticks
    private final int kActiveFramePeriod = 20;
    private final int kDormantFramePeriod = 100;

    // Subsystem States
    public enum SystemState {
        ASSESSING,
        BACKING,
        DISABLING,
        FEEDING,
        HOLDING,
        LOADING,
        MANUAL_CONTROLLING
    }

    public enum WantedState {
        ASSESS,
        BACK,
        DISABLE,
        FEED,
        HOLD,
        LOAD,
        MANUAL_CONTROL
    }

    private SystemState mSystemState;
    private WantedState mWantedState;
    private boolean mStateChanged;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private LatchedBoolean mLB_SystemStateChange = new LatchedBoolean();

    // Other
    private SubsystemManager mSubsystemManager;
    private double mAssessingStartPosition;
    private boolean mAssessmentResult;
    private boolean mAssessmentHandlerComplete;
    private LatchedBoolean mLB_handlerLoopCounter;
    private int kMotorAssessTime;
    private double mTestMotorDemand;
    private int mHandlerLoopCount;

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
        mFXMotor = TalonFXFactory.createDefaultTalon(Ports.INDEXER, Constants.kCanivoreName);
        mAIEnterBeamBreak = new AnalogInput(Ports.ENTRANCE_BEAM_BREAK);
        mAIExitBeamBreak = new AnalogInput(Ports.EXIT_BEAM_BREAK);
        mSubsystemManager = SubsystemManager.getInstance(sClassName);
        configMotors();
    }

    private void configMotors() {
        // must be run on every powerup
        FramePeriodSwitch.setFramePeriodsVolatile(mFXMotor); // set frame periods
        FramePeriodSwitch.setInvertedVolatile(mFXMotor);
        FramePeriodSwitch.setNeutralModeVolatile(mFXMotor, NeutralMode.Brake);


        // The following commands are stored in nonVolatile ram in the motor
        // They are repeated on boot incase a motor needs to replaced quickly
        FramePeriodSwitch.configFactoryDefaultPermanent(mFXMotor);
        FramePeriodSwitch.configStatorCurrentLimitPermanent(mFXMotor, new StatorCurrentLimitConfiguration(true, kIndexerCurrentLimit, kIndexerCurrentLimit, 0));
    
        // the following commands are stored in nonVolatile ram but they are
        // no longer deemed necessary. Keeping around for a while in case they
        // need to be brought back
        // talon.configNeutralDeadband(config.NEUTRAL_DEADBAND, kTimeoutMs);
        // talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, kTimeoutMs);
        // talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, kTimeoutMs);    

    }

    @Override
    public void onStart(Phase phase) {
        synchronized (Indexer.this) {
            switch (phase){
                case TELEOP:
                case AUTONOMOUS:
                    mSystemState = SystemState.HOLDING;
                    mWantedState = WantedState.HOLD;
                    break;
                case DISABLED:
                    mSystemState = SystemState.DISABLING;
                    mWantedState = WantedState.DISABLE;
                    break;
                case TEST:
                    mSystemState = SystemState.MANUAL_CONTROLLING;
                    mWantedState = WantedState.MANUAL_CONTROL;
                    break;
                // default:    leave commented so compiler will identify missing cases
            }
            mStateChanged = true;
            System.out.println(sClassName + " state " + mSystemState);
            mPeriodicIO.schedDeltaDesired = kSchedDeltaActive;
            mFeedingCompleted = false;
            mLoadingCompleted = false;
            mAssessmentHandlerComplete = false;
            mLB_SystemStateChange.update(false); // reset
            stop(); // put into a known state
        }
    }

    @Override
    public void onLoop(double timestamp) {
        synchronized (Indexer.this) {
            do {
                SystemState newState = null;
                switch (mSystemState) {
                    case ASSESSING:
                        newState = handleAssessing();
                        break;
                    case BACKING:
                        newState = handleBacking();
                        break;
                    case DISABLING:
                        newState = handleDisabling();
                        break;
                    case FEEDING:
                        newState = handleFeeding();
                        break;
                    case HOLDING:
                        newState = handleHolding();
                        break;
                    case LOADING:
                        newState = handleLoading();
                        break;
                    case MANUAL_CONTROLLING:
                        newState = handleManualControlling();    
                        break;
                        // default:   leave commented so compiler will identify missing cases
                }

                if (newState != mSystemState) {
                    System.out.println(
                            sClassName + " state " + mSystemState + " to " + newState + " (" + timestamp + ")");
                    mSystemState = newState;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            } while (mLB_SystemStateChange.update(mStateChanged));
        }
    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case ASSESS:
                return SystemState.ASSESSING;
            case BACK:
                return SystemState.BACKING;
            case DISABLE:
                return SystemState.DISABLING;
            case FEED:
                return SystemState.FEEDING;
            case HOLD:
                return SystemState.HOLDING;
            case LOAD:
                return SystemState.LOADING;
            case MANUAL_CONTROL:
                return SystemState.MANUAL_CONTROLLING;
            // default:
                // leave commented so compiler will identify missing cases
        }
        return null; // crash if this happens    }
    }

    public boolean isHandlerComplete(WantedState state) {
        switch(state) {
            case ASSESS:
                return mAssessmentHandlerComplete;
            case LOAD:
                return mLoadingCompleted;
            case FEED:
                return mFeedingCompleted;
            case BACK:
            case DISABLE:
            case HOLD:
            case MANUAL_CONTROL:
                System.out.println("Uh oh something is not right in "+sClassName);
                break;
        }
        return false;
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

    private SystemState handleAssessing() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = kSchedDeltaActive;
            setMotorControlModeAndDemand(ControlMode.PercentOutput,kAssessingMotorDemand);

            mAssessmentResult = false;
            mAssessmentHandlerComplete = false;
            mAssessingStartPosition = mPeriodicIO.motorPosition;
            mHandlerLoopCount = kMotorAssessTime / mPeriodicIO.schedDeltaDesired; // 250 is .25 sec to run collector forward
            mLB_handlerLoopCounter.update(false); // reset
        }
        // Run collector forward to deploy collector before backing
        if(mLB_handlerLoopCounter.update(mHandlerLoopCount-- <= 0)) {
            setMotorControlModeAndDemand(ControlMode.PercentOutput,0);
            if (mAssessingStartPosition >= mPeriodicIO.motorPosition + kMinAssessmentMovement){
                mAssessmentResult = true;
                System.out.println("ASSESSING: Indexer motor functioning");
            }
            else{
                System.out.println("ASSESSING: Indexer motor DID NOT DETECT MOVEMENT");
            }

            mAssessmentHandlerComplete = true;
        }

        if (mWantedState != WantedState.ASSESS){
            mAssessmentHandlerComplete = false;
        }

        return defaultStateTransfer();
    }

    private SystemState handleBacking() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = kSchedDeltaActive;
            setMotorControlModeAndDemand(ControlMode.PercentOutput, kBackingSpeed);
        }

        return defaultStateTransfer();
    }

    private SystemState handleDisabling() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = kSchedDeltaDormant;
            setMotorControlModeAndDemand(ControlMode.PercentOutput,0.0);
        }

        return defaultStateTransfer();
    }

    private SystemState handleFeeding() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = kSchedDeltaActive;
            setMotorControlModeAndDemand(ControlMode.PercentOutput, kFeedingSpeed);
            motorPositionTarget = mPeriodicIO.motorPosition + kIndexerLengthTicks;
            mFeedingCompleted = false;
        }

        if (mPeriodicIO.motorPosition>motorPositionTarget){
            mFeedingCompleted = true;
        }

        if (mWantedState != WantedState.FEED) {
            mFeedingCompleted = false;
        }

        return defaultStateTransfer();
    }

    private SystemState handleHolding() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = kSchedDeltaDormant;
            setMotorControlModeAndDemand(ControlMode.PercentOutput,0.0);
        }

        return defaultStateTransfer();
    }

    // this will run until stopped by call to setWantedState
    private SystemState handleLoading() {

        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = kSchedDeltaActive;
            if (mPeriodicIO.exitBeamBlocked){
                mHasBallOnLoadingStart = true;
                setMotorControlModeAndDemand(ControlMode.PercentOutput,0);
            }
            else{
                mHasBallOnLoadingStart = false;
                setMotorControlModeAndDemand(ControlMode.PercentOutput,kLoadingSpeed);
            }
        }

        // loading first ball
        if (!mHasBallOnLoadingStart){
            if (mPeriodicIO.exitBeamBlocked){
                // stop indexer when sensor trips
                setMotorControlModeAndDemand(ControlMode.PercentOutput,0);
                mLoadingCompleted = true;
            }
        }
        else{ // loading 2nd ball
            if (mPeriodicIO.enterBeamBlocked){
                mLoadingCompleted = true;
            }
        }

        if (mWantedState != WantedState.LOAD) {
            mLoadingCompleted = false;
        }

        return defaultStateTransfer();
    }

    private SystemState handleManualControlling() {
        if (mStateChanged) {
            mTestMotorDemand = 0;
            mPeriodicIO.schedDeltaDesired = kSchedDeltaActive;
        }

        setMotorControlModeAndDemand(ControlMode.PercentOutput, mTestMotorDemand);
        return defaultStateTransfer();
    }

    private void setMotorControlModeAndDemand(ControlMode controlMode, double demand){
        mPeriodicIO.motorControlMode = controlMode;
        mPeriodicIO.motorDemand = demand;
    }

    public boolean getLastAssessmentResult(){
        return mAssessmentResult;
    }

    public void setMotorTestDemand(double newTestDemand){
        mTestMotorDemand = newTestDemand;
    }

    @Override
    public void readPeriodicInputs() {
        double now = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart = now;

        mPeriodicIO.motorPosition = FramePeriodSwitch.getSelectedSensorPosition(mFXMotor);
        // true if beam is blocked
        mPeriodicIO.exitBeamBlocked = mAIExitBeamBreak.getVoltage()<kBeamBreakThreshold;
        mPeriodicIO.enterBeamBlocked = mAIEnterBeamBreak.getVoltage()<kBeamBreakThreshold;
    }

    @Override
    public void writePeriodicOutputs() {
        mFXMotor.set(mPeriodicIO.motorControlMode, mPeriodicIO.motorDemand);
    }

    @Override
    public void stop() {
        setMotorControlModeAndDemand(ControlMode.PercentOutput, 0);
        writePeriodicOutputs();
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
                sClassName+".exitBeamBlocked,"+
                sClassName+".enterBeamBlocked,"+
                sClassName+".motorDemand,"+
                sClassName+".motorPosition,"+
                sClassName+".motorStator,"+
                sClassName+".motorControlMode,"+
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
                mPeriodicIO.exitBeamBlocked+","+
                mPeriodicIO.enterBeamBlocked+","+
                mPeriodicIO.motorDemand+","+
                mPeriodicIO.motorPosition+","+
                mPeriodicIO.motorStator+","+
                mPeriodicIO.motorControlMode+","+
                mLoadingCompleted+","+
                mFeedingCompleted;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("IndexerEnterBeamBlocked", mPeriodicIO.enterBeamBlocked);
        SmartDashboard.putBoolean("IndexerExitBeamBlocked", mPeriodicIO.exitBeamBlocked);
        SmartDashboard.putNumber("IndexerPosition", mPeriodicIO.motorPosition);
        mPeriodicIO.motorStator = FramePeriodSwitch.getStatorCurrent(mFXMotor);
        SmartDashboard.putNumber("IndexerStator", mPeriodicIO.motorStator);
    }

    public static class PeriodicIO {
        // Logging
        public int schedDeltaDesired;
        public double schedDeltaActual;
        public double lastSchedStart;

        // Inputs
        public boolean exitBeamBlocked;
        public boolean enterBeamBlocked;
        public double motorPosition;
        public double motorStator;

        // Outputs
        private ControlMode motorControlMode;
        public double motorDemand;
    }
}