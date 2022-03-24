package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.constants.Ports;
import libraries.cheesylib.drivers.TalonFXFactory;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.subsystems.SubsystemManager;
import libraries.cheesylib.util.LatchedBoolean;
import libraries.cyberlib.control.FramePeriodSwitch;

public class Collector extends Subsystem {

    // Hardware
    private final TalonFX mFXMotor;
    private final Solenoid mSolenoid;

    private FramePeriodSwitch mFramePeriods;

    // Subsystem Constants
    private final double kCollectSpeed = 0.9;
    // time (msec) to run collector motor forward so collector can deploy
    // when backing
    private final double kBackingEjectDuration = 100; 
    private final double kMotorAssessTime = 250; // quarter second
    private final double kCurrentLimit = 60;
    private final double kAssessingMotorDemand = .5;
    private final int kSchedDeltaActive = 20;
    private final int kSchedDeltaDormant = 100;
    private final double kMinAssessmentMovement = 100; // ticks
    private final int kActiveFramePeriod = 20;
    private final int kDormantFramePeriod = 100;

    // subsystem variables
    private double mAssessingStartPosition;
    private boolean mAssessmentResult;
    private boolean mAssessmentHandlerComplete;
    private SolenoidState mTestSolenoidDemand;
    private double mTestMotorDemand;
    private double mHandlerLoopCount;

    // latched booleans
    private LatchedBoolean mLB_handlerLoopCounter = new LatchedBoolean();

    // Subsystem States
    public enum SolenoidState {
        EXTEND(true),
        RETRACT(false);

        private final boolean state;

        private SolenoidState(boolean state) {
            this.state = state;
        }

        public boolean get() {
            return state;
        }
    }

    public enum SystemState {
        ASSESSING,
        BACKING,
        COLLECTING,
        DISABLING,
        HOLDING,
        MANUAL_CONTROLLING
    }

    public enum WantedState {
        ASSESS,
        BACK,
        COLLECT,
        DISABLE,
        HOLD,
        MANUAL_CONTROL
    }

    private SystemState mSystemState;
    private WantedState mWantedState;
    private boolean mStateChanged;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private LatchedBoolean mLB_SystemStateChange = new LatchedBoolean();

    // Other
    private SubsystemManager mSubsystemManager;

    // Subsystem Creation
    private static String sClassName;
    private static int sInstanceCount;
    private static Collector sInstance = null;

    public static Collector getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new Collector(caller);
        } else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller) {
        System.out.println("(" + caller + ") " + " getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Collector(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mFXMotor = TalonFXFactory.createDefaultTalon(Ports.COLLECTOR, Constants.kCanivoreName);
        mSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.COLLECTOR_DEPLOY);
        mSubsystemManager = SubsystemManager.getInstance(sClassName);
        configMotors();
    }

    private void configMotors() {
        mFramePeriods = new FramePeriodSwitch(mFXMotor, kActiveFramePeriod, kDormantFramePeriod);
        mFramePeriods.switchToDormant();
        mFXMotor.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mFXMotor.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mFXMotor.setInverted(false);

        mFXMotor.setNeutralMode(NeutralMode.Coast);

        mFXMotor
                .configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kCurrentLimit, kCurrentLimit, 0));

    }

    @Override
    public void onStart(Phase phase) {
        synchronized (Collector.this) { // TODO Check if the key word synchronized is needed
            switch (phase){
                case TELEOP:
                case AUTONOMOUS:
                    mSystemState = SystemState.HOLDING;
                    mWantedState = WantedState.HOLD;
                    break;
                case TEST:
                    mSystemState = SystemState.MANUAL_CONTROLLING;
                    mWantedState = WantedState.MANUAL_CONTROL;
                    break;
                case DISABLED:
                    mSystemState = SystemState.DISABLING;
                    mWantedState = WantedState.DISABLE;
                    break;
            }
            mStateChanged = true;
            System.out.println(sClassName + " state " + mSystemState);
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
            mLB_SystemStateChange.update(false); // reset
            stop(); // put into a known state
        }
    }

    @Override
    public void onLoop(double timestamp) {
        synchronized (Collector.this) {
            do {
                SystemState newState = null;
                switch (mSystemState) {
                    case ASSESSING:
                        newState = handleAssessing();
                        break;
                    case BACKING:
                        newState = handleBacking();
                        break;
                    case COLLECTING:
                        newState = handleCollecting();
                        break;
                    case DISABLING:
                        newState = handleDisabling();
                        break;
                    case HOLDING:
                        newState = handleHolding();
                        break;
                    case MANUAL_CONTROLLING:
                        newState = handleManualControlling();
                        break;
                    // default:
                    // leave commented so compiler will identify missing cases
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
            case COLLECT:
                return SystemState.COLLECTING;
            case DISABLE:
                return SystemState.DISABLING;
            case HOLD:
                return SystemState.HOLDING;
            case MANUAL_CONTROL:
                return SystemState.MANUAL_CONTROLLING;
            // default:
                // leave commented so compiler will identify missing cases
        }
        return null; // crash if this happens
    }

    public boolean isCollectorStageDone(WantedState state) {
        switch(state) {
            case ASSESS:
                return mAssessmentHandlerComplete;
            default:
                System.out.println("Uh oh something is not right in "+sClassName);
                return false;
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

    private SystemState handleAssessing() {
        if (mStateChanged) {
            mFramePeriods.switchToActive();
            mAssessingStartPosition = mPeriodicIO.motorPosition;
            setSolenoidDemand(SolenoidState.RETRACT);
            setMotorControlModeAndDemand(ControlMode.PercentOutput,kAssessingMotorDemand);
            mPeriodicIO.schedDeltaDesired = kSchedDeltaActive;
            mAssessmentResult = false;
            mAssessmentHandlerComplete = false;
            mHandlerLoopCount = kMotorAssessTime / mPeriodicIO.schedDeltaDesired; // 250 is .25 sec to run collector forward
            mLB_handlerLoopCounter.update(false); // reset
        }

        // Run collector forward to deploy collector before backing
        if(mLB_handlerLoopCounter.update(mHandlerLoopCount-- <= 0)) {
            setMotorControlModeAndDemand(ControlMode.PercentOutput,0);
            if (mAssessingStartPosition >= mPeriodicIO.motorPosition + kMinAssessmentMovement){
                mAssessmentResult = true;
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
            mFramePeriods.switchToActive();
            setSolenoidDemand(SolenoidState.EXTEND);
            setMotorControlModeAndDemand(ControlMode.PercentOutput,kCollectSpeed);
            mPeriodicIO.schedDeltaDesired = kSchedDeltaActive;
            // need to run motor forward until collector extends beyond bumper
            // before reversing for backing
            mHandlerLoopCount = (kBackingEjectDuration / (double) mPeriodicIO.schedDeltaDesired); 
            mLB_handlerLoopCounter.update(false); // reset
        }

        // Run collector forward to deploy collector before backing
        if(mLB_handlerLoopCounter.update(mHandlerLoopCount-- <= 0)) {
            setMotorControlModeAndDemand(ControlMode.PercentOutput,-kCollectSpeed);
        }

        return defaultStateTransfer();
    }

    private SystemState handleCollecting() {
        if(mStateChanged) {
            mFramePeriods.switchToActive();
            setSolenoidDemand(SolenoidState.EXTEND);
            setMotorControlModeAndDemand(ControlMode.PercentOutput,kCollectSpeed);
            mPeriodicIO.schedDeltaDesired = kSchedDeltaActive;
        }
        
        return defaultStateTransfer();
    }


    private SystemState handleDisabling() {
        if (mStateChanged) {
            mFramePeriods.switchToDormant();
            setSolenoidDemand(SolenoidState.RETRACT);
            setMotorControlModeAndDemand(ControlMode.PercentOutput,0.0);
            mPeriodicIO.schedDeltaDesired = kSchedDeltaDormant;
        }

        return defaultStateTransfer();
    }

    private SystemState handleHolding() {
        if (mStateChanged) {
            mFramePeriods.switchToDormant();
            setSolenoidDemand(SolenoidState.RETRACT);
            setMotorControlModeAndDemand(ControlMode.PercentOutput,0.0);
            mPeriodicIO.schedDeltaDesired = kSchedDeltaDormant;
        }

        return defaultStateTransfer();
    }

    private SystemState handleManualControlling() {
        if (mStateChanged) {
            mFramePeriods.switchToActive();
            mTestMotorDemand = 0;
            mTestSolenoidDemand = SolenoidState.RETRACT;
            mPeriodicIO.schedDeltaDesired = kSchedDeltaActive;
        }

        setMotorControlModeAndDemand(ControlMode.PercentOutput, mTestMotorDemand);
        setSolenoidDemand(mTestSolenoidDemand);

        return defaultStateTransfer();
    }

    public boolean getLastAssessmentResult(){
        return mAssessmentResult;
    }

    public void setMotorDemand(double newDemand){
        mTestMotorDemand = newDemand;
    }

    public void setSolenoidDemand(boolean extend){
        if (extend){
            mTestSolenoidDemand = SolenoidState.EXTEND;
        }
        else{
            mTestSolenoidDemand = SolenoidState.RETRACT;
        }
    }

    private void setMotorControlModeAndDemand(ControlMode controlMode, double demand){
        mPeriodicIO.motorControlMode = controlMode;
        mPeriodicIO.motorDemand = demand;
    }

    private void setSolenoidDemand(SolenoidState newSolenoidState){
        mPeriodicIO.solenoidDemand = newSolenoidState;
    }

    @Override
    public void readPeriodicInputs() {
        double now = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart = now;

        mPeriodicIO.motorPosition = mFXMotor.getSelectedSensorPosition();

    }

    @Override
    public void writePeriodicOutputs() {
        if (mPeriodicIO.solenoidState != mPeriodicIO.solenoidDemand) {
            mPeriodicIO.solenoidState = mPeriodicIO.solenoidDemand;
            mSolenoid.set(mPeriodicIO.solenoidDemand.get());
        }
        mFXMotor.set(mPeriodicIO.motorControlMode, mPeriodicIO.motorDemand);
    }

    @Override
    public void stop() {
        setSolenoidDemand(SolenoidState.RETRACT);

        setMotorControlModeAndDemand(ControlMode.PercentOutput,0);
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
                sClassName+".motorControlMode,"+
                sClassName+".motorDemand,"+
                sClassName+".motorPosition,"+
                sClassName+".motorStator,"+
                sClassName+".solenoidState";
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
                mPeriodicIO.motorControlMode+","+
                mPeriodicIO.motorDemand+","+
                mPeriodicIO.motorPosition+","+
                mPeriodicIO.motorStator+","+
                mPeriodicIO.solenoidState;
    }

    @Override
    public void outputTelemetry() {
        mPeriodicIO.motorStator = mFXMotor.getStatorCurrent();
    }

    public static class PeriodicIO {
        // Logging

        private final int mDefaultSchedDelta = 20; // axis updated every 20 msec
        private int schedDeltaDesired;
        public double schedDeltaActual;
        private double lastSchedStart;

        // Inputs
        private double motorPosition;
        private double motorStator;

        // Outputs
        private ControlMode motorControlMode;
        private double motorDemand;
        private SolenoidState solenoidDemand;

        // other
        private SolenoidState solenoidState;
    }
}