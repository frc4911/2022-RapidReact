package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.states.LEDState;
import frc.robot.states.TimedLEDState;
import libraries.cheesylib.loops.Loop;
import libraries.cheesylib.subsystems.Subsystem;

/**
 * A subsystem for LED managing state and effects.
 */
public class LED extends Subsystem {
    private static final double kClimbingBlinkDuration = 0.5; // In sec
    private static final double kWantsCargoBlinkDuration = 0.075; // In sec
    private static final double kFaultBlinkDuration = 0.25; // In sec

    public enum WantedAction {
        DISPLAY_CLIMB,
        DISPLAY_SHOOT,
        DISPLAY_INDEXER_LOADED,
        DISPLAY_INDEXER_UNJAMMING,
        DISPLAY_COLLECTOR_DEPLOYED,
        DISPLAY_COLLECTOR_UNJAMMING,
        DISPLAY_DRIVE,
        DISPLAY_AUTO_START,
        DISPLAY_ZEROED
    }

    private enum SystemState {
        DISPLAYING_FAULT,
        DISPLAYING_CLIMB,
        DISPLAYING_SHOOT,
        DISPLAYING_INDEXER_LOADED,
        DISPLAYING_INDEXER_UNJAMMING,
        DISPLAYING_COLLECTOR_DEPLOYED,
        DISPLAYING_COLLECTOR_UNJAMMING,
        DISPLAYING_DRIVE,
        DISPLAYING_AUTO_START,
        DISPLAYING_ZEROED
    }

    private final Canifier mLEDCanifier;
    private SystemState mSystemState = SystemState.DISPLAYING_SHOOT;
    private WantedAction mWantedAction = WantedAction.DISPLAY_SHOOT;
    private final boolean mLoggingEnabled = false;              // used to disable logging for this subsystem only
    public PeriodicIO mPeriodicIO;

    // List subsystems that have LED state

    private boolean mFaultsEnabled = false;
    private boolean mErrorCondition = false;
    private boolean mBlinking = false;

    private LEDState mDesiredLEDState = new LEDState(0.0, 0.0, 0.0);
    private TimedLEDState mActiveLEDState = TimedLEDState.StaticLEDState.kStaticOff;
    private double mStateStartTime;

    private int mDefaultSchedDelta = 20;

    // Subsystem Creation
    private static String sClassName;
    private static int sInstanceCount;
    private static LED mInstance;

    public synchronized static LED getInstance(String caller) {
        if (mInstance == null) {
            mInstance = new LED(caller);
        }

        return mInstance;
    }

    private static void printUsage(String caller) {
        System.out.println("(" + caller + ") " + " getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private LED(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mLEDCanifier = Canifier.getInstance(caller);
        mPeriodicIO = new PeriodicIO();
    }

    public synchronized void setWantedAction(WantedAction wantedAction, boolean isBlinking) {
        mWantedAction = wantedAction;
        mBlinking = isBlinking;
    }

    @Override
    public void onStart(Loop.Phase phase) {
        synchronized (LED.this) {
            mStateStartTime = Timer.getFPGATimestamp();
            switch (phase) {
                case DISABLED:
                    mPeriodicIO.schedDeltaDesired = 0; // goto sleep
                    break;
                default:
                    mPeriodicIO.schedDeltaDesired = mDefaultSchedDelta;
                    break;
            }
        }
    }

    @Override
    public void onLoop(double timestamp) {
        synchronized (LED.this) {
            SystemState newState = getStateTransition();

            if (mSystemState != newState) {
                System.out.println(timestamp + ": LED changed state: " + mSystemState + " -> " + newState);
                mSystemState = newState;
                mStateStartTime = timestamp;
            }

            double timeInState = timestamp - mStateStartTime;

            switch (mSystemState) {
                case DISPLAYING_FAULT:
                    setFaultLEDCommand(timeInState);
                    break;
                case DISPLAYING_CLIMB:
                    setClimbLEDCommand(timeInState);
                    break;
                case DISPLAYING_SHOOT:
                case DISPLAYING_INDEXER_LOADED:
                case DISPLAYING_INDEXER_UNJAMMING:
                case DISPLAYING_COLLECTOR_DEPLOYED:
                case DISPLAYING_COLLECTOR_UNJAMMING:
                case DISPLAYING_DRIVE:
                case DISPLAYING_AUTO_START:
                case DISPLAYING_ZEROED:
                    mActiveLEDState.getCurrentLEDState(mDesiredLEDState, timeInState);
                    break;
                default:
                    System.out.println("Fell through on LED commands: " + mSystemState);
                    break;
            }

            mLEDCanifier.setLEDColor(mDesiredLEDState.red, mDesiredLEDState.green, mDesiredLEDState.blue);
        }
    }

    private void setFaultLEDCommand(double timeInState) {
        // Blink red.
        if ((int) (timeInState / kFaultBlinkDuration) % 2 == 0) {
            mDesiredLEDState.copyFrom(LEDState.kFault);
        } else {
            mDesiredLEDState.copyFrom(LEDState.kOff);
        }
    }

    private void setClimbLEDCommand(double timeInState) {
        // Blink orange
        if ((int) (timeInState / kClimbingBlinkDuration) % 2 == 0) {
            mDesiredLEDState.copyFrom(LEDState.kClimbing);
        } else {
            mDesiredLEDState.copyFrom(LEDState.kOff);
        }
    }

    private SystemState getStateTransition() {
        // if (mFaultsEnabled && !mSwerve.hasBeenZeroed()) {
        // return SystemState.DISPLAYING_FAULT;
        // }
        if (mErrorCondition) {
            return SystemState.DISPLAYING_FAULT;
        }

        switch (mWantedAction) {
            case DISPLAY_CLIMB:
                return SystemState.DISPLAYING_CLIMB;
            case DISPLAY_SHOOT:
                mActiveLEDState = mBlinking ? TimedLEDState.BlinkingLEDState.kShoot : TimedLEDState.StaticLEDState.kShoot;
                return SystemState.DISPLAYING_SHOOT;
            case DISPLAY_INDEXER_LOADED:
                mActiveLEDState = mBlinking ? TimedLEDState.BlinkingLEDState.kIndexerLoaded : TimedLEDState.StaticLEDState.kIndexerLoaded;
                return SystemState.DISPLAYING_INDEXER_LOADED;
            case DISPLAY_INDEXER_UNJAMMING:
                mActiveLEDState = mBlinking ? TimedLEDState.BlinkingLEDState.kIndexerUnjamming : TimedLEDState.StaticLEDState.kIndexerLoaded;
                return SystemState.DISPLAYING_INDEXER_LOADED;
            case DISPLAY_COLLECTOR_DEPLOYED:
                mActiveLEDState = mBlinking ? TimedLEDState.BlinkingLEDState.kCollectorDeployed: TimedLEDState.StaticLEDState.kCollectorDeployed;
                return SystemState.DISPLAYING_COLLECTOR_DEPLOYED;
            case DISPLAY_COLLECTOR_UNJAMMING:
                mActiveLEDState = mBlinking ? TimedLEDState.BlinkingLEDState.kCollectorUnjamming: TimedLEDState.StaticLEDState.kCollectorDeployed;
                return SystemState.DISPLAYING_COLLECTOR_DEPLOYED;
            case  DISPLAY_DRIVE:
                mActiveLEDState = mBlinking ? TimedLEDState.BlinkingLEDState.kDrive: TimedLEDState.StaticLEDState.kDrive;
                return SystemState.DISPLAYING_DRIVE;
            case DISPLAY_AUTO_START:
                mActiveLEDState = TimedLEDState.StaticLEDState.kAutoStart;
            default:
                System.out.println("Fell through on LED wanted action check: " + mWantedAction);
                return SystemState.DISPLAYING_SHOOT;
        }
    }

    public synchronized void setEnableFaults(boolean enable) {
        mFaultsEnabled = enable;
    }

//    boolean testRunOnce = false;
//    public boolean checkSystem() {
//        if (testRunOnce) {
//            return true;
//        }
//        // TODO:  Re-enable tests for  2022 states
//
//        System.out.println("Climb - blink orange for 0.5s");
//        this.setWantedAction(WantedAction.DISPLAY_CLIMB);
//        runLoop(3.0);
//
//        // Test intake sequence:  Intaking -> Blinking Has Cargo -> Solid Has Cargo
//        System.out.println("Intaking - solid green for 0.5s");
//        setIntakeLEDState(TimedLEDState.StaticLEDState.kIntaking);
//        this.setWantedAction(WantedAction.DISPLAY_BEAK_DOWN);
//        runLoop(3.0);
//
//        // Test intake sequence:  Intaking -> Blinking Has Cargo -> Solid Has Cargo
//        System.out.println("Intaking (has cargo) - blinking blue for 0.075s");
//        setIntakeLEDState(TimedLEDState.BlinkingLEDState.kHasCargo);
//        this.setWantedAction(WantedAction.DISPLAY_BEAK_DOWN);
//        runLoop(3.0);
//
//        // Test intake sequence:  Intaking -> Blinking Has Cargo -> Solid Has Cargo
//        System.out.println("Intaking (has cargo) - solid blue");
//        setIntakeLEDState(TimedLEDState.StaticLEDState.kHasCargo);
//        this.setWantedAction(WantedAction.DISPLAY_BEAK_DOWN);
//        runLoop(3.0);
//
//        // Test wants Cargo sequence
//        System.out.println("Wants Cargo - blinking orange");
//        this.setWantedAction(WantedAction.DISPLAY_HAS_CARGO);
//        runLoop(3.0);
//
//        // Test Fault LED
//        System.out.println("Wants Cargo - blinking red");
//        mErrorCondition = true;
//        runLoop(3.0);
//        mErrorCondition = false;
//
//        setIntakeLEDState(TimedLEDState.StaticLEDState.kStaticOff);
//        this.setWantedAction(WantedAction.DISPLAY_BEAK_DOWN);
//        runLoop(3.0);
//
//        mDesiredLEDState.blue = 0;
//        mDesiredLEDState.red= 0;
//        mDesiredLEDState.green = 0;
//
//        testRunOnce = true;
//        return true;
//    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.systemState = mSystemState;
    }

    @Override
    public String getLogHeaders() {
        return  sClassName + ".schedDeltaDesired," +
                sClassName + ".schedDeltaActual," +
                sClassName + ".schedDuration," +
                sClassName + ".mSystemState," +
                sClassName + ".mWantedState,";
    }

    @Override
    public String getLogValues(boolean telemetry) {
        String start;
        if (telemetry){
            start = ",,,";
        }
        else{
            start = mPeriodicIO.schedDeltaDesired + "," +
                    mPeriodicIO.schedDeltaActual + "," +
                    (Timer.getFPGATimestamp() - mPeriodicIO.lastSchedStart) + ",";
        }
        return  start+
            mSystemState + "," +
            mWantedAction + ",";
    }

    @Override
    public void outputTelemetry() {
    }

    @Override
    public void stop() {
    }


    private void runLoop(double duration) {
        double endTime = Timer.getFPGATimestamp() + duration;
        double timeStamp;

        do {
            timeStamp = Timer.getFPGATimestamp();
            onLoop(timeStamp );
            Canifier.getInstance(sClassName).readPeriodicInputs();
            Canifier.getInstance(sClassName).writePeriodicOutputs();
            Timer.delay(Constants.kLooperDt);
        } while (timeStamp < endTime);
    }

    public static class PeriodicIO {
        // LOGGING
        public int schedDeltaDesired;
        public double schedDeltaActual;
        public double schedDuration;
        private double lastSchedStart;
        public SystemState systemState;

        // INPUTS

        //OUTPUTS
    }
}
