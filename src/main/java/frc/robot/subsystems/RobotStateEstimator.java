package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;

public class RobotStateEstimator extends Subsystem {

    public enum SystemState {
        ESTIMATING,
    }

    public enum WantedState {
        ESTIMATE,
    }

    private SystemState mSystemState;
    private WantedState mWantedState;
    private final PeriodicIO mPeriodicIO;
    @SuppressWarnings("unused")
    private boolean mStateChanged;
    private final boolean mLoggingEnabled = true; // used to disable logging for this subsystem only
    private static int mDefaultSchedDelta = 20;
    RobotState robotState;// = RobotState.getInstance();
    Swerve mSwerve;
    private double prev_timestamp_ = -1.0;

    private static String sClassName;
    private static int sInstanceCount;
    private static RobotStateEstimator sInstance = null;

    public static RobotStateEstimator getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new RobotStateEstimator(caller);
        } else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller) {
        System.out.println("(" + caller + ") " + "getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private RobotStateEstimator(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mPeriodicIO = new PeriodicIO();
        mSwerve = Swerve.getInstance(sClassName);
        robotState = RobotState.getInstance(sClassName);
    }

    @Override
    public void onStart(Phase phase) {
        synchronized (RobotStateEstimator.this) {
            mSystemState = SystemState.ESTIMATING;
            mWantedState = WantedState.ESTIMATE;
            mStateChanged = true;
            System.out.println(sClassName + " state " + mSystemState);
            mPeriodicIO.schedDeltaDesired = mDefaultSchedDelta;
            prev_timestamp_ = Timer.getFPGATimestamp();
        }
    }

    @Override
    public void onLoop(double timestamp) {
        synchronized (RobotStateEstimator.this) {
            SystemState newState;
            switch (mSystemState) {
                case ESTIMATING:
                default:
                    newState = handleEstimating(timestamp);
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

    private SystemState handleEstimating(double timestamp) {
        final double dt = timestamp - prev_timestamp_;
        // TODO:  Add prediction values based on current velocities

        robotState.addFieldToVehicleObservation(timestamp, mSwerve.getPose());
        prev_timestamp_ = timestamp;
        return defaultStateTransfer();
    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case ESTIMATE:
            default:
                return SystemState.ESTIMATING;
        }
    }

    // this method should only be used by external subsystems.
    // if you want to change your own wantedState then simply set
    // it directly
    public synchronized void setWantedState(WantedState state, String who) {
        if (state != mWantedState) {
            mWantedState = state;
            //mSubsystemManager.scheduleMe(mListIndex, 1, true);
            System.out.println(who + " is setting wanted state of " + sClassName + " to "+state);
        }
        else{
            System.out.println(who + " is setting wanted state of " + sClassName + " to "+state + " again!!!");
        }
    }

    @Override
    public void stop() {
    }

    @Override
    public String getLogHeaders() {
        return  sClassName+".schedDeltaDesired,"+
                sClassName+".schedDeltaActual,"+
                sClassName+".schedDuration";
    }

    @Override
    public String getLogValues(boolean telemetry) {
        String start;
        if (telemetry){
            start = ",,";
        }
        else{
            start = mPeriodicIO.schedDeltaDesired+","+
                    mPeriodicIO.schedDeltaActual+","+
                    (Timer.getFPGATimestamp()-mPeriodicIO.lastSchedStart);
        }
        return  start;
    }

    @Override
    public void readPeriodicInputs() {
        double now = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart = now;

    }

    @Override
    public void writePeriodicOutputs() {
    }

    @Override
    public int whenRunAgain() {
        return mPeriodicIO.schedDeltaDesired;
    }

    @Override
    public void outputTelemetry() {
    }

    public static class PeriodicIO {
        // LOGGING
        public int schedDeltaDesired;
        public double schedDeltaActual;
        private double lastSchedStart;
    }
}