package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import libraries.cheesylib.geometry.Twist2d;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cyberlib.kinematics.ChassisSpeeds;

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
    RobotState mRobotState;
    Swerve mSwerve;

    private ChassisSpeeds mPrevChassisSpeeds = null;
    private double mPrevTimestamp = -1.0;

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
        mRobotState = RobotState.getInstance(sClassName);
    }

    @Override
    public void onStart(Phase phase) {
        synchronized (RobotStateEstimator.this) {
            mSystemState = SystemState.ESTIMATING;
            mWantedState = WantedState.ESTIMATE;
            mStateChanged = true;
            System.out.println(sClassName + " state " + mSystemState);
            mPeriodicIO.schedDeltaDesired = mDefaultSchedDelta;
            mPrevTimestamp = Timer.getFPGATimestamp();
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
        if (mPrevChassisSpeeds == null) {
            mPrevChassisSpeeds = mSwerve.getChassisSpeeds();
        }

        final double dt = timestamp - mPrevTimestamp;
        final ChassisSpeeds chassisSpeeds = mSwerve.getChassisSpeeds();

        final Twist2d measuredVelocity = new Twist2d(
                chassisSpeeds.vxInMetersPerSecond,
                chassisSpeeds.vyInMetersPerSecond,
                chassisSpeeds.omegaInRadiansPerSecond);

        final Twist2d predictedVelocity = new Twist2d(
                chassisSpeeds.vxInMetersPerSecond - mPrevChassisSpeeds.vxInMetersPerSecond,
                chassisSpeeds.vyInMetersPerSecond - mPrevChassisSpeeds.vyInMetersPerSecond,
                chassisSpeeds.omegaInRadiansPerSecond - mPrevChassisSpeeds.omegaInRadiansPerSecond
               ).scaled(1.0 / dt);

        mRobotState.addFieldToVehicleObservation(timestamp, mSwerve.getPose(), measuredVelocity, predictedVelocity);
        mPrevChassisSpeeds = chassisSpeeds;
        mPrevTimestamp = timestamp;
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
        System.out.println(sClassName + " stop()");
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