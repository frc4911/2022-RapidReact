package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.subsystems.SubsystemManager;
import libraries.cheesylib.util.LatchedBoolean;

public class Superstructure extends Subsystem {

    // Subsystem Instances
    @SuppressWarnings("unused")
    private Swerve mSwerve;
    private Indexer mIndexer;
    private Collector mCollector;
    private Shooter mShooter;
    private Climber mClimber;

    // Superstructure States
    public enum SystemState {
        DISABLING,
        HOLDING,
        COLLECTING,
        BACKING,
        AUTO_SHOOTING,
        MANUAL_SHOOTING,
        AUTO_CLIMBING
    }

    public enum WantedState {
        DISABLE,
        HOLD,
        COLLECT,
        BACK,
        AUTO_SHOOT,
        MANUAL_SHOOT,
        AUTO_CLIMB
    }

    private SystemState mSystemState;
    private WantedState mWantedState;
    private boolean mStateChanged;
    private LatchedBoolean mSystemStateChange = new LatchedBoolean();
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private int mFastCycle = 20;
    private int mSlowCycle = 100;

    private double mManualDistance;
    private boolean mShootSetup;
    private static String sClassName;
    private static int sInstanceCount;
    private static Superstructure sInstance = null;
    private boolean actionHasStarted;
    private SubsystemManager mSubsystemManager;

    public static Superstructure getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new Superstructure(caller);
        } else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller) {
        System.out.println("(" + caller + ") " + " getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Superstructure(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller); // this is done here so it appears before the subsequent prints for the other
                            // subsystems
        mSubsystemManager = SubsystemManager.getInstance(sClassName);
        mSwerve = Swerve.getInstance(sClassName);
        mIndexer = Indexer.getInstance(sClassName);
        mCollector = Collector.getInstance(sClassName);
        mShooter = Shooter.getInstance(sClassName);
        mClimber = Climber.getInstance(sClassName);
    }

    @Override
    public void onStart(Phase phase) {
        synchronized (Superstructure.this) {
            mStateChanged = true;
            switch (phase) {
                case DISABLED:
                    mSystemState = SystemState.DISABLING;
                    mWantedState = WantedState.DISABLE;
                    mPeriodicIO.schedDeltaDesired = 0; // goto sleep
                    break;
                default:
                    mSystemState = SystemState.HOLDING;
                    mWantedState = WantedState.HOLD;
                    mPeriodicIO.schedDeltaDesired = 100;
                    break;
            }
            System.out.println(sClassName + " state " + mSystemState);
        }
    }

    @Override
    public void onLoop(double timestamp) {
        synchronized (Superstructure.this) {
            do {
                SystemState newState;
                switch (mSystemState) {
                    case COLLECTING:
                        newState = handleCollecting();
                        break;
                    case BACKING:
                        newState = handleBacking();
                        break;
                    case AUTO_SHOOTING:
                        newState = handleAutoShooting();
                        break;
                    case MANUAL_SHOOTING:
                        newState = handleManualShooting();
                        break;
                    case AUTO_CLIMBING:
                        newState = handleAutoClimbing();
                        break;
                    case DISABLING:
                        newState = handleDisabling();
                        break;
                    case HOLDING:
                    default:
                        newState = handleHolding();
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

    // Handling methods
    private SystemState handleDisabling() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = mSlowCycle;
        }

        return defaultStateTransfer();
    }

    private SystemState handleHolding() {
        if (mStateChanged) {
            mCollector.setWantedState(Collector.WantedState.HOLD, sClassName);
            mIndexer.setWantedState(Indexer.WantedState.HOLD, sClassName);
            mShooter.setWantedState(Shooter.WantedState.HOLD, sClassName);
            mClimber.setWantedState(Climber.WantedState.HOLD, sClassName);
            mPeriodicIO.schedDeltaDesired = mSlowCycle;
        }

        return defaultStateTransfer();
    }

    private SystemState handleCollecting() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = mFastCycle;
            actionHasStarted = false;
        }

        if (!actionHasStarted && mIndexer.getBallCount() < 2) {
            mCollector.setWantedState(Collector.WantedState.COLLECT, sClassName);
            mIndexer.setWantedState(Indexer.WantedState.LOAD, sClassName);
            actionHasStarted = true;
        }

        if (mIndexer.getBallCount() >= 2) {
            mWantedState = WantedState.HOLD;
        }

        return defaultStateTransfer();
    }

    private SystemState handleBacking() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = mFastCycle;
            actionHasStarted = false;
        }

        if (!actionHasStarted && mIndexer.getBallCount() > 0) {
            mCollector.setWantedState(Collector.WantedState.BACK, sClassName);
            mIndexer.setWantedState(Indexer.WantedState.BACK, sClassName);
            actionHasStarted = true;
        }

        if (mIndexer.getBallCount() <= 0) {
            mWantedState = WantedState.HOLD;
        }

        return defaultStateTransfer();
    }

    // TODO: Get help with logic and limelight implementation - CURRENTLY UNUSED
    // If time constrains, may not be complete by Week 1
    private SystemState handleAutoShooting() {
        if (mStateChanged) {
            mShootSetup = true;
            mPeriodicIO.schedDeltaDesired = mFastCycle;
        }

        if (mShooter.readyToShoot() || !mShootSetup) {
            mIndexer.setWantedState(Indexer.WantedState.FEED, sClassName);
            mShootSetup = false;
        } else {
            mIndexer.setWantedState(Indexer.WantedState.HOLD, sClassName);
        }

        return defaultStateTransfer();
    }

    private SystemState handleManualShooting() {
        if (mStateChanged) {
            mShooter.setShootDistance(mManualDistance);
            // shooter must be in shoot state for readyToShoot to return true
            mShooter.setWantedState(Shooter.WantedState.SHOOT, sClassName);
            mPeriodicIO.schedDeltaDesired = mFastCycle; // Set aligned with shooter frequency
            actionHasStarted = false;
        }

        // when there are no balls then go to hold
        if (mIndexer.getBallCount() == 0) {
            mWantedState = WantedState.HOLD;
        }
        // now only do something if shooter is ready and we have not already started
        // shooting
        else if (!actionHasStarted && mShooter.readyToShoot()) {
            mIndexer.setWantedState(Indexer.WantedState.FEED, sClassName);
            actionHasStarted = true;
        }

        // everything is put into hold when the state changes
        return defaultStateTransfer();
    }

    // Unused: Lower priority in reference to other states
    private SystemState handleAutoClimbing() {
        if (mStateChanged) {

        }

        return defaultStateTransfer();
    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case DISABLE:
                return SystemState.DISABLING;
            case COLLECT:
                return SystemState.COLLECTING;
            case BACK:
                return SystemState.BACKING;
            case AUTO_SHOOT:
                return SystemState.AUTO_SHOOTING;
            case MANUAL_SHOOT:
                return SystemState.MANUAL_SHOOTING;
            case AUTO_CLIMB:
                return SystemState.AUTO_CLIMBING;
            case HOLD:
            default:
                return SystemState.HOLDING;
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

    public WantedState getWantedState() {
        return mWantedState;
    }

    public void setManualShootDistance(double distance) {
        mManualDistance = distance;
    }

    public void setOpenLoopClimb(double climbSpeed, int deploySlappyState) {
        mClimber.setClimbSpeed(climbSpeed);
        if (deploySlappyState == 0) {
            mClimber.setSlappyStickState(true);
        } else if (deploySlappyState == 1) {
            mClimber.setSlappyStickState(false);
        }
    }

    @Override
    public void writePeriodicOutputs() {
    }

    @Override
    public int whenRunAgain() {
        return mPeriodicIO.schedDeltaDesired;
    }

    @Override
    public void stop() {
        System.out.println(sClassName + " stop()");
        // TODO Auto-generated method stub

    }

    @Override
    public void readPeriodicInputs() {
        double now = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart = now;
    }

    @Override
    public String getLogHeaders() {
        // TODO Auto-generated method stub
        return "Superstructure";
    }

    @Override
    public String getLogValues(boolean telemetry) {
        // TODO Auto-generated method stub
        return "Superstructure.Values";
    }

    @Override
    public void outputTelemetry() {

    }

    public static class PeriodicIO {
        // Logging
        private int schedDeltaDesired;
        public double schedDeltaActual;
        public double schedDuration;
        private double lastSchedStart;
    }

}