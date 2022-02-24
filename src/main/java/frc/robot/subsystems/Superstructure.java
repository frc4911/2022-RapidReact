package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import libraries.cheesylib.loops.ILooper;
import libraries.cheesylib.loops.Loop;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.util.LatchedBoolean;

public class Superstructure extends Subsystem{
    
    //Subsystem Instances
    @SuppressWarnings("unused")
    private Swerve    mSwerve;
    private Indexer   mIndexer;
    private Collector mCollector;
    private Shooter   mShooter;
    private Climber   mClimber;

    //Superstructure States
    public enum SystemState{
        HOLDING,
        COLLECTING,
        BACKING,
        AUTO_SHOOTING,
        MANUAL_SHOOTING,
        AUTO_CLIMBING
    }
    
    public enum WantedState{
        HOLD,
        COLLECT,
        BACK,
        AUTO_SHOOT,
        MANUAL_SHOOT,
        AUTO_CLIMB
    }

    private SystemState    mSystemState;
    private WantedState    mWantedState;
    private boolean        mStateChanged;
    private LatchedBoolean mSystemStateChange = new LatchedBoolean();
    private PeriodicIO     mPeriodicIO = new PeriodicIO();
    private int            mFastCycle = 10;
    private int            mSlowCycle = 100;

    private double  mManualDistance;
    private boolean mShootSetup;
    private int mListIndex;

    private static String sClassName;
    private static int sInstanceCount;
    private static Superstructure sInstance = null;
    public  static Superstructure getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new Superstructure(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+" getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Superstructure(String caller){
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mSwerve =    Swerve.getInstance(sClassName);
        mIndexer =   Indexer.getInstance(sClassName);
        mCollector = Collector.getInstance(sClassName);
        mShooter =   Shooter.getInstance(sClassName);
        mClimber =   Climber.getInstance(sClassName);
    }

    @Override
    public void onStart(Phase phase){
        synchronized(Superstructure.this) {
            mSystemState = SystemState.HOLDING;
            mWantedState = WantedState.HOLD;
            mStateChanged = true;
            System.out.println(sClassName + " state " + mSystemState);
            switch (phase) {
                case DISABLED:
                    mPeriodicIO.schedDeltaDesired = 0; // goto sleep
                    break;
                default:
                    mPeriodicIO.schedDeltaDesired = 100;
                    break;
            }
            stop();
        }
    }

    @Override
    public void onLoop(double timestamp){
        synchronized(Superstructure.this) {
            do{
                SystemState newState;
                switch(mSystemState) {
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
                    case HOLDING:
                    default:
                        newState = handleHolding();
                }

                if (newState != mSystemState) {
                    System.out.println(sClassName + " state " + mSystemState + " to " + newState + " (" + timestamp + ")");
                    mSystemState = newState;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            } while(mSystemStateChange.update(mStateChanged));
        }
    }

    // Handling methods
    private SystemState handleHolding() {
        if(mStateChanged){
            mCollector.setWantedState(Collector.WantedState.HOLD);
            mIndexer.setWantedState(Indexer.WantedState.HOLD);
            mShooter.setWantedState(Shooter.WantedState.HOLD);
            mClimber.setWantedState(Climber.WantedState.HOLD);
            mPeriodicIO.schedDeltaDesired = mSlowCycle;
        }

        return defaultStateTransfer();
    }

    private SystemState handleCollecting() {
        if(mStateChanged){
            mPeriodicIO.schedDeltaDesired = mFastCycle;
        }
        
        if(!mIndexer.isFullyLoaded()){
            mCollector.setWantedState(Collector.WantedState.COLLECT);
            if(mIndexer.isBallEntering()){
                mIndexer.setWantedState(Indexer.WantedState.LOAD);
            } else {
                mCollector.setWantedState(Collector.WantedState.HOLD);
                mIndexer.setWantedState(Indexer.WantedState.HOLD);
            }
        } else {
            mCollector.setWantedState(Collector.WantedState.HOLD);
            mIndexer.setWantedState(Indexer.WantedState.HOLD);
        }

        return collectingStateTransfer();
    }

    private SystemState handleBacking() {
        if(mStateChanged){
            mPeriodicIO.schedDeltaDesired = mSlowCycle;
        }

        mCollector.setWantedState(Collector.WantedState.BACK);
        mIndexer.setWantedState(Indexer.WantedState.BACK);

        return backingStateTransfer();
    }

    // TODO: Get help with logic and limelight implementation - CURRENTLY UNUSED
    // If time constrains, may not be complete by Week 1
    private SystemState handleAutoShooting() {
        if(mStateChanged){
            mShootSetup = true;
            mPeriodicIO.schedDeltaDesired = mFastCycle;
        }

        if (mShooter.readyToShoot() || !mShootSetup) {
            mIndexer.setWantedState(Indexer.WantedState.FEED);
            mShootSetup = false;
        } else {
            mIndexer.setWantedState(Indexer.WantedState.HOLD);
        }

        return shootingStateTransfer();
    }

    private SystemState handleManualShooting() {
        if (mStateChanged) {
            mShooter.setShootDistance(mManualDistance);
            mShooter.setWantedState(Shooter.WantedState.SHOOT);
            mPeriodicIO.schedDeltaDesired = 20; // Set aligned with shooter frequency
        }

        if (mShooter.readyToShoot()) {
            mIndexer.setWantedState(Indexer.WantedState.FEED);
        } else {
            mIndexer.setWantedState(Indexer.WantedState.HOLD);
        }

        return shootingStateTransfer();
    }

    // Unused: Lower priority in reference to other states
    private SystemState handleAutoClimbing() {
        if(mStateChanged){
            
        }

        return climbingStateTransfer();
    }

    // State Transfers
    // Executes subsystem actions before switching to another state

    private SystemState collectingStateTransfer() {
        if(mSystemState != SystemState.COLLECTING){
            mCollector.setWantedState(Collector.WantedState.HOLD);
            mIndexer.setWantedState(Indexer.WantedState.HOLD);
        }

        return defaultStateTransfer();
    }

    private SystemState backingStateTransfer() {
        if(mSystemState != SystemState.BACKING){
            mCollector.setWantedState(Collector.WantedState.HOLD);
            mIndexer.setWantedState(Indexer.WantedState.HOLD);
        }

        return defaultStateTransfer();
    }

    private SystemState shootingStateTransfer() {
        if(mSystemState != SystemState.AUTO_SHOOTING && mSystemState != SystemState.MANUAL_SHOOTING){
            mShooter.setWantedState(Shooter.WantedState.HOLD);
            mIndexer.setWantedState(Indexer.WantedState.HOLD);
        }

        return defaultStateTransfer();
    }

    private SystemState climbingStateTransfer() {
        if(mSystemState != SystemState.AUTO_CLIMBING){
            mClimber.setWantedState(Climber.WantedState.HOLD);
        }

        return defaultStateTransfer();
    }

    private SystemState defaultStateTransfer() {
        switch(mWantedState){
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

    public synchronized void setWantedState(WantedState state) {
        if (mWantedState != state){
            System.out.println(sClassName + " to " +state);
            mPeriodicIO.schedDeltaDesired = 2;
        }
        mWantedState = state;
    }


    public WantedState getWantedState() {
        return mWantedState;
    }

    public void setManualShootDistance(double distance){
        mManualDistance = distance;
    }

    public void setOpenLoopClimb(double climbSpeed, int deploySlappyState){
        mClimber.setClimbSpeed(climbSpeed);
        if(deploySlappyState == 0){
            mClimber.setSlappyStickState(true);
        } else if(deploySlappyState == 1){
            mClimber.setSlappyStickState(false);
        }
    }

    @Override
    public void writePeriodicOutputs() {
    }

    @Override
    public int whenRunAgain () {
        return mPeriodicIO.schedDeltaDesired;
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void readPeriodicInputs() {
        double now = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart = now;
    }

    @Override
    public void passInIndex(int listIndex) {
        mListIndex = listIndex;
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
        // TODO Auto-generated method stub
        
    }

    public static class PeriodicIO{
        //Logging
        private int schedDeltaDesired;
        public  double schedDeltaActual;
        public  double schedDuration;
        private double lastSchedStart;
    }

}