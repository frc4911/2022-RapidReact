package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Ports;
import frc.robot.subsystems.Shooter.SystemState;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.subsystems.SubsystemManager;
import libraries.cheesylib.util.LatchedBoolean;
import libraries.cyberlib.utils.CyberMath;

public class Superstructure extends Subsystem {

    // Subsystem Instances
    @SuppressWarnings("unused")
    private Swerve mSwerve;
    private Indexer mIndexer;
    private Collector mCollector;
    private Shooter mShooter;
    private Climber mClimber;
  
    private final AnalogInput mAIPressureSensor;

    // Superstructure States
    public enum SystemState {
        DISABLING,
        HOLDING,
        COLLECTING,
        BACKING,
        AUTO_SHOOTING,
        MANUAL_SHOOTING,
        AUTO_CLIMBING,
        AUTO_PRE_CLIMBING
    }

    public enum WantedState {
        DISABLE,
        HOLD,
        COLLECT,
        BACK,
        AUTO_SHOOT,
        MANUAL_SHOOT,
        AUTO_PRE_CLIMB,
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
        mAIPressureSensor = new AnalogInput(Ports.PRESSURE_SENSOR);
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
                    case AUTO_PRE_CLIMBING:
                        newState = handlePreClimbing();
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
            mCollector.setWantedState(Collector.WantedState.COLLECT, sClassName);
            mIndexer.setWantedState(Indexer.WantedState.LOAD, sClassName);
        }

        return defaultStateTransfer();
    }

    private SystemState handleBacking() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = mFastCycle;
            mCollector.setWantedState(Collector.WantedState.BACK, sClassName);
            mIndexer.setWantedState(Indexer.WantedState.BACK, sClassName);
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
        }

        if (!mShooter.getWantedState().equals(Shooter.WantedState.HOMEHOOD) &&
            !mShooter.getWantedState().equals(Shooter.WantedState.SHOOT)){
            mShooter.setWantedState(Shooter.WantedState.SHOOT, sClassName);
        }
        // now only do something if shooter is ready and we have not already started shooting
        if (mShooter.readyToShoot()) {
            mIndexer.setWantedState(Indexer.WantedState.FEED, sClassName);
        }
        else{
            mIndexer.setWantedState(Indexer.WantedState.HOLD, sClassName);
        }

        // everything is put into hold when the state changes
        return defaultStateTransfer();
    }

    // Unused: Lower priority in reference to other states
    private SystemState handleAutoClimbing() {
        if (mStateChanged) {
            mClimber.setWantedState(Climber.WantedState.GRAB_BAR_DYNAMIC_CLAW, sClassName);
        }

        return defaultStateTransfer();
    }

    private SystemState handlePreClimbing() {
        if (mStateChanged) {
            mClimber.setWantedState(Climber.WantedState.PRECLIMB,sClassName);
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
            case AUTO_PRE_CLIMB:
                return SystemState.AUTO_PRE_CLIMBING;
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
        System.out.println("setManualShootDistance "+distance);
        mManualDistance = distance;
    }

    public void setOpenLoopClimb(double climbSpeed, int deploySlappyState) {
        // mClimber.setClimbSpeed(climbSpeed);
        // if (deploySlappyState == 0) {
        //     mClimber.setSlappyStickState(true);
        // } else if (deploySlappyState == 1) {
        //     mClimber.setSlappyStickState(false);
        // }
    }

    // used in auto
    public boolean autoShootingComplete() {
        return mIndexer.feedingComplete();
    }

    // max voltage is 2.6 which is 100 PSI
    // min voltage is 0.5 which is 0 PSI
    private double convertSensorToPSI(double sensorValue){
        return sensorValue * ((100-0)/(2.6-.5)) - 24.3;
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
    }

    @Override
    public void readPeriodicInputs() {
        double now = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart = now;

        mPeriodicIO.pressure = convertSensorToPSI(mAIPressureSensor.getVoltage());
    }

    @Override
    public String getLogHeaders() {
        return  sClassName+".schedDeltaDesired,"+
                sClassName+".schedDeltaActual,"+
                sClassName+".schedDuration,"+
                sClassName+".mSystemState,"+
                sClassName+".mWantedState,"+
                sClassName+".pressure";
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
        mPeriodicIO.pressure;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Pressure Sensor", CyberMath.cTrunc(mPeriodicIO.pressure, 10));
    }

    public static class PeriodicIO {
        // Logging
        private int schedDeltaDesired;
        public double schedDeltaActual;
        private double lastSchedStart;

        // Inputs
        private double pressure;
    }

}