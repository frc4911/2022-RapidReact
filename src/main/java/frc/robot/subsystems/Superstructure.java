package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.constants.Ports;
import frc.robot.limelight.LimelightManager;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.subsystems.SubsystemManager;
import libraries.cheesylib.util.LatchedBoolean;
import libraries.cheesylib.util.Util;
import libraries.cyberlib.utils.Angles;
import libraries.cyberlib.utils.CyberMath;

public class Superstructure extends Subsystem {

    // Subsystem Instances

    private final Swerve mSwerve;
    private final Indexer mIndexer;
    private final Collector mCollector;
    private final Shooter mShooter;
    private final Climber mClimber;
    private final LimelightManager mLLManager = LimelightManager.getInstance();
    private final AnalogInput mAIPressureSensor;

    // Superstructure States
    public enum SystemState {
        DISABLING,
        HOLDING,
        TESTING,
        COLLECTING,
        BACKING,
        AUTO_SHOOTING,
        MANUAL_SHOOTING,
        AUTO_CLIMBING,
        AUTO_PRE_CLIMBING,
        HOMING
    }

    public enum WantedState {
        DISABLE,
        HOLD,
        TEST,
        COLLECT,
        BACK,
        AUTO_SHOOT,
        MANUAL_SHOOT,
        AUTO_PRE_CLIMB,
        AUTO_CLIMB,
        HOME
    }

    private SystemState mSystemState;
    private WantedState mWantedState;
    private boolean mStateChanged;
    private final LatchedBoolean mSystemStateChange = new LatchedBoolean();

    private boolean mHasTarget = false;
    private boolean mOnTarget = false;
    private double mYOffset;
    private double mXOffset;

    private double mSwerveFeedforwardFromVision = 0.0;


    private boolean mOverrideLimelightLEDs = false;


    private final PeriodicIO mPeriodicIO = new PeriodicIO();
    private int mFastCycle = 20;
    private int mSlowCycle = 100;

    private double mManualDistance;
    private boolean mStartedShooting;

    private static String sClassName;
    private static int sInstanceCount;
    private static Superstructure sInstance = null;
    private final SubsystemManager mSubsystemManager;

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
            switch (phase) {
                case DISABLED:
                case TEST:
                    mSystemState = SystemState.DISABLING;
                    mWantedState = WantedState.DISABLE;
                    break;
                case AUTONOMOUS:
                case TELEOP:
                    mSystemState = SystemState.HOLDING;
                    mWantedState = WantedState.HOLD;
                    mOverrideLimelightLEDs = false;
                    break;
            }
            mStateChanged = true;
            mPeriodicIO.schedDeltaDesired = mFastCycle;
            System.out.println(sClassName + " state " + mSystemState);
        }
    }

    @Override
    public void onLoop(double timestamp) {
        synchronized (Superstructure.this) {
            do {
                SystemState newState = null;
                switch (mSystemState) {
                    case COLLECTING:
                        newState = handleCollecting();
                        break;
                    case BACKING:
                        newState = handleBacking();
                        break;
                    case AUTO_SHOOTING:
                        newState = handleAutoShooting(timestamp);
                        break;
                    case MANUAL_SHOOTING:
                        newState = handleManualShooting(timestamp);
                        break;
                    case AUTO_CLIMBING:
                        newState = handleAutoClimbing();
                        break;
                    case AUTO_PRE_CLIMBING:
                        newState = handlePreClimbing();
                        break;
                    case HOMING:
                        newState = handleHoming();
                        break;
                    case TESTING:
                        newState = handleTesting();
                        break;
                    case DISABLING:
                        newState = handleDisabling();
                        break;
                    case HOLDING:
                        newState = handleHolding();
                    // default: leave commented so compiler finds missed cases
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
            if (!mOverrideLimelightLEDs) {
                mLLManager.getLimelight().setLed(Limelight.LedMode.PIPELINE);
            }
            mPeriodicIO.schedDeltaDesired = mSlowCycle;
        }

        return defaultStateTransfer();
    }

    private SystemState handleHolding() {
        if (mStateChanged) {
            if (!mOverrideLimelightLEDs) {
                mLLManager.getLimelight().setLed(Limelight.LedMode.PIPELINE);
            }
            if(mCollector.getWantedState() != Collector.WantedState.HOLD){
                mCollector.setWantedState(Collector.WantedState.HOLD, sClassName);
            }
            if(mIndexer.getWantedState() != Indexer.WantedState.HOLD){
                mIndexer.setWantedState(Indexer.WantedState.HOLD, sClassName);
            }
            
            if(mShooter.getWantedState() != Shooter.WantedState.HOLD){
                mShooter.setWantedState(Shooter.WantedState.HOLD, sClassName);
            }
            
            if(mClimber.getWantedState() != Climber.WantedState.HOLD){
                mClimber.setWantedState(Climber.WantedState.HOLD, sClassName);
            }
            
            mPeriodicIO.schedDeltaDesired = mSlowCycle;
        }

        return defaultStateTransfer();
    }

    private SystemState handleTesting() {
        if (mStateChanged) {
            mClimber.setWantedState(Climber.WantedState.MANUAL_CONTROL, sClassName);
            mPeriodicIO.schedDeltaDesired = mFastCycle;
        }

        return defaultStateTransfer();
    }

    private SystemState handleHoming() {
        if (mStateChanged) {
            if (!mOverrideLimelightLEDs) {
                mLLManager.getLimelight().setLed(Limelight.LedMode.PIPELINE);
            }
            // mShooter.setWantedState(Shooter.WantedState.HOLD, sClassName);
            mClimber.setWantedState(Climber.WantedState.HOME, sClassName);
            mPeriodicIO.schedDeltaDesired = mFastCycle;
        }

        if (mWantedState != WantedState.HOME){
            // mShooter.setWantedState(Shooter.WantedState.TEST, sClassName);
            mClimber.setWantedState(Climber.WantedState.MANUAL_CONTROL, sClassName);
        }

        return defaultStateTransfer();
    }

    private SystemState handleCollecting() {
        if (mStateChanged) {
            if (!mOverrideLimelightLEDs) {
                mLLManager.getLimelight().setLed(Limelight.LedMode.PIPELINE);
            }
            mPeriodicIO.schedDeltaDesired = mFastCycle;
            mCollector.setWantedState(Collector.WantedState.COLLECT, sClassName);
            mIndexer.setWantedState(Indexer.WantedState.LOAD, sClassName);
        }

        return defaultStateTransfer();
    }

    private SystemState handleBacking() {
        if (mStateChanged) {
            if (!mOverrideLimelightLEDs) {
                mLLManager.getLimelight().setLed(Limelight.LedMode.PIPELINE);
            }
            mPeriodicIO.schedDeltaDesired = mFastCycle;
            mCollector.setWantedState(Collector.WantedState.BACK, sClassName);
            mIndexer.setWantedState(Indexer.WantedState.BACK, sClassName);
        }

        return defaultStateTransfer();
    }

     double mSetPointInRadians;
    private SystemState handleAutoShooting(double timestamp) {
        if (mStateChanged) {
            
            if (!mOverrideLimelightLEDs) {
                mLLManager.getLimelight().setLed(Limelight.LedMode.PIPELINE);
            }
            mStartedShooting = false;
            mPeriodicIO.schedDeltaDesired = mFastCycle;
            mShooter.setWantedState(Shooter.WantedState.SHOOT, sClassName);
            mSetPointInRadians = Double.NaN;
        }
        var setPointInRadians = getSwerveSetpointFromVision(timestamp);
        
        if (mHasTarget) {
            
            if (Double.isNaN(mSetPointInRadians)){
                mSetPointInRadians = setPointInRadians;
            }

            if (!mOnTarget){
                mSwerve.setAimingSetpoint(mSetPointInRadians, 0.0 /*mSwerveFeedforwardFromVision*/, timestamp);
            }
            mShooter.setShootDistanceFromTY(mYOffset);

            // Shoot while aiming
            if (mOnTarget || mStartedShooting) {
                // mSwerve.stop();
                if (mShooter.readyToShoot() || mStartedShooting) {
                    if (mIndexer.getWantedState() != Indexer.WantedState.FEED) {
                        mIndexer.setWantedState(Indexer.WantedState.FEED, sClassName);
                    }
                    mStartedShooting = true;
                } else {
                    if (mIndexer.getWantedState() != Indexer.WantedState.HOLD) {
                        mIndexer.setWantedState(Indexer.WantedState.HOLD, sClassName);
                    }
                }
            }
        }

        return defaultStateTransfer();
    }

    /**
     * Get the setpoint to be used when aiming the chassis.
     *
     * <p>The state variables mHasTarget and mOnTarget are updated to reflect
     * the validity of the setpoint.
     *
     * @param timestamp current time
     * @return The setpoint
     */
    public synchronized double getSwerveSetpointFromVision(double timestamp) {

        if(mLLManager.getLimelight().seesTarget()){
            mXOffset = -mLLManager.getLimelight().getXOffset();

            double setPointInRadians =  mSwerve.getHeading().getRadians() + Math.toRadians(mXOffset);
            mYOffset = mLLManager.getLimelight().getYOffset();

            mHasTarget = true;
            mOnTarget = Util.epsilonEquals(mXOffset, 0.0, 3);

            return Angles.normalizeAngle(setPointInRadians);        }
        else {
            mHasTarget = false;
            mOnTarget = false;
            System.out.println("Superstruct no aimingParameters");

            return mSwerve.getHeading().getRadians();
        }
    }

    private SystemState handleManualShooting(double timestamp) {
        if (mStateChanged) {
            if (!mOverrideLimelightLEDs) {
                mLLManager.getLimelight().setLed(Limelight.LedMode.PIPELINE);
            }
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
        if (mShooter.readyToShoot() && 
            !mIndexer.getWantedState().equals(Indexer.WantedState.FEED)) {
            mIndexer.setWantedState(Indexer.WantedState.FEED, sClassName);
        }

        // everything is put into hold when the state changes
        return defaultStateTransfer();
    }

    private SystemState handlePreClimbing() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = mFastCycle;
            mClimber.setWantedState(Climber.WantedState.PRECLIMB,sClassName);
        }

        return defaultStateTransfer();
    }

    // Unused: Lower priority in reference to other states
    private SystemState handleAutoClimbing() {
        if (mStateChanged) {
            if (!mOverrideLimelightLEDs) {
                mLLManager.getLimelight().setLed(Limelight.LedMode.OFF);
            }
            mPeriodicIO.schedDeltaDesired = mFastCycle;
            mClimber.setWantedState(Climber.WantedState.CLIMB_1_LIFT, sClassName);
        }

        switch(mClimber.getWantedState()) {
            case CLIMB_1_LIFT:
                if (mClimber.isClimbingStageDone(Climber.WantedState.CLIMB_1_LIFT)) {
                    mClimber.setWantedState(Climber.WantedState.CLIMB_2_ROTATE_UP, sClassName);
                }
                break;
            case CLIMB_2_ROTATE_UP:
                if (mClimber.isClimbingStageDone(Climber.WantedState.CLIMB_2_ROTATE_UP)) {
                    mClimber.setWantedState(Climber.WantedState.CLIMB_3_LIFT_MORE, sClassName);
                }
                break;
            case CLIMB_3_LIFT_MORE:
                if (mClimber.isClimbingStageDone(Climber.WantedState.CLIMB_3_LIFT_MORE)) {
                    mClimber.setWantedState(Climber.WantedState.CLIMB_4_ENGAGE_TRAV, sClassName);
                }
                break;
            case CLIMB_4_ENGAGE_TRAV:
                if (mClimber.isClimbingStageDone(Climber.WantedState.CLIMB_4_ENGAGE_TRAV)) {
                    mClimber.setWantedState(Climber.WantedState.CLIMB_5_RELEASE_MID, sClassName);
                }
                break;
            case CLIMB_5_RELEASE_MID: // Done with climb
                break;
            default:
                System.out.println("Climber Exiting Auto Sequence!!!");
                break;
        }

        return defaultStateTransfer();
    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case DISABLE:
                return SystemState.DISABLING;
            case TEST:
                return SystemState.TESTING;
            case COLLECT:
                return SystemState.COLLECTING;
            case BACK:
                return SystemState.BACKING;
            case HOME:
                return SystemState.HOMING;
            case AUTO_SHOOT:
                return SystemState.AUTO_SHOOTING;
            case MANUAL_SHOOT:
                return SystemState.MANUAL_SHOOTING;
            case AUTO_CLIMB:
                return SystemState.AUTO_CLIMBING;
            case AUTO_PRE_CLIMB:
                return SystemState.AUTO_PRE_CLIMBING;
            case HOLD:
                return SystemState.HOLDING;
            // default: // leave commented so compiler helps identify missing cases
            }
            return null;
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
        return mIndexer.isHandlerComplete(Indexer.WantedState.FEED);
    }

    // max voltage is 2.6 which is 100 PSI
    // min voltage is 0.5 which is 0 PSI
    private double convertSensorToPSI(double sensorValue){
        return sensorValue * ((100-0)/(2.6-.5)) - 24.3;
    }


    public boolean visionHasTarget() {
        return mHasTarget;
    }

    public void setOverrideLimelightLEDs(boolean should_override) {
        mOverrideLimelightLEDs = should_override;
    }

    /**
     * pre condition: getSwerveSetpointFromVision() is called
     *
     * @return swerve feedforward voltage
     */
    public synchronized double getSwerveFeedforwardVFromVision() {
        return mSwerveFeedforwardFromVision;
    }

    public synchronized void resetAimingParameters() {
        mHasTarget = false;
        mOnTarget = false;
        mSwerveFeedforwardFromVision = 0.0;
    }

    public synchronized boolean isOnTarget() {
        return mOnTarget;
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
                sClassName+".mHasTarget,"+
                sClassName+".mOnTarget,"+
                sClassName+".mXOffset,"+
                sClassName+".mYOffset,"+
                sClassName+".mManualDistance,"+
                sClassName+".mStartedShooting,"+
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
        mHasTarget+","+
        mOnTarget+","+
        mXOffset+","+
        mYOffset+","+
        mManualDistance+","+
        mStartedShooting+","+
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