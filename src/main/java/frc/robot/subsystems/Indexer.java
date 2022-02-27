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
import frc.robot.Constants;
import frc.robot.Ports;
import libraries.cheesylib.drivers.TalonFXFactory;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.subsystems.SubsystemManager;
import libraries.cheesylib.util.LatchedBoolean;

public class Indexer extends Subsystem{

    //Hardware
    private final TalonFX mFXIndexer;
    private final AnalogInput mAIBallEntering;
    private final AnalogInput mAIBallExiting;

    //Subsystem Constants
    private final double kFirstPositionDelta = 17000; // 15700; Continue tuning
    private final double kSecondPositionDelta = 21200; // 20200;
    private final double kBeamBreakThreshold = 3.0;

    //Configuration Constants
    private final double kIndexerKp = 0.15;
    private final double kIndexerKi = 0.0;
    private final double kIndexerKd = 0.0;
    private final double kIndexerKf = 0.0;
    private final double kClosedRamp = 0.0;
    private final double kClosedError = 0.0;

    private final double kIndexerCurrentLimit = 50;

    //Subsystem States
    public enum SystemState {
        HOLDING,
        LOADING,
        LOADING_FIRST,
        LOADING_SECOND,
        FEEDING,
        BACKING
    }

    public enum WantedState {
        HOLD,
        LOAD,
        LOAD_FIRST,
        LOAD_SECOND,
        FEED,
        BACK        
    }

    private SystemState mSystemState;
    private WantedState mWantedState;
    private boolean mStateChanged;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private LatchedBoolean mSystemStateChange = new LatchedBoolean();

    private boolean mCountedBalls = false;
    private int mBallCount;
    private boolean firstTime = true;

    //Other
    private SubsystemManager mSubsystemManager;
    
    //Subsystem Creation
    private static String sClassName;
    private static int sInstanceCount;
    private static Indexer sInstance = null;
    public static Indexer getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new Indexer(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+" getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Indexer(String caller){
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mFXIndexer = TalonFXFactory.createDefaultTalon(Ports.INDEXER);
        mAIBallEntering = new AnalogInput(Ports.ENTRANCE_BEAM_BREAK);
        mAIBallExiting = new AnalogInput(Ports.EXIT_BEAM_BREAK);
        mSubsystemManager = SubsystemManager.getInstance(sClassName);
        configMotors();
    }

    private void configMotors(){
        mFXIndexer.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        mFXIndexer.setControlFramePeriod(ControlFrame.Control_3_General,18);

        mFXIndexer.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mFXIndexer.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mFXIndexer.setInverted(true);
        mFXIndexer.setSensorPhase(false);

        mFXIndexer.setNeutralMode(NeutralMode.Brake);

        mFXIndexer.config_kP(0, kIndexerKp, Constants.kLongCANTimeoutMs);
        mFXIndexer.config_kI(0, kIndexerKi, Constants.kLongCANTimeoutMs);
        mFXIndexer.config_kD(0, kIndexerKd, Constants.kLongCANTimeoutMs);
        mFXIndexer.config_kF(0, kIndexerKf, Constants.kLongCANTimeoutMs);
        mFXIndexer.config_IntegralZone(0, 0, Constants.kLongCANTimeoutMs);
        mFXIndexer.configClosedloopRamp(kClosedRamp, Constants.kLongCANTimeoutMs);
        mFXIndexer.configAllowableClosedloopError(0, kClosedError, Constants.kLongCANTimeoutMs);

        mFXIndexer.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kIndexerCurrentLimit, kIndexerCurrentLimit, 0));

    }

    @Override
    public void onStart(Phase phase){
        synchronized (Indexer.this) {
            mSystemState = SystemState.HOLDING;
            mWantedState = WantedState.HOLD;
            mStateChanged = true;
            System.out.println(sClassName + " state " + mSystemState);
            // this subsystem is "on demand" so
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
            mCountedBalls = false;
            firstTime = true;
        }
    }

    @Override
    public void onLoop(double timestamp){
        synchronized (Indexer.this) {
            do{
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
                default:
                    newState = handleHolding();
                    break;
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

    public synchronized void setWantedState(WantedState state) {
        if (state != mWantedState) {
            mSubsystemManager.scheduleMe(mListIndex, 1, true);
            System.out.println("waking " + sClassName);
        }

        mWantedState = state;
    }

    private SystemState handleHolding() {
        if(mStateChanged){
            mPeriodicIO.indexerDemand = mPeriodicIO.indexerPosition;
        }
        if(!mCountedBalls) {
            if(isBallEntering()){
                mBallCount++;
            }
            if(isFullyLoaded()){
                mBallCount++;
            }
            mCountedBalls = true;
        }

        return defaultStateTransfer();
    }
    
    private SystemState handleLoading() {

        if(isBallEntering() || !firstTime) {
            if(mBallCount == 0) {
                // System.out.println("Loading First Ball");
                loadFirstBall();
            } else if (mBallCount == 1) {
                // System.out.println("Loading Second Ball");
                loadSecondBall();
            } else {
                mPeriodicIO.indexerDemand = mPeriodicIO.indexerPosition;
            }
        }

        return defaultStateTransfer();
    }

    private void loadFirstBall() {
        if(firstTime) {
            mPeriodicIO.startIndexerPosition = mPeriodicIO.indexerPosition;
            mPeriodicIO.indexerDemand = mPeriodicIO.startIndexerPosition + kFirstPositionDelta;
            firstTime = false;
        }

        if(Math.abs(mPeriodicIO.indexerPosition - mPeriodicIO.indexerDemand) <= 300) {
            mBallCount++;
            firstTime = true;
        }
    }

    private void loadSecondBall() {
        if(firstTime) {
            mPeriodicIO.indexerDemand = mPeriodicIO.startIndexerPosition + kSecondPositionDelta;
            // System.out.println("Demand 2 " + mPeriodicIO.indexerDemand);
            firstTime = false;
        }

        // System.out.println("Difference " + Math.abs(mPeriodicIO.indexerPosition - mPeriodicIO.indexerDemand));
        if(Math.abs(mPeriodicIO.indexerPosition - mPeriodicIO.indexerDemand) <= 300) {
            mBallCount++;
            firstTime = true;
        }
    }

    private SystemState handleFeeding() {
        if(mStateChanged){
            mPeriodicIO.indexerDemand = mPeriodicIO.indexerPosition + kSecondPositionDelta;
        }

        if(Math.abs(mPeriodicIO.indexerPosition - mPeriodicIO.indexerDemand) <= 300) {
            mBallCount = 0;
        }

        return defaultStateTransfer();
    }

    private SystemState handleBacking() {
        if(mStateChanged){
            mPeriodicIO.indexerDemand = mPeriodicIO.indexerPosition - kSecondPositionDelta;
        }

        if(Math.abs(mPeriodicIO.indexerPosition - mPeriodicIO.indexerDemand) <= 300) {
            mBallCount = 0;
        }

        return defaultStateTransfer();
    }

    private SystemState defaultStateTransfer(){
        switch(mWantedState){
            case LOAD:
                return SystemState.LOADING;
            case LOAD_FIRST:
                return SystemState.LOADING_FIRST;
            case LOAD_SECOND:
                return SystemState.LOADING_SECOND;
            case FEED:
                return SystemState.FEEDING;
            case BACK:
                return SystemState.BACKING;
            case HOLD:
            default:
                return SystemState.HOLDING;
        }
    }

    //Called in superstructure to manage loading balls
    public boolean isBallEntering() {
        return mAIBallEntering.getVoltage() < kBeamBreakThreshold;
    }

    public boolean isFullyLoaded() {
        return mAIBallExiting.getVoltage() < kBeamBreakThreshold;
    }

    public int getBallCount() {
        return mBallCount;
    }

    public void resetBallCount() {
        mBallCount = 0;
    }

    @Override
    public void readPeriodicInputs() {
        double now       = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart   = now;

        mPeriodicIO.indexerPosition = mFXIndexer.getSelectedSensorPosition();
    }

    @Override
    public void writePeriodicOutputs() {
        //System.out.println("Demand " + mPeriodicIO.indexerDemand);
        mFXIndexer.set(ControlMode.Position, mPeriodicIO.indexerDemand);
    }


    @Override
    public void stop() {
        mFXIndexer.set(ControlMode.Position, mPeriodicIO.indexerPosition);
    }

    @Override
    public int whenRunAgain () {
        return mPeriodicIO.schedDeltaDesired;
    }

    @Override
    public String getLogHeaders() {
        return "Indexer";
    }

    @Override
    public String getLogValues(boolean telemetry) {
        return "Indexer.Values";
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Ball Entering", isBallEntering());
        SmartDashboard.putBoolean("Fully Loaded", isFullyLoaded());
        SmartDashboard.putNumber("Indexer Position", mPeriodicIO.indexerPosition); 
        SmartDashboard.putNumber("Ball Count", mBallCount);      
    }

    public static class PeriodicIO{
            //Logging
        @SuppressWarnings("unused")
        private final int mDefaultSchedDelta = 20; // axis updated every 20 msec
        private int    schedDeltaDesired;
        public  double schedDeltaActual;
        public  double schedDuration;
        private double lastSchedStart;

        //Inputs
        private double indexerPosition;
        private double startIndexerPosition;

        //Outputs
        private double indexerDemand;
    }

}