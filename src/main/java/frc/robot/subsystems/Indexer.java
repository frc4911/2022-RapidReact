package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
    private final double kFeedSpeed = 0.40;
    private final double kLoadSpeed = 0.25; // Need to tune
    private final double kBackSpeed = 0.50; // Speed is kept as a magnitude; must make negative for backward

    private final double kBeamBreakThreshold = 3.0;

    //Subsystem States
    public enum SystemState {
        HOLDING,
        LOADING,
        FEEDING,
        BACKING
    }

    public enum WantedState {
        HOLD,
        LOAD,
        FEED,
        BACK        
    }

    private SystemState mSystemState;
    private WantedState mWantedState;
    private boolean mStateChanged;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private LatchedBoolean mSystemStateChange = new LatchedBoolean();

    double indexSpeed;

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
        indexSpeed = SmartDashboard.getNumber("Indexing Speed", -1.0);
        if(indexSpeed == -1){
            SmartDashboard.putNumber("Indexing Speed", 0.0);
        }
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
    }

    @Override
    public void onStart(Phase phase){
        synchronized (Indexer.this) {
            mSystemState = SystemState.HOLDING;
            mWantedState = WantedState.HOLD;
            mStateChanged = true;
            System.out.println(sClassName + " state " + mSystemState);
            // this subsystem is "on demand" so
            mPeriodicIO.schedDeltaDesired = 0;
            stop(); // put into a known state
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
            mSubsystemManager.scheduleMe(mListIndex, 1, false);
            System.out.println("waking " + sClassName);
        }

        mWantedState = state;
    }

    private SystemState handleHolding() {
        if(mStateChanged){
            mPeriodicIO.controlMode = ControlMode.PercentOutput;
            mPeriodicIO.indexerDemand = 0.0;
        }

        return defaultStateTransfer();
    }
    
    private SystemState handleLoading() {
        if(mStateChanged){
            mPeriodicIO.controlMode = ControlMode.PercentOutput; // TODO:: Change to position
            indexSpeed = SmartDashboard.getNumber("Indexing Speed", 0.0);
            mPeriodicIO.indexerDemand = indexSpeed; // kLoadSpeed;
        }

        return defaultStateTransfer();
    }

    private SystemState handleFeeding() {
        if(mStateChanged){
            mPeriodicIO.controlMode = ControlMode.PercentOutput; // TODO: Change to position
            mPeriodicIO.indexerDemand = kFeedSpeed; // TODO: Update demand to move balls out of indexer by position in one set
        }

        return defaultStateTransfer();
    }

    private SystemState handleBacking() {
        if(mStateChanged){
            mPeriodicIO.controlMode = ControlMode.PercentOutput;
            mPeriodicIO.indexerDemand = -kBackSpeed;
        }

        return defaultStateTransfer();
    }

    private SystemState defaultStateTransfer(){
        switch(mWantedState){
            case LOAD:
                return SystemState.LOADING;
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
    public boolean isBallEntering(){
        return mAIBallEntering.getVoltage() < kBeamBreakThreshold;
    }

    public boolean isFullyLoaded(){
        return mAIBallExiting.getVoltage() < kBeamBreakThreshold;
    }

    @Override
    public void readPeriodicInputs() {
        double now       = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart   = now;
    }

    @Override
    public void writePeriodicOutputs() {
        mFXIndexer.set(mPeriodicIO.controlMode, mPeriodicIO.indexerDemand);
    }


    @Override
    public void stop() {
        mFXIndexer.set(ControlMode.PercentOutput, 0.0);

        mPeriodicIO.indexerDemand = 0.0;
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
    }

    public static class PeriodicIO{
            //Logging
        @SuppressWarnings("unused")
        private final int mDefaultSchedDelta = 100; // axis updated every 100 msec
        private int    schedDeltaDesired;
        public  double schedDeltaActual;
        public  double schedDuration;
        private double lastSchedStart;

        //Inputs

        //Outputs
        private ControlMode controlMode;
        private double indexerDemand;
    }

}