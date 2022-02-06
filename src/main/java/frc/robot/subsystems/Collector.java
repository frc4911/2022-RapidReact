package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Ports;
import libraries.cheesylib.drivers.TalonFXFactory;
import libraries.cheesylib.loops.ILooper;
import libraries.cheesylib.loops.Loop;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.subsystems.SubsystemManager;

public class Collector extends Subsystem{

    //Hardware
    private final TalonFX mFXCollector;
    private final Solenoid mSolenoid;

    //Subsystem Constants
    private final double kCollectSpeed = 0.5;

    //Subsystem States
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
        HOLDING,
        COLLECTING,
        BACKING
    }

    public enum WantedState {
        HOLD,
        COLLECT,
        BACK
    }

    private SystemState mSystemState;
    private WantedState mWantedState;
    private boolean mStateChanged;
    private PeriodicIO mPeriodicIO;
    private SolenoidState mSolenoidState;

    //Other
    private SubsystemManager mSubsystemManager;
    private int              mListIndex;
    
    //Subsystem Creation
    private static String sClassName;
    private static int sInstanceCount;
    private static Collector sInstance = null;
    public static Collector getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new Collector(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+" getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Collector(String caller){
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mFXCollector = TalonFXFactory.createDefaultTalon(Ports.COLLECTOR);
        mSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.COLLECTOR_DEPLOY);
        configMotors();
    }

    private void configMotors(){
        mFXCollector.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mFXCollector.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mFXCollector.setInverted(false);

        mFXCollector.setNeutralMode(NeutralMode.Coast);
    }

    private Loop mLoop = new Loop() {
        
        @Override
        public void onStart(Phase phase){
            synchronized (Collector.this) {
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
            synchronized (Collector.this) {
                SystemState newState;
                switch (mSystemState) {
                case COLLECTING:
                    newState = handleCollecting();
                    break;
                case BACKING:
                    newState = handleBacking();
                    break;
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
            }
        }

        @Override
        public void onStop(double timestamp){
            stop();
        }

    };

    private SystemState handleHolding() {
        return defaultStateTransfer();
    }
    
    private SystemState handleCollecting() {
        return defaultStateTransfer();
    }

    private SystemState handleBacking() {
        return defaultStateTransfer();
    }

    public synchronized void setWantedState(WantedState state) {
        if (state != mWantedState) {
            mSubsystemManager.scheduleMe(mListIndex, 1, false);
            System.out.println("waking " + sClassName);
        }

        mWantedState = state;
    }

    private SystemState defaultStateTransfer(){
        switch(mWantedState){
            case COLLECT:
                return SystemState.COLLECTING;
            case BACK:
                return SystemState.BACKING;
            default:
                return SystemState.HOLDING;
        }
    }

    @Override
    public void readPeriodicInputs() {
        double now       = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart   = now;
    }

    @Override
    public void writePeriodicOutputs() {
        mFXCollector.set(ControlMode.PercentOutput, mPeriodicIO.collectorDemand);
        if(mSolenoidState != mPeriodicIO.solenoidDemand){
            mSolenoidState = mPeriodicIO.solenoidDemand;
        }
        mSolenoid.set(mPeriodicIO.solenoidDemand.get());
    }


    @Override
    public void stop() {
        mFXCollector.set(ControlMode.PercentOutput, 0.0);
        mSolenoid.set(SolenoidState.RETRACT.get());   

        mPeriodicIO.collectorDemand = 0.0;
        mSolenoidState = SolenoidState.RETRACT;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    @Override
    public int whenRunAgain () {
        if (mStateChanged && mPeriodicIO.schedDeltaDesired == 0){
            return 1; // one more loop before going to sleep
        }

        return mPeriodicIO.schedDeltaDesired;
    }

    @Override
    public String getLogHeaders() {
        // TODO Auto-generated method stub
        return "Collector";
    }

    @Override
    public String getLogValues(boolean telemetry) {
        // TODO Auto-generated method stub
        return "Collector.Values";
    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub
        
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
        private double collectorDemand;
        private SolenoidState solenoidDemand;
    }

}
