package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import libraries.cheesylib.loops.ILooper;
import libraries.cheesylib.loops.Loop;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.subsystems.SubsystemManager;

public class Shooter extends Subsystem{

    //Hardware

    //Subsystem Constants

    //Subsystem States
    public enum SystemState {
        RESTING,
        SHOOTING,
    }

    public enum WantedState {
        REST,
        SHOOT,
    }

    private SystemState mSystemState;
    private WantedState mWantedState;
    private boolean mStateChanged;

    //Logging
    @SuppressWarnings("unused")
    private final int mDefaultSchedDelta = 100; // axis updated every 100 msec
    private int    schedDeltaDesired;
    public  double schedDeltaActual;
    public  double schedDuration;
    private double lastSchedStart;

    //Other
    private SubsystemManager mSubsystemManager;
    private int              mListIndex;
    
    //Subsystem Creation
    private static String sClassName;
    private static int sInstanceCount;
    private static Shooter sInstance = null;
    public static Shooter getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new Shooter(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+" getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Shooter(String caller){

    }

    private Loop mLoop = new Loop() {
        
        @Override
        public void onStart(Phase phase){
            synchronized (Shooter.this) {
                mSystemState = SystemState.RESTING;
                mWantedState = WantedState.REST;
                mStateChanged = true;
                System.out.println(sClassName + " state " + mSystemState);
                // this subsystem is "on demand" so
                schedDeltaDesired = 0;
                stop(); // put into a known state
            }
        }

        @Override
        public void onLoop(double timestamp){
            synchronized (Shooter.this) {
                SystemState newState;
                switch (mSystemState) {
                case SHOOTING:
                    newState = handleCollecting();
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

    public synchronized void setWantedState(WantedState state) {
        if (state != mWantedState) {
            mSubsystemManager.scheduleMe(mListIndex, 1, false);
            System.out.println("waking " + sClassName);
        }

        mWantedState = state;
    }

    private SystemState defaultStateTransfer(){
        switch(mWantedState){
            case SHOOT:
                return SystemState.SHOOTING;
            default:
                return SystemState.RESTING;
        }
    }

    @Override
    public void readPeriodicInputs() {
        double now       = Timer.getFPGATimestamp();
        schedDeltaActual = now - lastSchedStart;
        lastSchedStart   = now;
    }

    @Override
    public void writePeriodicOutputs() {

    }


    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    @Override
    public int whenRunAgain () {
        if (mStateChanged && schedDeltaDesired == 0){
            return 1; // one more loop before going to sleep
        }

        return schedDeltaDesired;
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

}
