package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import libraries.cheesylib.loops.ILooper;
import libraries.cheesylib.loops.Loop;
import libraries.cheesylib.subsystems.Subsystem;

public class Superstructure extends Subsystem{
    
    //Subsystem Instances
    private Swerve    mSwerve;

    //Superstructure States
    public enum SystemState{
        HOLDING,
        COLLECTING,
        CLEARING,
        AUTO_SHOOTING,
        MANUAL_SHOOTING,
        AUTO_CLIMBING,
        MANUAL_CLIMBING
    }
    
    public enum WantedState{
        HOLD,
        COLLECT,
        CLEAR,
        AUTO_SHOOT,
        MANUAL_SHOOT,
        AUTO_CLIMB,
        MANUAL_CLIMB
    }

    private SystemState mSystemState;
    private WantedState mWantedState;
    private boolean     mStateChanged;

    //Logging
    private int schedDeltaDesired;
    public  double schedDeltaActual;
    public  double schedDuration;
    private double lastSchedStart;

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
    }

    // Looping methods for subsystem
    private Loop mLoop = new Loop(){
        
        @Override
        public void onStart(Phase phase){
            synchronized(Superstructure.this) {
                mSystemState = SystemState.HOLDING;
                mWantedState = WantedState.HOLD;
                mStateChanged = true;
                System.out.println(sClassName + " state " + mSystemState);
                switch (phase) {
                    case DISABLED:
                        schedDeltaDesired = 0; // goto sleep
                        break;
                    default:
                        schedDeltaDesired = 100;
                        break;
                }
                stop();
            }
        }

        @Override
        public void onLoop(double timestamp){
            synchronized(Superstructure.this) {
                SystemState newState;
                switch(mSystemState) {
                    case COLLECTING:
                        newState = handleCollecting();
                        break;
                    case CLEARING:
                        newState = handleClearing();
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
                    case MANUAL_CLIMBING:
                        newState = handleManualClimbing();
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
            }
        }

        @Override
        public void onStop(double timestamp){
            stop();
        }

    };

    // Handling methods
    private SystemState handleHolding() {
        return defaultStateTransfer();
    }

    private SystemState handleCollecting() {
        return collectingStateTransfer();
    }

    private SystemState handleClearing() {
        return clearingStateTransfer();
    }

    private SystemState handleAutoShooting() {
        return shootingStateTransfer();
    }

    private SystemState handleManualShooting() {
        return shootingStateTransfer();
    }

    private SystemState handleAutoClimbing() {
        return climbingStateTransfer();
    }

    private SystemState handleManualClimbing() {
        return climbingStateTransfer();
    }    

    //State Transfers
    //Executes subsystem actions before switching to another state

    private SystemState collectingStateTransfer() {
        return defaultStateTransfer();
    }

    private SystemState clearingStateTransfer() {
        return defaultStateTransfer();
    }

    private SystemState shootingStateTransfer() {
        return defaultStateTransfer();
    }

    private SystemState climbingStateTransfer() {
        return defaultStateTransfer();
    }

    private SystemState defaultStateTransfer() {
        switch(mWantedState){
            case COLLECT:
                return SystemState.COLLECTING;
            case CLEAR:
                return SystemState.CLEARING;
            case AUTO_SHOOT:
                return SystemState.AUTO_SHOOTING;
            case MANUAL_SHOOT:
                return SystemState.MANUAL_SHOOTING;
            case AUTO_CLIMB:
                return SystemState.AUTO_CLIMBING;
            case MANUAL_CLIMB:
                return SystemState.AUTO_SHOOTING;
            case HOLD:
            default:
                return SystemState.HOLDING;
        }
    }

    public synchronized void setWantedState(WantedState state) {
        if (mWantedState != state){
            System.out.println(state);
            schedDeltaDesired = 2;
        }
        mWantedState = state;
    }

    @Override
    public void writePeriodicOutputs() {
    }

    @Override
    public int whenRunAgain () {
        if (mStateChanged && schedDeltaDesired == 0){
            return 1; // one more loop before going to sleep
        }
        return schedDeltaDesired;
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void readPeriodicInputs() {
        double now = Timer.getFPGATimestamp();
        schedDeltaActual = now - lastSchedStart;
        lastSchedStart = now;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(mLoop);
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

}
