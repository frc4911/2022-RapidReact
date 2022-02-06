package frc.robot.subsystems;

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

public class Climber extends Subsystem{

    //Hardware
    private final TalonFX mFXLeftClimber, mFXRightClimber;
    private final Solenoid mSolenoid;

    //Subsystem Constants

    //Subsystem States
    public enum SystemState {
        RESTING,
        EXTENDING,
        RETRACTING,
        LEVELING
    }

    public enum WantedState {
        REST,
        EXTEND,
        RETRACT,
        LEVEL
    }

    private SystemState mSystemState;
    private WantedState mWantedState;
    private boolean mStateChanged;
    private PeriodicIO mPeriodicIO;

    //Other
    private SubsystemManager mSubsystemManager;
    private int              mListIndex;
    
    //Subsystem Creation
    private static String sClassName;
    private static int sInstanceCount;
    private static Climber sInstance = null;
    public static Climber getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new Climber(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+" getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Climber(String caller){
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mFXLeftClimber = TalonFXFactory.createDefaultTalon(Ports.LEFT_CLIMBER);
        mFXRightClimber = TalonFXFactory.createDefaultTalon(Ports.RIGHT_CLIMBER);
        mSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.CLIMBER_STAGE);
        mSubsystemManager = SubsystemManager.getInstance(sClassName);
        configMotors();
    }

    private void configMotors(){

        mFXLeftClimber.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mFXLeftClimber.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mFXRightClimber.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mFXRightClimber.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mFXLeftClimber.setInverted(false);
        mFXRightClimber.setInverted(false);
        
        mFXLeftClimber.setNeutralMode(NeutralMode.Coast);
        mFXRightClimber.setNeutralMode(NeutralMode.Coast);

    }

    private Loop mLoop = new Loop() {
        
        @Override
        public void onStart(Phase phase){
            synchronized (Climber.this) {
                mSystemState = SystemState.RESTING;
                mWantedState = WantedState.REST;
                mStateChanged = true;
                System.out.println(sClassName + " state " + mSystemState);
                // this subsystem is "on demand" so
                mPeriodicIO.schedDeltaDesired = 0;
                stop(); // put into a known state
            }
        }

        @Override
        public void onLoop(double timestamp){
            synchronized (Climber.this) {
                SystemState newState;
                switch (mSystemState) {
                case EXTENDING:
                    newState = handleLoading();
                    break;
                case RETRACTING:
                    newState = handleBacking();
                    break;
                case LEVELING:
                    newState = handleFeeding();
                    break;
                case RESTING:
                default:
                    newState = handleResting();
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

    private SystemState handleResting() {
        return defaultStateTransfer();
    }
    
    private SystemState handleLoading() {
        return defaultStateTransfer();
    }

    private SystemState handleBacking() {
        return defaultStateTransfer();
    }

    private SystemState handleFeeding() {
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
            case EXTEND:
                return SystemState.EXTENDING;
            case RETRACT:
                return SystemState.RETRACTING;
            case LEVEL:
                return SystemState.LEVELING;
            case REST:
            default:
                return SystemState.RESTING;
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
        if (mStateChanged && mPeriodicIO.schedDeltaDesired == 0){
            return 1; // one more loop before going to sleep
        }

        return mPeriodicIO.schedDeltaDesired;
    }

    @Override
    public String getLogHeaders() {
        // TODO Auto-generated method stub
        return "Climber";
    }

    @Override
    public String getLogValues(boolean telemetry) {
        // TODO Auto-generated method stub
        return "Climber.Values";
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
    }

}
