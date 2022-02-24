package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final Solenoid mSlapSticks;

    //Subsystem Constants

    //Subsystem States
    public enum SolenoidState {
        EXTEND(true), // Need to test
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
        CLIMBING
    }

    public enum WantedState {
        HOLD,
        CLIMB
    }

    private SystemState   mSystemState;
    private WantedState   mWantedState;
    private boolean       mStateChanged;
    private PeriodicIO    mPeriodicIO = new PeriodicIO();
    private SolenoidState mSolenoidState;
    private int           mDefaultSchedDelta = 20;

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
        mSlapSticks = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.CLIMBER_STAGE);
        mSubsystemManager = SubsystemManager.getInstance(sClassName);
        configMotors();
    }

    private void configMotors(){

        //Current limit motors
        
        mFXLeftClimber.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mFXLeftClimber.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mFXRightClimber.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mFXRightClimber.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mFXLeftClimber.setInverted(false);
        mFXRightClimber.setInverted(true);
        
        mFXLeftClimber.setNeutralMode(NeutralMode.Coast);
        mFXRightClimber.setNeutralMode(NeutralMode.Coast);

    }

    private Loop mLoop = new Loop() {
        
        @Override
        public void onStart(Phase phase){
            synchronized (Climber.this) {
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
            synchronized (Climber.this) {
                SystemState newState;
                switch (mSystemState) {
                case CLIMBING:
                    newState = handleClimbing();
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
            }
        }

        @Override
        public void onStop(double timestamp){
            stop();
        }

    };

    public synchronized void setWantedState(WantedState state) {
        if (state != mWantedState) {
            mSubsystemManager.scheduleMe(mListIndex, 1, false);
            System.out.println("waking " + sClassName);
        }

        mWantedState = state;
    }

    private SystemState handleHolding() {
        if(mStateChanged){
            System.out.println("climber demand 0");
            mPeriodicIO.climberDemand = 0.0;
            mPeriodicIO.slappyDemand = SolenoidState.RETRACT;
            mPeriodicIO.schedDeltaDesired = 0;
        }

        return defaultStateTransfer();
    }
    
    private SystemState handleClimbing() {
        if(mStateChanged){
            mPeriodicIO.slappyDemand = SolenoidState.RETRACT;
            mPeriodicIO.schedDeltaDesired = mDefaultSchedDelta; // stay awake
        }

        return defaultStateTransfer();
    }

    private SystemState defaultStateTransfer(){
        switch(mWantedState){
            case CLIMB:
                return SystemState.CLIMBING;
            case HOLD:
            default:
                return SystemState.HOLDING;
        }
    }

    public void setClimbSpeed(double speed){
        mPeriodicIO.climberDemand = speed;
        if(speed == 0.0){
            setWantedState(WantedState.HOLD);
        } else {
            setWantedState(WantedState.CLIMB);
        }
    }

    public void setSlappyStickState(boolean state){
        if(state){
            mPeriodicIO.slappyDemand = SolenoidState.EXTEND;
        } else {
            mPeriodicIO.slappyDemand = SolenoidState.RETRACT;
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
        // TODO: Change to Position control, moving elevator to max height or lowest height with a controlled velocity
        // Will still need to monitor encoder values, but it should prevent elevator from going too high
        if(mSystemState == SystemState.CLIMBING){
            mFXLeftClimber.set(ControlMode.PercentOutput, mPeriodicIO.climberDemand);
            mFXRightClimber.set(ControlMode.PercentOutput, mPeriodicIO.climberDemand);
        } else {
            // TODO Use pid to keep climber in place?
            mFXLeftClimber.set(ControlMode.PercentOutput, mPeriodicIO.climberDemand);
            mFXRightClimber.set(ControlMode.PercentOutput, mPeriodicIO.climberDemand);
        }

        if(mSolenoidState != mPeriodicIO.slappyDemand){
            mSolenoidState = mPeriodicIO.slappyDemand;
            mSlapSticks.set(mPeriodicIO.slappyDemand.get());
        }
    }


    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        mListIndex = enabledLooper.register(mLoop);
    }

    @Override
    public int whenRunAgain () {
        if (mStateChanged && mPeriodicIO.schedDeltaDesired == 0){
            return 1; // one more loop before going to sleep
        }

        return 20; //mPeriodicIO.schedDeltaDesired;
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
        SmartDashboard.putNumber("Left Climber Encoder", mFXLeftClimber.getSelectedSensorPosition());        
    }

    public static class PeriodicIO{
        //Logging
        @SuppressWarnings("unused")
        private final int mDefaultSchedDelta = 100; // axis updated every 100 msec
        private int    schedDeltaDesired;
        public  double schedDeltaActual;
        public  double schedDuration;
        private double lastSchedStart;

        private double climberDemand;
        private SolenoidState slappyDemand;
    }

}