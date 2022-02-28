package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.constants.Ports;
import libraries.cheesylib.drivers.TalonFXFactory;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.subsystems.SubsystemManager;
import libraries.cheesylib.util.LatchedBoolean;

public class Climber extends Subsystem{

    //Hardware
    private final TalonFX mFXLeftClimber, mFXRightClimber;
    private final Solenoid mSlapSticks;

    //Subsystem Constants
    private final double kClimberCurrentLimit = 60;

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
        CLIMBING,
        HOMING
    }

    public enum WantedState {
        HOLD,
        CLIMB,
        HOME
    }

    private SystemState   mSystemState;
    private WantedState   mWantedState;
    private boolean       mStateChanged;
    private PeriodicIO    mPeriodicIO = new PeriodicIO();
    private LatchedBoolean mSystemStateChange = new LatchedBoolean();
    private SolenoidState mSolenoidState;
    private int           mDefaultSchedDelta = 20;

    // Climber homing state variables
    // Homing is done by sending the Climber to a negative position
    // While watching for the climber encoder to stop changing for a sufficient amount of time
    private final double climberMovementThreshhold = 5; // encoder movements below this threshhold are considered stopped
    private final double climberNonMovementDuration = .25; // reading below threshhold encoder reads for this long is considered stopped
    private final double climberHomingDemand = -2 * 100000; // a number negative enough to drive past 0 regardless of where started
    private boolean climberHomed = true; // global flag
    private double climberNonMovementTimeout; // timestamp of when low readings are sufficient
    private WantedState wantedStateAfterHoming = WantedState.HOLD; // state to transition to after homed

    //Other
    private SubsystemManager mSubsystemManager;
    
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
        
        mFXLeftClimber.setNeutralMode(NeutralMode.Brake);
        mFXRightClimber.setNeutralMode(NeutralMode.Brake);

        mFXLeftClimber.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kClimberCurrentLimit, kClimberCurrentLimit, 0));
        mFXRightClimber.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kClimberCurrentLimit, kClimberCurrentLimit, 0));

    }

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
            do{
                SystemState newState;
                switch (mSystemState) {
                case CLIMBING:
                    newState = handleClimbing();
                    break;
                case HOMING:
                    newState = handleHoming();
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
            if (climberHomed){
                mPeriodicIO.climberDemand = 0.0;
                // mPeriodicIO.slappyDemand = SolenoidState.RETRACT;
                mPeriodicIO.schedDeltaDesired = 0;
            } else {
                mWantedState = WantedState.HOME;
                wantedStateAfterHoming = WantedState.HOLD;
            }
        }

        return defaultStateTransfer();
    }
    
    private SystemState handleClimbing() {
        if (mStateChanged){
            if(climberHomed) {
                mPeriodicIO.schedDeltaDesired = mDefaultSchedDelta; // stay awake
            } else {
                mWantedState = WantedState.HOME;
                wantedStateAfterHoming = WantedState.CLIMB;
            }
        }

        return defaultStateTransfer();
    }

    private SystemState handleHoming() {
        double now = Timer.getFPGATimestamp();
        if(mStateChanged){
            climberHomed = false;
            climberNonMovementTimeout = now+climberNonMovementDuration;
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
        }

        double distance = Math.abs(mPeriodicIO.climberPosition - mPeriodicIO.lastClimberPosition);
        if (distance > climberMovementThreshhold){
            climberNonMovementTimeout = now+climberNonMovementDuration;
        }

        if (now > climberNonMovementTimeout){
            mFXRightClimber.setSelectedSensorPosition(0);
            mFXLeftClimber.setSelectedSensorPosition(0);

            climberHomed = true;
            mWantedState = wantedStateAfterHoming;
        }

        return defaultStateTransfer();
    }

    private SystemState defaultStateTransfer(){
        switch(mWantedState){
            case CLIMB:
                return SystemState.CLIMBING;
            case HOME:
                return SystemState.HOMING;
            case HOLD:
            default:
                return SystemState.HOLDING;
        }
    }

    public void setClimbSpeed(double speed){
        mPeriodicIO.climberDemand = speed;
        if(speed == 0.0){
            mWantedState = WantedState.HOLD;
        } else {
            mWantedState = WantedState.CLIMB;
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
        mFXLeftClimber.set(ControlMode.PercentOutput, 0);
        mFXRightClimber.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public int whenRunAgain () {
        //return mPeriodicIO.schedDeltaDesired;
        return 20;
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
        SmartDashboard.putNumber("Left Climb Current", mFXLeftClimber.getStatorCurrent());        
        SmartDashboard.putNumber("Right Climb Current", mFXRightClimber.getStatorCurrent());        
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
        private double climberPosition;

        //Outputs
        private double climberDemand;
        private SolenoidState slappyDemand;

        //Other
        private double lastClimberPosition;

    }

}