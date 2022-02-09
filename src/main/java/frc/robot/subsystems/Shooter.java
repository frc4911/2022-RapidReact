package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Ports;
import libraries.cheesylib.drivers.TalonFXFactory;
import libraries.cheesylib.loops.ILooper;
import libraries.cheesylib.loops.Loop;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.subsystems.SubsystemManager;

public class Shooter extends Subsystem{

    //Hardware
    private final TalonFX mFXLeftFlyWheel, mFXRightFlyWheel;
    private final TalonFX mFXShooterHood; //TO-DO: Decide if hood adjustment will be controlled by SHOOTING state or by superstructure

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
    private PeriodicIO mPeriodicIO;

    private final double kSpeedTolerance = 250.0;
    private double mDistance;
    private double mHoldSpeed;

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
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mFXLeftFlyWheel = TalonFXFactory.createDefaultTalon(Ports.LEFT_FLYWHEEL);
        mFXRightFlyWheel = TalonFXFactory.createDefaultTalon(Ports.RIGHT_FLYWHEEL);
        mFXShooterHood = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_HOOD);
        mSubsystemManager = SubsystemManager.getInstance(sClassName);
        configMotors();
    }

    private void configMotors(){
        mFXLeftFlyWheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mFXRightFlyWheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        // only one encoder is needed
        mFXLeftFlyWheel.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, Constants.kLongCANTimeoutMs);
    
        mFXLeftFlyWheel.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mFXRightFlyWheel.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mFXLeftFlyWheel.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mFXRightFlyWheel.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mFXLeftFlyWheel.setInverted(true);
        mFXRightFlyWheel.setInverted(false);

        mFXLeftFlyWheel.setSensorPhase(true);
        mFXRightFlyWheel.setSensorPhase(false);

        mFXLeftFlyWheel.setNeutralMode(NeutralMode.Coast);
        mFXRightFlyWheel.setNeutralMode(NeutralMode.Coast);   
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
                mPeriodicIO.schedDeltaDesired = 0;
                stop(); // put into a known state
            }
        }

        @Override
        public void onLoop(double timestamp){
            synchronized (Shooter.this) {
                SystemState newState;
                switch (mSystemState) {
                case SHOOTING:
                    newState = handleShooting();
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

    public synchronized void setWantedState(WantedState state) {
        if (state != mWantedState) {
            mSubsystemManager.scheduleMe(mListIndex, 1, false);
            System.out.println("waking " + sClassName);
        }

        mWantedState = state;
    }

    private SystemState handleResting() {
        mPeriodicIO.flywheelPercentDemand = mHoldSpeed;

        return defaultStateTransfer();
    }
    
    private SystemState handleShooting() {
        //TO-DO: change mPeriodicIO.flywheelVelocityDemand based on distance

        return defaultStateTransfer();
    }

    private SystemState defaultStateTransfer(){
        switch(mWantedState){
            case SHOOT:
                return SystemState.SHOOTING;
            case REST:
            default:
                return SystemState.RESTING;
        }
    }

    // TO-DO: Add needed constants
    public synchronized boolean readyToShoot() {
        return mSystemState == SystemState.SHOOTING && mPeriodicIO.reachedDesiredSpeed && mPeriodicIO.flywheelRPM > kSpeedTolerance;
    }

    // TO-DO: Add kShootDistance constants
    public synchronized void setShootDistance(double distance) {
        if (mSystemState != SystemState.SHOOTING) {
            mWantedState = WantedState.SHOOT;
        }
        if (mSystemState != SystemState.SHOOTING || distance != mDistance) {
            mSubsystemManager.scheduleMe(mListIndex, 1, false);
            System.out.println("waking " + sClassName);
        }

        //mDistance = Math.max(kMinShootDistance, Math.min(distance, 35.0));
    }

    public synchronized void setHoldSpeed(double speed) {
        if (speed != mHoldSpeed && mSystemState == SystemState.RESTING) {
            mSubsystemManager.scheduleMe(mListIndex, 1, false);
            System.out.println("waking " + sClassName);
        }

        mHoldSpeed = speed;
    }

    private double ticksPer100MsToRPM(double speed) {
        return speed / 1365.0 * 1000.0 / 100.0 * 60.0; //TO-DO: 1365.0 conversion rate needs to be changed
    }

    private double rpmToTicksPer100Ms(double speed) {
        return speed * 1365.0 / 1000.0 * 100.0 / 60.0; //TO-DO: 1365.0 conversion rate needs to be changed
    }

    // TO-DO: Calculate and add kShootDistance constants to ensure calculations are correct for proper rpm
    // private double getDistanceToVelocityRPM(double distance) {
    //     return mShootRate * Math.abs(distance - kMidShootDistance) + kMinShootSpeed;
    // }

    @Override
    public void readPeriodicInputs() {
        double now       = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart   = now;
    }

    @Override
    public void writePeriodicOutputs() {
        if(mSystemState == SystemState.SHOOTING){
            mFXLeftFlyWheel.set(ControlMode.Velocity, mPeriodicIO.flywheelVelocityDemand);
            mFXRightFlyWheel.set(ControlMode.Velocity, mPeriodicIO.flywheelVelocityDemand);
        } else {
            mFXLeftFlyWheel.set(ControlMode.PercentOutput, mPeriodicIO.flywheelPercentDemand);
            mFXRightFlyWheel.set(ControlMode.PercentOutput, mPeriodicIO.flywheelPercentDemand);
        }
    }


    @Override
    public void stop() {
        mFXLeftFlyWheel.set(ControlMode.PercentOutput, 0.0);
        mFXRightFlyWheel.set(ControlMode.PercentOutput, 0.0);

        mPeriodicIO.flywheelVelocityDemand = 0.0;
        mPeriodicIO.flywheelPercentDemand = 0.0;
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
        private boolean reachedDesiredSpeed;
        private double flywheelRPM;

        //Outputs
        private double flywheelVelocityDemand;
        private double flywheelPercentDemand;
    }

}
