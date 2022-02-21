package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final TalonFX mFXShooterHood; //TODO: Decide if hood adjustment will be controlled by SHOOTING state or by superstructure

    //Subsystem States
    public enum SystemState {
        HOLDING,
        SHOOTING,
    }

    public enum WantedState {
        HOLD,
        SHOOT,
    }

    private SystemState mSystemState;
    private WantedState mWantedState;
    private boolean mStateChanged;
    private PeriodicIO mPeriodicIO = new PeriodicIO();

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

        // If flywheel makes clicking sound, test config line for both motors
        mFXLeftFlyWheel.setControlFramePeriod(ControlFrame.Control_3_General,18);
        mFXRightFlyWheel.setControlFramePeriod(ControlFrame.Control_3_General,18);
    
        mFXLeftFlyWheel.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mFXRightFlyWheel.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mFXLeftFlyWheel.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mFXRightFlyWheel.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mFXLeftFlyWheel.setInverted(true); // Need to test both
        mFXRightFlyWheel.setInverted(false);

        // mFXLeftFlyWheel.setSensorPhase(true);
        // mFXRightFlyWheel.setSensorPhase(false);

        mFXLeftFlyWheel.setNeutralMode(NeutralMode.Coast);
        mFXRightFlyWheel.setNeutralMode(NeutralMode.Coast);   

        // mFXLeftFlyWheel.configVoltageCompSaturation(0.0, Constants.kLongCANTimeoutMs);
        // mFXLeftFlyWheel.enableVoltageCompensation(mConstants.kSteerMotorEnableVoltageCompensation);
        // mFXLeftFlyWheel.configAllowableClosedloopError(0, mConstants.kSteerMotorClosedLoopAllowableError, Constants.kLongCANTimeoutMs);

        // parameters are enable, current limit after triggering, trigger limit, time allowed to exceed trigger limit before triggering
        mFXLeftFlyWheel.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10, 10,0));
        mFXRightFlyWheel.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10, 10,0));

        mFXLeftFlyWheel.follow(mFXRightFlyWheel);

        mFXRightFlyWheel.selectProfileSlot(0, 0);

        mFXRightFlyWheel.config_kP(0, .1, Constants.kLongCANTimeoutMs);
        mFXRightFlyWheel.config_kI(0, 0, Constants.kLongCANTimeoutMs);
        mFXRightFlyWheel.config_kD(0, 0, Constants.kLongCANTimeoutMs);
        mFXRightFlyWheel.config_kF(0, .2, Constants.kLongCANTimeoutMs);
        mFXRightFlyWheel.config_IntegralZone(0, 0, Constants.kLongCANTimeoutMs);
        mFXRightFlyWheel.configClosedloopRamp(.5,Constants.kLongCANTimeoutMs);
        mFXRightFlyWheel.configAllowableClosedloopError(0, 10, Constants.kLongCANTimeoutMs);

        // SmartDashboard.getNumber("")
    }

    private Loop mLoop = new Loop() {

        @Override
        public void onStart(Phase phase){
            synchronized (Shooter.this) {
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
            synchronized (Shooter.this) {
                SystemState newState;
                switch (mSystemState) {
                case SHOOTING:
                    newState = ();
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
        mPeriodicIO.flywheelPercentDemand = mHoldSpeed;

        return defaultStateTransfer();
    }
    
    private SystemState handleShooting() {
        mPeriodicIO.flywheelVelocityDemand = rpmToTicksPer100Ms(getDistanceToVelocityRPM(mDistance));

        return defaultStateTransfer();
    }

    private SystemState defaultStateTransfer(){
        switch(mWantedState){
            case SHOOT:
                return SystemState.SHOOTING;
            case HOLD:
            default:
                return SystemState.HOLDING;
        }
    }

    public synchronized boolean readyToShoot() {
        return mPeriodicIO.reachedDesiredSpeed;
    }

    public synchronized void setShootDistance(double distance) {
        if (mSystemState != SystemState.SHOOTING) {
            mWantedState = WantedState.SHOOT;
        }
        if (mSystemState != SystemState.SHOOTING || distance != mDistance) {
            mSubsystemManager.scheduleMe(mListIndex, 1, false);
            System.out.println("waking " + sClassName);
        }

        mDistance = Math.max(kMinShootDistance, Math.min(distance, 35.0));
    }

    public synchronized void setHoldSpeed(double speed) {
        if (speed != mHoldSpeed && mSystemState == SystemState.HOLDING) {
            mSubsystemManager.scheduleMe(mListIndex, 1, false);
            System.out.println("waking " + sClassName);
        }

        mHoldSpeed = speed;
    }

    // TODO: Figure out what units parameter will be in - potentially an angle?
    // Convert angle (or other unit) to falcon ticks
    // Find out gear ratio for hood - how many falcon rotations per angle are needed
    public void setFlywheelHood(double position){
        mPeriodicIO.hoodDemand = degreesToTicks(position);
    }

    // TODO: Determine if midshoot distance is needed
    private double getDistanceToVelocityRPM(double distance) {
        return kShootRate * Math.abs(distance - kMidShootDistance) + kMinShootSpeed;
    }

    // Motor to flywheel pulley ratio is 60/16 (15/4 reduced)
    // Multiply by 2048 (falcon ticks per revolution) to get 7567.5
    private double ticksPer100MsToRPM(double speed) {
        return speed / 7567.5 * 1000.0 / 100.0 * 60.0;
    }

    private double rpmToTicksPer100Ms(double speed) {
        return speed * 7567.5 / 1000.0 * 100.0 / 60.0;
    }

    // TODO: Find shooter hood conversion between angle and motor ticks
    // Motor to hood angle gear ratio is
    // Multiply gear ratios by 2048 to get 
    private double degreesToTicks(double degrees) {
        return degrees;
    }

    @Override
    public void readPeriodicInputs() {
        double now       = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart   = now;
        
        mPeriodicIO.flywheelRPM = ticksPer100MsToRPM(mFXLeftFlyWheel.getSelectedSensorVelocity());
        mPeriodicIO.reachedDesiredSpeed = mPeriodicIO.flywheelRPM >= ticksPer100MsToRPM(mPeriodicIO.flywheelVelocityDemand) - kSpeedTolerance;
    }

    // TODO: Double check if the shooter motors will run using velocity control
    // Motors did not function from velocity when testing on the test board
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
        private double hoodPosition;

        //Outputs
        private double flywheelVelocityDemand;
        private double flywheelPercentDemand;
        private double hoodDemand;
    }

}