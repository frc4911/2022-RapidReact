package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.constants.Ports;
import libraries.cheesylib.drivers.TalonFXFactory;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.subsystems.SubsystemManager;
import libraries.cheesylib.util.InterpolatingDouble;
import libraries.cheesylib.util.InterpolatingTreeMap;
import libraries.cheesylib.util.LatchedBoolean;
import libraries.cyberlib.control.FramePeriodSwitch;

public class Shooter extends Subsystem {

    // Hardware
    private final TalonFX mFXFlyLeft, mFXFlyRight;
    private final TalonFX mFXHood;

    // Subsystem Constants
    private final int mSchedActive = 20;
    private final int mSchedDormant = 100;
    private final double kMinShootDistance = 0; // Fender shot is 0
    private final double kMaxShootDistance = 144; // inches

    // Configuration Constants
    private final double kFlywheelKp = 0.13;
    private final double kFlywheelKi = 0.0;//001;
    private final double kFlywheelKd = 2.0;
    private final double kFlywheelKf = 0.05;
    private final double kClosedRamp = 1;
    private final double kClosedError = 25.0;
    private final double kFlyIntegralZone = 700;

    private final double kFlywheelCurrentLimit = 40;
    private final double kHoodCurrentLimitLow = 5;
    private final double kHoodCurrentLimitHigh = 15;
    private final double kHoodPositionAtFender = 8000;
    // Subsystem States
    public enum SystemState {
        ASSESSING,
        DISABLING,
        HOMINGHOOD,
        HOLDING,
        SHOOTING,
        MANUALCONTROLLING
    }

    public enum WantedState {
        ASSESS,
        DISABLE,
        HOMEHOOD,
        HOLD,
        SHOOT,
        MANUALCONTROL
    }

    private SystemState mSystemState;
    private WantedState mWantedState;
    private boolean mStateChanged;
    private LatchedBoolean mLB_SystemStateChange = new LatchedBoolean();
    private PeriodicIO mPeriodicIO = new PeriodicIO();

    // hood homing state variables
    // homing is done by sending the hood to a negative position
    // while watching for the hood encoder to stop changing for a sufficient amount
    // of time
    private boolean hoodHomed = true; // assume homed global flag
    private double kMaxHoodPosition = 28500;
    private final double hoodNonMovementThreshhold = 5; // encoder movements below this threshhold are considered
                                                        // stopped
    private final double hoodNonMovementDuration = .25; // reading below threshhold encoder reads for this long is
                                                        // considered stopped
    private final double hoodHomingDemand = -2 * kMaxHoodPosition; // a number negative enough to drive past 0
                                                                   // regardless of where started
    private double hoodNonMovementTimeout; // timestamp of when low readings are sufficient
    private double hoodEncoderOffset = 0; // used after homing to avoid resetting encoder position
    private final double kHoodHomingDemand = -.2;
    private AssessingState assessingState;
    private boolean assessingStateChange;
    private double assessingFlyLeftStartPosition;
    private double assessingFlyRightStartPosition;

    private boolean assessingComplete;
    private double assessingHoodStartPosition;
    private double assessingStopTime;
    private final double kAssessingTimeout = 250;
    
    InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shooterSpeedMap= new InterpolatingTreeMap<>();
    InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shooterHoodMap= new InterpolatingTreeMap<>();
    InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shooterLLTYDist= new InterpolatingTreeMap<>();

     private double mDistance = -1;
    private boolean turnOffFlywheel = false;

    // Other
    private SubsystemManager mSubsystemManager;
    // Subsystem Creation
    private static String sClassName;
    private static int sInstanceCount;
    private static Shooter sInstance = null;

    public static Shooter getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new Shooter(caller);
        } else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller) {
        System.out.println("(" + caller + ") " + " getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Shooter(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mFXFlyLeft = TalonFXFactory.createDefaultTalon(Ports.LEFT_FLYWHEEL, Constants.kCanivoreName);
        mFXFlyRight = TalonFXFactory.createDefaultTalon(Ports.RIGHT_FLYWHEEL, Constants.kCanivoreName);
        mFXHood = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_HOOD, Constants.kCanivoreName);
        mSubsystemManager = SubsystemManager.getInstance(sClassName);
        configMotors();
        buildInterpTree();

        // hoodEncoderOffset = mFXHood.getSelectedSensorPosition()+kHoodPositionAtFender-100;
        // System.out.println("initial hood home hoodEncoderOffset="+hoodEncoderOffset);
        mFXHood.setSelectedSensorPosition(8000);
        hoodEncoderOffset=0;
    }

    private void configMotors() {
        commonMotorConfig(mFXFlyRight, "Fly Right");
        commonMotorConfig(mFXFlyLeft, "Fly Left");
        commonMotorConfig(mFXHood, "Hood");

        FramePeriodSwitch.setInvertedVolatile(mFXFlyLeft);
        FramePeriodSwitch.setInvertedVolatile(mFXHood);

        FramePeriodSwitch.configStatorCurrentLimitPermanent(mFXFlyLeft, new StatorCurrentLimitConfiguration(true, kFlywheelCurrentLimit, kFlywheelCurrentLimit, 0));
        FramePeriodSwitch.configStatorCurrentLimitPermanent(mFXFlyRight, new StatorCurrentLimitConfiguration(true, kFlywheelCurrentLimit, kFlywheelCurrentLimit, 0));
        FramePeriodSwitch.configStatorCurrentLimitPermanent(mFXHood, new StatorCurrentLimitConfiguration(true, kHoodCurrentLimitHigh, kHoodCurrentLimitHigh, 0));

        // PID setting  are permanent but rerunning in case of motor change
        mFXFlyLeft.config_kP(0, kFlywheelKp, Constants.kLongCANTimeoutMs);
        mFXFlyLeft.config_kI(0, kFlywheelKi, Constants.kLongCANTimeoutMs);
        mFXFlyLeft.config_kD(0, kFlywheelKd, Constants.kLongCANTimeoutMs);
        mFXFlyLeft.config_kF(0, kFlywheelKf, Constants.kLongCANTimeoutMs);
        mFXFlyLeft.config_IntegralZone(0, kFlyIntegralZone, Constants.kLongCANTimeoutMs);
        mFXFlyLeft.configClosedloopRamp(kClosedRamp, Constants.kLongCANTimeoutMs);
        mFXFlyLeft.configAllowableClosedloopError(0, kClosedError, Constants.kLongCANTimeoutMs);

        mFXFlyRight.config_kP(0, kFlywheelKp, Constants.kLongCANTimeoutMs);
        mFXFlyRight.config_kI(0, kFlywheelKi, Constants.kLongCANTimeoutMs);
        mFXFlyRight.config_kD(0, kFlywheelKd, Constants.kLongCANTimeoutMs);
        mFXFlyRight.config_kF(0, kFlywheelKf, Constants.kLongCANTimeoutMs);
        mFXFlyRight.config_IntegralZone(0, kFlyIntegralZone, Constants.kLongCANTimeoutMs);
        mFXFlyRight.configClosedloopRamp(kClosedRamp, Constants.kLongCANTimeoutMs);
        mFXFlyRight.configAllowableClosedloopError(0, kClosedError, Constants.kLongCANTimeoutMs);

        mFXHood.config_kP(0, 0.7, Constants.kLongCANTimeoutMs); // May need to tune more? .7,.1,15
        mFXHood.config_kI(0, 0.04, Constants.kLongCANTimeoutMs);
        mFXHood.config_kD(0, 10.0, Constants.kLongCANTimeoutMs);
        mFXHood.config_kF(0, 0, Constants.kLongCANTimeoutMs);
        mFXHood.config_IntegralZone(0, 200, Constants.kLongCANTimeoutMs);
        mFXHood.configClosedloopRamp(0/* kClosedRamp */, Constants.kLongCANTimeoutMs);
        mFXHood.configAllowableClosedloopError(0, 15 /*kClosedError*/, Constants.kLongCANTimeoutMs);

        mFXHood.configMotionCruiseVelocity(10000); // ticks/100ms // measured 8800 ave ticks/second while traveling full path
        mFXHood.configMotionAcceleration(5400); // ticks/100mse/sec // 2500 ticks/(sec^2)
        mFXHood.configMotionSCurveStrength(0); // trapezoidal curve
    }

    private void commonMotorConfig(TalonFX motor, String motorName){
        System.out.println("configuring "+motorName+" motor");

        // The following commands are stored in nonVolatile ram in the motor
        // They are repeated on boot incase a motor needs to replaced quickly
        FramePeriodSwitch.configFactoryDefaultPermanent(motor);

        // the following commands are stored in nonVolatile ram but they are
        // no longer deemed necessary. Keeping around for a while in case they
        // need to be brought back
        // motor.configNeutralDeadband(.04, 100);
        // motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, 100);
        // motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, 100);

        // the following are volatile settings and must be run every power cycle
        FramePeriodSwitch.setFramePeriodsVolatile(motor); // set frame periods

    }

    private void buildInterpTree(){
        shooterHoodMap.put(new InterpolatingDouble(0.0),   new InterpolatingDouble(kHoodPositionAtFender));//8000.0
        shooterHoodMap.put(new InterpolatingDouble(24.0),  new InterpolatingDouble(12000.0 - 1000.0));
        shooterHoodMap.put(new InterpolatingDouble(48.0),  new InterpolatingDouble(16500.0 - 1000.0));
        shooterHoodMap.put(new InterpolatingDouble(72.0),  new InterpolatingDouble(22000.0 - 1000.0));
        shooterHoodMap.put(new InterpolatingDouble(96.0),  new InterpolatingDouble(28500.0 - 1000.0));
        shooterHoodMap.put(new InterpolatingDouble(120.0), new InterpolatingDouble(28500.0));
        shooterHoodMap.put(new InterpolatingDouble(144.0), new InterpolatingDouble(28500.0));

        shooterSpeedMap.put(new InterpolatingDouble(0.0),   new InterpolatingDouble(10900.0));
        shooterSpeedMap.put(new InterpolatingDouble(24.0),  new InterpolatingDouble(11300.0)); // 10900 for modified fender shot
        shooterSpeedMap.put(new InterpolatingDouble(48.0),  new InterpolatingDouble(11700.0)); // 10900 for modified fender shot 11200
        shooterSpeedMap.put(new InterpolatingDouble(72.0),  new InterpolatingDouble(12100.0)); //12300
        shooterSpeedMap.put(new InterpolatingDouble(96.0),  new InterpolatingDouble(12200.0)); //12750
        shooterSpeedMap.put(new InterpolatingDouble(120.0), new InterpolatingDouble(12800.0)); //13000
        shooterSpeedMap.put(new InterpolatingDouble(144.0), new InterpolatingDouble(14750.0)); //14750

        shooterLLTYDist.put(new InterpolatingDouble(2.7), new InterpolatingDouble(24.0));
        shooterLLTYDist.put(new InterpolatingDouble(-6.6), new InterpolatingDouble(48.0));
        shooterLLTYDist.put(new InterpolatingDouble(-13.3), new InterpolatingDouble(72.0));
        shooterLLTYDist.put(new InterpolatingDouble(-18.0), new InterpolatingDouble(96.0));
        shooterLLTYDist.put(new InterpolatingDouble(-22.4), new InterpolatingDouble(120.0));
        shooterLLTYDist.put(new InterpolatingDouble(-24.4), new InterpolatingDouble(144.0));

            }

    @Override
    public void onStart(Phase phase) {
        synchronized (Shooter.this) {
            mStateChanged = true;
            switch (phase) {
                case TEST:
                    mSystemState = SystemState.MANUALCONTROLLING;
                    mWantedState = WantedState.MANUALCONTROL;
                    break;
                case DISABLED:
                    mSystemState = SystemState.DISABLING;
                    mWantedState = WantedState.DISABLE;
                    break;
                case AUTONOMOUS:
                case TELEOP:
                    mSystemState = SystemState.HOLDING;
                    mWantedState = WantedState.HOLD;
                    setShootDistance(0);
                    break;
            }
            mPeriodicIO.schedDeltaDesired = mSchedActive;
            stop();
            mLB_SystemStateChange.update(false); // reset
            System.out.println(sClassName + " state " + mSystemState);
            FramePeriodSwitch.configStatorCurrentLimitPermanent(mFXHood, new StatorCurrentLimitConfiguration(true, kHoodCurrentLimitHigh, kHoodCurrentLimitHigh, 0));
        }
    }

    @Override
    public void onLoop(double timestamp) {
        synchronized (Shooter.this) {
            do {
                SystemState newState = null;
                switch (mSystemState) {
                    case DISABLING:
                        newState = handleDisabling();
                        break;
                    case MANUALCONTROLLING:
                        newState = handleManualControlling();
                        break;
                    case HOMINGHOOD:
                        newState = handleHomingHood();
                        break;
                    case SHOOTING:
                        newState = handleShooting();
                        break;
                    case HOLDING:
                        newState = handleHolding();
                        break;
                    case ASSESSING:
                        newState = handleAssessing();
                        break;
                    // default: leave commented so compiler identifies missing cases
                }

                if (newState != mSystemState) {
                    System.out.println(
                            sClassName + " state " + mSystemState + " to " + newState + " (" + timestamp + ")");
                    mSystemState = newState;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
                // this do/while does a second run through so state changes take place in one
                // onLoop call
            } while (mLB_SystemStateChange.update(mStateChanged));
        }
    }

    public boolean isHandlerComplete(WantedState wantedState){
        switch (wantedState){
            case ASSESS:
                return assessingComplete;
            default:
        }

        return false;
    }
    // this method should only be used by external subsystems.
    // if you want to change your own wantedState then simply set
    // it directly
    public synchronized void setWantedState(WantedState state, String who) {
        if (state != mWantedState) {
            mWantedState = state;
            mSubsystemManager.scheduleMe(mListIndex, 1, true);
            System.out.println(who + " is setting wanted state of " + sClassName + " to " + state);
        } else {
            System.out.println(who + " is setting wanted state of " + sClassName + " to " + state + " again!!!");
        }
    }

    public synchronized WantedState getWantedState() {
        return mWantedState;
    }

    public enum AssessingState {
        FLYLEFT,
        HOOD,
        FLYRIGHT
    }

    private SystemState handleAssessing() {
        double now = Timer.getFPGATimestamp();
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = mSchedActive;
            assessingComplete = false;
            assessingState = AssessingState.FLYLEFT;
            assessingStateChange = true;
        }

        switch (assessingState){
            case FLYLEFT:
            case FLYRIGHT:
            case HOOD:
                if (assessingStateChange){
                    assessingStopTime = now+kAssessingTimeout;
                    mPeriodicIO.flyControlMode = ControlMode.PercentOutput;
                    mPeriodicIO.flyDemand = .1;
                    mPeriodicIO.hoodControlMode = ControlMode.PercentOutput;
                    mPeriodicIO.hoodDemand = .1;
                    assessingFlyLeftStartPosition = mFXFlyLeft.getSelectedSensorPosition();
                    assessingFlyRightStartPosition = mFXFlyRight.getSelectedSensorPosition();
                    assessingHoodStartPosition = mPeriodicIO.hoodPosition;
                    assessingStateChange = false;
                }

                if (now>assessingStopTime){
                    mPeriodicIO.flyDemand = 0;
                    mPeriodicIO.hoodDemand = 0;
                    assessingStopTime += 20000; // increase so code does not enter here any more
                    assessingComplete = true;

                    if (mFXFlyLeft.getSelectedSensorPosition() > assessingFlyLeftStartPosition){
                        System.out.println("ASSESSING: Shooter Fly Left motor functioning");
                    }
                    else{
                        System.out.println("ASSESSING: Shooter Fly Left motor DID NOT DETECT MOVEMENT");
                    }
                    if (mFXFlyRight.getSelectedSensorPosition() > assessingFlyRightStartPosition){
                        System.out.println("ASSESSING: Shooter Fly Right motor functioning");
                    }
                    else{
                        System.out.println("ASSESSING: Shooter Fly Right motor DID NOT DETECT MOVEMENT");
                    }
                    if (mPeriodicIO.hoodPosition > assessingHoodStartPosition){
                        System.out.println("ASSESSING: Shooter Hood motor functioning");
                    }
                    else{
                        System.out.println("ASSESSING: Shooter Hood motor DID NOT DETECT MOVEMENT");
                    }
                }
        }

        if (mWantedState != WantedState.ASSESS){
            assessingComplete = false;
        }

        return defaultStateTransfer();
    }

    private SystemState handleDisabling() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = mSchedDormant;
            stop();
        }

        return defaultStateTransfer();
    }

    private SystemState handleHomingHood() {
        double now = Timer.getFPGATimestamp();
        if (mStateChanged) {
            System.out.println("Start homing hood");
            FramePeriodSwitch.configStatorCurrentLimitPermanent(mFXHood,new StatorCurrentLimitConfiguration(true, kHoodCurrentLimitLow, kHoodCurrentLimitLow, 0));    
            hoodHomed = false;
            hoodEncoderOffset = 0;
            hoodNonMovementTimeout = now + hoodNonMovementDuration;
            mPeriodicIO.schedDeltaDesired = mSchedActive;
            mPeriodicIO.hoodControlMode = ControlMode.PercentOutput;
            mPeriodicIO.hoodDemand = kHoodHomingDemand;
        }

        if(!hoodHomed){
            double distance = Math.abs(mPeriodicIO.hoodPosition - mPeriodicIO.lastHoodPosition);
            if (distance > hoodNonMovementThreshhold) {
                hoodNonMovementTimeout = now + hoodNonMovementDuration;
            }

            if (now > hoodNonMovementTimeout) {
                // instead of resetting the sensor the code remembers the offset
                // hoodEncoderOffset = mPeriodicIO.hoodPosition+kHoodPositionAtFender-100; // -100 is so the PID can be off a little
                mFXHood.setSelectedSensorPosition(8000);
                hoodEncoderOffset=0;
                hoodHomed = true;
                FramePeriodSwitch.configStatorCurrentLimitPermanent(mFXHood,new StatorCurrentLimitConfiguration(true, kHoodCurrentLimitHigh, kHoodCurrentLimitHigh, 0));    

                mPeriodicIO.hoodDemand = 0;
                setShootDistance(0);
                System.out.println("homing hood is complete offset="+hoodEncoderOffset);
            }
        }

        // System.out.println("hood pos="+mPeriodicIO.hoodPosition+ " demand "+mPeriodicIO.hoodDemand + " control="+mPeriodicIO.hoodControlMode);
        return defaultStateTransfer();
    }

    private SystemState handleHolding() {
        if (mStateChanged) {
            mPeriodicIO.hoodControlMode = ControlMode.MotionMagic;
            mPeriodicIO.flyControlMode = ControlMode.Velocity;

            mPeriodicIO.schedDeltaDesired = mSchedDormant;
            updateShooterDemandsBasedOnDist();
        }
        // updateShooterStatus();

        return holdingStateTransfer();
    }

    private SystemState handleManualControlling() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = mSchedActive;
            mPeriodicIO.hoodTestDemand = 0;
            mPeriodicIO.flyTestDemand = 0;
            mPeriodicIO.hoodControlMode = ControlMode.PercentOutput;
            mPeriodicIO.flyControlMode = ControlMode.PercentOutput;
            // test code runs open loop so don't care if it is homed and current limit must be low
            FramePeriodSwitch.configStatorCurrentLimitPermanent(mFXHood, new StatorCurrentLimitConfiguration(true, kHoodCurrentLimitLow, kHoodCurrentLimitLow, 0));
        }

        mPeriodicIO.hoodDemand = mPeriodicIO.hoodTestDemand;
        mPeriodicIO.flyDemand = mPeriodicIO.flyTestDemand;

        return defaultStateTransfer();
    }

    private SystemState handleShooting() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = mSchedActive;
            mPeriodicIO.hoodControlMode = ControlMode.MotionMagic;
            mPeriodicIO.flyControlMode = ControlMode.Velocity;
            updateShooterDemandsBasedOnDist();
        }
        updateShooterStatus();

        return defaultStateTransfer();
    }

    private SystemState holdingStateTransfer() {
        if (mWantedState != WantedState.HOLD) {
            turnOffFlywheel = false;
        }
        return defaultStateTransfer();
    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case SHOOT:
                return SystemState.SHOOTING;
            case MANUALCONTROL:
                return SystemState.MANUALCONTROLLING;
            case DISABLE:
                return SystemState.DISABLING;
            case HOMEHOOD:
                return SystemState.HOMINGHOOD;
            case HOLD:
                return SystemState.HOLDING;
            case ASSESS:
                return SystemState.ASSESSING;
            // default:
        }
        return null;
    }

    public synchronized boolean readyToShoot() {
        return mSystemState == SystemState.SHOOTING
                && (mPeriodicIO.reachedDesiredSpeed && mPeriodicIO.reachedDesiredHoodPosition);
    }

    public synchronized void setShootDistance(double distance) {
        if (distance != mDistance) {
            mDistance = Math.max(kMinShootDistance, Math.min(distance, kMaxShootDistance));
            updateShooterDemandsBasedOnDist();
            updateShooterStatus();
        }
    }

    public synchronized void setShootDistanceFromTY(double TY) {
        double dist = (shooterLLTYDist.getInterpolated(new InterpolatingDouble(TY))).value;
        System.out.println("setShootDistanceFromTY TY="+TY+" dist="+dist);
        setShootDistance(dist);
    }

    public synchronized void setMotorTestDemand(double hoodTestDemand, double flyTestDemand) {
        mPeriodicIO.hoodTestDemand = hoodTestDemand;
        mPeriodicIO.flyTestDemand = flyTestDemand;
    }

    private void updateShooterDemandsBasedOnDist(){
        mPeriodicIO.flyDemand = distanceToFlyVelocity(mDistance);
        mPeriodicIO.hoodDemand = distanceToHoodPos(mDistance);
    }

    private void updateShooterStatus() {
        mPeriodicIO.reachedDesiredSpeed = Math.abs(mPeriodicIO.flyVelocity - mPeriodicIO.flyDemand) <= 300;
        mPeriodicIO.reachedDesiredHoodPosition = Math.abs(mPeriodicIO.hoodPosition - mPeriodicIO.hoodDemand) <= 100;
    }

    public void stopFlywheel() {
        if (mSystemState == SystemState.HOLDING) {
            turnOffFlywheel = true;
        }
    }

    // fly velocity is from 0 to 20500 ticks/100msec
    private double distanceToFlyVelocity(double distance) {
        // units are ticks per 100msec
        return (shooterSpeedMap.getInterpolated(new InterpolatingDouble(distance))).value;
    }

    // Hood range is from 0 to 27800 ticks
    // Hood range is from 15.82 degrees to 35.82 degrees
    // 1390 falcon ticks per degree
    private double distanceToHoodPos(double distance) {
        return (shooterHoodMap.getInterpolated(new InterpolatingDouble(distance))).value;
    }

    @Override
    public void readPeriodicInputs() {
        double now = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart = now;

        mPeriodicIO.flyVelocity = mFXFlyLeft.getSelectedSensorVelocity();
        mPeriodicIO.lastHoodPosition = mPeriodicIO.hoodPosition;
        mPeriodicIO.hoodPosition = adjustHoodEncoderPosition(mFXHood.getSelectedSensorPosition());

        updateShooterStatus();
    }

    // these next two methods are used in liew of resetting the encoder.
    // resetting the encoder can take a few milliseconds which could
    // cause unexpected behavior if the encoder values jump while PID'ing
    private double adjustHoodEncoderPosition(double rawPosition) {
        return rawPosition - hoodEncoderOffset;
    }

    private double unadjustHoodEncoderPosition(double adjustedPosition) {
        return adjustedPosition + hoodEncoderOffset;
    }

    @Override
    public void writePeriodicOutputs() {
        double flyDemand = mPeriodicIO.flyDemand;
        if (mPeriodicIO.flyControlMode == ControlMode.Velocity && turnOffFlywheel){
            flyDemand = 0;
        }

        double hoodDemand = mPeriodicIO.hoodDemand;
        if (mPeriodicIO.hoodControlMode == ControlMode.Position || mPeriodicIO.hoodControlMode == ControlMode.MotionMagic){
            hoodDemand = unadjustHoodEncoderPosition(mPeriodicIO.hoodDemand);
        }


        mFXHood.set(mPeriodicIO.hoodControlMode, hoodDemand);
        mFXFlyLeft.set(mPeriodicIO.flyControlMode, flyDemand);
        mFXFlyRight.set(mPeriodicIO.flyControlMode, flyDemand);
        // System.out.println("writePer hood pos="+mPeriodicIO.hoodPosition+" demand="+mPeriodicIO.hoodDemand+" control="+mPeriodicIO.hoodControlMode);
    }

    @Override
    public void stop() {
        mPeriodicIO.flyControlMode = ControlMode.PercentOutput;
        mPeriodicIO.hoodControlMode = ControlMode.PercentOutput;
        mPeriodicIO.hoodDemand = 0;
        mPeriodicIO.flyDemand = 0;

        writePeriodicOutputs();
    }

    @Override
    public int whenRunAgain() {
        return mPeriodicIO.schedDeltaDesired;
    }

    @Override
    public String getLogHeaders() {
        return  sClassName+".schedDeltaDesired,"+
                sClassName+".schedDeltaActual,"+
                sClassName+".schedDuration,"+
                sClassName+".mSystemState,"+
                sClassName+".flywheelVelocity,"+
                sClassName+".hoodPosition,"+
                sClassName+".flyDemand,"+
                sClassName+".hoodDemand,"+
                sClassName+".flyControlMode,"+
                sClassName+".hoodControlMode,"+
                sClassName+".reachedDesiredSpeed,"+
                sClassName+".reachedDesiredHoodPosition,"+
                sClassName+".flyLeftStator,"+
                sClassName+".flyRightStator,"+
                sClassName+".hoodStator,"+
                sClassName+".mDistance,"+
                sClassName+".hoodHomed";
    }

    @Override
    public String getLogValues(boolean telemetry) {
        String start;
        if (telemetry){
            start = ",,,";
        }
        else{
            start = mPeriodicIO.schedDeltaDesired+","+
                    mPeriodicIO.schedDeltaActual+","+
                    (Timer.getFPGATimestamp()-mPeriodicIO.lastSchedStart)+",";
        }
        return  start+
        mSystemState+","+
        mPeriodicIO.flyVelocity+","+
        mPeriodicIO.hoodPosition+","+
        mPeriodicIO.flyDemand+","+
        mPeriodicIO.hoodDemand+","+
        mPeriodicIO.flyControlMode+","+
        mPeriodicIO.hoodControlMode+","+
        mPeriodicIO.reachedDesiredSpeed+","+
        mPeriodicIO.reachedDesiredHoodPosition+","+
        mPeriodicIO.flyLeftStator+","+
        mPeriodicIO.flyRightStator+","+
        mPeriodicIO.hoodStator+","+
        mDistance+","+
        hoodHomed;
    }

    @Override
    public void outputTelemetry() {
        mPeriodicIO.flyLeftStator = FramePeriodSwitch.getStatorCurrent(mFXFlyLeft);
        mPeriodicIO.flyRightStator = FramePeriodSwitch.getStatorCurrent(mFXFlyRight);
        mPeriodicIO.hoodStator = FramePeriodSwitch.getStatorCurrent(mFXHood);

        SmartDashboard.putNumber("Hood demand", mPeriodicIO.hoodDemand);
        SmartDashboard.putNumber("Flywheel demand", mPeriodicIO.flyDemand);
        SmartDashboard.putNumber("Hood Position", mPeriodicIO.hoodPosition);
        SmartDashboard.putNumber("Flywheel Speed", mPeriodicIO.flyVelocity);
        SmartDashboard.putBoolean("Reached Desired Speed", mPeriodicIO.reachedDesiredSpeed);
        SmartDashboard.putBoolean("Reached Desired Hood", mPeriodicIO.reachedDesiredHoodPosition);
        // SmartDashboard.putBoolean("Ready To Shoot", readyToShoot());
        // // these next values are bypassing readPeriodicInputs to reduce the ctre errors
        // // in
        // // riolog
        // SmartDashboard.putNumber("FlyStatorRight", mFXRightFlyWheel.getStatorCurrent());
        // SmartDashboard.putNumber("FlyStatorLeft", mFXLeftFlyWheel.getStatorCurrent());
        // SmartDashboard.putNumber("HoodStator", mFXHood.getStatorCurrent());
        // SmartDashboard.putNumber("FlySupplyRight", mFXRightFlyWheel.getSupplyCurrent());
        // SmartDashboard.putNumber("FlySupplyLeft", mFXLeftFlyWheel.getSupplyCurrent());
        // SmartDashboard.putNumber("HoodSupply", mFXHood.getSupplyCurrent());
    }

    public static class PeriodicIO {
        // Logging
        public int schedDeltaDesired;
        public double schedDeltaActual;
        public double lastSchedStart;

        // Inputs
        public double flyVelocity;
        public double hoodPosition;

        public double flyLeftStator;
        public double flyRightStator;
        public double hoodStator;

        // Outputs
        public double flyDemand;
        public double hoodDemand;

        // Other
        public ControlMode flyControlMode;
        public ControlMode hoodControlMode;
        public double hoodTestDemand;
        public double flyTestDemand;

        public double lastHoodPosition;
        public boolean reachedDesiredSpeed;
        public boolean reachedDesiredHoodPosition;
    }

}
