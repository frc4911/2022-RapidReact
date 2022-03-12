package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.constants.Ports;
import frc.robot.subsystems.Shooter.SystemState;
import libraries.cheesylib.drivers.TalonFXFactory;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.subsystems.SubsystemManager;
import libraries.cheesylib.util.LatchedBoolean;

public class Climber extends Subsystem {

    // Hardware
    private final TalonFX mFXLeftClimber, mFXRightClimber;
    private final Solenoid mSlapSticks;

    // Subsystem Constants
    private final double kClimberCurrentLimit = 60; // temp change while testing autoclimb reset to 80 when done
    private final int kHomingZeroAdjustment = -1000; // Tick adjustment to accomodate physical overdriving on the elevator
    private final int kSlappySticksElevatorConflictLimit = 140000; // Max theory is 140k
    private final int kElevatorMaxHeight = 162000; // Max theory is 165k

    // Subsystem States
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
        DISABLING,
        HOLDING,
        HOMING,
        CLIMBING,
        MOVINGELEVATOR,
        GRABBING_BAR_DYNAMIC_CLAW,
        PRECLIMBING,
        AUTOCLIMBING
    }

    public enum WantedState {
        DISABLE,
        HOLD,
        HOME,
        CLIMB,
        MOVEELEVATOR,
        GRAB_BAR_DYNAMIC_CLAW,
        PRECLIMB,
        AUTOCLIMB
    }

    public enum ElevatorPositions {
        BAR(80000),
        CLAWS(40000),
        DOWN(0),
        NOTSET(0);

        double position;

        private ElevatorPositions(double position) {
            this.position = position;
        }

        public double get() {
            return position;
        }
    }

    private enum GrabBarDynamicClawSubState {
        EXTENDTOTOP,
        RETRACTTOENGAGECLAW,
        EXTENDSLAPPIES,
        RETRACTSLAPPIES,
        DISENGAGECLAW,
        DONE
    }

    private GrabBarDynamicClawSubState gbdcSubState = GrabBarDynamicClawSubState.EXTENDTOTOP;
    private boolean gbdcSubStateChange = false;
    // TODO: get the correct values for these constants
    private final double kExtendToTopPos = kElevatorMaxHeight-2000;
    private final double kDescendToBar = 2000;
    private final double kElevPosTolerance = 1000;
    private double mSlappyExtendTimeout;
    private final double kSlappyExtendDuration = 1;



    private ElevatorPositions desiredElevatorPosition = ElevatorPositions.NOTSET;
    private SystemState mSystemState;
    private WantedState mWantedState;
    private boolean mStateChanged;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private LatchedBoolean mSystemStateChange = new LatchedBoolean();
    private SolenoidState mSolenoidState;
    private int mDefaultSchedDelta = 20;

    private boolean elevatorAtDesiredPosition;
    private double desiredElevatorPositionTicks;
    private double climbSpeed = 0;

    // Climber homing state variables
    // Homing is done by sending the Climber to a negative position
    // While watching for the climber encoder to stop changing for a sufficient
    // amount of time
    private final double climberMovementThreshhold = 50; // encoder movements below this threshhold are considered
                                                        // stopped
    private final double climberNonMovementDuration = .1; // reading below threshhold encoder reads for this long is
                                                           // considered stopped
    private final double climberHomingDemand = -0.1;
    private boolean climberHomed = true; // global flag
    private double climberNonMovementTimeout; // timestamp of when low readings are sufficient
    private WantedState wantedStateAfterHoming = WantedState.HOLD; // state to transition to after homed

    // Other
    private SubsystemManager mSubsystemManager;

    // Subsystem Creation
    private static String sClassName;
    private static int sInstanceCount;
    private static Climber sInstance = null;

    public static Climber getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new Climber(caller);
        } else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller) {
        System.out.println("(" + caller + ") " + " getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Climber(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mFXLeftClimber = TalonFXFactory.createDefaultTalon(Ports.LEFT_CLIMBER);
        mFXRightClimber = TalonFXFactory.createDefaultTalon(Ports.RIGHT_CLIMBER);
        mSlapSticks = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.CLIMBER_STAGE);
        mSubsystemManager = SubsystemManager.getInstance(sClassName);
        configMotors();
    }

    private void configMotors() {

        // only one encoder is needed
        mFXLeftClimber.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, Constants.kLongCANTimeoutMs);

        mFXLeftClimber.setControlFramePeriod(ControlFrame.Control_3_General, 18);
        mFXRightClimber.setControlFramePeriod(ControlFrame.Control_3_General, 18);

        mFXLeftClimber.configForwardSoftLimitEnable(true, Constants.kLongCANTimeoutMs);
        mFXLeftClimber.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mFXRightClimber.configForwardSoftLimitEnable(true, Constants.kLongCANTimeoutMs);
        mFXRightClimber.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mFXLeftClimber.setInverted(false);
        mFXRightClimber.setInverted(true);

        mFXLeftClimber.setNeutralMode(NeutralMode.Brake);
        mFXRightClimber.setNeutralMode(NeutralMode.Brake);

        // parameters are enable, current limit after triggering, trigger limit, time
        // allowed to exceed trigger limit before triggering
        // Current limit motors
        mFXLeftClimber.configStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, kClimberCurrentLimit, kClimberCurrentLimit, 0));
        mFXRightClimber.configStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, kClimberCurrentLimit, kClimberCurrentLimit, 0));
        mFXLeftClimber.configStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, kClimberCurrentLimit, kClimberCurrentLimit, 0));
        mFXRightClimber.configStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, kClimberCurrentLimit, kClimberCurrentLimit, 0));

        // TODO: all values must be tuned
        double kClosedError = 50;
        double config_kP = 0.2;
        double config_kI = 0;
        double config_kD = 0;
        double config_kF = 0;
        double integralZone = 0;
        double kClosedRamp = 0; // this makes a difference in Magic and Position modes 
                                // but when using magic i prefer to use the Magic config values
                                // so this should only be nonzero when using Position mode
        double kCruiseVelocity = 0; 
        double kAcceleration = 0;
        int kCurveStrength = 0; // 0 is trapizoidal, larger values make the path more soft (curved) at velocity changes

        mFXLeftClimber.config_kP(0, config_kP, Constants.kLongCANTimeoutMs);
        mFXLeftClimber.config_kI(0, config_kI, Constants.kLongCANTimeoutMs);
        mFXLeftClimber.config_kD(0, config_kD, Constants.kLongCANTimeoutMs);
        mFXLeftClimber.config_kF(0, config_kF, Constants.kLongCANTimeoutMs);
        mFXLeftClimber.config_IntegralZone(0, integralZone, Constants.kLongCANTimeoutMs);
        mFXLeftClimber.configClosedloopRamp(kClosedRamp, Constants.kLongCANTimeoutMs);
        mFXLeftClimber.configAllowableClosedloopError(0, kClosedError, Constants.kLongCANTimeoutMs);

        mFXLeftClimber.configMotionCruiseVelocity(kCruiseVelocity); // 10000 ticks/second
        mFXLeftClimber.configMotionAcceleration(kAcceleration); // 2500 ticks/(sec^2)
        mFXLeftClimber.configMotionSCurveStrength(kCurveStrength); // trapizoidal curve
        
        mFXRightClimber.config_kP(0, config_kP, Constants.kLongCANTimeoutMs);
        mFXRightClimber.config_kI(0, config_kI, Constants.kLongCANTimeoutMs);
        mFXRightClimber.config_kD(0, config_kD, Constants.kLongCANTimeoutMs);
        mFXRightClimber.config_kF(0, config_kF, Constants.kLongCANTimeoutMs);
        mFXRightClimber.config_IntegralZone(0, integralZone, Constants.kLongCANTimeoutMs);
        mFXRightClimber.configClosedloopRamp(kClosedRamp, Constants.kLongCANTimeoutMs);
        mFXRightClimber.configAllowableClosedloopError(0, kClosedError, Constants.kLongCANTimeoutMs);

        mFXRightClimber.configMotionCruiseVelocity(kCruiseVelocity); // 10000 ticks/second
        mFXRightClimber.configMotionAcceleration(kAcceleration); // 2500 ticks/(sec^2)
        mFXRightClimber.configMotionSCurveStrength(kCurveStrength); // trapizoidal curve
        
    }

    @Override
    public void onStart(Phase phase) {
        synchronized (Climber.this) {
            mStateChanged = true;
            switch (phase) {
                case DISABLED:
                case TEST:
                    mSystemState = SystemState.DISABLING;
                    mWantedState = WantedState.DISABLE;
                    mPeriodicIO.schedDeltaDesired = 0; // goto sleep
                    break;
                case AUTONOMOUS:
                case TELEOP:
                    mSystemState = SystemState.HOLDING;
                    mWantedState = WantedState.HOLD;
                    mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
                    break;
            }
            mPeriodicIO.slappyDemand = SolenoidState.RETRACT;
            mSolenoidState = SolenoidState.EXTEND;
            System.out.println(sClassName + " state " + mSystemState);
            stop(); // put into a known state
        }
    }

    @Override
    public void onLoop(double timestamp) {
        synchronized (Climber.this) {
            do {
                SystemState newState;
                switch (mSystemState) {
                    case DISABLING:
                        newState = handleDisabling();
                        break;
                    case HOMING:
                        newState = handleHoming();
                        break;
                    case CLIMBING:
                        newState = handleClimbing();
                        break;
                    case PRECLIMBING:
                        newState = handlePreclimbing();
                        break;
                    case MOVINGELEVATOR:
                        newState = handleMovingElevator();
                        break;
                    case GRABBING_BAR_DYNAMIC_CLAW:
                        newState = handleGrabbingBarDynamicClaw();
                        break;
                    case HOLDING:
                    default:
                        newState = handleHolding();
                        break;
                }

                if (newState != mSystemState) {
                    System.out.println(
                            sClassName + " state " + mSystemState + " to " + newState + " (" + timestamp + ")");
                    mSystemState = newState;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            } while (mSystemStateChange.update(mStateChanged));
        }
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

    private SystemState handleDisabling() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = 0;
            mPeriodicIO.climberDemand = 0;
            mPeriodicIO.climberControlMode = ControlMode.PercentOutput;
        }

        return defaultStateTransfer();
    }

    private SystemState handleHolding() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = mDefaultSchedDelta; // stay awake
            if (climberHomed) {
                System.out.println("climber hold position = "+mPeriodicIO.climberPosition);
                mPeriodicIO.climberDemand = Math.max(mPeriodicIO.climberPosition,0);
                mPeriodicIO.climberControlMode = ControlMode.Position;
            } else {
                mWantedState = WantedState.HOME;
                wantedStateAfterHoming = WantedState.HOLD;
                return defaultStateTransfer();
            }
        }

        return defaultStateTransfer();
    }

    private SystemState handleHoming() {
        double now = Timer.getFPGATimestamp();
        if (mStateChanged) {
            climberHomed = false;
            climberNonMovementTimeout = now + climberNonMovementDuration;
            mPeriodicIO.climberDemand = climberHomingDemand;
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
        }

        double distance = Math.abs(mPeriodicIO.climberPosition - mPeriodicIO.lastClimberPosition);
        if (distance > climberMovementThreshhold) {
            climberNonMovementTimeout = now + climberNonMovementDuration;
        }
        //System.out.println("Climber Homing Now: distance " + distance);
        if (now > climberNonMovementTimeout) {

            System.out.println("Climber Homing Sequence Complete");
            mFXRightClimber.setSelectedSensorPosition(kHomingZeroAdjustment);
            mFXLeftClimber.setSelectedSensorPosition(kHomingZeroAdjustment);
            mPeriodicIO.climberPosition = 0;

            // Set maximum climber height after homing
            mFXLeftClimber.configForwardSoftLimitThreshold(kElevatorMaxHeight, Constants.kLongCANTimeoutMs);
            mFXRightClimber.configForwardSoftLimitThreshold(kElevatorMaxHeight, Constants.kLongCANTimeoutMs);

            climberHomed = true;
            mWantedState = wantedStateAfterHoming;
        }

        return defaultStateTransfer();
    }

    private SystemState handleClimbing() {
        if(mStateChanged) {
            mPeriodicIO.climberControlMode = ControlMode.PercentOutput;
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
        }
        mPeriodicIO.climberDemand = climbSpeed;

        return defaultStateTransfer();
    }

    private SystemState handleMovingElevator() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
            if (!climberHomed) {
                wantedStateAfterHoming = WantedState.MOVEELEVATOR;
                mWantedState = WantedState.HOME;
                return defaultStateTransfer();
            } else {
                if (desiredElevatorPosition.equals(ElevatorPositions.NOTSET)) {
                    elevatorAtDesiredPosition = true;
                } else {
                    desiredElevatorPositionTicks = desiredElevatorPosition.get();
                    mPeriodicIO.climberDemand = desiredElevatorPositionTicks;
                    mPeriodicIO.climberControlMode = ControlMode.Position;
                    elevatorAtDesiredPosition = false;
                }
            }
        }

        double distance = Math.abs(mPeriodicIO.climberPosition - desiredElevatorPositionTicks);
        if (distance < 300) {
            elevatorAtDesiredPosition = true;
        }

        if (mWantedState != WantedState.MOVEELEVATOR) {
            elevatorAtDesiredPosition = false;
            desiredElevatorPosition = ElevatorPositions.NOTSET;
        }
        return defaultStateTransfer();
    }

    public boolean elevatorHasReachedDesiredPosition() {
        return elevatorAtDesiredPosition;
    }

    private SystemState handlePreclimbing() {
        if(mStateChanged) {
            mPeriodicIO.slappyDemand = SolenoidState.RETRACT;
            mPeriodicIO.climberDemand = kExtendToTopPos;
            mPeriodicIO.climberControlMode = ControlMode.Position;
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
        }

        return defaultStateTransfer();
    }

    // This is preliminary only. It is intended to give us a running start if we decide to automate any of the climb
    private SystemState handleGrabbingBarDynamicClaw(){
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
            if (!climberHomed) {
                wantedStateAfterHoming = WantedState.GRAB_BAR_DYNAMIC_CLAW;
                mWantedState = WantedState.HOME;
                return defaultStateTransfer();
            } else {
                gbdcSubState = GrabBarDynamicClawSubState.RETRACTTOENGAGECLAW;
                gbdcSubStateChange = true;
            }
        }

        if (gbdcSubStateChange){
            gbdcSubStateChange = false;
            switch (gbdcSubState){
                case RETRACTTOENGAGECLAW:
                System.out.println("Climber substate RETRACTTOENGAGECLAW 1st time");
                    mPeriodicIO.climberDemand = kDescendToBar;
                    mPeriodicIO.climberControlMode = ControlMode.Position;
                    mPeriodicIO.slappyDemand = SolenoidState.RETRACT;
                    break;
                case EXTENDSLAPPIES:
                System.out.println("Climber substate EXTENDSLAPPIES 1st time");
                    mPeriodicIO.slappyDemand = SolenoidState.EXTEND;
                    mSlappyExtendTimeout = Timer.getFPGATimestamp() + kSlappyExtendDuration;
                    break;
                case EXTENDTOTOP:
                    System.out.println("Climber substate EXTENDTOTOP 1st time");
                        mPeriodicIO.climberDemand = kSlappySticksElevatorConflictLimit;
                        mPeriodicIO.climberControlMode = ControlMode.Position;
                        mPeriodicIO.slappyDemand = SolenoidState.RETRACT;
                        break;
                case RETRACTSLAPPIES:
                    System.out.println("Climber substate RETRACTSLAPPIES 1st time");
                        mPeriodicIO.slappyDemand = SolenoidState.RETRACT;
                        mSlappyExtendTimeout = Timer.getFPGATimestamp() + kSlappyExtendDuration;
                        break;
                case DISENGAGECLAW:
                        mPeriodicIO.climberDemand = kSlappySticksElevatorConflictLimit-20000;
                        mPeriodicIO.climberControlMode = ControlMode.Position;
                        mPeriodicIO.slappyDemand = SolenoidState.RETRACT;
                        break;
                
                        case DONE:
                    break;
            }
        }

        double distance;
        distance = Math.abs(mPeriodicIO.climberPosition - mPeriodicIO.climberDemand);

        switch (gbdcSubState){
            case RETRACTTOENGAGECLAW:
            System.out.println("Climber substate RETRACTTOENGAGECLAW dist "+distance);
                if (distance < kElevPosTolerance) {
                    gbdcSubStateChange = true;
                    gbdcSubState = GrabBarDynamicClawSubState.EXTENDSLAPPIES;
                }
                break;
            case EXTENDSLAPPIES:
                double now = Timer.getFPGATimestamp();
                System.out.println("EXTENDSLAPPIES now = "+now+" "+mSlappyExtendTimeout);
                if (Timer.getFPGATimestamp() >= mSlappyExtendTimeout){
                    gbdcSubStateChange = true;
                    gbdcSubState = GrabBarDynamicClawSubState.EXTENDTOTOP;
                }
                break;
            case EXTENDTOTOP:
                System.out.println("Climber substate EXTENDTOTOP dist "+distance);
                if (distance < kElevPosTolerance) {
                    gbdcSubStateChange = true;
                    gbdcSubState = GrabBarDynamicClawSubState.RETRACTSLAPPIES;
                }
                break;
            case RETRACTSLAPPIES:
                // double now = Timer.getFPGATimestamp();
                // System.out.println("RETRACTSLAPPIES now = "+now+" "+mSlappyExtendTimeout);
                if (Timer.getFPGATimestamp() >= mSlappyExtendTimeout){
                    gbdcSubStateChange = true;
                    gbdcSubState = GrabBarDynamicClawSubState.DISENGAGECLAW;
                }
                break;
            case DISENGAGECLAW:
                System.out.println("Climber substate DISENGAGECLAW dist "+distance);
                if (distance < kElevPosTolerance) {
                    gbdcSubStateChange = true;
                    gbdcSubState = GrabBarDynamicClawSubState.DONE;
                }
                break;
            case DONE:
                System.out.println("Climber substate DONE");
                break;
        }

        if (mWantedState != WantedState.GRAB_BAR_DYNAMIC_CLAW) {
            gbdcSubState = GrabBarDynamicClawSubState.RETRACTTOENGAGECLAW; // anystate that is not DONE
        }
        return defaultStateTransfer();
    }

    public boolean ElevatorHasGrabbedBar() {
        return gbdcSubState.equals(GrabBarDynamicClawSubState.DONE);
    }


    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case HOME:
                return SystemState.HOMING;
            case CLIMB:
                return SystemState.CLIMBING;
            case MOVEELEVATOR:
                return SystemState.MOVINGELEVATOR;
            case DISABLE:
                return SystemState.DISABLING;
            case GRAB_BAR_DYNAMIC_CLAW:
                return SystemState.GRABBING_BAR_DYNAMIC_CLAW;
            case AUTOCLIMB:
                return SystemState.AUTOCLIMBING;
            case PRECLIMB:
                return SystemState.PRECLIMBING;
            case HOLD:
                default:
                    return SystemState.HOLDING;
            }
    }

    public void setDesiredElevatorPosition(ElevatorPositions desiredElevatorPosition) {
        this.desiredElevatorPosition = desiredElevatorPosition;
    }

    public void setClimbSpeed(double speed) {
        // if(mSystemState != SystemState.HOMING && mSystemState != SystemState.DISABLING) {
        //     climbSpeed = speed;
        //     if(speed == 0.0) {
        //         mWantedState = WantedState.HOLD;
        //     } else {
        //         mWantedState = WantedState.CLIMB;
        //     }
        // }
    }

    public void setSlappyStickState(boolean state) {
        if (state) {
            mPeriodicIO.slappyDemand = SolenoidState.EXTEND;
        } else {
            mPeriodicIO.slappyDemand = SolenoidState.RETRACT;
        }
    }

    @Override
    public void readPeriodicInputs() {
        double now = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart = now;

        mPeriodicIO.lastClimberPosition = mPeriodicIO.climberPosition;
        mPeriodicIO.climberPosition = mFXLeftClimber.getSelectedSensorPosition();
    }

    @Override
    public void writePeriodicOutputs() {
        if (mSolenoidState != mPeriodicIO.slappyDemand) {
            // If we're extending, only do it if the elevator is below the height where
            // they'll hit
            if (!mPeriodicIO.slappyDemand.get() || (mPeriodicIO.slappyDemand.get()
                    && mPeriodicIO.climberPosition < kSlappySticksElevatorConflictLimit)) {
                mSolenoidState = mPeriodicIO.slappyDemand;
                mSlapSticks.set(mPeriodicIO.slappyDemand.get());
            }

            if (mSolenoidState.get()) {
                mFXLeftClimber.configForwardSoftLimitThreshold(kSlappySticksElevatorConflictLimit,
                        Constants.kLongCANTimeoutMs);
                mFXRightClimber.configForwardSoftLimitThreshold(kSlappySticksElevatorConflictLimit,
                        Constants.kLongCANTimeoutMs);
            } else {
                mFXLeftClimber.configForwardSoftLimitThreshold(kElevatorMaxHeight, Constants.kLongCANTimeoutMs);
                mFXRightClimber.configForwardSoftLimitThreshold(kElevatorMaxHeight, Constants.kLongCANTimeoutMs);
            }
        }

        // We don't need safety checks here because the motors can't extend through the
        // soft limits
        mFXLeftClimber.set(mPeriodicIO.climberControlMode, mPeriodicIO.climberDemand);
        mFXRightClimber.set(mPeriodicIO.climberControlMode, mPeriodicIO.climberDemand);
    }

    @Override
    public void stop() {
        System.out.println(sClassName + " stop()");
        mFXLeftClimber.set(ControlMode.PercentOutput, 0);
        mFXRightClimber.set(ControlMode.PercentOutput, 0);
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
                sClassName+".mWantedState,"+
                sClassName+".gbdcSubState,"+
                sClassName+".climberPosition,"+
                sClassName+".climberDemand,"+
                sClassName+".climberControlMode,"+
                sClassName+".slappyDemand,"+
                sClassName+".climberHomed";
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
                mWantedState+","+
                mPeriodicIO.climberPosition+","+
                gbdcSubState+","+
                mPeriodicIO.climberDemand+","+
                mPeriodicIO.climberControlMode+","+
                mPeriodicIO.slappyDemand+","+
                climberHomed;
    }

    @Override
    public void outputTelemetry() {
        // SmartDashboard.putNumber("Left Climber Encoder", mPeriodicIO.climberPosition);
        // SmartDashboard.putNumber("Left Climb Current", mFXLeftClimber.getStatorCurrent());
        // SmartDashboard.putNumber("Right Climb Current", mFXRightClimber.getStatorCurrent());
        // SmartDashboard.putNumber("Left Climb Supply Current", mFXLeftClimber.getSupplyCurrent());
        // SmartDashboard.putNumber("Right Climb Supply Current", mFXRightClimber.getSupplyCurrent());
    }

    public static class PeriodicIO {
        // Logging
        public final int mDefaultSchedDelta = 20; // axis updated every 20 msec
        public int schedDeltaDesired;
        public double schedDeltaActual;
        public double lastSchedStart;

        // Inputs
        public double climberPosition;

        // Outputs
        public double climberDemand;
        public ControlMode climberControlMode;
        public SolenoidState slappyDemand;

        // Other
        public double lastClimberPosition;

    }

}
