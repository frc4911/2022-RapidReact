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
import libraries.cheesylib.drivers.TalonFXFactory;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.subsystems.SubsystemManager;
import libraries.cheesylib.util.LatchedBoolean;

public class Climber extends Subsystem {

    // Hardware
    private final TalonFX mFXLeftClimber, mFXRightClimber;
    private final Solenoid mSolenoidDeploy;

    // Subsystem Constants
    private final double kClimberCurrentLimitLow = 60; // temp change while testing autoclimb reset to 80 when done
    private final double kClimberCurrentLimitHigh = 60; // temp change while testing autoclimb reset to 80 when done
    private final double kStatusFramePeriodActive = 20;
    private final double kControlFrameActive = 18;
    private final double kStatusFramePeriodDormant = 100;
    private final double kControlFrameDormant = 100;
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
        CLIMBING_1_LIFT,
        CLIMBING_2_ROTATE_UP,
        CLIMBING_3_LIFT_MORE,
        CLIMBING_4_ENGAGE_TRAV,
        CLIMBING_5_RELEASE_MID,
        PRECLIMBING,
        TESTING
    }

    public enum WantedState {
        DISABLE,
        HOLD,
        HOME,
        CLIMB_1_LIFT,
        CLIMB_2_ROTATE_UP,
        CLIMB_3_LIFT_MORE,
        CLIMB_4_ENGAGE_TRAV,
        CLIMB_5_RELEASE_MID,
        PRECLIMB,
        TEST
    }

    public enum MidArmPosition {
        BAR(80000),
        CLAWS(40000),
        DOWN(0),
        NOTSET(0);

        double position;

        private MidArmPosition(double position) {
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
    private boolean stageOneComplete;
    private boolean stageTwoComplete;
    private boolean stageThreeComplete;
    private boolean stageFourComplete;
    private boolean stageFiveComplete;

    private SystemState mSystemState;
    private WantedState mWantedState;
    private boolean mStateChanged;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private LatchedBoolean mSystemStateChange = new LatchedBoolean();
    private SolenoidState mSolenoidState;
    private int mDefaultSchedDelta = 20;

    private double testClimberDemand;

    // Climber homing state variables
    // Homing is done by sending the Climber to a negative position
    // While watching for the climber encoder to stop changing for a sufficient
    // amount of time
    private final double climberMovementThreshhold = 50; // encoder movements below this threshhold are considered
                                                        // stopped
    private final double climberNonMovementDuration = .1; // reading below threshhold encoder reads for this long is
                                                           // considered stopped
    private final double climberHomingDemand = -0.1;
    private boolean climberHomed = false; // global flag
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
        mSolenoidDeploy = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.CLIMBER_DEPLOY);
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
        mFXLeftClimber.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kClimberCurrentLimitLow, kClimberCurrentLimitLow, 0));
        mFXRightClimber.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kClimberCurrentLimitLow, kClimberCurrentLimitLow, 0));

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
                    mSystemState = SystemState.DISABLING;
                    mWantedState = WantedState.DISABLE;
                    mPeriodicIO.schedDeltaDesired = 0; // goto sleep
                    break;
                case TEST:
                    mSystemState = SystemState.TESTING;
                    mWantedState = WantedState.TEST;
                    mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
                    break;
                case AUTONOMOUS:
                case TELEOP:
                    mSystemState = SystemState.HOLDING;
                    mWantedState = WantedState.HOLD;
                    mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
                    break;
            }
            stageOneComplete = false;
            stageTwoComplete = false;
            stageThreeComplete = false;
            stageFourComplete = false;
            stageFiveComplete = false;
            mPeriodicIO.deployDemand = SolenoidState.RETRACT;
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
                    case PRECLIMBING:
                        newState = handlePreclimbing();
                        break;
                    case CLIMBING_1_LIFT:
                        newState = handleClimbing_1_Lift();
                        break;
                    case CLIMBING_2_ROTATE_UP:
                        newState = handleClimbing_2_RotateUp();
                        break;
                    case CLIMBING_3_LIFT_MORE:
                        newState = handleClimbing_3_LiftMore();
                        break;
                    case CLIMBING_4_ENGAGE_TRAV:
                        newState = handleClimbing_4_EngageTrav();
                        break;
                    case CLIMBING_5_RELEASE_MID:
                        newState = handleClimbing_5_ReleaseMid();
                        break;
                    case TESTING:
                        newState = handleTesting();
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

    public WantedState getWantedState() {
        return mWantedState;
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
            mPeriodicIO.climberControlMode = ControlMode.PercentOutput;
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
        }

        double distance = Math.abs(mPeriodicIO.climberPosition - mPeriodicIO.lastClimberPosition);
        if (distance > climberMovementThreshhold) {
            climberNonMovementTimeout = now + climberNonMovementDuration;
        }

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

    private SystemState handlePreclimbing() {
        if(mStateChanged) {
            masterConfig(kClimberCurrentLimitHigh, true,
                         Double.NaN, false, //TODO: Check what soft limits need to be
                         Double.NaN, false, //TODO: Check what soft limits need to be
                         kStatusFramePeriodActive, kControlFrameActive, mPeriodicIO.mDefaultSchedDelta);
            mPeriodicIO.deployDemand = SolenoidState.RETRACT;
            mPeriodicIO.climberDemand = kExtendToTopPos;
            mPeriodicIO.climberControlMode = ControlMode.Position;
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
        }

        return defaultStateTransfer();
    }

    private SystemState handleTesting(){
        if(mStateChanged) {
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
            testClimberDemand = 0;
            mPeriodicIO.climberControlMode = ControlMode.PercentOutput;

            mFXLeftClimber.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
            mFXLeftClimber.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
    
            mFXRightClimber.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
            mFXRightClimber.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
    
        }
        if (testClimberDemand != mPeriodicIO.climberDemand){
            mPeriodicIO.climberDemand = testClimberDemand;
            mPeriodicIO.climberControlMode = ControlMode.PercentOutput;
        }

        return defaultStateTransfer();
    }

    private SystemState handleClimbing_1_Lift(){
        if (mStateChanged) {
            masterConfig(Double.NaN, true,
                         Double.NaN, false,
                         Double.NaN, false,
                         Double.NaN, Double.NaN, Double.NaN);
            stageOneComplete = false; // redundant
        }
        if (true /*Mid claw motor is to position*/) {
            stageOneComplete = true;
        }
        if (mWantedState != WantedState.CLIMB_1_LIFT) {
            stageOneComplete = false;
        }
        return defaultStateTransfer();
    }

    public boolean ElevatorHasGrabbedBar() {
        return gbdcSubState.equals(GrabBarDynamicClawSubState.DONE);
    }

    private SystemState handleClimbing_2_RotateUp(){
        if (mStateChanged) {
            masterConfig(Double.NaN, true,
                         Double.NaN, false,
                         Double.NaN, false,
                         Double.NaN, Double.NaN, Double.NaN);
            // set demands
            // set magic motion params?
            stageTwoComplete = false; // redundant
        }
        if (true /*Move Slappy sticks to position*/) {
            stageTwoComplete = true;
        }
        if (mWantedState != WantedState.CLIMB_2_ROTATE_UP) {
            stageTwoComplete = false;
        }
        return defaultStateTransfer();
    }

    private SystemState handleClimbing_3_LiftMore(){
        if (mStateChanged) {
            masterConfig(Double.NaN, true,
                         Double.NaN, false,
                         Double.NaN, false,
                         Double.NaN, Double.NaN, Double.NaN);
            // set demands
            // set magic motion params?
            stageThreeComplete = false; // redundant
        }
        if (true /*Mid claw motor is to position*/) {
            stageThreeComplete = true;
        }
        if (mWantedState != WantedState.CLIMB_3_LIFT_MORE) {
            stageThreeComplete = false;
        }
        return defaultStateTransfer();
    }

    private SystemState handleClimbing_4_EngageTrav(){
        if (mStateChanged) {
            masterConfig(Double.NaN, true,
                         Double.NaN, false,
                         Double.NaN, false,
                         Double.NaN, Double.NaN, Double.NaN);
            // set demands
            // set magic motion params?
            stageFourComplete = false; // redundant
        }
        if (true /*Mid claw motor is to position*/) {
            stageFourComplete = true;
        }
        if (mWantedState != WantedState.CLIMB_4_ENGAGE_TRAV) {
            stageFourComplete = false;
        }
        return defaultStateTransfer();

    }

    private SystemState handleClimbing_5_ReleaseMid(){
        if (mStateChanged) {
            masterConfig(Double.NaN, true,
                         Double.NaN, false,
                         Double.NaN, false,
                         Double.NaN, Double.NaN, Double.NaN);
            // set demands
            // set magic motion params?
            stageFiveComplete = false; // redundant
        }
        if (true /*Mid claw motor is to position*/) {
            stageFiveComplete = true;
            masterConfig(0, false, //TODO: Confirm if this actually disables stator currents
                         Double.NaN, false,
                         Double.NaN, false,
                         kStatusFramePeriodDormant, kControlFrameDormant, Double.NaN);
            System.out.println("* * * Finished Climbing Sequence * * *");
        }
        if (mWantedState != WantedState.CLIMB_5_RELEASE_MID) {
            stageFiveComplete = false;
        }
        return defaultStateTransfer();
    }

    private void masterConfig(double statorLimit, boolean statorEnable, 
                              double softLimitFwd, boolean softLimitFwdEnable, 
                              double softLimitRev, boolean softLimitRevEnable, 
                              double statusFramePeriod, double controlFramePeriod, double whenRun){
        if (statorLimit != Double.NaN){
            mFXLeftClimber.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(statorEnable, statorLimit, statorLimit, 0));
            mFXRightClimber.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(statorEnable, statorLimit, statorLimit, 0));    
        }

        if (softLimitFwd != Double.NaN){
            mFXLeftClimber.configForwardSoftLimitThreshold(softLimitFwd, Constants.kLongCANTimeoutMs);
            mFXRightClimber.configForwardSoftLimitThreshold(softLimitFwd, Constants.kLongCANTimeoutMs);

            mFXLeftClimber.configForwardSoftLimitEnable(softLimitFwdEnable, Constants.kLongCANTimeoutMs);    
            mFXRightClimber.configForwardSoftLimitEnable(softLimitFwdEnable, Constants.kLongCANTimeoutMs);
        }

        if (softLimitRev != Double.NaN){
            mFXLeftClimber.configReverseSoftLimitThreshold(softLimitFwd, Constants.kLongCANTimeoutMs);
            mFXRightClimber.configReverseSoftLimitThreshold(softLimitFwd, Constants.kLongCANTimeoutMs);
            
            mFXLeftClimber.configReverseSoftLimitEnable(softLimitRevEnable, Constants.kLongCANTimeoutMs);
            mFXRightClimber.configReverseSoftLimitEnable(softLimitRevEnable, Constants.kLongCANTimeoutMs);
        }
        if (statusFramePeriod != Double.NaN){
            mFXLeftClimber.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, (int)statusFramePeriod, Constants.kLongCANTimeoutMs);
        }
        if (controlFramePeriod != Double.NaN){
            mFXLeftClimber.setControlFramePeriod(ControlFrame.Control_3_General, (int)controlFramePeriod);
            mFXRightClimber.setControlFramePeriod(ControlFrame.Control_3_General, (int)controlFramePeriod);
        }
        if (whenRun != Double.NaN){
            mPeriodicIO.schedDeltaDesired = (int) whenRun;
        }

    }
    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case HOME:
                return SystemState.HOMING;
            case DISABLE:
                return SystemState.DISABLING;
            case CLIMB_1_LIFT:
                return SystemState.CLIMBING_1_LIFT;
            case CLIMB_2_ROTATE_UP:
                return SystemState.CLIMBING_2_ROTATE_UP;
            case CLIMB_3_LIFT_MORE:
                return SystemState.CLIMBING_3_LIFT_MORE;
            case CLIMB_4_ENGAGE_TRAV:
                return SystemState.CLIMBING_4_ENGAGE_TRAV;
            case CLIMB_5_RELEASE_MID:
                return SystemState.CLIMBING_5_RELEASE_MID;
            case PRECLIMB:
                return SystemState.PRECLIMBING;
            case TEST:
                return SystemState.TESTING;
            case HOLD:
                default:
                    return SystemState.HOLDING;
        }
    }

    public boolean isClimbingStageDone(WantedState state) {
        switch(state) {
            case CLIMB_1_LIFT:
                return stageOneComplete;
            case CLIMB_2_ROTATE_UP:
                return stageTwoComplete;
            case CLIMB_3_LIFT_MORE:
                return stageThreeComplete;
            case CLIMB_4_ENGAGE_TRAV:
                return stageFourComplete;
            case CLIMB_5_RELEASE_MID:
                return stageFiveComplete;
            default:
                System.out.println("Uh oh something is not right");
                return false;
        }
    }

    public void setClimberTestDemand(double newDemand){
        testClimberDemand = newDemand;
    }

    public void setSlappyStickState(boolean state) {
        if (state) {
            mPeriodicIO.deployDemand = SolenoidState.EXTEND;
        } else {
            mPeriodicIO.deployDemand = SolenoidState.RETRACT;
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
        if (mSolenoidState != mPeriodicIO.deployDemand) {
            // If we're extending, only do it if the elevator is below the height where
            // they'll hit
            if (!mPeriodicIO.deployDemand.get() || (mPeriodicIO.deployDemand.get()
                    && mPeriodicIO.climberPosition < kSlappySticksElevatorConflictLimit)) {
                mSolenoidState = mPeriodicIO.deployDemand;
                mSolenoidDeploy.set(mPeriodicIO.deployDemand.get());
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
                mPeriodicIO.deployDemand+","+
                climberHomed;
    }

    @Override
    public void outputTelemetry() {
        // SmartDashboard.putNumber("Left Climber Encoder", mPeriodicIO.climberPosition);
        // SmartDashboard.putNumber("ClimbStatorLeft", mFXLeftClimber.getStatorCurrent());
        // SmartDashboard.putNumber("ClimbStatorRight", mFXRightClimber.getStatorCurrent());
    }

    public static class PeriodicIO {
        // Logging
        public final int mDefaultSchedDelta = 20; // axis updated every 20 msec
        public int schedDeltaDesired;
        public double schedDeltaActual;
        public double lastSchedStart;

        // Inputs
        public double climberPosition;
        public double slappyPosition;

        // Outputs
        public double climberDemand;
        public double slappyDemand;
        public ControlMode climberControlMode;
        public SolenoidState deployDemand;

        // Other
        public double lastClimberPosition;
        public double lastSlappyPosition;

    }

}
