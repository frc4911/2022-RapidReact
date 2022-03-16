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
    private final TalonFX mFXMidArm, mFXSlappy;
    private final Solenoid mSolenoidDeploy;

    // Subsystem Constants
    private final double kMidArmCurrentLimitLow = 10; // temp change while testing autoclimb reset to 80 when done
    private final double kMidArmCurrentLimitHigh = 80; // temp change while testing autoclimb reset to 80 when done
    private final double kSlappyCurrentLimitLow = 5;
    private final double kSlappyCurrentLimitHigh = 30;

    private final double kStatusFramePeriodActive = 20;
    private final double kControlFrameActive = 18;
    private final double kStatusFramePeriodDormant = 100;
    private final double kControlFrameDormant = 100;

    // Subsystem States
    public enum SolenoidState {
        RELEASE(true),
        LOCK(false);

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
        MIDBAR(370000),
        MIDDLE(92500),
        DOWN(-15900),
        HOME(-400),
        NOTSET(0);

        double position;

        private MidArmPosition(double position) {
            this.position = position;
        }

        public double get() {
            return position;
        }
    }

    public enum SlappyPosition {
        MAX(160000),
        MID(120000), //165000 physical max
        DOWN(10000),
        HOME(-1000),
        NOTSET(0);

        double position;

        private SlappyPosition(double position) {
            this.position = position;
        }

        public double get() {
            return position;
        }
    }

    private boolean preClimbComplete;
    private boolean stageOneComplete;
    private boolean stageTwoComplete;
    private boolean stageThreeComplete;
    private boolean stageFourComplete;
    private boolean stageFiveComplete;

    private boolean midArmHomingComplete;
    private boolean slappyHomingComplete;
    private boolean midArmJustFinishedHoming;

    private SystemState mSystemState;
    private WantedState mWantedState;
    private boolean mStateChanged;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private LatchedBoolean mSystemStateChange = new LatchedBoolean();
    private SolenoidState mSolenoidState;

    private final double midBarPosTolerance = 2000;
    private final double slappyPosTolerance = 2000;
    private final double kSolenoidDuration = .25;
    private double solenoidTimeout = 0;  

    private double testMidArmDemand;
    private double testSlappyDemand;
    private boolean testSolenoidDemand;

    // Climber homing state variables
    // Homing is done by sending the Climber to a negative position
    // While watching for the climber encoder to stop changing for a sufficient
    // amount of time
    private final double midArmMovementThreshhold = 50; // encoder movements below this threshhold are considered
                                                        // stopped
    private final double midArmNonMovementDuration = .1; // reading below threshhold encoder reads for this long is
                                                           // considered stopped
    private final double midArmHomingDemand = -0.4;
    private double midArmNonMovementTimeout; // timestamp of when low readings are sufficient

    private final double slappyMovementThreshhold = 50; // encoder movements below this threshhold are considered
                                                        // stopped
    private final double slappyNonMovementDuration = 0.5; // reading below threshhold encoder reads for this long is
                                                           // considered stopped
    private final double slappyHomingDemand = -0.4;
    private double slappyNonMovementTimeout; // timestamp of when low readings are sufficient

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
        mFXMidArm = TalonFXFactory.createDefaultTalon(Ports.LEFT_CLIMBER);
        mFXSlappy = TalonFXFactory.createDefaultTalon(Ports.RIGHT_CLIMBER);
        mSolenoidDeploy = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.CLIMBER_DEPLOY);
        mSubsystemManager = SubsystemManager.getInstance(sClassName);
        configMotors();
    }

    private void configMotors() {

        // only one encoder is needed
        mFXMidArm.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, Constants.kLongCANTimeoutMs);

        mFXMidArm.setControlFramePeriod(ControlFrame.Control_3_General, 18);
        mFXSlappy.setControlFramePeriod(ControlFrame.Control_3_General, 18);

        // mFXMidArm.configForwardSoftLimitEnable(true, Constants.kLongCANTimeoutMs);
        // mFXMidArm.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        // mFXSlappy.configForwardSoftLimitEnable(true, Constants.kLongCANTimeoutMs);
        // mFXSlappy.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mFXMidArm.setInverted(false);
        mFXSlappy.setInverted(false);

        mFXMidArm.setNeutralMode(NeutralMode.Brake);
        mFXSlappy.setNeutralMode(NeutralMode.Brake);

        // parameters are enable, current limit after triggering, trigger limit, time
        // allowed to exceed trigger limit before triggering
        // Current limit motors
        // mFXMidArm.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kClimberCurrentLimitLow, kClimberCurrentLimitLow, 0));
        // mFXSlappy.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kClimberCurrentLimitLow, kClimberCurrentLimitLow, 0));

        // TODO: all values must be tuned
        double kClosedError = 50;

        double config_arm_kP = 0.2;
        double config_arm_kI = 0;
        double config_arm_kD = 0;
        double config_arm_kF = 0;
        double armIntegralZone = 0;
        double kClosedRamp = 0; // this makes a difference in Magic and Position modes 
                                // but when using magic i prefer to use the Magic config values
                                // so this should only be nonzero when using Position mode

        double kArmCruiseVelocity = 10000; 
        double kArmAcceleration = 10000;
        int kArmCurveStrength = 0; // 0 is trapizoidal, larger values make the path more soft (curved) at velocity changes

        double config_slappy_kP = 0.05;
        double config_slappy_kI = 0;
        double config_slappy_kD = 0;
        double config_slappy_kF = 0;
        double slappyIntegralZone = 0;

        double kSlappyCruiseVelocity = 10000; 
        double kSlappyAcceleration = 10000;
        int kSlappyCurveStrength = 0; // 0 is trapizoidal, larger values make the path more soft (curved) at velocity changes

        mFXMidArm.config_kP(0, config_arm_kP, Constants.kLongCANTimeoutMs);
        mFXMidArm.config_kI(0, config_arm_kI, Constants.kLongCANTimeoutMs);
        mFXMidArm.config_kD(0, config_arm_kD, Constants.kLongCANTimeoutMs);
        mFXMidArm.config_kF(0, config_arm_kF, Constants.kLongCANTimeoutMs);
        mFXMidArm.config_IntegralZone(0, armIntegralZone, Constants.kLongCANTimeoutMs);
        mFXMidArm.configClosedloopRamp(kClosedRamp, Constants.kLongCANTimeoutMs);
        mFXMidArm.configAllowableClosedloopError(0, kClosedError, Constants.kLongCANTimeoutMs);

        mFXMidArm.configMotionCruiseVelocity(kArmCruiseVelocity); // 10000 ticks/second
        mFXMidArm.configMotionAcceleration(kArmAcceleration); // 2500 ticks/(sec^2)
        mFXMidArm.configMotionSCurveStrength(kArmCurveStrength); // trapizoidal curve
        
        mFXSlappy.config_kP(0, config_slappy_kP, Constants.kLongCANTimeoutMs);
        mFXSlappy.config_kI(0, config_slappy_kI, Constants.kLongCANTimeoutMs);
        mFXSlappy.config_kD(0, config_slappy_kD, Constants.kLongCANTimeoutMs);
        mFXSlappy.config_kF(0, config_slappy_kF, Constants.kLongCANTimeoutMs);
        mFXSlappy.config_IntegralZone(0, slappyIntegralZone, Constants.kLongCANTimeoutMs);
        mFXSlappy.configClosedloopRamp(kClosedRamp, Constants.kLongCANTimeoutMs);
        mFXSlappy.configAllowableClosedloopError(0, kClosedError, Constants.kLongCANTimeoutMs);

        mFXSlappy.configMotionCruiseVelocity(kSlappyCruiseVelocity); // 10000 ticks/second
        mFXSlappy.configMotionAcceleration(kSlappyAcceleration); // 2500 ticks/(sec^2)
        mFXSlappy.configMotionSCurveStrength(kSlappyCurveStrength); // trapizoidal curve
        
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
            preClimbComplete = false;
            midArmHomingComplete = false;
            slappyHomingComplete = false;
            midArmJustFinishedHoming = false;

            mPeriodicIO.solenoidDemand = SolenoidState.LOCK;
            if (mSolenoidDeploy.get()){
                mSolenoidState = SolenoidState.RELEASE;
            }
            else{
                mSolenoidState = SolenoidState.LOCK;
            }

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
            masterConfig(0, true, 0, true, 0, false, 0, false, kStatusFramePeriodDormant, kControlFrameDormant, mPeriodicIO.mDefaultSchedDelta);
            mPeriodicIO.midArmDemand = 0;
            mPeriodicIO.midArmControlMode = ControlMode.PercentOutput;
            mPeriodicIO.slappyDemand = 0;
            mPeriodicIO.slappyControlMode = ControlMode.PercentOutput;
    
        }

        return defaultStateTransfer();
    }

    private SystemState handleHolding() {
        if (mStateChanged) {
            masterConfig(0, true, 0, true, 0, false, 0, false, kStatusFramePeriodActive, kControlFrameActive, mPeriodicIO.mDefaultSchedDelta);
            mPeriodicIO.midArmDemand = mPeriodicIO.midArmPosition;
            mPeriodicIO.midArmControlMode = ControlMode.Position;
            mPeriodicIO.slappyDemand = 0;
            mPeriodicIO.slappyControlMode = ControlMode.PercentOutput;
        }

        return defaultStateTransfer();
    }

    private SystemState handleHoming() {
        if (mStateChanged) {
            masterConfig(kMidArmCurrentLimitLow, true,
                         kSlappyCurrentLimitLow, true,
                         Double.NaN, false, //TODO: Check what soft limits need to be
                         Double.NaN, false, //TODO: Check what soft limits need to be
                         kStatusFramePeriodActive, kControlFrameActive, mPeriodicIO.mDefaultSchedDelta);
            mPeriodicIO.solenoidDemand = SolenoidState.RELEASE;
        }

        if (!midArmHomingComplete) {
            homeArms();
        } else if (!slappyHomingComplete) {
            homeSlappySticks();
        }

        if (mWantedState != WantedState.HOME){
            midArmHomingComplete = false;
            slappyHomingComplete = false;
        }
        return defaultStateTransfer();
    }

    private void homeArms() {
        double now = Timer.getFPGATimestamp();

        if (mStateChanged) {
            midArmNonMovementTimeout = now + midArmNonMovementDuration;
            mPeriodicIO.midArmDemand = midArmHomingDemand;
            mPeriodicIO.midArmControlMode = ControlMode.PercentOutput;
        }

        double distance = Math.abs(mPeriodicIO.midArmPosition - mPeriodicIO.lastClimberPosition);
        if (distance > midArmMovementThreshhold) {
            midArmNonMovementTimeout = now + midArmNonMovementDuration;
        }

        if (now > midArmNonMovementTimeout) {

            System.out.println("Mid Arm Homing Sequence Complete");
            mFXMidArm.setSelectedSensorPosition(MidArmPosition.HOME.get()); //kHomingZeroAdjustment);
            mPeriodicIO.midArmPosition = MidArmPosition.HOME.get();
            mPeriodicIO.midArmDemand = 0.0;

            midArmJustFinishedHoming = true;
            midArmHomingComplete = true;
            mPeriodicIO.solenoidDemand = SolenoidState.LOCK;

        }
    }

    private void homeSlappySticks() {
        double now = Timer.getFPGATimestamp();

        if (midArmJustFinishedHoming) {
            midArmJustFinishedHoming = false;
            slappyNonMovementTimeout = now + slappyNonMovementDuration;
            mPeriodicIO.slappyDemand = slappyHomingDemand;
            mPeriodicIO.slappyControlMode = ControlMode.PercentOutput;
        }

        double distance = Math.abs(mPeriodicIO.slappyPosition - mPeriodicIO.lastSlappyPosition);
        if (distance > slappyMovementThreshhold) {
            slappyNonMovementTimeout = now + slappyNonMovementDuration;
        }

        if (now > slappyNonMovementTimeout) {

            System.out.println("Slappy Sticks Homing Sequence Complete");
            mFXSlappy.setSelectedSensorPosition(SlappyPosition.HOME.get());
            mPeriodicIO.slappyPosition = SlappyPosition.HOME.get();
            mPeriodicIO.slappyDemand = 0.0;

            slappyHomingComplete = true;

        }
    }

    private SystemState handlePreclimbing() {
        double now = Timer.getFPGATimestamp();

        if(mStateChanged) {
            masterConfig(kMidArmCurrentLimitHigh, true,
                         kSlappyCurrentLimitHigh, true,
                         Double.NaN, false, //TODO: Check what soft limits need to be
                         Double.NaN, false, //TODO: Check what soft limits need to be
                         kStatusFramePeriodActive, kControlFrameActive, mPeriodicIO.mDefaultSchedDelta);
            mPeriodicIO.solenoidDemand = SolenoidState.RELEASE;
            mPeriodicIO.midArmDemand = mPeriodicIO.midArmPosition;
            mPeriodicIO.midArmControlMode = ControlMode.MotionMagic;
            solenoidTimeout = now + kSolenoidDuration;
        }

        if (now > solenoidTimeout){
            mPeriodicIO.midArmDemand = MidArmPosition.MIDBAR.get();
            mPeriodicIO.midArmControlMode = ControlMode.MotionMagic;
            solenoidTimeout += 360000; // set to big number so only come in here once
        }

        if (Math.abs(mPeriodicIO.midArmDemand - mPeriodicIO.midArmPosition) <= midBarPosTolerance){
            preClimbComplete = true;
        }

        return defaultStateTransfer();
    }

    private SystemState handleClimbing_1_Lift(){
        if(mStateChanged) {
            masterConfig(kMidArmCurrentLimitHigh, true,
                         kSlappyCurrentLimitHigh, true,
                         Double.NaN, false, //TODO: Check what soft limits need to be
                         Double.NaN, false, //TODO: Check what soft limits need to be
                         kStatusFramePeriodActive, kControlFrameActive, mPeriodicIO.mDefaultSchedDelta);
            mPeriodicIO.midArmDemand = MidArmPosition.MIDDLE.get();
            mPeriodicIO.midArmControlMode = ControlMode.MotionMagic;
            stageOneComplete = false;
        }

        if (Math.abs(mPeriodicIO.midArmPosition - mPeriodicIO.midArmDemand) < midBarPosTolerance){
            stageOneComplete = true;
        }

        if (mWantedState != WantedState.CLIMB_1_LIFT) {
            stageOneComplete = false;
        }
        return defaultStateTransfer();
    }

    private SystemState handleClimbing_2_RotateUp(){
        if (mStateChanged) {
            masterConfig(kMidArmCurrentLimitHigh, true,
                         kSlappyCurrentLimitHigh, true,
                         Double.NaN, false, //TODO: Check what soft limits need to be
                         Double.NaN, false, //TODO: Check what soft limits need to be
                         kStatusFramePeriodActive, kControlFrameActive, mPeriodicIO.mDefaultSchedDelta);
            mPeriodicIO.slappyDemand = SlappyPosition.MID.get();
            mPeriodicIO.slappyControlMode = ControlMode.MotionMagic;
            stageTwoComplete = false; // redundant
        }

        if (Math.abs(mPeriodicIO.slappyPosition - mPeriodicIO.slappyDemand) < slappyPosTolerance) {
            stageTwoComplete = true;
        }

        if (mWantedState != WantedState.CLIMB_2_ROTATE_UP) {
            stageTwoComplete = false;
        }
        return defaultStateTransfer();
    }

    private SystemState handleClimbing_3_LiftMore(){
        if(mStateChanged) {
            masterConfig(kMidArmCurrentLimitHigh, true,
                         kSlappyCurrentLimitHigh, true,
                         Double.NaN, false, //TODO: Check what soft limits need to be
                         Double.NaN, false, //TODO: Check what soft limits need to be
                         kStatusFramePeriodActive, kControlFrameActive, mPeriodicIO.mDefaultSchedDelta);
            mPeriodicIO.slappyDemand = SlappyPosition.MAX.get();
            mPeriodicIO.midArmDemand = MidArmPosition.DOWN.get();
            mPeriodicIO.midArmControlMode = ControlMode.Position;
            stageOneComplete = false;
        }
        System.out.println(mPeriodicIO.midArmPosition);
        if (Math.abs(mPeriodicIO.midArmPosition - mPeriodicIO.midArmDemand) < midBarPosTolerance){
            stageThreeComplete = true;
        }

        if (mWantedState != WantedState.CLIMB_3_LIFT_MORE) {
            stageThreeComplete = false;
        }
        return defaultStateTransfer();
    }

    private SystemState handleClimbing_4_EngageTrav(){
        if (mStateChanged) {
            masterConfig(kMidArmCurrentLimitHigh, true,
                         kSlappyCurrentLimitHigh, true,
                         Double.NaN, false, //TODO: Check what soft limits need to be
                         Double.NaN, false, //TODO: Check what soft limits need to be
                         kStatusFramePeriodActive, kControlFrameActive, mPeriodicIO.mDefaultSchedDelta);
            mPeriodicIO.slappyDemand = SlappyPosition.DOWN.get();
            mPeriodicIO.slappyControlMode = ControlMode.MotionMagic;
            stageFourComplete = false; // redundant
        }

        if (Math.abs(mPeriodicIO.slappyPosition - mPeriodicIO.slappyDemand) < slappyPosTolerance) {
            stageFourComplete = true;
        }

        if (mWantedState != WantedState.CLIMB_4_ENGAGE_TRAV) {
            stageFourComplete = false;
        }
        return defaultStateTransfer();

    }

    private SystemState handleClimbing_5_ReleaseMid(){
        if(mStateChanged) {
            masterConfig(kMidArmCurrentLimitHigh, true,
                         kSlappyCurrentLimitHigh, true,
                         Double.NaN, false, //TODO: Check what soft limits need to be
                         Double.NaN, false, //TODO: Check what soft limits need to be
                         kStatusFramePeriodActive, kControlFrameActive, mPeriodicIO.mDefaultSchedDelta);
            mPeriodicIO.midArmDemand = MidArmPosition.MIDDLE.get();
            mPeriodicIO.midArmControlMode = ControlMode.MotionMagic;
            stageFiveComplete = false;
        }

        if (Math.abs(mPeriodicIO.midArmPosition - mPeriodicIO.midArmDemand) < midBarPosTolerance){
            stageFiveComplete = true;
        }

        if (mWantedState != WantedState.CLIMB_5_RELEASE_MID) {
            stageFiveComplete = false;
        }
        return defaultStateTransfer();
    }

    private SystemState handleTesting(){
        if(mStateChanged) {
            masterConfig(80, true, kSlappyCurrentLimitHigh, true, 0, false, 0, false, 
                         kControlFrameActive, kStatusFramePeriodActive, mPeriodicIO.mDefaultSchedDelta);
            testMidArmDemand = 0;
            testSlappyDemand = 0;
            mPeriodicIO.midArmControlMode = ControlMode.PercentOutput;
            mPeriodicIO.slappyControlMode = ControlMode.PercentOutput;

            // mPeriodicIO.slappyControlMode = ControlMode.MotionMagic;
            // testSlappyDemand = mPeriodicIO.slappyPosition;
        }

        if (testMidArmDemand != mPeriodicIO.midArmDemand){
            mPeriodicIO.midArmDemand = testMidArmDemand;
        }

        if (testSlappyDemand != mPeriodicIO.slappyDemand){
            mPeriodicIO.slappyDemand = testSlappyDemand;
        }

        if (testSolenoidDemand) {
            mPeriodicIO.solenoidDemand = SolenoidState.RELEASE;
        } else {
            mPeriodicIO.solenoidDemand = SolenoidState.LOCK;
        }
        System.out.println(mPeriodicIO.midArmPosition);

        return defaultStateTransfer();
    }

    private void masterConfig(double midArmStatorLimit, boolean midArmStatorEnable,
                              double slappyStatorLimit, boolean slappyStatorEnable, 
                              double softLimitFwd, boolean softLimitFwdEnable, 
                              double softLimitRev, boolean softLimitRevEnable, 
                              double statusFramePeriod, double controlFramePeriod, double whenRun){
        if (!Double.isNaN(midArmStatorLimit)){
            mFXMidArm.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(midArmStatorEnable, midArmStatorLimit, midArmStatorLimit, 0));
        }

        if (!Double.isNaN(slappyStatorLimit)){
            mFXSlappy.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(slappyStatorEnable, slappyStatorLimit, slappyStatorLimit, 0));
        }

        if (!Double.isNaN(softLimitFwd)){
            mFXMidArm.configForwardSoftLimitThreshold(softLimitFwd, Constants.kLongCANTimeoutMs);
            mFXSlappy.configForwardSoftLimitThreshold(softLimitFwd, Constants.kLongCANTimeoutMs);

            mFXMidArm.configForwardSoftLimitEnable(softLimitFwdEnable, Constants.kLongCANTimeoutMs);    
            mFXSlappy.configForwardSoftLimitEnable(softLimitFwdEnable, Constants.kLongCANTimeoutMs);
        }

        if (!Double.isNaN(softLimitRev)){
            mFXMidArm.configReverseSoftLimitThreshold(softLimitFwd, Constants.kLongCANTimeoutMs);
            mFXSlappy.configReverseSoftLimitThreshold(softLimitFwd, Constants.kLongCANTimeoutMs);
            
            mFXMidArm.configReverseSoftLimitEnable(softLimitRevEnable, Constants.kLongCANTimeoutMs);
            mFXSlappy.configReverseSoftLimitEnable(softLimitRevEnable, Constants.kLongCANTimeoutMs);
        }
        if (!Double.isNaN(statusFramePeriod)){
            mFXMidArm.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, (int)statusFramePeriod, Constants.kLongCANTimeoutMs);
        }
        if (!Double.isNaN(controlFramePeriod)){
            mFXMidArm.setControlFramePeriod(ControlFrame.Control_3_General, (int)controlFramePeriod);
            mFXSlappy.setControlFramePeriod(ControlFrame.Control_3_General, (int)controlFramePeriod);
        }
        if (!Double.isNaN(whenRun)){
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
            case PRECLIMB:
                return preClimbComplete;
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
            case HOME:
                return midArmHomingComplete && slappyHomingComplete;
            default:
                System.out.println("Uh oh something is not right");
                return false;
        }
    }

    public void setClimberTestDemand(double newMidArmDemand, double newSlappyDemand){
        testMidArmDemand = newMidArmDemand;
        testSlappyDemand = newSlappyDemand;
    }

    public void setSolenoidState(boolean state) {
        testSolenoidDemand = state;
    }

    @Override
    public void readPeriodicInputs() {
        double now = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart = now;

        mPeriodicIO.lastClimberPosition = mPeriodicIO.midArmPosition;
        mPeriodicIO.midArmPosition = mFXMidArm.getSelectedSensorPosition();

        mPeriodicIO.lastSlappyPosition = mPeriodicIO.slappyPosition;
        mPeriodicIO.slappyPosition = mFXSlappy.getSelectedSensorPosition();
    }

    @Override
    public void writePeriodicOutputs() {
        if (mSolenoidState != mPeriodicIO.solenoidDemand) {
            mSolenoidState = mPeriodicIO.solenoidDemand;
            mSolenoidDeploy.set(mPeriodicIO.solenoidDemand.get());
            System.out.println("climber solenoid "+mSolenoidState.toString());
        }

        mFXMidArm.set(mPeriodicIO.midArmControlMode, mPeriodicIO.midArmDemand);
        mFXSlappy.set(mPeriodicIO.slappyControlMode, mPeriodicIO.slappyDemand);
    }

    @Override
    public void stop() {
        mFXMidArm.set(ControlMode.PercentOutput, 0);
        mFXSlappy.set(ControlMode.PercentOutput, 0);
        // should solenoid be set
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
                sClassName+".climberPosition,"+
                sClassName+".climberDemand,"+
                sClassName+".climberControlMode,"+
                sClassName+".slappyDemand";
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
                mPeriodicIO.midArmPosition+","+
                mPeriodicIO.midArmDemand+","+
                mPeriodicIO.midArmControlMode+","+
                mPeriodicIO.solenoidDemand;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("midArmPos", mPeriodicIO.midArmPosition);
        SmartDashboard.putNumber("slappyPos", mPeriodicIO.slappyPosition);
        SmartDashboard.putNumber("midArmCurrent", mFXMidArm.getStatorCurrent());
        SmartDashboard.putNumber("slappyCurrent", mFXSlappy.getStatorCurrent());
    }

    public static class PeriodicIO {
        // Logging
        public final int mDefaultSchedDelta = 20; // axis updated every 20 msec
        public int schedDeltaDesired;
        public double schedDeltaActual;
        public double lastSchedStart;

        // Inputs
        public double midArmPosition;
        public double slappyPosition;

        // Outputs
        public double midArmDemand;
        public double slappyDemand;
        public ControlMode midArmControlMode;
        public ControlMode slappyControlMode;
        public SolenoidState solenoidDemand;

        // Other
        public double lastClimberPosition;
        public double lastSlappyPosition;

    }

}
