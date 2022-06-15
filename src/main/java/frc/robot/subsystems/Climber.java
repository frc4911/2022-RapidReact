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
import libraries.cyberlib.control.FramePeriodSwitch;

public class Climber extends Subsystem {

    // Hardware
    private final TalonFX mFXMidArm, mFXSlappy;
    private final Solenoid mSolenoidDeploy;

    // Constants
    private final int kSchedActive = 20;
    private final int kSchedDormant = 100;

    private final double kMidArmCurrentLimitLow = 10;
    private final double kMidArmCurrentLimitHigh = 80;
    private final double kSlappyCurrentLimitLow = 10;
    private final double kSlappyCurrentLimitHigh = 70;

    private final double kClosedError = 50;

    private final double kConfig_arm_kP = 0.3;
    private final double kConfig_arm_kI = 0;
    private final double kConfig_arm_kD = 0;
    private final double kConfig_arm_kF = 0;
    private final double kArmIntegralZone = 0;
    private final double kClosedRamp = 0; // this makes a difference in Magic and Position modes 
                                          // but when using magic i prefer to use the Magic config values
                                          // so this should only be nonzero when using Position mode
 
    private final double kArmCruiseVelocity = 16400; 
    private final double kArmAcceleration = 10000;
    private final int kArmCurveStrength = 0; // 0 is trapizoidal, larger values make the path more soft (curved) at velocity changes
 
    private final double kConfig_slappy_kP = 0.1; //.05
    private final double kConfig_slappy_kI = 0;
    private final double kConfig_slappy_kD = 0;
    private final double kConfig_slappy_kF = 0;
    private final double kSlappyIntegralZone = 0;

    private final double kSlappyCruiseVelocity = 16000; 
    private final double kSlappyAcceleration = 6400;
    private final int kSlappyCurveStrength = 0; // 0 is trapizoidal, larger values make the path more soft (curved) at velocity changes

    // Subsystem States
    public enum SolenoidState {
        RELEASE(true), // Release passive traversal hooks
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
        ASSESSING,
        DISABLING,
        HOLDING,
        HOMING,
        CLIMBING_1_LIFT_MORE,
        CLIMBING_2_ENGAGE_TRAV,
        CLIMBING_3_RELEASE_MID,
        PRECLIMBING,
        MANUAL_CONTROLLING
    }

    public enum WantedState {
        ASSESS,
        DISABLE,
        HOLD,
        HOME,
        CLIMB_1_LIFT_MORE,
        CLIMB_2_ENGAGE_TRAV,
        CLIMB_3_RELEASE_MID,
        PRECLIMB,
        MANUAL_CONTROL
    }

    public enum MidArmPosition {
        MIDBAR(369000), //345000: mid bar position with belt nu climber (old)
        MIDDLE(92500),
        DOWN(-6000), // -14900
        HOME(-400),
        RELEASE(250000),
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
        MAX(165000),
        MID(140000), //165000 physical max
        DOWN(-2600),
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

    private boolean assessingComplete;
    private boolean preClimbComplete;
    private boolean stageOneComplete;
    private boolean stageTwoComplete;
    private boolean stageThreeComplete;
    private int currentStage;

    private boolean midArmHomingComplete;
    private boolean slappyHomingComplete;
    private boolean slappyJustFinishedHoming;

    private double assessingMidArmStartPosition;
    private double assessingSlappyStartPosition;
    private double assessingStopTime;
    private final double kAssessingTimeout = 250;
    private boolean assessingReturn;
    
    private SystemState mSystemState;
    private WantedState mWantedState;
    private boolean mStateChanged;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private LatchedBoolean mLB_SystemStateChange = new LatchedBoolean();
    private SolenoidState mSolenoidState;

    private final double midBarPosTolerance = 2000;
    private final double slappyPosTolerance = 2000;
    private final double kSolenoidDuration = .25;
    private double solenoidTimeout = 0;  

    private double testMidArmDemand;
    private double testSlappyDemand;
    private boolean testSolenoidDemand;
    @SuppressWarnings("unused")
    private boolean mPrintIt = false;

    // Climber homing state variables
    // Homing is done by sending the Climber to a negative position
    // While watching for the climber encoder to stop changing for a sufficient
    // amount of time
    private final double midArmMovementThreshhold = 50; // encoder movements below this threshhold are considered
                                                        // stopped
    private final double midArmNonMovementDuration = .1; // reading below threshhold encoder reads for this many
                                                         // seconds is considered stopped
    private final double midArmHomingDemand = -0.4; // Percent output for motor
    private double midArmNonMovementTimeout; // timestamp of when low readings are sufficient

    private final double slappyMovementThreshhold = 50; // encoder movements below this threshhold are considered
                                                        // stopped
    private final double slappyNonMovementDuration = 0.5; // reading below threshhold encoder reads for this many
                                                           // seconds is considered stopped
    private final double slappyHomingDemand = -0.4; // Percent output of motor
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
        mFXMidArm = TalonFXFactory.createDefaultTalon(Ports.LEFT_CLIMBER, Constants.kCanivoreName);
        mFXSlappy = TalonFXFactory.createDefaultTalon(Ports.RIGHT_CLIMBER, Constants.kCanivoreName);
        mSolenoidDeploy = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.CLIMBER_DEPLOY);
        mSubsystemManager = SubsystemManager.getInstance(sClassName);
        configMotors();
    }

    private void configMotors() {
        commonMotorConfig(mFXMidArm, "Mid Arm");
        commonMotorConfig(mFXSlappy, "Slappy");

        mFXMidArm.config_kP(0, kConfig_arm_kP, Constants.kLongCANTimeoutMs);
        mFXMidArm.config_kI(0, kConfig_arm_kI, Constants.kLongCANTimeoutMs);
        mFXMidArm.config_kD(0, kConfig_arm_kD, Constants.kLongCANTimeoutMs);
        mFXMidArm.config_kF(0, kConfig_arm_kF, Constants.kLongCANTimeoutMs);
        mFXMidArm.config_IntegralZone(0, kArmIntegralZone, Constants.kLongCANTimeoutMs);
        mFXMidArm.configClosedloopRamp(kClosedRamp, Constants.kLongCANTimeoutMs);
        mFXMidArm.configAllowableClosedloopError(0, kClosedError, Constants.kLongCANTimeoutMs);

        mFXMidArm.configMotionCruiseVelocity(kArmCruiseVelocity); // 10000 ticks/second
        mFXMidArm.configMotionAcceleration(kArmAcceleration); // 2500 ticks/(sec^2)
        mFXMidArm.configMotionSCurveStrength(kArmCurveStrength); // trapizoidal curve
        
        mFXSlappy.config_kP(0, kConfig_slappy_kP, Constants.kLongCANTimeoutMs);
        mFXSlappy.config_kI(0, kConfig_slappy_kI, Constants.kLongCANTimeoutMs);
        mFXSlappy.config_kD(0, kConfig_slappy_kD, Constants.kLongCANTimeoutMs);
        mFXSlappy.config_kF(0, kConfig_slappy_kF, Constants.kLongCANTimeoutMs);
        mFXSlappy.config_IntegralZone(0, kSlappyIntegralZone, Constants.kLongCANTimeoutMs);
        mFXSlappy.configClosedloopRamp(kClosedRamp, Constants.kLongCANTimeoutMs);
        mFXSlappy.configAllowableClosedloopError(0, kClosedError, Constants.kLongCANTimeoutMs);

        mFXSlappy.configMotionCruiseVelocity(kSlappyCruiseVelocity); // 10000 ticks/second
        mFXSlappy.configMotionAcceleration(kSlappyAcceleration); // 2500 ticks/(sec^2)
        mFXSlappy.configMotionSCurveStrength(kSlappyCurveStrength); // trapizoidal curve
        
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

        FramePeriodSwitch.setNeutralModeVolatile(motor, NeutralMode.Brake);
    }

    @Override
    public void onStart(Phase phase) {
        synchronized (Climber.this) {
            mStateChanged = true;
            switch (phase) {
                case DISABLED:
                    mSystemState = SystemState.DISABLING;
                    mWantedState = WantedState.DISABLE;
                    break;
                case TEST:
                    mSystemState = SystemState.MANUAL_CONTROLLING;
                    mWantedState = WantedState.MANUAL_CONTROL;
                    currentStage = 0;
                    break;
                case AUTONOMOUS:
                case TELEOP:
                    mSystemState = SystemState.HOLDING;
                    mWantedState = WantedState.HOLD;
                    currentStage = 0;
                    break;
            }
            mPeriodicIO.schedDeltaDesired = kSchedActive;
            assessingComplete = false;
            preClimbComplete = false;
            stageOneComplete = false;
            stageTwoComplete = false;
            stageThreeComplete = false;
            midArmHomingComplete = false;
            slappyHomingComplete = false;
            slappyJustFinishedHoming = false;

            mPeriodicIO.solenoidDemand = SolenoidState.LOCK;
            if (mSolenoidDeploy.get()){
                mSolenoidState = SolenoidState.RELEASE;
            }
            else{
                mSolenoidState = SolenoidState.LOCK;
            }
            mLB_SystemStateChange.update(false); // reset
            System.out.println(sClassName + " state " + mSystemState);
            stop(); // put into a known state
        }
    }

    @Override
    public void onLoop(double timestamp) {
        synchronized (Climber.this) {
            do {
                SystemState newState = null;
                switch (mSystemState) {
                    case DISABLING:
                        newState = handleDisabling();
                        break;
                    case PRECLIMBING:
                        newState = handlePreclimbing();
                        break;
                    case CLIMBING_1_LIFT_MORE:
                        newState = handleClimbing_1_LiftMore();
                        break;
                    case CLIMBING_2_ENGAGE_TRAV:
                        newState = handleClimbing_2_EngageTrav();
                        break;
                    case CLIMBING_3_RELEASE_MID:
                        newState = handleClimbing_3_ReleaseMid();
                        break;
                    case MANUAL_CONTROLLING:
                        newState = handleManualControlling();
                        break;
                    case HOMING:
                        newState = handleHoming();
                        break;
                    case HOLDING:
                        newState = handleHolding();
                        break;
                    case ASSESSING:
                        newState = handleAssessing();
                        break;
                    // default: leave commented so compiler will identify missing cases
                    //     break;
                }

                if (newState != mSystemState) {
                    System.out.println(
                            sClassName + " state " + mSystemState + " to " + newState + " (" + timestamp + ")");
                    mSystemState = newState;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            } while (mLB_SystemStateChange.update(mStateChanged));
        }
    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case ASSESS:
                return SystemState.ASSESSING;
            case HOME:
                return SystemState.HOMING;
            case DISABLE:
                return SystemState.DISABLING;
            case CLIMB_1_LIFT_MORE:
                return SystemState.CLIMBING_1_LIFT_MORE;
            case CLIMB_2_ENGAGE_TRAV:
                return SystemState.CLIMBING_2_ENGAGE_TRAV;
            case CLIMB_3_RELEASE_MID:
                return SystemState.CLIMBING_3_RELEASE_MID;
            case PRECLIMB:
                return SystemState.PRECLIMBING;
            case MANUAL_CONTROL:
                return SystemState.MANUAL_CONTROLLING;
            case HOLD:
                    return SystemState.HOLDING;
        }
        return null;
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
            if (currentStage == 2 || currentStage == 3) {
                // Activate stator current limits on disable to allow mid arm to slide off the mid bar for a traversal climb
                masterConfig(0, true, 0, true, kSchedDormant);
            } else {
                // Deactivate stator current limits to lock the mid arm if disabled, allowing a mid climb if needed
                masterConfig(0, false, 0, true, kSchedDormant);
            }
            mPeriodicIO.midArmDemand = 0;
            mPeriodicIO.midArmControlMode = ControlMode.PercentOutput;
            mPeriodicIO.slappyDemand = 0;
            mPeriodicIO.slappyControlMode = ControlMode.PercentOutput;
    
        }

        return defaultStateTransfer();
    }

    private SystemState handleAssessing() {
        double now = Timer.getFPGATimestamp();
        if (mStateChanged) {
            masterConfig(0, true, 0, true, kSchedActive);
            mPeriodicIO.midArmDemand = .1;
            mPeriodicIO.midArmControlMode = ControlMode.PercentOutput;
            mPeriodicIO.slappyDemand = .1;
            mPeriodicIO.slappyControlMode = ControlMode.PercentOutput;
            assessingMidArmStartPosition = mPeriodicIO.midArmPosition;
            assessingSlappyStartPosition = mPeriodicIO.slappyPosition;
            assessingStopTime = now+kAssessingTimeout;
            assessingComplete = false;
            assessingReturn = false;
        }

        if (!assessingComplete){
            // move in one direction for .25 seconds and look for encoder changes
            if (!assessingReturn){
                if (now >= assessingStopTime){
                    if (mPeriodicIO.midArmPosition != assessingMidArmStartPosition){
                        System.out.println("ASSESSING: climber Mid Arm motor functioning");
                    }
                    else{
                        System.out.println("ASSESSING: climber Mid Arm motor DID NOT DETECT MOVEMENT");
                    }
                    if (mPeriodicIO.slappyPosition != assessingSlappyStartPosition){
                        System.out.println("ASSESSING: climber Slappy motor functioning");
                    }
                    else{
                        System.out.println("ASSESSING: climber Slappy motor DID NOT DETECT MOVEMENT");
                    }
                    mPeriodicIO.midArmDemand = -mPeriodicIO.midArmDemand;
                    mPeriodicIO.slappyDemand = -mPeriodicIO.slappyDemand;
                    assessingStopTime = now+kAssessingTimeout;
                    assessingReturn = true;
                }    
            }
            else{
                // move back for .25 seconds then stop
                if (now >= assessingStopTime){
                    mPeriodicIO.midArmDemand = 0;
                    mPeriodicIO.slappyDemand = 0;
                    assessingComplete = true;
                }
            }
        }

        if (mWantedState != WantedState.ASSESS){
            assessingComplete = false;
        }

        return defaultStateTransfer();
    }

    private SystemState handleHolding() {
        if (mStateChanged) {
            if(currentStage >= 1){
                // If auto climb has started, change to active configs and deltadesires
                    masterConfig(0, false, 0, false, kSchedActive);
            } else {
                // If climb has not been started
                    masterConfig(0, false, 0, false, kSchedDormant);
            }
            mPeriodicIO.midArmDemand = mPeriodicIO.midArmPosition;
            mPeriodicIO.midArmControlMode = ControlMode.Position;
            mPeriodicIO.slappyDemand = 0;
            mPeriodicIO.slappyControlMode = ControlMode.PercentOutput;
        }

        return defaultStateTransfer();
    }

    private SystemState handlePreclimbing() {
        double now = Timer.getFPGATimestamp();

        if(mStateChanged) {
        masterConfig(kMidArmCurrentLimitHigh, true,
                         kSlappyCurrentLimitHigh, true,
                         kSchedActive);
            mPeriodicIO.solenoidDemand = SolenoidState.RELEASE;
            mPeriodicIO.midArmDemand = mPeriodicIO.midArmPosition;
            mPeriodicIO.midArmControlMode = ControlMode.Position;
            solenoidTimeout = now + kSolenoidDuration;
        }

        if (now > solenoidTimeout){
            mPeriodicIO.midArmDemand = MidArmPosition.MIDBAR.get();
            mPeriodicIO.midArmControlMode = ControlMode.Position;
            solenoidTimeout += 360000; // set to big number so only come in here once
        }

        if (Math.abs(mPeriodicIO.midArmDemand - mPeriodicIO.midArmPosition) <= midBarPosTolerance){
            preClimbComplete = true;
        }

        if (mWantedState != WantedState.PRECLIMB){
            preClimbComplete = false;
        }

        return defaultStateTransfer();
    }

    private double slappyTravelDist = SlappyPosition.MAX.get()-SlappyPosition.HOME.get();
    private double midArmTravelDist = MidArmPosition.MIDBAR.get()-MidArmPosition.DOWN.get();

    // Tracks the progress of the slappy sticks' position as the mid bar pulls the robot upward
    // 0-100 percent of the slappy's total travel distance
    // Each element represents 10% of the mid arm's travel, starting at 0%
    private double[] slappyProgressBar = {0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.5, 1.0, 1.0, 1.0, 1.0};
    private SystemState handleClimbing_1_LiftMore(){
        if(mStateChanged) {
            mPrintIt = true;
            masterConfig(kMidArmCurrentLimitHigh, true,
                         kSlappyCurrentLimitHigh, true,
                         kSchedActive);
            configPIDF(mFXSlappy,
                       kConfig_slappy_kP, //kP
                       kConfig_slappy_kI, //kI
                       kConfig_slappy_kD, //kD
                       kConfig_slappy_kF, //kF
                       kSlappyCruiseVelocity, kSlappyAcceleration); //cruise veloctiy, cruise acceleration
            mPeriodicIO.slappyDemand = SlappyPosition.HOME.get()+slappyTravelDist*slappyProgressBar[0];  // slappy is already down so don't move yet
            mPeriodicIO.slappyControlMode = ControlMode.Position;
            mPeriodicIO.midArmDemand = MidArmPosition.DOWN.get();
            mPeriodicIO.midArmControlMode = ControlMode.Position;
            stageOneComplete = false;
            currentStage = 1;
        }

        // System.out.println("MIDARM: Pos "+ mPeriodicIO.midArmPosition + ", Demand " + mPeriodicIO.midArmDemand + ", Current "+ mPeriodicIO.midArmStatorCurrent 
        //                 + " SLAPPY: Pos " + mPeriodicIO.slappyPosition + ", Demand " + mPeriodicIO.slappyDemand + ", Current "+ mPeriodicIO.slappyStatorCurrent);
    
        double midArmPercent = (MidArmPosition.MIDBAR.get()-mPeriodicIO.midArmPosition)/midArmTravelDist;
        int slapIndex = (int)(((double)slappyProgressBar.length)*midArmPercent);
        if (slapIndex >= slappyProgressBar.length){ // Prevents throwing an indexOutOfBounds exception: mid arm will overdrive on high battery and crash without this
            slapIndex = slappyProgressBar.length-1;
        }
        mPeriodicIO.slappyDemand = SlappyPosition.HOME.get()+slappyTravelDist*slappyProgressBar[slapIndex];
        // System.out.println("midPos:"+mPeriodicIO.midArmPosition+" mid%:"+midArmPercent+" slapIndex:"+slapIndex+" slapDemand:"+mPeriodicIO.slappyDemand);

        // System.out.println(slappyHasMoved+","+slappyIsStopped+","+midArmHasMoved+","+midArmIsStopped);
        if (((Math.abs(mPeriodicIO.midArmPosition - mPeriodicIO.midArmDemand) < midBarPosTolerance) /*|| midArmIsStopped*/) && 
            ((Math.abs(mPeriodicIO.slappyPosition - mPeriodicIO.slappyDemand) < slappyPosTolerance) /*|| slappyIsStopped*/)){
            // System.out.println("Climber 3 stage is complete. midArmIsStopped:"+midArmIsStopped+" slappyIsStopped:"+slappyIsStopped);
            stageOneComplete = true;
        }
       
        if (mWantedState != WantedState.CLIMB_1_LIFT_MORE) {
            stageOneComplete = false;
            mPrintIt = false;
        }
        return defaultStateTransfer();
    }

    private SystemState handleClimbing_2_EngageTrav(){
        if (mStateChanged) {
            masterConfig(kMidArmCurrentLimitHigh, true,
                         kSlappyCurrentLimitHigh, true,
                         kSchedActive);
            configPIDF(mFXSlappy,
                       kConfig_slappy_kP, //kP
                       kConfig_slappy_kI, //kI
                       kConfig_slappy_kD, //kD
                       kConfig_slappy_kF, //kF
                       10000, 10000); //cruise veloctiy, cruise acceleration
            mPeriodicIO.slappyDemand = SlappyPosition.DOWN.get();
            mPeriodicIO.slappyControlMode = ControlMode.Position;
            stageTwoComplete = false; // redundant
            currentStage = 2;
        }

        // System.out.println("MIDARM: Pos "+ mPeriodicIO.midArmPosition + ", Demand " + mPeriodicIO.midArmDemand + ", Current "+ mPeriodicIO.midArmStatorCurrent 
        //                 + " SLAPPY: Pos " + mPeriodicIO.slappyPosition + ", Demand " + mPeriodicIO.slappyDemand + ", Current "+ mPeriodicIO.slappyStatorCurrent);
        
        if (Math.abs(mPeriodicIO.slappyPosition - mPeriodicIO.slappyDemand) < slappyPosTolerance) {
            stageTwoComplete = true;
        }

        if (mWantedState != WantedState.CLIMB_2_ENGAGE_TRAV) {
            stageTwoComplete = false;
        }
        return defaultStateTransfer();

    }

    private SystemState handleClimbing_3_ReleaseMid(){
        if(mStateChanged) {
            masterConfig(0, false,
                         0, false,
                         kSchedActive);
            mPeriodicIO.midArmDemand = MidArmPosition.RELEASE.get();
            mPeriodicIO.midArmControlMode = ControlMode.Position;
            stageThreeComplete = false;
            currentStage = 3;
        }

        // System.out.println("MIDARM: Pos "+ mPeriodicIO.midArmPosition + ", Demand " + mPeriodicIO.midArmDemand + ", Current "+ mPeriodicIO.midArmStatorCurrent 
        //                 + " SLAPPY: Pos " + mPeriodicIO.slappyPosition + ", Demand " + mPeriodicIO.slappyDemand + ", Current "+ mPeriodicIO.slappyStatorCurrent);
        
        if (Math.abs(mPeriodicIO.midArmPosition - mPeriodicIO.midArmDemand) < midBarPosTolerance){
            stageThreeComplete = true;
        }

        if (mWantedState != WantedState.CLIMB_3_RELEASE_MID) {
            stageThreeComplete = false;
            mPrintIt = false;
        }
        return defaultStateTransfer();
    }

    private SystemState handleManualControlling(){
        if(mStateChanged) {
            masterConfig(kMidArmCurrentLimitLow, true, kSlappyCurrentLimitLow, true, kSchedActive);
            testMidArmDemand = 0;
            testSlappyDemand = 0;
            mPeriodicIO.midArmControlMode = ControlMode.PercentOutput;
            mPeriodicIO.slappyControlMode = ControlMode.PercentOutput;
        }
        
        mPeriodicIO.midArmDemand = testMidArmDemand;
        mPeriodicIO.slappyDemand = testSlappyDemand;

        if (testSolenoidDemand) {
            mPeriodicIO.solenoidDemand = SolenoidState.RELEASE;
        } else {
            mPeriodicIO.solenoidDemand = SolenoidState.LOCK;
        }

        return defaultStateTransfer();
    }

    private SystemState handleHoming() {
        if (mStateChanged) {
            masterConfig(kMidArmCurrentLimitLow, true,
                         kSlappyCurrentLimitLow, true,
                         kSchedActive);
            mPeriodicIO.solenoidDemand = SolenoidState.RELEASE;
        }

        // Switch if statement arguments to flip homing sequence order
        // Also see individual homing methods
        if (!slappyHomingComplete) {
            homeSlappySticks();
        } else if (!midArmHomingComplete) {
            homeArms();
        }

        if (mWantedState != WantedState.HOME){
            midArmHomingComplete = false;
            slappyHomingComplete = false;
        }
        return defaultStateTransfer();
    }

    private void homeArms() {
        double now = Timer.getFPGATimestamp();

        // if (mStateChanged) { // Keep to home mid arm first
        //     midArmNonMovementTimeout = now + midArmNonMovementDuration;
        //     mPeriodicIO.midArmDemand = midArmHomingDemand;
        //     mPeriodicIO.midArmControlMode = ControlMode.PercentOutput;
        // }

        if (slappyJustFinishedHoming) { // Keep to home slappy first
            slappyJustFinishedHoming = false;
            midArmNonMovementTimeout = now + slappyNonMovementDuration;
            mPeriodicIO.midArmDemand = midArmHomingDemand;
            mPeriodicIO.midArmControlMode = ControlMode.PercentOutput;
        }

        // if still moving - difference between last position and now is large
        double distance = Math.abs(mPeriodicIO.midArmPosition - mPeriodicIO.lastClimberPosition);
        if (distance > midArmMovementThreshhold) {
            // increase timeout to continue
            midArmNonMovementTimeout = now + midArmNonMovementDuration;
        }

        //  if timeout has stopped moving, then finished
        if (now > midArmNonMovementTimeout) {

            System.out.println("Mid Arm Homing Sequence Complete");
            FramePeriodSwitch.setSelectedSensorPositionVolatile(mFXMidArm, MidArmPosition.HOME.get());
            mPeriodicIO.midArmPosition = MidArmPosition.HOME.get();
            mPeriodicIO.midArmDemand = 0.0;

            midArmHomingComplete = true;
            mPeriodicIO.solenoidDemand = SolenoidState.LOCK;
        }
    }

    private void homeSlappySticks() {
        double now = Timer.getFPGATimestamp();

        if (mStateChanged) { // Keep to home slappy first
            slappyNonMovementTimeout = now + slappyNonMovementDuration;
            mPeriodicIO.slappyDemand = slappyHomingDemand;
            mPeriodicIO.slappyControlMode = ControlMode.PercentOutput;
        }

        // if (midArmJustFinishedHoming) { // Keep to home mid arm first
        //     midArmJustFinishedHoming = false;
        //     slappyNonMovementTimeout = now + slappyNonMovementDuration;
        //     mPeriodicIO.slappyDemand = slappyHomingDemand;
        //     mPeriodicIO.slappyControlMode = ControlMode.PercentOutput;
        // }

        double distance = Math.abs(mPeriodicIO.slappyPosition - mPeriodicIO.lastSlappyPosition);
        if (distance > slappyMovementThreshhold) {
            slappyNonMovementTimeout = now + slappyNonMovementDuration;
        }

        if (now > slappyNonMovementTimeout) {

            System.out.println("Slappy Sticks Homing Sequence Complete");
            
            FramePeriodSwitch.setSelectedSensorPositionVolatile(mFXSlappy, SlappyPosition.HOME.get());

            mPeriodicIO.slappyPosition = SlappyPosition.HOME.get();
            mPeriodicIO.slappyDemand = 0.0;

            slappyJustFinishedHoming = true;
            slappyHomingComplete = true;
        }
    }

    private void masterConfig(double midArmStatorLimit, boolean midArmStatorEnable,
                              double slappyStatorLimit, boolean slappyStatorEnable, 
                              double whenRun){

        if (!Double.isNaN(midArmStatorLimit)){
            FramePeriodSwitch.configStatorCurrentLimitPermanent(mFXMidArm, new StatorCurrentLimitConfiguration(midArmStatorEnable, midArmStatorLimit, midArmStatorLimit, 0));
        }

        if (!Double.isNaN(slappyStatorLimit)){
            FramePeriodSwitch.configStatorCurrentLimitPermanent(mFXSlappy, new StatorCurrentLimitConfiguration(slappyStatorEnable, slappyStatorLimit, slappyStatorLimit, 0));
        }

        if (!Double.isNaN(whenRun)){
            mPeriodicIO.schedDeltaDesired = (int) whenRun;
        }
    }

    private void configPIDF(TalonFX motor,
                            double kP,
                            double kI,
                            double kD,
                            double kF,
                            double cruiseVelocity, double cruiseAcceleration) {

        motor.config_kP(0, kP, Constants.kLongCANTimeoutMs);
        motor.config_kI(0, kI, Constants.kLongCANTimeoutMs);
        motor.config_kD(0, kD, Constants.kLongCANTimeoutMs);
        motor.config_kF(0, kF, Constants.kLongCANTimeoutMs);

        motor.configMotionCruiseVelocity(cruiseVelocity);
        motor.configMotionAcceleration(cruiseAcceleration);

    }

    public boolean isHandlerComplete(WantedState state) {
        switch(state) {
            case ASSESS:
                return assessingComplete;
            case PRECLIMB:
                return preClimbComplete;
            case CLIMB_1_LIFT_MORE:
                return stageOneComplete;
            case CLIMB_2_ENGAGE_TRAV:
                return stageTwoComplete;
            case CLIMB_3_RELEASE_MID:
                return stageThreeComplete;
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

    public void setSolenoidTestState(boolean state) {
        testSolenoidDemand = state;
    }

    @Override
    public void readPeriodicInputs() {
        double now = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart = now;

        mPeriodicIO.lastClimberPosition = mPeriodicIO.midArmPosition;
        mPeriodicIO.midArmPosition = FramePeriodSwitch.getSelectedSensorPosition(mFXMidArm);

        mPeriodicIO.lastSlappyPosition = mPeriodicIO.slappyPosition;
        mPeriodicIO.slappyPosition = FramePeriodSwitch.getSelectedSensorPosition(mFXSlappy);
    }

    @Override
    public void writePeriodicOutputs() {
        if (mSolenoidState != mPeriodicIO.solenoidDemand) {
            mSolenoidState = mPeriodicIO.solenoidDemand;
            mSolenoidDeploy.set(mPeriodicIO.solenoidDemand.get());
            // System.out.println("climber solenoid "+mSolenoidState.toString());
        }

        mFXMidArm.set(mPeriodicIO.midArmControlMode, mPeriodicIO.midArmDemand);
        mFXSlappy.set(mPeriodicIO.slappyControlMode, mPeriodicIO.slappyDemand);
        // if (mPrintIt){
        //     System.out.println("MidArm "+mPeriodicIO.midArmControlMode+" "+mPeriodicIO.midArmDemand+" "+mPeriodicIO.midArmPosition+" "+mPeriodicIO.midArmStator);
        //     System.out.println("Slappy "+mPeriodicIO.slappyControlMode+" "+mPeriodicIO.slappyDemand+" "+mPeriodicIO.slappyPosition+" "+mPeriodicIO.slappyStator);
        // }
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

                sClassName+".midBarPosition,"+
                sClassName+".midBarDemand,"+
                sClassName+".midBarControlMode,"+
                sClassName+".midBarStator,"+

                sClassName+".slappyPosition,"+
                sClassName+".slappyDemand,"+
                sClassName+".slappyControlMode,"+
                sClassName+".slappyStator,"+

                sClassName+".solenoidState";
    }

    @Override
    public String getLogValues(boolean telemetry) {
        String start;
        if (telemetry){
            start = ",,,";
        }
        else{
            start = 
            mPeriodicIO.schedDeltaDesired+","+
            mPeriodicIO.schedDeltaActual+","+
            (Timer.getFPGATimestamp()-mPeriodicIO.lastSchedStart)+",";
        }
        return  start+
                mSystemState+","+
                mPeriodicIO.midArmPosition+","+
                mPeriodicIO.midArmDemand+","+
                mPeriodicIO.midArmControlMode+","+
                mPeriodicIO.midArmStator+","+

                mPeriodicIO.slappyPosition+","+
                mPeriodicIO.slappyDemand+","+
                mPeriodicIO.slappyControlMode+","+
                mPeriodicIO.slappyStator+","+

                mSolenoidState;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Mid Arm Pos", mPeriodicIO.midArmPosition);
        SmartDashboard.putNumber("Slappy Pos", mPeriodicIO.slappyPosition);
        mPeriodicIO.slappyStator = FramePeriodSwitch.getStatorCurrent(mFXSlappy);
        mPeriodicIO.midArmStator = FramePeriodSwitch.getStatorCurrent(mFXMidArm);
        SmartDashboard.putNumber("Mid Arm Current", mPeriodicIO.midArmStator);
        SmartDashboard.putNumber("Slappy Current", mPeriodicIO.slappyStator);
        
    }

    public static class PeriodicIO {
        // Logging
        public int schedDeltaDesired;
        public double schedDeltaActual;
        public double lastSchedStart;

        // Inputs
        public double midArmPosition;
        public double slappyPosition;
        public double slappyStator;
        public double midArmStator;

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
