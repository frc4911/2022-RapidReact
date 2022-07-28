package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter.SystemState;
import frc.robot.subsystems.Swerve.ControlState;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.util.LatchedBoolean;
import libraries.cheesylib.util.TimeDelayedBoolean;
import libraries.cyberlib.control.SwerveHeadingController;
import libraries.cyberlib.io.CW;
import libraries.cyberlib.io.LogitechPS4;
import libraries.cyberlib.io.Xbox;

public class JSticks extends Subsystem {

    // Heading controller methods
    private final SwerveHeadingController mHeadingController = SwerveHeadingController.getInstance();
    private final LatchedBoolean shouldChangeHeadingSetpoint = new LatchedBoolean();
    private final TimeDelayedBoolean mShouldMaintainHeading = new TimeDelayedBoolean();

    public enum SystemState {
        READINGBUTTONS,
        READINGTESTBUTTONS,
        DISABLING,
    }

    public enum WantedState {
        READBUTTONS,
        READTESTBUTTONS,
        DISABLE,
    }

    private SystemState mSystemState = SystemState.READINGBUTTONS;
    private WantedState mWantedState = WantedState.READBUTTONS;
    private boolean mStateChanged;
    private boolean slowToggle = false;
    private final CW mDriver;
    private final CW mOperator;
    // private LogitechPS4 mTest;
    private final PeriodicIO mPeriodicIO = new PeriodicIO();
    private final double mDeadBand = 0.15; // for the turnigy (driver) swerve controls
    private final Superstructure mSuperstructure;
    private final Swerve mSwerve;
    private final Shooter mShooter;
    private final Indexer mIndexer;
    private final Climber mClimber;

    @SuppressWarnings("unused")
    private final LatchedBoolean mSystemStateChange = new LatchedBoolean();

    private static String sClassName;
    private static int sInstanceCount;
    private static JSticks sInstance = null;

    public static JSticks getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new JSticks(caller);
        } else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller) {
        System.out.println("(" + caller + ") " + " getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private JSticks(String caller) {
        sClassName = this.getClass().getSimpleName();
        mSuperstructure = Superstructure.getInstance(sClassName);
        mSwerve = Swerve.getInstance(sClassName);
        mShooter = Shooter.getInstance(sClassName);
        mIndexer = Indexer.getInstance(sClassName); // Getting instance to reset ball count: hopefully will be a
                                                    // temporary fix
        mClimber = Climber.getInstance(sClassName);
        mHeadingController.setPIDFConstants(
                mSwerve.mSwerveConfiguration.kSwerveHeadingKp,
                mSwerve.mSwerveConfiguration.kSwerveHeadingKi,
                mSwerve.mSwerveConfiguration.kSwerveHeadingKd,
                mSwerve.mSwerveConfiguration.kSwerveHeadingKf);
        mDriver = new Xbox();
        mOperator = new Xbox();
        // mTest = new LogitechPS4();

        printUsage(caller);
    }

    @Override
    public void onStart(Phase phase) {
        synchronized (JSticks.this) {
            mStateChanged = true;

            // Force true on first iteration of teleop periodic
            shouldChangeHeadingSetpoint.update(false);

            switch (phase) {
                case DISABLED:
                case AUTONOMOUS:
                    mPeriodicIO.schedDeltaDesired = 0; // goto sleep
                    mSystemState = SystemState.DISABLING;
                    mWantedState = WantedState.DISABLE;
                    break;
                case TEST:
                    mSystemState = SystemState.READINGTESTBUTTONS;
                    mWantedState = WantedState.READTESTBUTTONS;
                    mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
                    break;
                default:
                    mSystemState = SystemState.READINGBUTTONS;
                    mWantedState = WantedState.READBUTTONS;
                    mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
                    break;
            }
            System.out.println(sClassName + " state " + mSystemState);
        }
    }

    @Override
    public void onLoop(double timestamp) {
        synchronized (JSticks.this) {
            do {
                SystemState newState;
                switch (mSystemState) {
                    case DISABLING:
                        newState = handleDisabling();
                        break;
                    case READINGTESTBUTTONS:
                        newState = handleReadingTestButtons();
                        break;
                    case READINGBUTTONS:
                    default:
                        newState = handleReadingButtons();
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

    @Override
    public void stop() {
    }

    private SystemState handleDisabling() {

        return defaultStateTransfer();
    }

    private SystemState handleReadingButtons() {
        if (mStateChanged){
            mDriver.rumble(5, 2);
        }
        teleopRoutines();

        return defaultStateTransfer();
    }
    
    public void teleopRoutines() {
        Superstructure.WantedState currentState = mSuperstructure.getWantedState();
        Superstructure.WantedState previousState = currentState;

        // if(mPeriodicIO.dr_AButton_ToggleDriveMode) {
        //     mSwerve.toggleThroughDriveModes();
        // }

        // NEW SWERVE
        if (mSuperstructure.getWantedState() != Superstructure.WantedState.AUTO_SHOOT) {
        // All driver assist to raw inputs should be implemented Swerve#handleManual()
            double swerveYInput = mPeriodicIO.dr_LeftStickX_Translate;
            double swerveXInput = mPeriodicIO.dr_LeftStickY_Translate;
            double swerveRotationInput = mPeriodicIO.dr_RightStickX_Rotate;

            // if (mPeriodicIO.dr_LeftStickButton_SlowSpeed) {
            //     slowToggle = !slowToggle;
            //     if (slowToggle){
            //         swerveYInput *= 0.25;
            //         swerveXInput *= 0.25;
            //         swerveRotationInput *= 0.5;
            //     }
            // }

            boolean maintainHeading = mShouldMaintainHeading.update(swerveRotationInput == 0, 0.2);
            boolean changeHeadingSetpoint = shouldChangeHeadingSetpoint.update(maintainHeading);

            if (!maintainHeading) {
                mHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.OFF);
            } else if (changeHeadingSetpoint) {
                mHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.MAINTAIN);
                mHeadingController.setGoal(mSwerve.getHeading().getDegrees());
            }

            var isFieldOriented = !mPeriodicIO.dr_RightBumper_RobotOrient;
            if (mHeadingController.getHeadingControllerState() != SwerveHeadingController.HeadingControllerState.OFF) {
                mSwerve.setTeleopInputs(swerveXInput, swerveYInput, mHeadingController.update(),
                        false, isFieldOriented, true);
            } else {
                mSwerve.setTeleopInputs(swerveXInput, swerveYInput, swerveRotationInput,
                        false, isFieldOriented, false);
            }
        }

        if (mPeriodicIO.dr_YButton_ResetIMU) {
            // Seems safest to disable heading controller if were resetting IMU.
            mHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.OFF);
            mSwerve.setRobotPosition(Constants.kRobotStartingPose);
            // mHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.MAINTAIN); // Reenable heading controller?
        }

        if (mPeriodicIO.dr_StartButton_ResetWheels){
            mSwerve.convertCancoderToFX();
        }

        if (mPeriodicIO.dr_XButton_HomeHood){
            mShooter.setWantedState(Shooter.WantedState.HOMEHOOD, sClassName);
        }
        
        if (mPeriodicIO.dr_XButton_HomeHood_Stop){
            mShooter.setWantedState(Shooter.WantedState.HOLD, sClassName);
        }
        
        // END NEW SWERVE

        // CLIMBER CONTROL
        if(mPeriodicIO.dr_RightBumper_ClimberLockout) {
            if (mPeriodicIO.dr_AButton_PreClimb) {
                mSuperstructure.setWantedState(Superstructure.WantedState.AUTO_PRE_CLIMB, sClassName);
            } else if (mPeriodicIO.dr_AButton_PreClimb_Stop) {
                mSuperstructure.setWantedState(Superstructure.WantedState.HOLD, sClassName);
            } else if (mPeriodicIO.dr_BButton_AutoClimb) {
                mSuperstructure.setWantedState(Superstructure.WantedState.AUTO_CLIMB, sClassName);
            } 
            // else if (mPeriodicIO.op_YButton_MidBarClimb) {
            //     mSuperstructure.setLastClimbState(Climber.WantedState.CLIMB_3_LIFT_MORE);
            //     mSuperstructure.setWantedState(Superstructure.WantedState.AUTO_CLIMB, sClassName);
            // }

        } else {
            if(mPeriodicIO.dr_LeftBumper_BackingLock){
                if (mPeriodicIO.dr_LeftTrigger_Collect) {
                    mSuperstructure.setWantedState(Superstructure.WantedState.BACK, sClassName);
                }
            } else {
                if (mPeriodicIO.dr_LeftTrigger_Collect) {
                    mSuperstructure.setWantedState(Superstructure.WantedState.COLLECT, sClassName);
                }
            }
        }

        if (mPeriodicIO.dr_POV180_ManualShot_StopShooter) {
            mShooter.stopFlywheel();
        }

        if (mPeriodicIO.dr_RightTrigger_AutoShoot) {
            mHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.OFF);
            mSuperstructure.setWantedState(Superstructure.WantedState.AUTO_SHOOT, sClassName);
            mSwerve.EnableAimingController();
        }

        if (mPeriodicIO.dr_RightTrigger_AutoShoot_Stop) {
            mSwerve.DisableAimingController();
            mSuperstructure.setWantedState(Superstructure.WantedState.HOLD, sClassName);
            mHeadingController.setGoal(mSwerve.getHeading().getDegrees());
        }

        if (mPeriodicIO.dr_POV0_ManualShot_Fender) {
            mSuperstructure.setManualShootDistance(0);
            mSuperstructure.setWantedState(Superstructure.WantedState.MANUAL_SHOOT, sClassName);
        }

        if (mPeriodicIO.dr_POV90_ManualShot_Ball) {
            mSuperstructure.setManualShootDistance(48);
            mSuperstructure.setWantedState(Superstructure.WantedState.MANUAL_SHOOT, sClassName);
        }

        // if (mPeriodicIO.dr_POV180_ManualShot_Robot) {
        //     mSuperstructure.setManualShootDistance(120);
        //     mSuperstructure.setWantedState(Superstructure.WantedState.MANUAL_SHOOT, sClassName);
        // }

        if (mPeriodicIO.dr_POV270_ManualShot_Tarmac) {
            mSuperstructure.setManualShootDistance(96);
            mSuperstructure.setWantedState(Superstructure.WantedState.MANUAL_SHOOT, sClassName);
        }

        if (mPeriodicIO.dr_POV0_ManualShot_Fender_Stop || mPeriodicIO.dr_POV90_ManualShot_Ball_Stop
             || mPeriodicIO.dr_POV270_ManualShot_Tarmac_Stop) {
            mSuperstructure.setWantedState(Superstructure.WantedState.HOLD, sClassName);
        }

        if (mPeriodicIO.dr_LeftTrigger_Collect_Stop) {
            mSuperstructure.setWantedState(Superstructure.WantedState.HOLD, sClassName);
        }

        if (mPeriodicIO.dr_LeftBumper_Back_Stop) {
            mSuperstructure.setWantedState(Superstructure.WantedState.HOLD, sClassName);
        }

        // if (mPeriodicIO.tst_POV90_flyup){
        //     mShooter.setTempDemands(mShooter.getHoodDemand(), mShooter.getFlyDemand()+100);
        // }
        // if (mPeriodicIO.tst_POV270_flyDn){
        //     mShooter.setTempDemands(mShooter.getHoodDemand(), mShooter.getFlyDemand()-50);
        // }
        // if (mPeriodicIO.tst_POV0_hoodup){
        //     mShooter.setTempDemands(mShooter.getHoodDemand()+100, mShooter.getFlyDemand());
        // }
        // if (mPeriodicIO.tst_POV180_hooddn){
        //     mShooter.setTempDemands(mShooter.getHoodDemand()-50, mShooter.getFlyDemand());
        // }
    }

    private SystemState handleReadingTestButtons() {
        if (mPeriodicIO.dr_AButton_PreClimb){
            mSuperstructure.setWantedState(Superstructure.WantedState.AUTO_PRE_CLIMB, sClassName);
        }

        if (mPeriodicIO.dr_AButton_PreClimb_Stop){
            mSuperstructure.setWantedState(Superstructure.WantedState.TEST, sClassName);
        }

        if (mPeriodicIO.dr_BackButton_TestHome){
            mSuperstructure.setWantedState(Superstructure.WantedState.HOME, sClassName);
        }

        if (mPeriodicIO.dr_BackButton_TestHome_Stop){
            mSuperstructure.setWantedState(Superstructure.WantedState.TEST, sClassName);
        }

        // mShooter.setMotorTestDemand(-mOperator.getRaw(Xbox.LEFT_STICK_Y), -mOperator.getRaw(Xbox.RIGHT_STICK_Y));

        if (mPeriodicIO.dr_LeftTrigger_Collect) {
            mSuperstructure.setWantedState(Superstructure.WantedState.COLLECT, sClassName);
        }
        if (mPeriodicIO.dr_LeftTrigger_Collect_Stop) {
            mSuperstructure.setWantedState(Superstructure.WantedState.HOLD, sClassName);
        }

        if (mPeriodicIO.dr_LeftBumper_BackingLock) {
            mSuperstructure.setWantedState(Superstructure.WantedState.BACK, sClassName);
        }
        
        if (mPeriodicIO.dr_LeftBumper_Back_Stop) {
            mSuperstructure.setWantedState(Superstructure.WantedState.HOLD, sClassName);
        }

        // mShooter.setHoodTestDemand(mPeriodicIO.tst_LeftAxis_TestDemand);
        return defaultStateTransfer();
    }

    @Override
    public void readPeriodicInputs() {
        double now = Timer.getFPGATimestamp();

        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart = now;

        //Driving
        mPeriodicIO.dr_LeftStickX_Translate = -mDriver.getRaw(Xbox.LEFT_STICK_X, mDeadBand);
        mPeriodicIO.dr_LeftStickY_Translate = -mDriver.getRaw(Xbox.LEFT_STICK_Y, mDeadBand);
        mPeriodicIO.dr_RightStickX_Rotate = -mDriver.getRaw(Xbox.RIGHT_STICK_X, mDeadBand);
        mPeriodicIO.dr_LeftStickButton_SlowSpeed = mDriver.getButton(Xbox.LEFT_STICK_BUTTON, CW.PRESSED_LEVEL);
        mPeriodicIO.dr_RightTrigger_AutoShoot = mDriver.getButton(Xbox.RIGHT_TRIGGER, CW.PRESSED_EDGE);
        mPeriodicIO.dr_RightTrigger_AutoShoot_Stop = mDriver.getButton(Xbox.RIGHT_TRIGGER, CW.RELEASED_EDGE);
        //mPeriodicIO.dr_RightBumper_RobotOrient = mDriver.getButton(Xbox.RIGHT_BUMPER, CW.PRESSED_LEVEL); // field/robot
        mPeriodicIO.dr_YButton_ResetIMU = mDriver.getButton(Xbox.Y_BUTTON, CW.PRESSED_EDGE);
        //mPeriodicIO.dr_AButton_ToggleDriveMode = mDriver.getButton(Xbox.A_BUTTON, CW.PRESSED_EDGE);
        mPeriodicIO.dr_StartButton_ResetWheels = mDriver.getButton(Xbox.START_BUTTON, CW.PRESSED_EDGE);
        mPeriodicIO.dr_XButton_HomeHood = mDriver.getButton(Xbox.X_BUTTON, CW.PRESSED_EDGE);
        mPeriodicIO.dr_XButton_HomeHood_Stop = mDriver.getButton(Xbox.X_BUTTON, CW.RELEASED_EDGE);

        //Climbing
        mPeriodicIO.op_LeftStickY_TestMidArms = -mOperator.getRaw(Xbox.LEFT_STICK_Y, mDeadBand);
        mPeriodicIO.op_RightStickY_TestSlappy = -mOperator.getRaw(Xbox.RIGHT_STICK_Y, mDeadBand);
        mPeriodicIO.dr_RightBumper_ClimberLockout = mDriver.getButton(Xbox.RIGHT_BUMPER, CW.PRESSED_LEVEL);
        mPeriodicIO.dr_AButton_PreClimb = mDriver.getButton(Xbox.A_BUTTON, CW.PRESSED_EDGE);
        mPeriodicIO.dr_AButton_PreClimb_Stop = mDriver.getButton(Xbox.A_BUTTON, CW.RELEASED_EDGE);
        mPeriodicIO.dr_BButton_AutoClimb = mDriver.getButton(Xbox.B_BUTTON, CW.PRESSED_EDGE);
        // mPeriodicIO.op_XButton_AutoClimb_Stop = mOperator.getButton(Xbox.X_BUTTON, CW.RELEASED_EDGE);
        mPeriodicIO.op_YButton_MidBarClimb = mOperator.getButton(Xbox.Y_BUTTON, CW.PRESSED_EDGE);

        mPeriodicIO.dr_BackButton_TestHome = mDriver.getButton(Xbox.BACK_BUTTON, CW.PRESSED_EDGE);
        mPeriodicIO.dr_BackButton_TestHome_Stop = mDriver.getButton(Xbox.BACK_BUTTON, CW.RELEASED_EDGE);
        mPeriodicIO.op_LeftBumper_TestRelease = mDriver.getButton(Xbox.LEFT_BUMPER, CW.PRESSED_EDGE);
        mPeriodicIO.op_RightBumper_TestLock = mDriver.getButton(Xbox.RIGHT_BUMPER, CW.PRESSED_EDGE);

        //Collecting
        mPeriodicIO.dr_LeftTrigger_Collect = mDriver.getButton(Xbox.LEFT_TRIGGER, CW.PRESSED_EDGE);
        mPeriodicIO.dr_LeftTrigger_Collect_Stop = mDriver.getButton(Xbox.LEFT_TRIGGER, CW.RELEASED_EDGE);

        mPeriodicIO.dr_LeftBumper_BackingLock = mDriver.getButton(Xbox.LEFT_BUMPER, CW.PRESSED_LEVEL);
        mPeriodicIO.dr_LeftBumper_Back_Stop = mDriver.getButton(Xbox.LEFT_BUMPER, CW.RELEASED_EDGE);

        //Shooting
        mPeriodicIO.op_BButton_StopShooter = mOperator.getButton(Xbox.B_BUTTON, CW.PRESSED_EDGE);

        mPeriodicIO.dr_POV0_ManualShot_Fender = mDriver.getButton(Xbox.POV0_0, CW.PRESSED_EDGE);
        mPeriodicIO.dr_POV90_ManualShot_Ball = mDriver.getButton(Xbox.POV0_90, CW.PRESSED_EDGE);
        mPeriodicIO.dr_POV180_ManualShot_StopShooter = mDriver.getButton(Xbox.POV0_180, CW.PRESSED_EDGE);
        mPeriodicIO.dr_POV270_ManualShot_Tarmac = mDriver.getButton(Xbox.POV0_270, CW.PRESSED_EDGE);

        mPeriodicIO.dr_POV0_ManualShot_Fender_Stop = mDriver.getButton(Xbox.POV0_0, CW.RELEASED_EDGE);
        mPeriodicIO.dr_POV90_ManualShot_Ball_Stop = mDriver.getButton(Xbox.POV0_90, CW.RELEASED_EDGE);
        mPeriodicIO.dr_POV180_ManualShot_FlyStop_Stop = mDriver.getButton(Xbox.POV0_180, CW.RELEASED_EDGE);
        mPeriodicIO.dr_POV270_ManualShot_Tarmac_Stop = mDriver.getButton(Xbox.POV0_270, CW.RELEASED_EDGE);

        // mPeriodicIO.tst_AButton_AutoElev = mTest.getButton(LogitechPS4.A_BUTTON, CW.PRESSED_EDGE);
        // mPeriodicIO.tst_AButton_AutoElev_Stop = mTest.getButton(LogitechPS4.A_BUTTON, CW.RELEASED_EDGE);
        // mPeriodicIO.tst_BButton_AutoPre = mTest.getButton(LogitechPS4.B_BUTTON, CW.PRESSED_EDGE);
        // mPeriodicIO.tst_BButton_AutoPre_Stop = mTest.getButton(LogitechPS4.B_BUTTON, CW.RELEASED_EDGE);
        // mPeriodicIO.tst_LeftAxis_TestDemand = mTest.getRaw(LogitechPS4.LEFT_STICK_Y,.15);
        // mPeriodicIO.tst_RightAxis_TestDemand = mTest.getRaw(LogitechPS4.RIGHT_STICK_Y,.15);
        // mPeriodicIO.tst_LeftBumper_TestSolEngage = mTest.getButton(LogitechPS4.LEFT_BUMPER, CW.PRESSED_EDGE);
        // mPeriodicIO.tst_RightBumper_TestSolDisengage = mTest.getButton(LogitechPS4.RIGHT_BUMPER, CW.PRESSED_EDGE);
        // mPeriodicIO.tst_XButton_HOME = mTest.getButton(LogitechPS4.X_BUTTON, CW.PRESSED_EDGE);
        // mPeriodicIO.tst_XButton_HOME_STOP = mTest.getButton(LogitechPS4.X_BUTTON, CW.RELEASED_EDGE);

        // mPeriodicIO.tst_POV0_hoodup = mTest.getButton(LogitechPS4.POV0_0, CW.PRESSED_EDGE);
        // mPeriodicIO.tst_POV90_flyup = mTest.getButton(LogitechPS4.POV0_90, CW.PRESSED_EDGE);
        // mPeriodicIO.tst_POV180_hooddn = mTest.getButton(LogitechPS4.POV0_180, CW.PRESSED_EDGE);
        // mPeriodicIO.tst_POV270_flyDn = mTest.getButton(LogitechPS4.POV0_270, CW.PRESSED_EDGE);

    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case DISABLE:
                return SystemState.DISABLING;
            case READTESTBUTTONS:
                return SystemState.READINGTESTBUTTONS;
            case READBUTTONS:
            default:
                return SystemState.READINGBUTTONS;
        }
    }

    @Override
    public String getLogHeaders() {
        return  sClassName+".schedDeltaDesired,"+
                sClassName+".schedDeltaActual,"+
                sClassName+".schedDuration,"+
                sClassName+".dr_LeftStickX_Translate,"+
                sClassName+".dr_LeftStickY_Translate,"+
                sClassName+".dr_RightStickX_Rotate,"+
                sClassName+".dr_RightTrigger_AutoShoot,"+
                sClassName+".dr_RightTrigger_AutoShoot_Stop,"+
                sClassName+".dr_LeftTrigger_SlowSpeed,"+
                sClassName+".dr_RightBumper_RobotOrient,"+
                sClassName+".dr_YButton_ResetIMU = false,"+
                sClassName+".dr_AButton_ToggleDriveMode,"+
                sClassName+".dr_StartButton_ResetWheels,"+
                sClassName+".dr_XButton_HomeHood,"+
                sClassName+".dr_XButton_HomeHood_Stop,"+
                sClassName+".op_RightTrigger_Collect,"+
                sClassName+".op_RightTrigger_Collect_Stop,"+
                sClassName+".op_LeftTrigger_Back,"+
                sClassName+".op_LeftTrigger_Back_Stop,"+
                sClassName+".op_BButton_StopShooter,"+

                sClassName+".op_LeftBumper_ClimberLockout,"+
                sClassName+".op_XButton_AutoClimb,"+
                sClassName+".op_YButton_MidBarClimb,"+
                sClassName+".op_AButton_PreClimb,"+
                sClassName+".op_POV0_ManualShot_Fender,"+
                sClassName+".op_POV90_ManualShot_Ball,"+
                sClassName+".op_POV180_ManualShot_Robot,"+
                sClassName+".op_POV270_ManualShot_Tarmac,"+
                sClassName+".op_ManualShoot,"+
                sClassName+".op_POV0_ManualShot_Fender_Stop,"+
                sClassName+".op_POV90_ManualShot_Ball_Stop,"+
                sClassName+".op_POV180_ManualShot_Robot_Stop,"+
                sClassName+".op_POV270_ManualShot_Tarmac_Stop";
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
        mPeriodicIO.dr_LeftStickX_Translate+","+
        mPeriodicIO.dr_LeftStickY_Translate+","+
        mPeriodicIO.dr_RightStickX_Rotate+","+
        mPeriodicIO.dr_RightTrigger_AutoShoot+","+
        mPeriodicIO.dr_RightTrigger_AutoShoot_Stop+","+
        mPeriodicIO.dr_LeftTrigger_SlowSpeed+","+
        mPeriodicIO.dr_RightBumper_RobotOrient+","+
        mPeriodicIO.dr_YButton_ResetIMU+","+
        mPeriodicIO.dr_AButton_ToggleDriveMode+","+
        mPeriodicIO.dr_StartButton_ResetWheels+","+
        mPeriodicIO.dr_XButton_HomeHood+","+
        mPeriodicIO.dr_XButton_HomeHood_Stop+","+
        mPeriodicIO.dr_LeftTrigger_Collect+","+
        mPeriodicIO.dr_LeftTrigger_Collect_Stop+","+
        mPeriodicIO.dr_LeftBumper_BackingLock+","+
        mPeriodicIO.dr_LeftBumper_Back_Stop+","+
        mPeriodicIO.op_BButton_StopShooter+","+
        mPeriodicIO.dr_RightBumper_ClimberLockout+","+
        mPeriodicIO.dr_BButton_AutoClimb+","+
        mPeriodicIO.op_YButton_MidBarClimb+","+
        mPeriodicIO.dr_AButton_PreClimb+","+
        mPeriodicIO.dr_POV0_ManualShot_Fender+","+
        mPeriodicIO.dr_POV90_ManualShot_Ball+","+
        mPeriodicIO.dr_POV180_ManualShot_StopShooter+","+
        mPeriodicIO.dr_POV270_ManualShot_Tarmac+","+
        mPeriodicIO.dr_ManualShoot+","+
        mPeriodicIO.dr_POV0_ManualShot_Fender_Stop+","+
        mPeriodicIO.dr_POV90_ManualShot_Ball_Stop+","+
        mPeriodicIO.dr_POV180_ManualShot_FlyStop_Stop+","+
        mPeriodicIO.dr_POV270_ManualShot_Tarmac_Stop;
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public int whenRunAgain() {
        return mPeriodicIO.schedDeltaDesired;
    }

    public static class PeriodicIO {
        // Logging
        private final int mDefaultSchedDelta = 20; // axis updated every 20 msec
        public int schedDeltaDesired;
        public double schedDeltaActual;
        private double lastSchedStart;

        // Joystick Inputs
        public double dr_LeftStickX_Translate; // drive
        public double dr_LeftStickY_Translate; // drive
        public double dr_RightStickX_Rotate; // drive
        public boolean dr_RightTrigger_AutoShoot = false;
        public boolean dr_RightTrigger_AutoShoot_Stop = false;
        public boolean dr_LeftTrigger_SlowSpeed = false;
        public boolean dr_RightBumper_RobotOrient = false; // field/robot oriented
        public boolean dr_YButton_ResetIMU = false; // reset direction
        public boolean dr_AButton_ToggleDriveMode = false;
        public boolean dr_StartButton_ResetWheels = false;
        public boolean dr_XButton_HomeHood = false;
        public boolean dr_XButton_HomeHood_Stop = false;
        public boolean dr_LeftStickButton_SlowSpeed = false;

        public boolean dr_LeftTrigger_Collect = false;
        public boolean dr_LeftTrigger_Collect_Stop = false;
        public boolean dr_LeftBumper_BackingLock = false;
        public boolean dr_LeftBumper_Back_Stop = false;
        public boolean dr_RightBumper_ClimberLockout = false;
        public boolean dr_AButton_PreClimb = false;
        public boolean dr_AButton_PreClimb_Stop = false;
        public boolean dr_BButton_AutoClimb = false;
        public boolean dr_BButton_AutoClimb_Stop = false;
        public boolean dr_BackButton_TestHome = false;
        public boolean dr_BackButton_TestHome_Stop = false;

        public boolean dr_POV0_ManualShot_Fender = false;
        public boolean dr_POV90_ManualShot_Ball = false;
        public boolean dr_POV180_ManualShot_StopShooter = false;
        public boolean dr_POV270_ManualShot_Tarmac = false;
        public boolean dr_ManualShoot = false; // Move to Pov once read

        public boolean dr_POV0_ManualShot_Fender_Stop = false;
        public boolean dr_POV90_ManualShot_Ball_Stop = false;
        public boolean dr_POV180_ManualShot_FlyStop_Stop = false;
        public boolean dr_POV270_ManualShot_Tarmac_Stop = false;

        public double op_LeftStickY_TestMidArms;
        public double op_RightStickY_TestSlappy;
        public boolean op_LeftBumper_TestRelease;
        public boolean op_RightBumper_TestLock;
        public boolean op_YButton_MidBarClimb = false;
        public boolean op_BButton_StopShooter = false;

        // public boolean tst_AButton_AutoElev = false;
        // public boolean tst_AButton_AutoElev_Stop = false;
        // public boolean tst_BButton_AutoPre = false;
        // public boolean tst_BButton_AutoPre_Stop = false;
        // public double  tst_LeftAxis_TestDemand = 0;
        // public double  tst_RightAxis_TestDemand = 0;
        // public boolean tst_LeftBumper_TestSolEngage = false;
        // public boolean tst_RightBumper_TestSolDisengage = false;
        // public boolean tst_XButton_HOME = false;
        // public boolean tst_XButton_HOME_STOP = false;
        // public boolean tst_POV0_hoodup = false;
        // public boolean tst_POV180_hooddn = false;
        // public boolean tst_POV90_flyup = false;
        // public boolean tst_POV270_flyDn = false;
    }
}