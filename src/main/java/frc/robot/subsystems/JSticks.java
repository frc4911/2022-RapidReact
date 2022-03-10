package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
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
        DISABLING,
    }

    public enum WantedState {
        READBUTTONS,
        DISABLE,
    }

    private SystemState mSystemState = SystemState.READINGBUTTONS;
    private WantedState mWantedState = WantedState.READBUTTONS;
    private boolean mStateChanged;
    private CW mDriver;
    private CW mOperator;
    private LogitechPS4 mTest;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private final double mDeadBand = 0.15; // for the turnigy (driver) swerve controls
    private Superstructure mSuperstructure;
    private Swerve mSwerve;
    private Shooter mShooter;
    private Indexer mIndexer;

    @SuppressWarnings("unused")
    private LatchedBoolean mSystemStateChange = new LatchedBoolean();

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
        mHeadingController.setPIDFConstants(
                mSwerve.mSwerveConfiguration.kSwerveHeadingKp,
                mSwerve.mSwerveConfiguration.kSwerveHeadingKi,
                mSwerve.mSwerveConfiguration.kSwerveHeadingKd,
                mSwerve.mSwerveConfiguration.kSwerveHeadingKf);
        mDriver = new Xbox();
        mOperator = new Xbox();
        mTest = new LogitechPS4();

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
        System.out.println(sClassName + " stop()");
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

        // disable for the competition
        // if(mPeriodicIO.dr_AButton_ToggleDriveMode) {
        //     mSwerve.toggleThroughDriveModes();
        // }

        // All driver assist to raw inputs should be implemented Swerve#handleManual()
        double swerveYInput = mPeriodicIO.dr_LeftStickX_Translate;
        double swerveXInput = mPeriodicIO.dr_LeftStickY_Translate;
        double swerveRotationInput = mPeriodicIO.dr_RightStickX_Rotate;

        if (mPeriodicIO.dr_LeftTrigger_SlowSpeed) {
            swerveYInput *= 0.5;
            swerveXInput *= 0.5;
            swerveRotationInput *= 0.5;
        }

        // NEW SWERVE
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
                    false/*mPeriodicIO.dr_LeftTrigger_SlowSpeed*/, isFieldOriented, true);
        } else {
            mSwerve.setTeleopInputs(swerveXInput, swerveYInput, swerveRotationInput,
                    false/*mPeriodicIO.dr_LeftTrigger_SlowSpeed*/, isFieldOriented, false);
        }

        if (mPeriodicIO.dr_YButton_ResetIMU) {
            // Seems safest to disable heading controller if were resetting IMU.
            mHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.OFF);
            mSwerve.setRobotPosition(Constants.kRobotStartingPose);
        }

        if (mPeriodicIO.dr_StartButton_ResetWheels){
            mSwerve.convertCancoderToFX();
        }
        
        // END NEW SWERVE

        // CLIMBER CONTROL
        if(mPeriodicIO.op_AButton_ClimberLockout) {
            // -1: Do nothing
            // 0: Extend
            // 1: Retract
            int deploySlappyState = -1;
            if (mPeriodicIO.op_RightBumper_ExtendSlappySticks) {
                deploySlappyState = 0;
            } else if (mPeriodicIO.op_LeftBumper_RetractSlappySticks) {
                deploySlappyState = 1;
            }
            mSuperstructure.setOpenLoopClimb(mPeriodicIO.op_LeftStickY_ClimberElevator, deploySlappyState);
        } else {
            if (mPeriodicIO.op_RightTrigger_Collect) {
                mSuperstructure.setWantedState(Superstructure.WantedState.COLLECT, sClassName);
            }
    
            if (mPeriodicIO.op_LeftTrigger_Back) {
                mSuperstructure.setWantedState(Superstructure.WantedState.BACK, sClassName);
            }

            mSuperstructure.setOpenLoopClimb(0.0, -1);
        }

        if (mPeriodicIO.op_BButton_StopShooter) {
            mShooter.stopFlywheel();
        }

        if (mPeriodicIO.op_POV0_ManualShot_Fender) {
            mSuperstructure.setManualShootDistance(0);
            mSuperstructure.setWantedState(Superstructure.WantedState.MANUAL_SHOOT, sClassName);
        }

        if (mPeriodicIO.op_POV90_ManualShot_Ball) {
            mSuperstructure.setManualShootDistance(36);
            mSuperstructure.setWantedState(Superstructure.WantedState.MANUAL_SHOOT, sClassName);
        }

        if (mPeriodicIO.op_POV180_ManualShot_Robot) {
            mSuperstructure.setManualShootDistance(60);
            mSuperstructure.setWantedState(Superstructure.WantedState.MANUAL_SHOOT, sClassName);
        }

        if (mPeriodicIO.op_POV270_ManualShot_Tarmac) {
            mSuperstructure.setManualShootDistance(102);
            mSuperstructure.setWantedState(Superstructure.WantedState.MANUAL_SHOOT, sClassName);
        }

        if (mPeriodicIO.op_POV0_ManualShot_Fender_Stop || mPeriodicIO.op_POV90_ManualShot_Ball_Stop
            || mPeriodicIO.op_POV180_ManualShot_Robot_Stop || mPeriodicIO.op_POV270_ManualShot_Tarmac_Stop) {
            mSuperstructure.setWantedState(Superstructure.WantedState.HOLD, sClassName);
        }

        if (mPeriodicIO.op_RightTrigger_Collect_Stop) {
            mSuperstructure.setWantedState(Superstructure.WantedState.HOLD, sClassName);
        }

        if (mPeriodicIO.op_LeftTrigger_Back_Stop) {
            mSuperstructure.setWantedState(Superstructure.WantedState.HOLD, sClassName);
        }

        if (mPeriodicIO.tst_AButton_AutoElev){
            mSuperstructure.setWantedState(Superstructure.WantedState.AUTO_CLIMB, sClassName);
        }

        if (mPeriodicIO.tst_AButton_AutoElev_Stop){
            mSuperstructure.setWantedState(Superstructure.WantedState.HOLD, sClassName);
        }

        if (mPeriodicIO.tst_BButton_AutoPre){
            mSuperstructure.setWantedState(Superstructure.WantedState.AUTO_PRE_CLIMB, sClassName);
        }

        if (mPeriodicIO.tst_BButton_AutoPre_Stop){
            mSuperstructure.setWantedState(Superstructure.WantedState.HOLD, sClassName);
        }

        // // Will add clauses for different shoot distances, aimed/auto shooting, auto
        // climbing, and others
        // currentState = activeBtnIsReleased(currentState);
        // if (currentState == Superstructure.WantedState.HOLD) {
        // // if (mPeriodicIO.op_POV0_ManualShot_Fender) {
        // // mSuperstructure.setManualShootDistance(0);
        // // mSuperstructure.setWantedState(Superstructure.WantedState.MANUAL_SHOOT,
        // sClassName);
        // // } else

        // if (mPeriodicIO.op_POV90_ManualShot_Tarmac) {
        // mSuperstructure.setManualShootDistance(60); //5 feet away: temporary test
        // value
        // mSuperstructure.setWantedState(Superstructure.WantedState.MANUAL_SHOOT,
        // sClassName);
        // } else if (mPeriodicIO.op_RightTrigger_Collect) {
        // mSuperstructure.setWantedState(Superstructure.WantedState.COLLECT,
        // sClassName);
        // } else if (mPeriodicIO.op_LeftTrigger_Back) {
        // mSuperstructure.setWantedState(Superstructure.WantedState.BACK, sClassName);
        // } else if (previousState != currentState) {
        // // mSuperstructure.setWantedState(Superstructure.WantedState.HOLD,
        // sClassName);
        // }
        // }
    }

    // private Superstructure.WantedState
    // activeBtnIsReleased(Superstructure.WantedState currentState) {
    // switch (currentState) {
    // // case MANUAL_SHOOT:
    // // return !mPeriodicIO.op_POV0_ManualShot_Fender ?
    // Superstructure.WantedState.HOLD : currentState;
    // case COLLECT:
    // return !mPeriodicIO.op_RightTrigger_Collect ? Superstructure.WantedState.HOLD
    // : currentState;
    // case BACK:
    // return !mPeriodicIO.op_LeftTrigger_Back ? Superstructure.WantedState.HOLD :
    // currentState;
    // default:
    // return Superstructure.WantedState.HOLD;
    // }
    // }

    @Override
    public void readPeriodicInputs() {
        double now = Timer.getFPGATimestamp();

        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart = now;

        //Driving
        mPeriodicIO.dr_LeftStickX_Translate = -mDriver.getRaw(Xbox.LEFT_STICK_X, mDeadBand);
        mPeriodicIO.dr_LeftStickY_Translate = -mDriver.getRaw(Xbox.LEFT_STICK_Y, mDeadBand);
        mPeriodicIO.dr_RightStickX_Rotate = -mDriver.getRaw(Xbox.RIGHT_STICK_X, mDeadBand);
        mPeriodicIO.dr_LeftTrigger_SlowSpeed = mDriver.getButton(Xbox.LEFT_TRIGGER, CW.PRESSED_LEVEL);
        mPeriodicIO.dr_RightBumper_RobotOrient = mDriver.getButton(Xbox.RIGHT_BUMPER, CW.PRESSED_LEVEL); // field/robot
        mPeriodicIO.dr_YButton_ResetIMU = mDriver.getButton(Xbox.Y_BUTTON, CW.PRESSED_EDGE);
        mPeriodicIO.dr_AButton_ToggleDriveMode = mDriver.getButton(Xbox.A_BUTTON, CW.PRESSED_EDGE);
        mPeriodicIO.dr_StartButton_ResetWheels = mDriver.getButton(Xbox.START_BUTTON, CW.PRESSED_EDGE);

        //Climbing
        mPeriodicIO.op_LeftStickY_ClimberElevator = -mOperator.getRaw(Xbox.LEFT_STICK_Y, mDeadBand);
        mPeriodicIO.op_AButton_ClimberLockout = mOperator.getButton(Xbox.A_BUTTON, CW.PRESSED_LEVEL);
        mPeriodicIO.op_LeftBumper_RetractSlappySticks = mOperator.getButton(Xbox.LEFT_BUMPER, CW.PRESSED_EDGE);
        mPeriodicIO.op_RightBumper_ExtendSlappySticks = mOperator.getButton(Xbox.RIGHT_BUMPER, CW.PRESSED_EDGE);

        //Collecting
        mPeriodicIO.op_RightTrigger_Collect = mOperator.getButton(Xbox.RIGHT_TRIGGER, CW.PRESSED_EDGE);
        mPeriodicIO.op_RightTrigger_Collect_Stop = mOperator.getButton(Xbox.RIGHT_TRIGGER, CW.RELEASED_EDGE);

        mPeriodicIO.op_LeftTrigger_Back = mOperator.getButton(Xbox.LEFT_TRIGGER, CW.PRESSED_EDGE);
        mPeriodicIO.op_LeftTrigger_Back_Stop = mOperator.getButton(Xbox.LEFT_TRIGGER, CW.RELEASED_EDGE);

        //Shooting
        mPeriodicIO.op_BButton_StopShooter = mOperator.getButton(Xbox.B_BUTTON, CW.PRESSED_EDGE);

        mPeriodicIO.op_POV0_ManualShot_Fender = mOperator.getButton(Xbox.POV0_0, CW.PRESSED_EDGE);
        mPeriodicIO.op_POV90_ManualShot_Ball = mOperator.getButton(Xbox.POV0_90, CW.PRESSED_EDGE);
        mPeriodicIO.op_POV180_ManualShot_Robot = mOperator.getButton(Xbox.POV0_180, CW.PRESSED_EDGE);
        mPeriodicIO.op_POV270_ManualShot_Tarmac = mOperator.getButton(Xbox.POV0_270, CW.PRESSED_EDGE);

        mPeriodicIO.op_POV0_ManualShot_Fender_Stop = mOperator.getButton(Xbox.POV0_0, CW.RELEASED_EDGE);
        mPeriodicIO.op_POV90_ManualShot_Ball_Stop = mOperator.getButton(Xbox.POV0_90, CW.RELEASED_EDGE);
        mPeriodicIO.op_POV180_ManualShot_Robot_Stop = mOperator.getButton(Xbox.POV0_180, CW.RELEASED_EDGE);
        mPeriodicIO.op_POV270_ManualShot_Tarmac_Stop = mOperator.getButton(Xbox.POV0_270, CW.RELEASED_EDGE);

        mPeriodicIO.tst_AButton_AutoElev = mTest.getButton(LogitechPS4.A_BUTTON, CW.PRESSED_EDGE);
        mPeriodicIO.tst_AButton_AutoElev_Stop = mTest.getButton(LogitechPS4.A_BUTTON, CW.RELEASED_EDGE);
        mPeriodicIO.tst_BButton_AutoPre = mTest.getButton(LogitechPS4.B_BUTTON, CW.PRESSED_EDGE);
        mPeriodicIO.tst_BButton_AutoPre_Stop = mTest.getButton(LogitechPS4.B_BUTTON, CW.RELEASED_EDGE);
    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case DISABLE:
                return SystemState.DISABLING;
            case READBUTTONS:
            default:
                return SystemState.READINGBUTTONS;
        }
    }

    @Override
    public String getLogHeaders() {
        return "Jsticks";
    }

    @Override
    public String getLogValues(boolean telemetry) {
        return "Jsticks.Values";
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
        private final int mDefaultSchedDelta = 100; // axis updated every 100 msec
        public int schedDeltaDesired;
        public double schedDeltaActual;
        public double schedDuration;
        private double lastSchedStart;

        // Joystick Inputs
        public double dr_LeftStickX_Translate; // drive
        public double dr_LeftStickY_Translate; // drive
        public double dr_RightStickX_Rotate; // drive
        public boolean dr_RightTrigger_AutoShoot = false;
        public boolean dr_LeftTrigger_SlowSpeed = false;
        public boolean dr_RightBumper_RobotOrient = false; // field/robot oriented
        public boolean dr_YButton_ResetIMU = false; // reset direction
        public boolean dr_AButton_ToggleDriveMode = false;
        public boolean dr_StartButton_ResetWheels = false;

        public double op_LeftStickY_ClimberElevator;
        public boolean op_RightTrigger_Collect = false;
        public boolean op_RightTrigger_Collect_Stop = false;
        public boolean op_LeftTrigger_Back = false;
        public boolean op_LeftTrigger_Back_Stop = false;
        public boolean op_BButton_StopShooter = false;
        public boolean op_AButton_ClimberLockout = false;
        public boolean op_LeftBumper_RetractSlappySticks = false;
        public boolean op_RightBumper_ExtendSlappySticks = false;

        public boolean op_POV0_ManualShot_Fender = false;
        public boolean op_POV90_ManualShot_Ball = false;
        public boolean op_POV180_ManualShot_Robot = false;
        public boolean op_POV270_ManualShot_Tarmac = false;
        public boolean op_ManualShoot = false; // Move to Pov once read

        public boolean op_POV0_ManualShot_Fender_Stop = false;
        public boolean op_POV90_ManualShot_Ball_Stop = false;
        public boolean op_POV180_ManualShot_Robot_Stop = false;
        public boolean op_POV270_ManualShot_Tarmac_Stop = false;

        public boolean tst_AButton_AutoElev = false;
        public boolean tst_AButton_AutoElev_Stop = false;
        public boolean tst_BButton_AutoPre = false;
        public boolean tst_BButton_AutoPre_Stop = false;
    }
}