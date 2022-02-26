package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.config.RobotConfiguration;
import libraries.cheesylib.loops.ILooper;
import libraries.cheesylib.loops.Loop;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.util.LatchedBoolean;
import libraries.cheesylib.util.TimeDelayedBoolean;
import libraries.cyberlib.control.SwerveHeadingController;
import libraries.cyberlib.io.CW;
import libraries.cyberlib.io.Xbox;
import libraries.cyberlib.utils.RobotName;

public class JSticks extends Subsystem{

    // Heading controller methods
    private final SwerveHeadingController mHeadingController = SwerveHeadingController.getInstance();
    private final LatchedBoolean shouldChangeHeadingSetpoint = new LatchedBoolean();
    private final TimeDelayedBoolean mShouldMaintainHeading = new TimeDelayedBoolean();

    public enum SystemState {
        READINGBUTTONS,
    }

    public enum WantedState {
        READBUTTONS,
    }

    private SystemState mSystemState = SystemState.READINGBUTTONS;
    private WantedState mWantedState = WantedState.READBUTTONS;
    @SuppressWarnings("unused")
    private boolean mStateChanged;
    private CW mDriver;
    private CW mOperator;
    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private final double mDeadBand = 0.15; // for the turnigy (driver) swerve controls
	private final Superstructure mSuperstructure;
    private final Swerve mSwerve;

    @SuppressWarnings("unused")
    private int mListIndex;

    private static String sClassName;
    private static int sInstanceCount;
    private static JSticks sInstance = null;
    public  static JSticks getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new JSticks(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+" getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private JSticks(String caller){
        sClassName = this.getClass().getSimpleName();
        mSuperstructure = Superstructure.getInstance(sClassName);
        mSwerve = Swerve.getInstance(sClassName);
        var mSwerveConfiguration = RobotConfiguration.getRobotConfiguration(RobotName.name).getSwerveConfiguration();
        mHeadingController.setPIDFConstants(
            mSwerveConfiguration.kSwerveHeadingKp,
            mSwerveConfiguration.kSwerveHeadingKi,
            mSwerveConfiguration.kSwerveHeadingKd,
            mSwerveConfiguration.kSwerveHeadingKf);
        // double testKp = SmartDashboard.getNumber("kP", -1.0);
        // if(testKp == -1.0){
        //     SmartDashboard.putNumber("kP", 0.0);
        // }
        mDriver = new Xbox();
        mOperator = new Xbox();

        printUsage(caller);
    }

    private Loop mLoop = new Loop(){
        
        @Override
        public void onStart(Phase phase){
            synchronized (JSticks.this) {
                mSystemState = SystemState.READINGBUTTONS;
                mWantedState = WantedState.READBUTTONS;
                mStateChanged = true;

                // Force true on first iteration of teleop periodic
                shouldChangeHeadingSetpoint.update(false);

                System.out.println(sClassName + " state " + mSystemState);
                switch (phase) {
                    case DISABLED:
                    case AUTONOMOUS: 
                        mPeriodicIO.schedDeltaDesired = 0; // goto sleep
                        break;
                    default:
                        mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
                        break;
                }
            }
        }

        @Override
        public void onLoop(double timestamp){
            synchronized (JSticks.this) {
                SystemState newState;
                switch (mSystemState) {
                case READINGBUTTONS:
                default:
                    newState = handleReadingButtons();
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

    @Override
    public void stop() {

    }

    private SystemState handleReadingButtons() {
        teleopRoutines();
        
        return defaultStateTransfer();
    }

    public void teleopRoutines() {
        Superstructure.WantedState currentState = mSuperstructure.getWantedState();
		Superstructure.WantedState previousState = currentState;

        //Swerve control
		double swerveYInput = mPeriodicIO.dr_LeftStickX_Translate;
		double swerveXInput = mPeriodicIO.dr_LeftStickY_Translate;
		double swerveRotationInput = mPeriodicIO.dr_RightStickX_Rotate;
 
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
                    false, isFieldOriented, true);
        } else {
            mSwerve.setTeleopInputs(swerveXInput, swerveYInput, swerveRotationInput,
                    false, isFieldOriented, false);
        }

		if (mPeriodicIO.dr_YButton_ResetIMU) {
            // Seems safest to disable heading controller if were resetting IMU.
            mHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.OFF);
            mSwerve.zeroSensors(Constants.kRobotStartingPose);
		}
        // END NEW SWERVE

        // CLIMBER CONTROL
        // -1: Do nothing
        //  0: Extend
        //  1: Retract
        int deploySlappyState = -1;
        if(mPeriodicIO.op_YButton_ExtendSlappySticks){
            deploySlappyState = 0;
        } else if(mPeriodicIO.op_XButton_RetractSlappySticks){
            deploySlappyState = 1;
        }
		mSuperstructure.setOpenLoopClimb(mPeriodicIO.op_LeftStickY_ClimberElevator, deploySlappyState);
        
        // Will add clauses for different shoot distances, aimed/auto shooting, auto climbing, and others
        currentState = activeBtnIsReleased(currentState);
        if (currentState == Superstructure.WantedState.HOLD) {
            if (mPeriodicIO.op_POV0_ManualShot_Fender) {
                mSuperstructure.setManualShootDistance(0);
                mSuperstructure.setWantedState(Superstructure.WantedState.MANUAL_SHOOT);
            } else if (mPeriodicIO.op_POV90_ManualShot_Tarmac) {
                mSuperstructure.setManualShootDistance(60); //5 feet away: temporary test value
                mSuperstructure.setWantedState(Superstructure.WantedState.MANUAL_SHOOT);
            } else if (mPeriodicIO.op_RightTrigger_Collect) {
                mSuperstructure.setWantedState(Superstructure.WantedState.COLLECT);
            } else if (mPeriodicIO.op_LeftTrigger_Back) {
                mSuperstructure.setWantedState(Superstructure.WantedState.BACK);
            } else if (previousState != currentState) {
                mSuperstructure.setWantedState(Superstructure.WantedState.HOLD);
            }

        }
	}

    private Superstructure.WantedState activeBtnIsReleased(Superstructure.WantedState currentState) {
        switch (currentState) {
            case MANUAL_SHOOT:
                return !mPeriodicIO.op_POV0_ManualShot_Fender ? Superstructure.WantedState.HOLD : currentState;
            case COLLECT:
                return !mPeriodicIO.op_RightTrigger_Collect ? Superstructure.WantedState.HOLD : currentState;
            case BACK:
                return !mPeriodicIO.op_LeftTrigger_Back ? Superstructure.WantedState.HOLD : currentState;
            default:
                return Superstructure.WantedState.HOLD;
        }
    }

    @Override
    public void readPeriodicInputs() {
        double now       = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart   = now;

        mPeriodicIO.dr_LeftStickX_Translate = -mDriver.getRaw(Xbox.LEFT_STICK_X, mDeadBand);
        mPeriodicIO.dr_LeftStickY_Translate = -mDriver.getRaw(Xbox.LEFT_STICK_Y, mDeadBand);
        mPeriodicIO.dr_RightStickX_Rotate = -mDriver.getRaw(Xbox.RIGHT_STICK_X, mDeadBand);
        mPeriodicIO.dr_LeftTrigger_SlowSpeed = mDriver.getButton(Xbox.LEFT_TRIGGER, CW.PRESSED_LEVEL);
        mPeriodicIO.dr_RightBumper_RobotOrient = mDriver.getButton(Xbox.RIGHT_BUMPER, CW.PRESSED_LEVEL); // field/robot oriented
        mPeriodicIO.dr_YButton_ResetIMU = mDriver.getButton(Xbox.Y_BUTTON, CW.PRESSED_EDGE);

        mPeriodicIO.op_LeftStickY_ClimberElevator = -mOperator.getRaw(Xbox.LEFT_STICK_Y, mDeadBand);
        mPeriodicIO.op_RightTrigger_Collect = mOperator.getButton(Xbox.RIGHT_TRIGGER, CW.PRESSED_LEVEL);
        mPeriodicIO.op_LeftTrigger_Back = mOperator.getButton(Xbox.LEFT_TRIGGER, CW.PRESSED_LEVEL);
        mPeriodicIO.op_LeftBumper_LoadBall = mOperator.getButton(Xbox.LEFT_BUMPER, CW.PRESSED_EDGE);
        mPeriodicIO.op_BButton_StopShooter = mOperator.getButton(Xbox.B_BUTTON, CW.PRESSED_EDGE);
        mPeriodicIO.op_XButton_RetractSlappySticks = mOperator.getButton(Xbox.X_BUTTON, CW.PRESSED_EDGE);
        mPeriodicIO.op_YButton_ExtendSlappySticks = mOperator.getButton(Xbox.Y_BUTTON, CW.PRESSED_EDGE);
        mPeriodicIO.op_POV0_ManualShot_Fender = mOperator.getButton(Xbox.POV0_0, CW.PRESSED_LEVEL);
        mPeriodicIO.op_POV90_ManualShot_Tarmac = mOperator.getButton(Xbox.POV0_90, CW.PRESSED_LEVEL);
    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
        case READBUTTONS:
        default:
            return SystemState.READINGBUTTONS;
        }
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        mListIndex = enabledLooper.register(mLoop);
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
    public int whenRunAgain () {
        return mPeriodicIO.schedDeltaDesired;
    }

    public static class PeriodicIO{
        //Logging
        private final int mDefaultSchedDelta = 100; // axis updated every 100 msec
        public  int    schedDeltaDesired;
        public  double schedDeltaActual;
        public  double schedDuration;
        private double lastSchedStart;

        //Joystick Inputs
        public double  dr_LeftStickX_Translate ; // drive
        public double  dr_LeftStickY_Translate; // drive
        public double  dr_RightStickX_Rotate;     // drive
        public boolean dr_RightTrigger_AutoShoot = false;
        public boolean dr_LeftTrigger_SlowSpeed = false;
        public boolean dr_RightBumper_RobotOrient = false; // field/robot oriented
        public boolean dr_YButton_ResetIMU = false;      // reset direction

        public double  op_LeftStickY_ClimberElevator;
        public boolean op_RightTrigger_Collect = false;
        public boolean op_LeftTrigger_Back = false;
        public boolean op_LeftBumper_LoadBall = false;
        public boolean op_BButton_StopShooter = false;
        public boolean op_XButton_RetractSlappySticks = false;
        public boolean op_YButton_ExtendSlappySticks = false;
        public boolean op_POV0_ManualShot_Fender = false;
        public boolean op_POV90_ManualShot_Tarmac = false;
        public boolean op_ManualShoot = false; // Move to Pov once read

    }
}