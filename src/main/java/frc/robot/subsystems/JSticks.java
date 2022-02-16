package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import libraries.cheesylib.loops.ILooper;
import libraries.cheesylib.loops.Loop;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.util.LatchedBoolean;
import libraries.cheesylib.util.TimeDelayedBoolean;
import libraries.cyberlib.control.PidGains;
import libraries.cyberlib.control.SwerveHeadingController;
import libraries.cyberlib.io.CW;
import libraries.cyberlib.io.LogitechExtreme;
import libraries.cyberlib.io.Xbox;

public class JSticks extends Subsystem{

    // Heading controller methods
    private final SwerveHeadingController mHeadingController;
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
    private final CW mDriver;
    private CW mOperator;
    private final LogitechExtreme mDriver2;

    private final double mDeadBand = 0.15; // for the turnigy (driver) swerve controls
	// private Superstructure mSuperstructure;
    private final Swerve mSwerve;

    //Logging
    private final int mDefaultSchedDelta = 100; // axis updated every 100 msec
    public  int    schedDeltaDesired;
    public  double schedDeltaActual;
    public  double schedDuration;
    private double lastSchedStart;

    //Joystick Inputs
    public double  dr_RightStickX_Translate; // drive
    public double  dr_RightStickY_Translate; // drive
    public double  dr_LeftStickX_Rotate;     // drive
    public boolean dr_YButton_ResetIMU = false;      // reset direction
    public boolean dr_LeftToggleDown_RobotOrient = false; // field/robot oriented
    private double controlledSpeed = .4;

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
        // mSuperstructure = Superstructure.getInstance(sClassName);
        mSwerve = Swerve.getInstance(sClassName);
        mHeadingController = new SwerveHeadingController(
                new PidGains(
                        mSwerve.mSwerveConfiguration.kSwerveHeadingKp,
                        mSwerve.mSwerveConfiguration.kSwerveHeadingKi,
                        mSwerve.mSwerveConfiguration.kSwerveHeadingKd
                ));
        mDriver = new Xbox();
        mDriver2 = new LogitechExtreme();
        mOperator = new Xbox();

        printUsage(caller);
    }

    private Loop mLoop = new Loop(){
        
        @Override
        public void onStart(Phase phase) {
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
                        schedDeltaDesired = 0; // goto sleep
                        break;
                    default:
                        schedDeltaDesired = mDefaultSchedDelta;
                        break;
                }
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (JSticks.this) {
                SystemState newState;
                switch (mSystemState) {
                case READINGBUTTONS:
                default:
                    newState = handleReadingButtons(timestamp);
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
        // TODO Auto-generated method stub
        
    }

    private SystemState handleReadingButtons(double timestamp) {
        teleopRoutines(timestamp);
        
        return defaultStateTransfer();
    }

    public void teleopRoutines(double timestamp) {
        //Swerve control
		double swerveYInput = dr_RightStickX_Translate;
		double swerveXInput = dr_RightStickY_Translate;
		double swerveRotationInput = dr_LeftStickX_Rotate;
 
        // NEW SWERVE
        boolean maintainHeading = mShouldMaintainHeading.update(swerveRotationInput == 0, 0.2);
        boolean changeHeadingSetpoint = shouldChangeHeadingSetpoint.update(maintainHeading);

        if (!maintainHeading) {
            mHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.OFF);
        } else if (changeHeadingSetpoint) {
            mHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.MAINTAIN);
            mHeadingController.setGoal(mSwerve.getHeading().getDegrees());
        }

        var isFieldOriented = !dr_LeftToggleDown_RobotOrient;
        if (mHeadingController.getHeadingControllerState() != SwerveHeadingController.HeadingControllerState.OFF) {
            mSwerve.setTeleopInputs(swerveXInput,
                    swerveYInput,
                    mHeadingController.update(mSwerve.getHeading().getDegrees(), timestamp),
                    false,
                    isFieldOriented,
                    true);
        } else {
            mSwerve.setTeleopInputs(swerveXInput,
                    swerveYInput,
                    swerveRotationInput,
                    false,
                    isFieldOriented,
                    false);
        }

		if (dr_YButton_ResetIMU) {
            // Seems safest to disable heading controller if were resetting IMU.
            mHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.OFF);
            mSwerve.zeroSensors(Constants.kRobotStartingPose);
		}
        // END NEW SWERVE
	}

    @Override
    public void readPeriodicInputs() {
        double now       = Timer.getFPGATimestamp();
        schedDeltaActual = now - lastSchedStart;
        lastSchedStart   = now;

        if (mDriver.joystickFound()) {
            dr_RightStickX_Translate = -mDriver.getRaw(Xbox.RIGHT_STICK_X, mDeadBand);
            dr_RightStickY_Translate = -mDriver.getRaw(Xbox.RIGHT_STICK_Y, mDeadBand);
            dr_LeftStickX_Rotate = mDriver.getRaw(Xbox.LEFT_STICK_X, mDeadBand);
            dr_YButton_ResetIMU = mDriver.getButton(Xbox.Y_BUTTON, CW.PRESSED_EDGE);
        }
        else {
            if (mDriver2.getButton(LogitechExtreme.TOP_FOUR, CW.PRESSED_EDGE)){
                controlledSpeed-=.01;
                System.out.println("Controlled speed is now "+controlledSpeed);
            }
            else if (mDriver2.getButton(LogitechExtreme.TOP_SIX, CW.PRESSED_EDGE)){
                controlledSpeed+=.01;
                System.out.println("Controlled speed is now "+controlledSpeed);
            }
            dr_LeftStickX_Rotate = 0; 
            if (mDriver2.getButton(LogitechExtreme.LEFT_SEVEN, CW.PRESSED_LEVEL)){
                dr_RightStickX_Translate = 0;
                dr_RightStickY_Translate = controlledSpeed;
            }
            else if (mDriver2.getButton(LogitechExtreme.LEFT_EIGHT, CW.PRESSED_LEVEL)){
                dr_RightStickX_Translate = -controlledSpeed;
                dr_RightStickY_Translate = 0;
            }
            else if (mDriver2.getButton(LogitechExtreme.LEFT_NINE, CW.PRESSED_LEVEL)){
                dr_RightStickX_Translate = controlledSpeed;
                dr_RightStickY_Translate = 0;
            }
            else if (mDriver2.getButton(LogitechExtreme.LEFT_TEN, CW.PRESSED_LEVEL)){
                dr_RightStickX_Translate = 0;
                dr_RightStickY_Translate = -controlledSpeed;
            }
            else {
                dr_RightStickX_Translate = -mDriver2.getRaw(LogitechExtreme.X, mDeadBand);
                dr_RightStickX_Translate = Math.copySign(Math.pow(dr_RightStickX_Translate,2), dr_RightStickX_Translate);
                dr_RightStickY_Translate = -mDriver2.getRaw(LogitechExtreme.Y, mDeadBand);
                dr_RightStickY_Translate = Math.copySign(Math.pow(dr_RightStickY_Translate,2), dr_RightStickY_Translate);
            }
            // make it easier to drive w/o rotate
            if (mDriver2.getButton(LogitechExtreme.TOP_THREE, CW.PRESSED_LEVEL)){
                dr_LeftStickX_Rotate = 0;    
            }
            else{
                dr_LeftStickX_Rotate = -mDriver2.getRaw(LogitechExtreme.Z, mDeadBand);
                dr_LeftStickX_Rotate = Math.copySign(Math.pow(dr_LeftStickX_Rotate,2), dr_LeftStickX_Rotate);
            }
            dr_YButton_ResetIMU = mDriver2.getButton(LogitechExtreme.THUMB_BUTTON, CW.PRESSED_EDGE);
            // hold trigger to switch to robot oriented
            dr_LeftToggleDown_RobotOrient = mDriver2.getButton(LogitechExtreme.TRIGGER, CW.PRESSED_LEVEL);
            
        }
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
        enabledLooper.register(mLoop);
    }

    @Override
    public String getLogHeaders() {
        // TODO Auto-generated method stub
        return "Jsticks";
    }

    @Override
    public String getLogValues(boolean telemetry) {
        // TODO Auto-generated method stub
        return "Jsticks.Values";
    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub
        
    }   

    @Override
    public int whenRunAgain () {
        return schedDeltaDesired;
    }
}
