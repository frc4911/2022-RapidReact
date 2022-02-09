package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import libraries.cheesylib.loops.ILooper;
import libraries.cheesylib.loops.Loop;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.util.LatchedBoolean;
import libraries.cheesylib.util.TimeDelayedBoolean;
import libraries.cyberlib.control.SwerveHeadingController;
import libraries.cyberlib.io.CW;
import libraries.cyberlib.io.LogitechExtreme;
import libraries.cyberlib.io.Xbox;

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
    private LogitechExtreme mDriver2;

    private final double mDeadBand = 0.15; // for the turnigy (driver) swerve controls
	// private Superstructure mSuperstructure;
    private Swerve mSwerve;

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
        mHeadingController.setPIDFConstants(
            mSwerve.mFrontRight.mConstants.kSwerveHeadingKp,
            mSwerve.mFrontRight.mConstants.kSwerveHeadingKi,
            mSwerve.mFrontRight.mConstants.kSwerveHeadingKd,
            0);
        mDriver = new Xbox();
        mDriver2 = new LogitechExtreme();
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
                        schedDeltaDesired = 0; // goto sleep
                        break;
                    default:
                        schedDeltaDesired = mDefaultSchedDelta;
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
        // TODO Auto-generated method stub
        
    }

    private SystemState handleReadingButtons() {
        teleopRoutines();
        
        return defaultStateTransfer();
    }

    public void teleopRoutines() {
        //Swerve control
		double swerveYInput = dr_RightStickX_Translate;
		double swerveXInput = dr_RightStickY_Translate;
		double swerveRotationInput = dr_LeftStickX_Rotate;
        // brian temp debug code
        // mSwerve.passThru(swerveXInput, swerveYInput, swerveRotationInput);
//        mSwerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, dr_LeftToggleDown_RobotOrient, false);

        // NEW SWERVE
        boolean maintainHeading = mShouldMaintainHeading.update(swerveRotationInput == 0, 0.2);
        boolean changeHeadingSetpoint = shouldChangeHeadingSetpoint.update(maintainHeading);

        if (!maintainHeading) {
            mHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.OFF);
        } else if (changeHeadingSetpoint) {
            mHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.MAINTAIN);
            mHeadingController.setGoal(mSwerve.getHeading().getDegrees());
        }

        // brian field oriented driving is not working
        if (mHeadingController.getHeadingControllerState() != SwerveHeadingController.HeadingControllerState.OFF) {
            mSwerve.setTeleopInputs(swerveXInput, swerveYInput, mHeadingController.update(),false, !dr_LeftToggleDown_RobotOrient, true);
        } else {
            mSwerve.setTeleopInputs(swerveXInput, swerveYInput,swerveRotationInput,false, !dr_LeftToggleDown_RobotOrient, false);
        }

		if (dr_YButton_ResetIMU) {
            // Seems safest to disable heading controller if were resetting IMU.
            mHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.OFF);
            mSwerve.zeroSensors(Constants.kRobotStartingPose);
		}
        // brian temp debug
        // if(throttlePrints%printFreq==0){
        //     System.out.println("01 js teleopRoutines (x,y,z) ("+swerveXInput+","+swerveYInput+","+swerveRotationInput+")");
        // }
        // END NEW SWERVE
	}
        // brian temp debug
    // int throttlePrints;
    // final int printFreq = 10;

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
            dr_RightStickX_Translate = -mDriver2.getRaw(LogitechExtreme.X, mDeadBand);
            dr_RightStickX_Translate = Math.copySign(Math.pow(dr_RightStickX_Translate,2), dr_RightStickX_Translate);
            dr_RightStickY_Translate = -mDriver2.getRaw(LogitechExtreme.Y, mDeadBand);
            dr_RightStickY_Translate = Math.copySign(Math.pow(dr_RightStickY_Translate,2), dr_RightStickY_Translate);
            // brian make it easier to drive w/o rotate
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

        // brian temp debug
        // if(++throttlePrints%printFreq==0){
        //     System.out.println("00 js readPeriodicInputs (x,y,z) ("+dr_RightStickX_Translate+","+dr_RightStickY_Translate+","+dr_LeftStickX_Rotate+")");
        // }
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
