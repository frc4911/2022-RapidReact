package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import libraries.cheesylib.loops.ILooper;
import libraries.cheesylib.loops.Loop;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cyberlib.io.CW;
import libraries.cyberlib.io.Xbox;

public class JSticks extends Subsystem{

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
    private PeriodicIO mPeriodicIO;

    private CW mDriver;
    private CW mOperator;
    private final double mDeadBand = 0.15; // for the turnigy (driver) swerve controls
	private Superstructure mSuperstructure;
    private Swerve mSwerve;

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
        // TODO Auto-generated method stub
        
    }

    private SystemState handleReadingButtons() {
        teleopRoutines();
        
        return defaultStateTransfer();
    }

    public void teleopRoutines() {
        Superstructure.WantedState currentState = mSuperstructure.getWantedState();
		Superstructure.WantedState previousState = currentState;
        
        //Swerve control
		double swerveYInput = mPeriodicIO.dr_RightStickX_Translate;
		double swerveXInput = mPeriodicIO.dr_RightStickY_Translate;
		double swerveRotationInput = mPeriodicIO.dr_LeftStickX_Rotate;

        mSwerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, mPeriodicIO.dr_LeftTrigger_RobotOrient, false);

		if (mPeriodicIO.dr_YButton_ResetIMU) {
			mSwerve.temporarilyDisableHeadingController();
			mSwerve.zeroSensors(Constants.kRobotStartingPose);
			mSwerve.resetAveragedDirection();
		}
        // if (currentState == Superstructure.WantedState.CLIMB) {                             
		// 	mSuperstructure.setClimbOpenLoop(mPeriodicIO.opLeftStickY_ClimbSpeed);             
		// }

        currentState = activeBtnIsReleased(currentState);
		if (currentState == Superstructure.WantedState.HOLD) {
			if (mPeriodicIO.op_LeftTrigger_ManualShoot) {
				mSuperstructure.setWantedState(Superstructure.WantedState.MANUAL_SHOOT);
			} else if (mPeriodicIO.op_XButton_Collect) {
				mSuperstructure.setWantedState(Superstructure.WantedState.COLLECT);
			} else if (mPeriodicIO.op_YButton_SlappySticks) {
				mSuperstructure.setWantedState(Superstructure.WantedState.MANUAL_CLIMB);
			} else if (mPeriodicIO.op_AButton_Clear) {
                mSuperstructure.setWantedState(Superstructure.WantedState.CLEAR);
            } else if (previousState != currentState) {
				mSuperstructure.setWantedState(Superstructure.WantedState.HOLD);
			}
            //AUTO SHOOT AND CLIMB ARE MISSING BUT WE DONT KNOW HOW TO ADD THEM

        }

	}

    private Superstructure.WantedState activeBtnIsReleased(Superstructure.WantedState currentState) {
		switch (currentState) {
			case MANUAL_SHOOT:
				return !mPeriodicIO.op_LeftTrigger_ManualShoot ? Superstructure.WantedState.HOLD : currentState;
			case COLLECT:
				return !mPeriodicIO.op_XButton_Collect ? Superstructure.WantedState.HOLD : currentState;
			case MANUAL_CLIMB:
				return !mPeriodicIO.op_LeftTrigger_ManualShoot ? Superstructure.WantedState.HOLD : currentState;
			// case MANUAL_SHOOT:                                                                                         IDK ABOUT THIS EITHER -CALEB WEST
			// 	if (!mPeriodicIO.opPOV0_MANUAL10 && !mPeriodicIO.opPOV90_MANUAL15 && !mPeriodicIO.opPOV180_MANUAL20 && !mPeriodicIO.opPOV270_MANUAL25) {
			// 		return Superstructure.WantedState.HOLD;
			// 	}
			// 	return currentState;
			case CLEAR:
                return !mPeriodicIO.op_AButton_Clear ? Superstructure.WantedState.HOLD : currentState;
			default:
                return Superstructure.WantedState.HOLD;
        }        
    }

    @Override
    public void readPeriodicInputs() {
        double now       = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart   = now;

        mPeriodicIO.dr_RightStickX_Translate = mDriver.getRaw(Xbox.RIGHT_STICK_X, mDeadBand);
        mPeriodicIO.dr_RightStickY_Translate = mDriver.getRaw(Xbox.RIGHT_STICK_Y, mDeadBand);
        mPeriodicIO.dr_LeftStickX_Rotate = mDriver.getRaw(Xbox.LEFT_STICK_X, mDeadBand);
        mPeriodicIO.dr_YButton_ResetIMU = mDriver.getButton(Xbox.Y_BUTTON, CW.PRESSED_EDGE);
        mPeriodicIO.op_LeftStickY_ClimberArms = mOperator.getRaw(Xbox.LEFT_STICK_X, mDeadBand);
        mPeriodicIO.dr_LeftTrigger_RobotOrient = mDriver.getButton(Xbox.LEFT_TRIGGER, CW.PRESSED_EDGE); // field/robot oriented
        mPeriodicIO.op_XButton_Collect = mOperator.getButton(Xbox.X_BUTTON, CW.PRESSED_LEVEL);
        mPeriodicIO.op_AButton_Clear = mOperator.getButton(Xbox.A_BUTTON, CW.PRESSED_LEVEL);
        mPeriodicIO.op_YButton_SlappySticks = mOperator.getButton(Xbox.X_BUTTON, CW.PRESSED_EDGE);
        mPeriodicIO.op_BButton_StopShooter = mOperator.getButton(Xbox.B_BUTTON, CW.PRESSED_LEVEL);
        mPeriodicIO.op_LeftTrigger_ManualShoot = mOperator.getButton(Xbox.LEFT_TRIGGER, CW.PRESSED_LEVEL);
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
        return mPeriodicIO.schedDeltaDesired;
    }

    public static class PeriodicIO{
        //Logging
        private final int mDefaultSchedDelta = 100; // axis updated every 100 msec
        public  int    schedDeltaDesired;
        public  double schedDeltaActual;
        public  double schedDuration;
        private double lastSchedStart;

        //THESE VALUES ARE TEMPORARY AND WILL CHANGE ONCE RANIA GIVES HER PREFERENCES
        //Joystick Inputs
        public double  dr_RightStickX_Translate ; // drive
        public double  dr_RightStickY_Translate; // drive
        public double  dr_LeftStickX_Rotate;     // drive
        public double op_LeftStickY_ClimberArms;
        public boolean dr_YButton_ResetIMU = false;      // reset direction
        public boolean dr_LeftTrigger_RobotOrient = false; // field/robot oriented
        public boolean op_XButton_Collect = false;
        public boolean op_AButton_Clear = false;
        public boolean op_YButton_SlappySticks = false;
        public boolean op_BButton_StopShooter = false;
        public boolean op_LeftTrigger_ManualShoot = false;
    }
}
