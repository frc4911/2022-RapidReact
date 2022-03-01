package libraries.cyberlib.io;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

// TODO: make threadsafe
// TODO: add rescan

// this is the workhorse for our joystick interface
// CW is short for ControllerWrapper. CW is prefered because
// the button press constants below are used in the top
// level API and CW is compact

// one CW is created for each joystick
public class CW {
    // Logitech
    // Controller (Gamepad F310)

    // Logitech
    // Logitech Extreme 3D

    // Logitech
    // Logitech Attack 3

    // Thrustmaster
    // T.16000M

    // Turnigy
    // Evolution PRO

    // Xbox
    // Controller (Xbox One For Windows)

    // be careful changing these 3 numbers. Reletive size matters
    // these constants allow the buttons, axis, and POVs to all
    // be referenced with a single number
    // it makes the top level API compact
    protected static final int BUTTONS_OFFSET = 0;
    protected static final int AXIS_OFFSET = 1000;
    protected static final int POVS_OFFSET = 10000;

    protected final double AXISPRESSTHRESHOLD = .5;
    // joystick, slot (0-5) of the driverstation USB slots
    // and name of joystick supplied by manufacturer
    private Joystick mStick = null;
    private String mName;

    // these numbers are filled in by querying the
    // joystick instance
    private int mButtonCount = 0;
    private int mAxisCount = 0;
    private int mPOVCount = 0;

    // these are arrays of buttons, axis, and POVs
    // the size of the arrays depends on what the
    // joystick supports
    private JoystickButton[] mJButtons = null;
    private JoystickAxis[] mJAxis = null;
    private JoystickPOV[][] mJPOVs = null;

    // this is used to efficiently look for
    // desired joysticks that were not plugged in
    // at powerup
    // look for 1 missing joystick every 1/2 second
    private double mNextCheck;
    private static double mNextCheckBase = 0;
    private final double TIMETONEXTCHECK = .5;

    // JoystickFinder is a class that keeps a
    // list of all discovered joysticks
    private JoystickFinder mJoyFinder = null;
    private Rumbler mRumbler = null;

    // These are for the different types of button presses
    // edge means it just happened
    // level means current state
    // quick is <= .25 seconds
    // long is > .25 seconds
    public static final int PRESSED_EDGE = 0;
    public static final int RELEASED_EDGE = 1;
    public static final int PRESSED_LEVEL_LONG = 2;
    public static final int RELEASED_EDGE_QUICK = 3;
    public static final int RELEASED_EDGE_LONG = 4;
    public static final int PRESSED_LEVEL = 5;

    // constructor
    // name is the manufacturer's description
    public CW(String name) {
        this.mName = name;
        mJoyFinder = JoystickFinder.getInstance("CW");
        Joystick js = findJoystick(name);
        if (js != null) {
            // joystick found so set things up
            initialize(js);
        } else {
            // setup to check again in .5 seconds
            if (mNextCheckBase == 0) {
                mNextCheckBase = Timer.getFPGATimestamp() + TIMETONEXTCHECK;
            } else {
                mNextCheckBase += TIMETONEXTCHECK;
            }
            mNextCheck = mNextCheckBase;
        }
    }

    private Joystick findJoystick(String name) {
        return mJoyFinder.findThisJoystick(name);
    }

    private void initialize(Joystick newJoystick) {
        mStick = newJoystick;

        // get button count and create JoystickButton class
        // for each one
        mButtonCount = mStick.getButtonCount() + 1;
        if (mButtonCount > 1) {
            // buttons start counting at 1
            mJButtons = new JoystickButton[mButtonCount];
            for (int i = 1; i < mButtonCount; i++) {
                mJButtons[i] = new JoystickButton(mStick, i);
            }
        }

        // do the same for Axis
        mAxisCount = mStick.getAxisCount();
        if (mAxisCount > 0) {
            // axis start counting at 0
            mJAxis = new JoystickAxis[mAxisCount];
            for (int i = 0; i < mAxisCount; i++) {
                mJAxis[i] = new JoystickAxis(mStick, i);
            }
        }

        // and POV's
        // POV's have the added twist of having
        // 8 possible values each of which is
        // treated as a button
        mPOVCount = mStick.getPOVCount();
        if (mPOVCount > 0) {
            // pov start counting at 0
            mJPOVs = new JoystickPOV[mPOVCount][8];
            for (int i = 0; i < mPOVCount; i++) {
                for (int j = 0; j < 8; j++) {
                    mJPOVs[i][j] = new JoystickPOV(mStick, i, j * 45);
                }
            }
        }

        // allocate rumbler even if it is not supported
        mRumbler = new Rumbler(mStick.getPort());

        System.out.println(
                "found Joystick \'" + mStick.getName() + "\', slot " + mStick.getPort() + ", buttons " + mButtonCount
                        + " , axes " + mAxisCount + " , pov " + mPOVCount);
    }

    // before checking for button press etc. check if the
    // joystick has been found. This prevents the continuous
    // warning msg on the console
    public boolean joystickFound() {
        // mStick == null means the joystick has not yet been found
        // this happens if the joystick was not inserted at powerup
        if (mStick != null) {
            // process button press
            return true;
        }
        // else look for joystick if more than .5 seconds have
        // passed
        if (Timer.getFPGATimestamp() > mNextCheck) {
            mNextCheckBase += TIMETONEXTCHECK;
            mNextCheck = mNextCheckBase;

            Joystick js = findJoystick(mName);
            if (js != null) {
                // joystick found so set things up
                initialize(js);
                return true;
            }
        }
        return false;
    }

    // check for a button press
    public boolean getButton(final int inputNum, final int pressType) {
        if (joystickFound()) {
            return pressTypes(inputNum, pressType);
        }
        return false;
    }

    // this unpacks the inputNum to extract button, axis, or pov number.
    // if it is a POV then it also gets the angle
    // then it calls the joystick api to get the value
    // each press type has to be handled seperately
    public boolean pressTypes(int inputNum, int pressType) {
        if (inputNum >= POVS_OFFSET) {
            inputNum -= POVS_OFFSET;
            int povNum = inputNum / 10;
            int angleIndex = inputNum % 10;

            // this handles each angle seperately
            switch (pressType) {
                case CW.PRESSED_EDGE:
                    return mJPOVs[povNum][angleIndex].isPressedEdge();
                case CW.RELEASED_EDGE:
                    return mJPOVs[povNum][angleIndex].isReleasedEdge();
                case CW.PRESSED_LEVEL_LONG:
                    return mJPOVs[povNum][angleIndex].isPressedLevelLong();
                case CW.RELEASED_EDGE_QUICK:
                    return mJPOVs[povNum][angleIndex].isReleasedEdgeQuick();
                case CW.RELEASED_EDGE_LONG:
                    return mJPOVs[povNum][angleIndex].isReleasedEdgeLong();
                case CW.PRESSED_LEVEL:
                    return mJPOVs[povNum][angleIndex].isPressedLevel();
                default:
            }
        } else if (inputNum >= AXIS_OFFSET) {
            // this maps axis to buttons by using .5 as a cutoff
            inputNum -= AXIS_OFFSET;
            switch (pressType) {
                case CW.PRESSED_EDGE:
                    return mJAxis[inputNum].isPressedEdge();
                case CW.RELEASED_EDGE:
                    return mJAxis[inputNum].isReleasedEdge();
                case CW.PRESSED_LEVEL_LONG:
                    return mJAxis[inputNum].isPressedLevelLong();
                case CW.RELEASED_EDGE_QUICK:
                    return mJAxis[inputNum].isReleasedEdgeQuick();
                case CW.RELEASED_EDGE_LONG:
                    return mJAxis[inputNum].isReleasedEdgeLong();
                case CW.PRESSED_LEVEL:
                    return mJAxis[inputNum].isPressedLevel();
                default:
            }
        } else {
            switch (pressType) {
                case CW.PRESSED_EDGE:
                    return mJButtons[inputNum].isPressedEdge();
                case CW.RELEASED_EDGE:
                    return mJButtons[inputNum].isReleasedEdge();
                case CW.PRESSED_LEVEL_LONG:
                    return mJButtons[inputNum].isPressedLevelLong();
                case CW.RELEASED_EDGE_QUICK:
                    return mJButtons[inputNum].isReleasedEdgeQuick();
                case CW.RELEASED_EDGE_LONG:
                    return mJButtons[inputNum].isReleasedEdgeLong();
                case CW.PRESSED_LEVEL:
                    return mJButtons[inputNum].isPressedLevel();
                default:
            }
        }
        return false;
    }

    int timeCnt = 0;

    // this gets the raw value plus adds a deadband for AXIS.
    public double getRaw(int inputNum, double deadband) {
        if (joystickFound()) {
            return getRawValue(inputNum, deadband);
        }
        if (inputNum >= POVS_OFFSET) {
            return -1;
        }
        return 0;
    }

    // this gets the raw value.
    public double getRaw(int inputNum) {
        return getRaw(inputNum, 0);
    }

    // it maps buttons to 0 and 1.0
    private double getRawValue(int inputNum, double deadband) {
        if (inputNum >= POVS_OFFSET) {
            inputNum -= POVS_OFFSET;
            int povNum = inputNum / 10;
            int angleIndex = inputNum % 10;

            return mJPOVs[povNum][angleIndex].getRaw(deadband);
        } else if (inputNum >= AXIS_OFFSET) {
            inputNum -= AXIS_OFFSET;
            return mJAxis[inputNum].getRaw(deadband);
        } else {
            return mJButtons[inputNum].getRaw(deadband);
        }
    }

    // same logic as above for rumble
    public void rumble(final double rumblesPerSecond, final double numberOfSeconds) {
        if (joystickFound()) {
            mRumbler.rumble(rumblesPerSecond, numberOfSeconds);
        }
    }

    // This class handles all of the button/axis/pov inputs
    // it makes them all buttons and raw inputs
    // it implements all special button activations
    // One of these is created for every button, axis, POV angle
    abstract class InputLogic {
        final double QUICKDURATION = .25;
        double pressedTime = 0;
        boolean longPressActivated = false;
        boolean pressedEdgeLastState = false;
        boolean releasedEdgeLastState = false;
        boolean releasedEdgeQuickLastState = false;
        double releasedEdgeQuickLastPressedDuration = 0;
        boolean releasedEdgeLongLastState = false;
        double releasedEdgeLongLastPressedDuration = 0;

        public InputLogic() {
        }

        // these two routines are supplied to convert
        // axis and POVs into buttons
        // and buttons into raw
        abstract protected boolean isPressed();

        abstract public double getRaw(double deadband);

        // this tracks how long a button has been pressed
        private double getPressedDuration() {
            if (isPressed()) {
                if (pressedTime == 0) {
                    pressedTime = Timer.getFPGATimestamp();
                    return .0001; // small positive # implying isPressed
                }
                return Timer.getFPGATimestamp() - pressedTime;
            }
            pressedTime = 0;
            return 0;
        }

        // this is really isPressedRaw but I called
        // isPressedLevel to differentiate from getRaw for axis
        public boolean isPressedLevel() {
            return isPressed();
        }

        // this looks for an edge
        // i.e. was it pressed since it was last checked
        public boolean isPressedEdge() {
            boolean last = pressedEdgeLastState;
            pressedEdgeLastState = isPressed();
            return !last && pressedEdgeLastState;
        }

        // same for release
        public boolean isReleasedEdge() {
            boolean last = releasedEdgeLastState;
            releasedEdgeLastState = isPressed();
            return last && !releasedEdgeLastState;
        }

        // is it pressed for .25 seconds
        public boolean isPressedLevelLong() {
            double duration = getPressedDuration();

            if (duration > 0) {
                if (!longPressActivated) {
                    if (duration > QUICKDURATION) {
                        longPressActivated = true;
                        return true;
                    }
                }
            } else {
                longPressActivated = false;
            }
            return false;
        }

        // is released in < .25 seconds
        public boolean isReleasedEdgeQuick() {
            double pressedDuration = getPressedDuration();

            if (pressedDuration > 0) {
                releasedEdgeQuickLastState = true;
                releasedEdgeQuickLastPressedDuration = pressedDuration;
                return false;
            }
            // only get here if not pressed now
            boolean last = releasedEdgeQuickLastState;
            releasedEdgeQuickLastState = false;
            return last && releasedEdgeQuickLastPressedDuration <= QUICKDURATION;
        }

        // is released in > .25 seconds
        public boolean isReleasedEdgeLong() {
            double pressedDuration = getPressedDuration();

            if (pressedDuration > 0) {
                releasedEdgeLongLastState = true;
                releasedEdgeLongLastPressedDuration = pressedDuration;
                return false;
            }
            // only get here if not pressed now
            boolean last = releasedEdgeLongLastState;
            releasedEdgeLongLastState = false;
            return last && releasedEdgeLongLastPressedDuration > QUICKDURATION;
        }
    }

    // one created for each button
    class JoystickButton extends InputLogic {
        Joystick stick;
        int num;

        public JoystickButton(Joystick js, int buttonNum) {
            stick = js;
            num = buttonNum;
        }

        protected boolean isPressed() {
            return stick.getRawButton(num);
        }

        public double getRaw(double deadband) {
            return stick.getRawButton(num) ? 1.0 : 0;
        }
    }

    // one created for each axis
    class JoystickAxis extends InputLogic {
        Joystick stick;
        int num;

        public JoystickAxis(Joystick js, int axisNum) {
            stick = js;
            num = axisNum;
        }

        // turn an axis into a button
        protected boolean isPressed() {
            double value = stick.getRawAxis(num);
            return Math.abs(value) <= AXISPRESSTHRESHOLD ? false : true;
        }

        public double getRaw(double deadband) {
            double value = stick.getRawAxis(num);
            double scalar = 1 / (1 - deadband); // Calculates the input scalar with the given deadband
            return Math.abs(value) <= deadband ? 0
                    : (value > 0) ? (value - deadband) * scalar : (value + deadband) * scalar;
        }
    }

    // one created for each POV angle
    class JoystickPOV extends InputLogic {
        Joystick stick;
        int num;
        int angle;

        public JoystickPOV(Joystick js, int POVNum, int a) {
            stick = js;
            num = POVNum;
            angle = a;
        }

        // turn a POV into one of 8 buttons
        protected boolean isPressed() {
            double value = stick.getPOV(num);
            return value == angle ? true : false;
        }

        public double getRaw(double deadband) {
            return stick.getPOV(num);
        }
    }
}