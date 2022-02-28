package libraries.cyberlib.io;

public class Turnigy extends CW {

    public static final int MID_BUTTON = CW.BUTTONS_OFFSET + 7;

    public static final int RIGHT_STICK_X = CW.AXIS_OFFSET + 0;
    public static final int RIGHT_STICK_Y = CW.AXIS_OFFSET + 1;
    public static final int LEFT_STICK_Y = CW.AXIS_OFFSET + 2;
    public static final int LEFT_STICK_X = CW.AXIS_OFFSET + 3;
    public static final int LEFT_TOGGLE = CW.AXIS_OFFSET + 4;
    public static final int RIGHT_TOGGLE = CW.AXIS_OFFSET + 5;

    public static final int TOGGLE_UP = 0;
    public static final int TOGGLE_MID = 1;
    public static final int TOGGLE_DOWN = 2;

    // note Right and Left Toggle
    // up = -0.890625
    // mid = -0.046875
    // down = 0.890625

    public boolean getToggle(int leftRight, int position) {
        double value = getRaw(leftRight);
        if (position == TOGGLE_DOWN) {
            if (value > AXISPRESSTHRESHOLD) {
                return true;
            }
            return false;
        } else if (position == TOGGLE_UP) {
            if (value < -AXISPRESSTHRESHOLD) {
                return true;
            }
            return false;
        }

        if (value > -AXISPRESSTHRESHOLD && value < AXISPRESSTHRESHOLD) {
            return true;
        }
        return false;
    }

    static String name = "Evolution PRO";

    public Turnigy() {
        super(name);
    }
}