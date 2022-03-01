package libraries.cyberlib.io;

public class Xbox extends CW {

    public static final int A_BUTTON = CW.BUTTONS_OFFSET + 1;
    public static final int B_BUTTON = CW.BUTTONS_OFFSET + 2;
    public static final int X_BUTTON = CW.BUTTONS_OFFSET + 3;
    public static final int Y_BUTTON = CW.BUTTONS_OFFSET + 4;
    public static final int LEFT_BUMPER = CW.BUTTONS_OFFSET + 5;
    public static final int RIGHT_BUMPER = CW.BUTTONS_OFFSET + 6;
    public static final int BACK_BUTTON = CW.BUTTONS_OFFSET + 7;
    public static final int START_BUTTON = CW.BUTTONS_OFFSET + 8;

    public static final int LEFT_STICK_X = CW.AXIS_OFFSET + 0;
    public static final int LEFT_STICK_Y = CW.AXIS_OFFSET + 1;
    public static final int LEFT_TRIGGER = CW.AXIS_OFFSET + 2;
    public static final int RIGHT_TRIGGER = CW.AXIS_OFFSET + 3;
    public static final int RIGHT_STICK_X = CW.AXIS_OFFSET + 4;
    public static final int RIGHT_STICK_Y = CW.AXIS_OFFSET + 5;

    // POV# + ANGLE/45 which is an index
    public static final int POV0_0 = CW.POVS_OFFSET + 0 + 0 / 45;
    public static final int POV0_45 = CW.POVS_OFFSET + 0 + 45 / 45;
    public static final int POV0_90 = CW.POVS_OFFSET + 0 + 90 / 45;
    public static final int POV0_135 = CW.POVS_OFFSET + 0 + 135 / 45;
    public static final int POV0_180 = CW.POVS_OFFSET + 0 + 180 / 45;
    public static final int POV0_225 = CW.POVS_OFFSET + 0 + 225 / 45;
    public static final int POV0_270 = CW.POVS_OFFSET + 0 + 270 / 45;
    public static final int POV0_315 = CW.POVS_OFFSET + 0 + 315 / 45;

    static String name = "Controller (Xbox One For Windows)";

    public Xbox() {
        super(name);
    }
}