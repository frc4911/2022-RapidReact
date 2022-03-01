package libraries.cyberlib.io;

public class LogitechExtreme extends CW {

    public static final int TRIGGER = CW.BUTTONS_OFFSET + 1;
    public static final int THUMB_BUTTON = CW.BUTTONS_OFFSET + 2;
    public static final int TOP_THREE = CW.BUTTONS_OFFSET + 3;
    public static final int TOP_FOUR = CW.BUTTONS_OFFSET + 4;
    public static final int TOP_FIVE = CW.BUTTONS_OFFSET + 5;
    public static final int TOP_SIX = CW.BUTTONS_OFFSET + 6;
    public static final int LEFT_SEVEN = CW.BUTTONS_OFFSET + 7;
    public static final int LEFT_EIGHT = CW.BUTTONS_OFFSET + 8;
    public static final int LEFT_NINE = CW.BUTTONS_OFFSET + 9;
    public static final int LEFT_TEN = CW.BUTTONS_OFFSET + 10;
    public static final int LEFT_ELEVEN = CW.BUTTONS_OFFSET + 11;
    public static final int LEFT_TWELVE = CW.BUTTONS_OFFSET + 12;

    public static final int X = CW.AXIS_OFFSET + 0;
    public static final int Y = CW.AXIS_OFFSET + 1;
    public static final int Z = CW.AXIS_OFFSET + 2;
    public static final int SLIDER = CW.AXIS_OFFSET + 2;

    // POV# + ANGLE/45 which is an index
    public static final int POV0_0 = CW.POVS_OFFSET + 0 + 0 / 45;
    public static final int POV0_45 = CW.POVS_OFFSET + 0 + 45 / 45;
    public static final int POV0_90 = CW.POVS_OFFSET + 0 + 90 / 45;
    public static final int POV0_135 = CW.POVS_OFFSET + 0 + 135 / 45;
    public static final int POV0_180 = CW.POVS_OFFSET + 0 + 180 / 45;
    public static final int POV0_225 = CW.POVS_OFFSET + 0 + 225 / 45;
    public static final int POV0_270 = CW.POVS_OFFSET + 0 + 270 / 45;
    public static final int POV0_315 = CW.POVS_OFFSET + 0 + 315 / 45;

    static String name = "Logitech Extreme 3D";

    public LogitechExtreme() {
        super(name);
    }
}