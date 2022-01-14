package libraries.cyberlib.io;

public class Thrustmaster extends CW{

    public static final int  BACK_BUTTON        = CW.BUTTONS_OFFSET + 1;
    public static final int  DOWN_BUTTON        = CW.BUTTONS_OFFSET + 2;
    public static final int  LEFT_BUTTON	    = CW.BUTTONS_OFFSET + 3;
    public static final int  RIGHT_BUTTON 	    = CW.BUTTONS_OFFSET + 4;
    public static final int  LEFT_FIVE          = CW.BUTTONS_OFFSET + 5;
    public static final int  LEFT_SIX           = CW.BUTTONS_OFFSET + 6;
    public static final int  LEFT_SEVEN         = CW.BUTTONS_OFFSET + 7;
	public static final int  LEFT_EIGHT         = CW.BUTTONS_OFFSET + 8;
	public static final int  LEFT_NINE          = CW.BUTTONS_OFFSET + 9;
	public static final int  LEFT_TEN           = CW.BUTTONS_OFFSET + 10;
	public static final int  RIGHT_ELEVEN       = CW.BUTTONS_OFFSET + 11;
    public static final int  RIGHT_TWELVE       = CW.BUTTONS_OFFSET + 12;
    public static final int  RIGHT_THIRTEEN     = CW.BUTTONS_OFFSET + 13;
	public static final int  RIGHT_FOURTEEN     = CW.BUTTONS_OFFSET + 14;
	public static final int  RIGHT_FIFTEEN      = CW.BUTTONS_OFFSET + 15;
	public static final int  RIGHT_SIXTEEN      = CW.BUTTONS_OFFSET + 16;

    public static final int  X = CW.AXIS_OFFSET + 0;
    public static final int  Y = CW.AXIS_OFFSET + 1;
    public static final int  Z = CW.AXIS_OFFSET + 2;

                                    //               POV# + ANGLE/45 which is an index
    public static final int POV0_0   = CW.POVS_OFFSET + 0 +   0/45;
    public static final int POV0_45  = CW.POVS_OFFSET + 0 +  45/45;
    public static final int POV0_90  = CW.POVS_OFFSET + 0 +  90/45;
    public static final int POV0_135 = CW.POVS_OFFSET + 0 + 135/45;
    public static final int POV0_180 = CW.POVS_OFFSET + 0 + 180/45;
    public static final int POV0_225 = CW.POVS_OFFSET + 0 + 225/45;
    public static final int POV0_270 = CW.POVS_OFFSET + 0 + 270/45;
    public static final int POV0_315 = CW.POVS_OFFSET + 0 + 315/45;

	static String name = "T.16000M";

    public Thrustmaster(){
		super(name);
	}
}