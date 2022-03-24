package frc.robot.states;

public class LEDState {
    public static final LEDState kOff = new LEDState(0.0, 0.0, 0.0);

//    public static final LEDState kIntakeOpenLoop = new LEDState(0.0, 0.0, 0.0);

    // Collector  State
    //  Deployed = Blue
    //  Indexer = Orange
    public static final LEDState kCollectorDeployed = new LEDState(1.0, 0.0, 0.0);
    public static final LEDState kIndexerLoaded = new LEDState(0.0, 0.25, 1.0);

    // Drive (purple)
    public static final LEDState kDrive = new LEDState(0.5, 0.0, 0.5);

    // Shoot (green)
    public static final LEDState kShoot = new LEDState(0.0, 1.0, 0.0);

    // Red
    public static final LEDState kFault = new LEDState(0.0, 0.0, 1.0);

    // Dark orange
    public static final LEDState kClimbing = new LEDState(0.0, 0.3, 1.0);

    // Yellow
    public static final LEDState kAutoStart = new LEDState(0.0, 1.0, 1.0);

    public static final LEDState kControlPanelRed = new LEDState(0.0, 0.0, 1.0);
    public static final LEDState kControlPanelBlue = new LEDState(1.0, 0.0, 0.0);
    public static final LEDState kControlPanelGreen = new LEDState(0.0, 1.0, 0.0);
    public static final LEDState kControlPanelYellow = new LEDState(0.0, 1.0, 1.0);

    public static final LEDState kRobotZeroed = new LEDState(0.0, 1.0, 0.0);

    public LEDState() {
    }

    public LEDState(double b, double g, double r) {
        blue = b;
        green = g;
        red = r;
    }

    public void copyFrom(LEDState other) {
        this.blue = other.blue;
        this.green = other.green;
        this.red = other.red;
    }

    public double blue;
    public double green;
    public double red;
}
