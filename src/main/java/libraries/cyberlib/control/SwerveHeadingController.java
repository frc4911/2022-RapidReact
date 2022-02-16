package libraries.cyberlib.control;

import libraries.cyberlib.utils.Angles;

/**
 * Controls overall swerve heading of the robot.
 */
public class SwerveHeadingController {

    public enum HeadingControllerState {
        OFF,
        //SNAP, // for dpad snapping to cardinals
        MAINTAIN, // maintaining current heading while driving
    }

    private final PidController mHeadingController;

    private HeadingControllerState mHeadingControllerState = HeadingControllerState.OFF;

    private double lastTimeStamp = Double.POSITIVE_INFINITY;

    public SwerveHeadingController(PidGains gains) {
        mHeadingController = new PidController(gains);
        mHeadingController.setInputRange(0, 360);
        mHeadingController.setContinuous(true);
    }

    public HeadingControllerState getHeadingControllerState() {
        return mHeadingControllerState;
    }

    public void setHeadingControllerState(HeadingControllerState state) {
        mHeadingControllerState = state;
    }

    /**
     * Sets the goal position.
     * <p>
     * @param goalPositionInDegrees goal position in degrees
     */
    public void setGoal(double goalPositionInDegrees) {
        mHeadingController.setSetpoint(goalPositionInDegrees);
    }

    /**
     * Should be called from a looper at a constant dt
     */
    public double update(double currentAngleInDegrees, double timeStamp) {
        if (mHeadingControllerState == HeadingControllerState.OFF) {
            return 0;
        }

        var dt = timeStamp - lastTimeStamp;
        lastTimeStamp = timeStamp;

        if (dt < 1E-6) {
            dt = 1E-6;
        }

        // Normalize angle into -pi to pi range
        var angle = Angles.normalizeAngle(Math.toRadians(currentAngleInDegrees));

        return mHeadingController.calculate(Math.toDegrees(angle), dt);
    }
}