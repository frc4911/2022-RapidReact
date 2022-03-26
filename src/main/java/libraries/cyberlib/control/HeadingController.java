package libraries.cyberlib.control;

import frc.robot.constants.Constants;
import frc.robot.subsystems.Swerve;
import libraries.cheesylib.util.SynchronousPIDF;

/**
 * Controls overall swerve heading of the robot.
 */
public class HeadingController {
    public enum HeadingControllerState {
        OFF,
        MAINTAIN, // maintaining current heading while driving
    }

    private final Swerve mSwerve;
    private final SynchronousPIDF mPIDFController;
    private final String sClassName;
    private double mSetpoint = 0.0;
    private HeadingControllerState mHeadingControllerState = HeadingControllerState.OFF;

    public HeadingController(double kP, double kI, double kD, double kF) {
        sClassName = this.getClass().getSimpleName();
        mPIDFController = new SynchronousPIDF(kP, kI, kD, kF);
        mSwerve = Swerve.getInstance(sClassName);
    }

    public void setPIDFConstants(double kP, double kI, double kD, double kF) {
        mPIDFController.setPIDF(kP, kI, kD, kF);
    }

    public HeadingControllerState getHeadingControllerState() {
        return mHeadingControllerState;
    }

    public void setHeadingControllerState(HeadingControllerState state) {
        mHeadingControllerState = state;
    }

    /**
     * Sets the goal position.
     *
     * @param goalPositionInDegrees goal position in degrees
     */
    public void setGoal(double goalPositionInDegrees) {
        mSetpoint = goalPositionInDegrees;
    }

    public boolean isAtGoal() {
        return mPIDFController.onTarget(Constants.kSwerveHeadingControllerErrorTolerance);
    }

    public void reset() {
        mPIDFController.reset();
    }

    /**
     * Should be called from a looper at a constant dt
     */
    public double update() {
        mPIDFController.setSetpoint(mSetpoint);
        // double current_angle = Swerve.getInstance(sClassName).getHeading().getDegrees();
        double current_angle = mSwerve.getHeading().getDegrees();
        double current_error = mSetpoint - current_angle;

        if (current_error > 180) {
            current_angle += 360;
        } else if (current_error < -180) {
            current_angle -= 360;
        }

        if (mHeadingControllerState == HeadingControllerState.OFF) {
            return 0;
        }

        return mPIDFController.calculate(current_angle);
    }
}
