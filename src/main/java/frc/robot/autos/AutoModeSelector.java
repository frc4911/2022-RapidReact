package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autos.modes.DoNothingMode;
import frc.robot.autos.modes.DriveBackwardMode;
import frc.robot.autos.modes.DriveForwardMode;
import frc.robot.autos.modes.ShootOneAndDriveBackwardMode;
import frc.robot.autos.modes.TestTrajectoryFollowingMode;
import frc.robot.autos.modes.ThreeBallMode;
import frc.robot.autos.modes.FiveBallMode;
import frc.robot.autos.modes.TwoBallLeftMode;
import frc.robot.autos.modes.TwoBallMode;
import libraries.cheesylib.autos.AutoModeBase;

import java.util.Optional;

public class AutoModeSelector {
    public enum DesiredMode {
        DO_NOTHING,
        TEST_TRAJECTORY_FOLLOWING_MODE,
        DRIVE_FORWARD_MODE,
        DRIVE_BACKWARD_MODE,
        SHOOT_ONE_AND_DRIVE_BACKWARD_MODE,
        TWO_BALL_MODE,
        THREE_BALL_MODE,
        FIVE_BALL_MODE
    }

    private SendableChooser<DesiredMode> mModeChooser;

    private DesiredMode mCachedDesiredMode = null;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    public AutoModeSelector() {
        mModeChooser = new SendableChooser<DesiredMode>();
        mModeChooser.setDefaultOption("None", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Test Trajectory Following Mode", DesiredMode.TEST_TRAJECTORY_FOLLOWING_MODE);
        mModeChooser.addOption("Drive Forward Mode", DesiredMode.DRIVE_FORWARD_MODE);
        mModeChooser.addOption("Drive Backward Mode", DesiredMode.DRIVE_BACKWARD_MODE);
        mModeChooser.addOption("Shoot One and Drive Backward", DesiredMode.SHOOT_ONE_AND_DRIVE_BACKWARD_MODE);
        mModeChooser.addOption("Two Ball Auto Mode", DesiredMode.TWO_BALL_MODE);
        mModeChooser.addOption("Three Ball Auto Mode", DesiredMode.THREE_BALL_MODE);
        mModeChooser.addOption("Five Ball Auto Mode", DesiredMode.FIVE_BALL_MODE);

        SmartDashboard.putData("Auto Mode", mModeChooser);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();

        if (desiredMode == null) {
            desiredMode = DesiredMode.DO_NOTHING;
        }

        if (mCachedDesiredMode != desiredMode) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name());
            mAutoMode = getAutoModeForParams(desiredMode);
        }
        mCachedDesiredMode = desiredMode;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode) {
        switch (mode) {
            case DO_NOTHING:
                return Optional.of(new DoNothingMode());
            case TEST_TRAJECTORY_FOLLOWING_MODE:
                return Optional.of(new TestTrajectoryFollowingMode());
            case DRIVE_FORWARD_MODE:
                return Optional.of(new DriveForwardMode());
            case DRIVE_BACKWARD_MODE:
                return Optional.of(new DriveBackwardMode());
            case SHOOT_ONE_AND_DRIVE_BACKWARD_MODE:
                return Optional.of(new ShootOneAndDriveBackwardMode());
            case TWO_BALL_MODE:
                return Optional.of(new TwoBallLeftMode());
            case THREE_BALL_MODE:
                return Optional.of(new ThreeBallMode());
            case FIVE_BALL_MODE:
                return Optional.of(new FiveBallMode());
            default:
                break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        return mAutoMode;
    }
}
