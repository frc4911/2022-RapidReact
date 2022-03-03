package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autos.modes.DoNothingMode;
import frc.robot.autos.modes.DriveBackwardMode;
import frc.robot.autos.modes.DriveForwardMode;
import frc.robot.autos.modes.TestTrajectoryFollowingMode;
import frc.robot.autos.modes.TwoBallMode;
import libraries.cheesylib.autos.AutoModeBase;

import java.util.Optional;

public class AutoModeSelector {
    public enum DesiredMode {
        DO_NOTHING,
        TEST_TRAJECTORY_FOLLOWING_MODE,
        DRIVE_FORWARD_MODE,
        DRIVE_BACKWARD_MODE,
        TWO_BALL_AUTO_MODE
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
        mModeChooser.addOption("Two Ball Auto Mode", DesiredMode.TWO_BALL_AUTO_MODE);
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
            case TWO_BALL_AUTO_MODE:
                return Optional.of(new TwoBallMode());
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
