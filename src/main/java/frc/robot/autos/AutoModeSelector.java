package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autos.modes.DoNothingMode;
import frc.robot.autos.modes.TestTrajectoryFollowingMode;
import libraries.cheesylib.autos.AutoModeBase;

public class AutoModeSelector {
    public enum AutoModeChoice {
        TEST_TRAJECTORY_FOLLOWING_MODE,
        NONE
    }

    private SendableChooser<AutoModeChoice> mAutoModeChooser;

    public AutoModeSelector() {
        mAutoModeChooser = new SendableChooser<AutoModeChoice>();
        mAutoModeChooser.addOption("Test Trajectory Following Mode", AutoModeChoice.TEST_TRAJECTORY_FOLLOWING_MODE);
        mAutoModeChooser.setDefaultOption("None", AutoModeChoice.NONE);
        SmartDashboard.putData("Auto Mode", mAutoModeChooser);
    }

    public AutoModeBase getSelectedAutoMode() {
        AutoModeChoice choice = mAutoModeChooser.getSelected();
        SmartDashboard.putString("Selected Auto Mode", choice.toString());
        switch (choice) {
            case TEST_TRAJECTORY_FOLLOWING_MODE:
                return new TestTrajectoryFollowingMode();
            default:
                return new DoNothingMode();
        }
    }
}
