package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.limelight.LimelightConfiguration;
import frc.robot.limelight.PipelineConfiguration;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.util.Units;
import libraries.cheesylib.vision.TargetInfo;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;


public class LimelightTest {
    static double[] nearCorners = new double[] {180.0, 15.0, 180.0, 19.0, 169.0, 19.0, 169.0, 15.0, 156.0, 17.0, 156.0, 20.0, 145.0, 22.0, 145.0, 19.0, 203.0, 24.0, 195.0, 22.0, 195.0, 19.0, 202.0, 19.0, 134.0, 25.0, 134.0, 27.0, 133.0, 28.0, 131.0, 28.0, 131.0, 25.0};
    static double[] farCorners = new double[] {112.0, 97.0, 112.0, 100.0, 107.0, 100.0, 107.0, 97.0, 100.0, 99.0, 100.0, 101.0, 95.0, 102.0, 95.0, 99.0, 138.0, 99.0, 138.0, 102.0, 135.0, 102.0, 135.0, 99.0, 126.0, 97.0, 126.0, 100.0, 123.0, 100.0, 123.0, 97.0};

    @Test
    public void test() {
        List<TargetInfo> mTargets = new ArrayList<>();
        PipelineConfiguration mPipelineConfig = Constants.kLowRes1xZoom;
        LimelightConfiguration mConfig = new LimelightConfiguration(
                1, // label id
                LimelightConfiguration.Type.Shooter,
                "Shooter Limelight #1", // name
                "limelight", // table name
                Units.inches_to_meters(26.25), // height
                Pose2d.identity(), // shooter to lens
                Rotation2d.fromDegrees(20.25), // horizontalPlaneToLens,
                65.0, //64.03840065743408,
                50.0 //50.34836606499798
        );

        var limelight = new Limelight(mConfig, Constants.kLowRes1xZoom);

        double acceptedError = 6.0;

        var topCorners = getTopCorners(limelight, nearCorners);

        var targets = limelight.getRawTargetInfos(topCorners,
                mPipelineConfig, mTargets, mConfig.getHorizontalFOV(), mConfig.getVerticalFOV());

        double[] distances0 = averageTargetInfos(targets, limelight.getLensHeight(),limelight.getHorizontalPlaneToLens());
        assertEquals(Units.feet_to_meters(3), distances0[0], acceptedError);


        topCorners = getTopCorners(limelight, farCorners);

        targets = limelight.getRawTargetInfos(topCorners,
                mPipelineConfig, mTargets, mConfig.getHorizontalFOV(), mConfig.getVerticalFOV());

        double[] distances1 = averageTargetInfos(targets, limelight.getLensHeight(),limelight.getHorizontalPlaneToLens());
        assertEquals(Units.feet_to_meters(14.5), distances1[0], acceptedError);

        var robotState = RobotState.getInstance("test");
        robotState.addVisionUpdate(Timer.getFPGATimestamp(), targets, limelight);

        // Get distance between LL and vision tape
        var aimingParams = robotState.getAimingParameters(-1,
                Constants.kMaxGoalTrackAge, Pose2d.identity());

        if (aimingParams.isPresent()) {
            // if (Constants.kIsHoodTuning) {
            var range = aimingParams.get().getRange();
            System.out.format("Range To Target = %.3f m, %.3f ft.\n", range, Units.meters_to_feet(range));

        }
    }

    /**
     * Returns raw top-left and top-right corners
     *
     * @return list of corners: index 0 - top left, index 1 - top right
     */
    private List<double[]> getTopCorners(Limelight limelight, double[] corners) {

        double[] xCorners = new double[corners.length / 2];
        double[] yCorners = new double[corners.length / 2];

        for (int i = 0; i < corners.length; i++) {
            if (i % 2 == 0) {
                xCorners[i / 2] = corners[i];
            } else {
                yCorners[i / 2] = corners[i];
            }
        }

        return limelight.extractTopCornersFromBoundingBoxes(xCorners, yCorners);
    }

    private List<double[]> getBottomCorners(Limelight limelight, double[] corners) {

        double[] xCorners = new double[corners.length / 2];
        double[] yCorners = new double[corners.length / 2];

        for (int i = 0; i < corners.length; i++) {
            if (i % 2 == 0) {
                xCorners[i / 2] = corners[i];
            } else {
                yCorners[i / 2] = corners[i];
            }
        }

        return limelight.extractBottomCornersFromBoundingBoxes(xCorners, yCorners);
    }


    private double[] averageTargetInfos(List<TargetInfo> targets, double cameraHeight, Rotation2d cameraPitch) {
        double x = 0;
        double y = 0;
        for (TargetInfo target : targets) {
            Translation2d distance = RobotState.getCameraToVisionTargetTranslation(target, cameraHeight, cameraPitch);
            assertNotNull(distance);
            x += distance.x();
            y += distance.y();
        }
        return new double[]{x/2, y/2};
    }

}