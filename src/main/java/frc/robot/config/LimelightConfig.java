package frc.robot.config;

import frc.robot.constants.Constants.Target;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Rotation2d;

public class LimelightConfig {
    public String kName = "";
    public String kTableName = "";
    public double kHeight = 0.0;
    public Pose2d kSubsystemToLens = Pose2d.identity();
    public Rotation2d kHorizontalPlaneToLens = Rotation2d.identity();
    // assign a zoom to each pipeline in use (EVERY pipleine must have a zoom!!!)
    public int[] kPipelineZoom = new int[] { 1 }; // all pipeline zooms default to 1
    public double[] kExpectedTargetCount = new double[] { 0, 0 };
    public Target[] kTargets = new Target[0];
}