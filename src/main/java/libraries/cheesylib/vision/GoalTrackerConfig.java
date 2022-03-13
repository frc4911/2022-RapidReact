package libraries.cheesylib.vision;

public class GoalTrackerConfig {
    public double maxTrackerDistance  = 4.0;//9.0;
    public double maxGoalTrackAge = 2.5;
    public double maxGoalTrackSmoothingTime = 0.5;
    public double cameraFrameRate = 90.0;

    public GoalTrackerConfig(
            double maxTrackerDistance,
            double maxGoalTrackAge,
            double maxGoalTrackSmoothingTime,
            double cameraFrameRate) {
        // These came from Constants.java and should be initialized in Robot.java
        this.maxTrackerDistance = maxTrackerDistance;
        this.maxGoalTrackAge = maxGoalTrackAge;
        this.maxGoalTrackSmoothingTime = maxGoalTrackSmoothingTime;
        this.cameraFrameRate = cameraFrameRate;
    }
}
