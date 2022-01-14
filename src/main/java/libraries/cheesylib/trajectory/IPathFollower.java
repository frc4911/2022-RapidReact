package libraries.cheesylib.trajectory;

import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Translation2d;
//import libraries.cheesylib.geometry.Twist2d;

public interface IPathFollower {
    public Translation2d steer(Pose2d current_pose);

    public boolean isDone();
}
