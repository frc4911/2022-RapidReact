package libraries.cheesylib.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Small class filled with static util methods for the SmartDashboard.
 */
public class SmartDashboardUtil {

    public static void deletePersistentKeys() {
        //SmartDashboard smartDashboard = SmartDashboard.ge`;
        for (String key : SmartDashboard.getKeys()) {
            if (SmartDashboard.isPersistent(key)) {
                SmartDashboard.delete(key);
            }
        }
    }

    public static void deleteAllKeys() {
        //SmartDashboard smartDashboard = new SmartDashboard();
        for (String key : SmartDashboard.getKeys()) {
            SmartDashboard.delete(key);
        }
    }
}
