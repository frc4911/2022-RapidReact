package libraries.cyberlib.utils;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import edu.wpi.first.wpilibj.Filesystem;

public class RobotName {
    public static String name = "Robot1";

    public RobotName(String defaultName) {
        Scanner scn = null;
        name = defaultName;
        try {

            File path = Filesystem.getDeployDirectory();
            File f = new File(path.toPath() + "/RobotName.txt");

            if (f.exists()) {
                scn = new Scanner(f);
                name = scn.nextLine();
            }
            // TODO: Throw - need to add appropriate file on the roborio and possible create
            // dart classes

        } catch (FileNotFoundException e) {
            System.out.println("RobotName.txt not found, using default name");
        } finally {
            if (scn != null) {
                scn.close();
                scn = null;
            }
        }
        System.out.println("Robot name is " + name);
    }
}