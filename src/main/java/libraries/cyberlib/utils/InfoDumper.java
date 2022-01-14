package libraries.cyberlib.utils;

// import java.lang.reflect.Constructor;
// import java.lang.reflect.Field;
// import java.lang.reflect.Method;

public class InfoDumper {

    // private static String info = null;

    // public void dumpInfo(Class main, Class constants, Class leftDriveController, Class rightDriveController, Class operatorController){
    //     if (info == null) {
    //         info = "";
    //         info += getManifest(main);
    //         info += portMapping(constants);
    //         info += buttonMappings(leftDriveController, rightDriveController,  operatorController);
    //         info += "Robot name is "+RobotName.name+"\n";
    //     }
    //     System.out.println(info);
    // }

    // private String getManifest(Class main){
    //     ManifestManager m = new ManifestManager(main);
    //     String info = "---------Manifest----------\n";
    //     info += m.getManifestString() + "\n";
    //     return info;
    // }

    // private String portMapping(Class constants) {
    //     String info = "---------Port-Mappings---------\n";
    //     info += controllerMapper (constants);
    //     return info;
    // }

    // private String buttonMappings(Class leftDriveController, Class rightDriveController, Class operatorController) {
    //     String info = "----------Button-Mappings---------\n";

    //     info += controllerMapper (leftDriveController);
    //     info += controllerMapper (rightDriveController);
    //     info += controllerMapper (operatorController);
        
    //     return info;
    // }

    // private String controllerMapper (Class theClass){
    //     String info = "";

	// 	for(Constructor construct : theClass.getConstructors()) {
	// 		Annot4911 annot = (Annot4911)construct.getAnnotation(Annot4911.class);
	// 		if(annot != null) {
	// 			info += "------"+annot.description()+"-----\n";
	// 		}
	// 	}

	// 	for(Method method : theClass.getMethods()) {
	// 		Annot4911 annot = (Annot4911)method.getAnnotation(Annot4911.class);
	// 		if(annot != null) {
	// 			info += annot.description() +"\n";
	// 		}
	// 	}

	// 	for(Field field : theClass.getFields()) {
	// 		Annot4911 annot = (Annot4911)field.getAnnotation(Annot4911.class);
	// 		if(annot != null) {
	// 			info += annot.description() +"\n";
	// 		}
	// 	}

    //     return info;
    // }
}