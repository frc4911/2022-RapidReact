package libraries.cyberlib.utils;

// import java.io.IOException;
// import java.util.*;
// import java.util.jar.*;

public class ManifestManager {
//     private String manifestStr;

//     public ManifestManager (Class main) {
//         readManifest(main);
//     }

//     private void readManifest(Class main) {
//        try {
//             String pathAndName =  main.getResource(main.getSimpleName() + ".class").getFile();
//             pathAndName = pathAndName.substring(pathAndName.indexOf(':')+1, pathAndName.lastIndexOf("!")) ;
            
//             JarFile jarfile = new JarFile(pathAndName);
            
//             Manifest manifest = jarfile.getManifest();
//             Attributes attrs = (Attributes) manifest.getMainAttributes();

//             manifestStr = "";
                
//             for (Iterator<?> it = attrs.keySet().iterator(); it.hasNext();) {
//                 Attributes.Name attrName = (Attributes.Name) it.next();
//                 String attrValue = attrs.getValue(attrName);
//                 manifestStr += attrName + ": " + attrValue + "\n";
//             }

//            jarfile.close();

//        } catch (IOException e) {
//            System.out.println(e);
//        }
//    }

//    public String getManifestString() {
//        return manifestStr;
//    }

}      