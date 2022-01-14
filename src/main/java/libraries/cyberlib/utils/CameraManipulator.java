package libraries.cyberlib.utils;

import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.UsbCameraInfo;
//import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.cscore.VideoMode;
//import edu.wpi.first.cscore.VideoProperty;
//import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
//import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class CameraManipulator {

    public UsbCamera startCamera(String name, String path){
        UsbCamera camera = new UsbCamera("temp_"+name, path);

        if (uploadVideoModes(camera)) {
            camera.close();
            camera = CameraServer.startAutomaticCapture(name, path);
            setVideoModeFromDashboard(camera);
            dumpCameraData(camera);
        } else {
            camera.close();
            camera = null;
        }
        return camera;
    }

    public boolean uploadVideoModes(UsbCamera camera){
        boolean success = false;

        //CameraServerJNI.setTelemetryPeriod(1); // needed for actualFPS call below

        try {
            System.out.println(camera.getName()+" is "+camera.getDescription());
            
            // this will throw an exception if camera is not detected
            // VideoMode[] videoModes = camera.enumerateVideoModes();

            // uncomment to print modes
            // String modes = "";
            
            // int cc = 0;
            // for (VideoMode v:videoModes){
            //     modes += (cc++)+": "+v.width+", "+v.height+", "+v.fps+", "+v.pixelFormat.toString()+"\n";
            // }

            // System.out.println(modes);
            //SmartDashboard.putString("VideoModes for "+camera.getName(), modes); // not using, only printing
            success = true;

        } catch (Exception e) {
            System.out.println("problem finding camera");
        }

        return success;
    }

    public boolean setVideoModeFromDashboard(UsbCamera camera){
        boolean success = false;
        final int defaultVideoModeIndex = 46;
        final int invalidModeIndex = -1;

        try {
            String str = "Input mode index for ";
            VideoMode[] videoModes = camera.enumerateVideoModes();

            int desiredModeIndex = (int) SmartDashboard.getNumber(str+camera.getName(),invalidModeIndex);

            if (desiredModeIndex >= videoModes.length || desiredModeIndex < 0){
                desiredModeIndex = defaultVideoModeIndex;
            }

            camera.setVideoMode(videoModes[desiredModeIndex]);
            displayCurrentVideoMode(camera);
            SmartDashboard.putNumber(str+camera.getName(), desiredModeIndex);
            success = true;
        } catch (Exception e) {
            System.out.println("problem finding camera");
        }

        return success;
    }

    public void displayCurrentVideoMode(UsbCamera camera){

        VideoMode current = camera.getVideoMode();
        VideoMode[] videoModes = camera.enumerateVideoModes();
        int i;

        for (i=0; i<videoModes.length; i++){
            VideoMode vm = videoModes[i];
            if (vm.width != current.width)
                continue;
            if (vm.height != current.height)
                continue;
            if (vm.fps != current.fps)
                continue;
            if (!vm.pixelFormat.toString().equals(current.pixelFormat.toString()))
                continue;
            break;
        }

        if (i < videoModes.length){
            String c = i+": "+current.width+", "+current.height+", "+current.fps+", "+current.pixelFormat.toString();
            SmartDashboard.putString("current mode for "+camera.getName(), c);
            System.out.println(c);
        }
        else{
            SmartDashboard.putString("current mode for "+camera.getName(), "unknown");
            System.out.println("unknown");
        }
    }

    public void setBrightness(UsbCamera camera){

        int desiredBrightness = (int) SmartDashboard.getNumber("Camera Brightness",-1);

        // first time so upload to dashboard
        if (desiredBrightness == -1){
            SmartDashboard.putNumber("Camera Brightness", 50);
            desiredBrightness = 50;
        }

        camera.setBrightness(desiredBrightness);
        
    }

    public void setExposure(UsbCamera camera){

        int desiredExposure = (int) SmartDashboard.getNumber("Camera Exposure",-1);

        // first time so upload to dashboard
        if (desiredExposure == -1){
            SmartDashboard.putNumber("Camera Exposure", 50);
            desiredExposure = -2;
        }

        switch (desiredExposure){
            case -2:
            camera.setExposureAuto();
            break;
            case -3:
            camera.setExposureHoldCurrent();
            break;
            default:
            camera.setExposureManual(desiredExposure);
            break;
        }

    }

    public void setWhiteBalance(UsbCamera camera){

        int desiredWhiteBalance = (int) SmartDashboard.getNumber("Camera WhiteBalance",-1);

        // first time so upload to dashboard
        if (desiredWhiteBalance == -1){
            SmartDashboard.putNumber("Camera WhiteBalance", 50);
            desiredWhiteBalance = -2;
        }

        switch (desiredWhiteBalance){
            case -2:
            camera.setWhiteBalanceAuto();
            break;
            case -3:
            camera.setWhiteBalanceHoldCurrent();
            break;
            default:
            camera.setWhiteBalanceManual(desiredWhiteBalance);
            break;
        }
    }

    public void dumpCameraData(UsbCamera camera){

        //camera.setConnectionStrategy(ConnectionStrategy.kForceClose);

        // camera.setConnectVerbose(1);
        VideoSource.Kind k = camera.getKind();
        System.out.println("camera Kind = "+k.toString());

        UsbCameraInfo[] uci = UsbCamera.enumerateUsbCameras();

        for (UsbCameraInfo u:uci){
            System.out.println("name = "+u.name+", path = "+u.path);
        }

        // VideoProperty[] vp =  camera.enumerateProperties();

        // if (vp != null){
        //     for (VideoProperty v:vp){
        //         if (v != null)
        //             System.out.println("video property = "+ v.getName()+" "+v.get());//+" ,"+v.getString());
        //     }
        // }

        // VideoSink[] vs = camera.enumerateSinks();

        // for (VideoSink v:vs){
        //     vp = v.enumerateProperties();
        //     for (VideoProperty v2:vp){
        //         System.out.println("video sinks"+v2.getName()+" "+v2.get());//+", "+v2.getString());
        //     }
        // }
        // CameraServerJNI.setTelemetryPeriod(1); // needed for actualFPS call below
        // System.out.println("dataRate "+camera.getActualDataRate());
        // System.out.println("dataFPS "+camera.getActualFPS());

        // try {
        //     CameraServerJNI.setTelemetryPeriod(1); // needed for actualFPS call below
        //     System.out.println("dataRate "+camera.getActualDataRate());
        //     System.out.println("dataFPS "+camera.getActualFPS());
        // } catch (VideoException e) {
        //     CameraServerJNI.setTelemetryPeriod(1); // needed for actualFPS call below
        //     System.out.println("dataRate "+camera.getActualDataRate());
        //     System.out.println("dataFPS "+camera.getActualFPS());
        // }

        System.out.println("Host name = "+CameraServerJNI.getHostname());
    }
}
