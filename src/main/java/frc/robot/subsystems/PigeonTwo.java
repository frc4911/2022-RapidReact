package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.Constants;
import frc.robot.Ports;
import libraries.cheesylib.geometry.Rotation2d;

public class PigeonTwo{

    private static PigeonTwo instance = null;

    private Pigeon2 pigeon;

    private PigeonTwo(){
        try{
            pigeon = new Pigeon2(Ports.PIGEON);
        }catch(Exception e){
            System.out.println(e);
        }
    }

    public static PigeonTwo getInstance(){
        if(instance == null){
            instance = new PigeonTwo();
        }
        return instance;
    }

    public double getYaw(){
        return pigeon.getYaw();
    }

    public Rotation2d getRotation2dYaw(){
        return Rotation2d.fromDegrees(pigeon.getYaw());
    }

    public double getPitch(){
        return pigeon.getPitch();
    }

    public double getRoll(){
        return pigeon.getRoll();
    }

    public double[] getYPR(){
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr;
    }

    public void setAngle(double angle){
        pigeon.setYaw(-angle, Constants.kLongCANTimeoutMs);
        System.out.println("Pigeon angle set to: " + angle);
    }

}