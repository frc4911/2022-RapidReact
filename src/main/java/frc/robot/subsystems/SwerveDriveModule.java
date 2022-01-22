package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import libraries.cheesylib.drivers.TalonFXFactory;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.loops.ILooper;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.util.Util;
import libraries.cyberlib.utils.CheckFaults;
import libraries.madtownlib.util.Utils;

/**
 * Represents a swerve module consisting of a drive motor that controls the speed of the wheel and
 * a rotation motor (aka. azimuth) that controls the angle the wheel is pointing towards relative to
 * the chassis frame.
 */
public class SwerveDriveModule extends Subsystem {

    // Hardware
    TalonFX rotationMotor, driveMotor;
    CANCoder canCoder;

    int moduleID;
    int encSlot;
    String name = "Module ";
    int rotationSetpoint = 0;
    double driveSetpoint = 0;
    int cancoderOffsetDegrees;
    int encoderReverseFactor = 1;
    boolean useDriveEncoder = true;
    boolean tenVoltRotationMode = false;
    private double previousEncDistance = 0;
    private Translation2d position;
    private Translation2d startingPosition;
    private Pose2d estimatedRobotPose = new Pose2d();
    private final boolean mLoggingEnabled = true;    // used to disable logging for this subsystem only
    boolean standardCarpetDirection = true;
    boolean hasEmergency = false;
    PeriodicIO periodicIO = new PeriodicIO();
    private CheckFaults cf = new CheckFaults();

    public SwerveDriveModule(int rotationSlot, int driveSlot, int encSlot, int moduleID,
                             int encoderOffset, Translation2d startingPose){
        name += (moduleID + " ");
        rotationMotor = TalonFXFactory.createDefaultTalon(rotationSlot);
        driveMotor = TalonFXFactory.createDefaultTalon(driveSlot);
        canCoder = new CANCoder(encSlot);
        canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, Constants.kLongCANTimeoutMs);
        canCoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255);
        canCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
        this.moduleID = moduleID;
        this.cancoderOffsetDegrees = encoderOffset;
        configureMotors();
        previousEncDistance = 0;
        position = startingPose;
        this.startingPosition = startingPose;
        getRawAngle();
        periodicIO.moduleID = moduleID;
    }

    private void configureMotors(){
        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        rotationMotor.setSensorPhase(true);
        rotationMotor.setInverted(true); // Alex
        rotationMotor.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        rotationMotor.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        // multiple (usually 2) sets were needed to set new encoder value
        int fxCurrentEnc = (int)rotationMotor.getSelectedSensorPosition();
        System.out.println(name+" current fx encoder ticks: "+fxCurrentEnc);
        double cancoderDegrees = canCoder.getAbsolutePosition();
        System.out.println(name+" cancoder degrees-offset: "+cancoderDegrees+"-"+cancoderOffsetDegrees+"="+(cancoderDegrees-cancoderOffsetDegrees));
        int startEncoderValue = (int)degreesToEncUnits(cancoderDegrees-cancoderOffsetDegrees);

        int count = 0;
        while (Math.abs(fxCurrentEnc-startEncoderValue)>10 && count<5){
            count++;
            rotationMotor.setSelectedSensorPosition(startEncoderValue, 0, Constants.kLongCANTimeoutMs);
            Timer.delay(.1);
            fxCurrentEnc = (int)rotationMotor.getSelectedSensorPosition();
        }
        System.out.println(name+" loops needed to set fx encoder: "+count);

        rotationMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, Constants.kLongCANTimeoutMs);
        rotationMotor.setNeutralMode(NeutralMode.Brake);
        rotationMotor.configVoltageCompSaturation(7.0, Constants.kLongCANTimeoutMs);
        rotationMotor.enableVoltageCompensation(false);
        rotationMotor.configAllowableClosedloopError(0, 0, Constants.kLongCANTimeoutMs);
        rotationMotor.configMotionAcceleration((int)(Constants.kSwerveRotationMaxSpeed*12.5), Constants.kLongCANTimeoutMs);
        rotationMotor.configMotionCruiseVelocity((int)(Constants.kSwerveRotationMaxSpeed), Constants.kLongCANTimeoutMs);
        rotationMotor.selectProfileSlot(0, 0);
        //Slot 1 is for normal use (tuned for fx integrated encoder)
        rotationMotor.config_kP(0, 0.07, Constants.kLongCANTimeoutMs);
        rotationMotor.config_kI(0, 0.0, Constants.kLongCANTimeoutMs);
        rotationMotor.config_kD(0, 0.84, Constants.kLongCANTimeoutMs);
        rotationMotor.config_kF(0, 0.025, Constants.kLongCANTimeoutMs);
        //Slot 2 is reserved for the beginning of auto (tuned for cancoders needs retune)
        // rotationMotor.config_kP(1, 0.07, 10);
        // rotationMotor.config_kI(1, 0.0, 10);
        // rotationMotor.config_kD(1, 0.84, 10);
        // rotationMotor.config_kF(1, 0.05, 10);
        rotationMotor.config_kP(1, 0.1, Constants.kLongCANTimeoutMs); // TODO: test this
        rotationMotor.config_kI(1, 0.0, Constants.kLongCANTimeoutMs);
        rotationMotor.config_kD(1, 0.84, Constants.kLongCANTimeoutMs);
        rotationMotor.config_kF(1, 0.05, Constants.kLongCANTimeoutMs);
        rotationMotor.set(ControlMode.MotionMagic, rotationMotor.getSelectedSensorPosition(0));
        // if(!isRotationSensorConnected()){
        // 	DriverStation.reportError(name + "rotation encoder not detected!", false);
        // 	hasEmergency = true;
        // }

        rotationMotor.setControlFramePeriod(ControlFrame.Control_3_General,18);

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        driveMotor.setSelectedSensorPosition(0, 0, Constants.kLongCANTimeoutMs);
        driveMotor.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        driveMotor.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        driveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, Constants.kLongCANTimeoutMs);
        driveMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, Constants.kLongCANTimeoutMs);
        driveMotor.configVelocityMeasurementWindow(32, Constants.kLongCANTimeoutMs);
        driveMotor.configNominalOutputForward(1.5/12.0, Constants.kLongCANTimeoutMs);
        driveMotor.configNominalOutputReverse(-1.5/12.0, Constants.kLongCANTimeoutMs);
        driveMotor.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        driveMotor.enableVoltageCompensation(true);
        driveMotor.configOpenloopRamp(0.25, Constants.kLongCANTimeoutMs);
        driveMotor.configClosedloopRamp(0.0);
        driveMotor.configAllowableClosedloopError(0, 0, Constants.kLongCANTimeoutMs);
        driveMotor.setInverted(true);
        driveMotor.setSensorPhase(true);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        // Slot 0 is reserved for MotionMagic
        driveMotor.selectProfileSlot(0, 0);
        driveMotor.config_kP(0, 2.0, Constants.kLongCANTimeoutMs);
        driveMotor.config_kI(0, 0.0, Constants.kLongCANTimeoutMs);
        driveMotor.config_kD(0, 24.0, Constants.kLongCANTimeoutMs);
        driveMotor.config_kF(0, 1023.0/Constants.kSwerveDriveMaxSpeed, Constants.kLongCANTimeoutMs);
        driveMotor.configMotionCruiseVelocity((int)(Constants.kSwerveDriveMaxSpeed*0.9), Constants.kLongCANTimeoutMs);
        driveMotor.configMotionAcceleration((int)(Constants.kSwerveDriveMaxSpeed), Constants.kLongCANTimeoutMs);
        // Slot 1 corresponds to velocity mode (USED FOR AUTO)
        driveMotor.config_kP(1, 0.03, Constants.kLongCANTimeoutMs);
        driveMotor.config_kI(1, 0.0, Constants.kLongCANTimeoutMs);
        driveMotor.config_kD(1, 0.0, Constants.kLongCANTimeoutMs);
        driveMotor.config_kF(1, 0.05, Constants.kLongCANTimeoutMs);//0.3
        // if(!isDriveSensorConnected()){
        // 	DriverStation.reportError(name + "drive encoder not detected!", false);
        // 	hasEmergency = true;
        // }
        driveMotor.setControlFramePeriod(ControlFrame.Control_3_General,18);
    }

    public void setCarpetDirection(boolean standardDirection) {
        standardCarpetDirection = standardDirection;
    }

    @Override
    public synchronized void stop(){
        setDriveOpenLoop(0.0);
    }

    @Override
    public synchronized void zeroSensors() {
        zeroSensors(new Pose2d());
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {

    }

    public synchronized void invertDriveMotor(boolean invert){
        driveMotor.setInverted(invert);
    }

    public synchronized void invertRotationMotor(boolean invert){
        rotationMotor.setInverted(invert);
    }

    public synchronized void reverseDriveSensor(boolean reverse){
        driveMotor.setSensorPhase(reverse);
    }

    public synchronized void reverseRotationSensor(boolean reverse){
        encoderReverseFactor = reverse ? -1 : 1;
        rotationMotor.setSensorPhase(reverse);
    }

    public synchronized void setNominalDriveOutput(double voltage){
        driveMotor.configNominalOutputForward(voltage / 12.0, Constants.kLongCANTimeoutMs);
        driveMotor.configNominalOutputReverse(-voltage / 12.0, Constants.kLongCANTimeoutMs);
    }

    public synchronized void setMaxRotationSpeed(double maxSpeed){
        rotationMotor.configMotionCruiseVelocity((int)maxSpeed, 0); // timout is 0 in the hot path
    }

    public synchronized void disableDriveEncoder(){
        useDriveEncoder = false;
    }

    public synchronized void updatePose(Rotation2d robotHeading){
        double currentEncDistance = getDriveDistanceInches();
        double deltaEncDistance = (currentEncDistance - previousEncDistance) * Constants.kWheelScrubFactors[moduleID];
        Rotation2d currentWheelAngle = getFieldCentricAngle(robotHeading);
        Translation2d deltaPosition = new Translation2d(currentWheelAngle.cos()*deltaEncDistance,
                currentWheelAngle.sin()*deltaEncDistance);

        double xScrubFactor = Constants.kXScrubFactor;
        double yScrubFactor = Constants.kYScrubFactor;
        if(Constants.kSimulateReversedCarpet){
            if(Util.epsilonEquals(Math.signum(deltaPosition.x()), 1.0)){
                if(standardCarpetDirection){
                    xScrubFactor = 1.0 / Constants.kXScrubFactor;
                }else{
                    xScrubFactor = 1.0;
                }
            }else{
                if(standardCarpetDirection){
                    xScrubFactor = Constants.kXScrubFactor * Constants.kXScrubFactor;
                }else{

                }
            }
            if(Util.epsilonEquals(Math.signum(deltaPosition.y()), 1.0)){
                if(standardCarpetDirection){
                    yScrubFactor = 1.0 / Constants.kYScrubFactor;
                }else{
                    yScrubFactor = 1.0;
                }
            }else{
                if(standardCarpetDirection){
                    yScrubFactor = Constants.kYScrubFactor * Constants.kYScrubFactor;
                }else{

                }
            }
        }else{
            if(Util.epsilonEquals(Math.signum(deltaPosition.x()), 1.0)){
                if(standardCarpetDirection){
                    xScrubFactor = 1.0;
                }else{

                }
            }else{
                if(standardCarpetDirection){

                }else{
                    xScrubFactor = 1.0;
                }
            }
            if(Util.epsilonEquals(Math.signum(deltaPosition.y()), 1.0)){
                if(standardCarpetDirection){
                    yScrubFactor = 1.0;
                }else{

                }
            }else{
                if(standardCarpetDirection){

                }else{
                    yScrubFactor = 1.0;
                }
            }
        }

        deltaPosition = new Translation2d(deltaPosition.x() * xScrubFactor,
                deltaPosition.y() * yScrubFactor);
        Translation2d updatedPosition = position.translateBy(deltaPosition);
        Pose2d staticWheelPose = new Pose2d(updatedPosition, robotHeading);
        Pose2d robotPose = staticWheelPose.transformBy(Pose2d.fromTranslation(startingPosition).inverse());
        position = updatedPosition;
        estimatedRobotPose =  robotPose;
        previousEncDistance = currentEncDistance;
    }

    private double getRawAngle(){
        return encUnitsToDegrees(periodicIO.rotationPosition);
    }

    public Rotation2d getModuleAngle(){
        return Rotation2d.fromDegrees(getRawAngle()); // - encUnitsToDegrees(cancoderOffsetDegrees));
    }

    public Rotation2d getFieldCentricAngle(Rotation2d robotHeading){
        Rotation2d normalizedAngle = getModuleAngle();
        return normalizedAngle.rotateBy(robotHeading);
    }

    public void setModuleAngle(double goalAngle){
        double newAngle = Utils.placeInAppropriate0To360Scope(getRawAngle(), goalAngle); // + encUnitsToDegrees(cancoderOffsetDegrees));
        int setpoint = degreesToEncUnits(newAngle);
        periodicIO.rotationControlMode = ControlMode.MotionMagic;
        periodicIO.rotationDemand = setpoint;
    }

    public boolean angleOnTarget(){
        double error = encUnitsToDegrees(Math.abs(periodicIO.rotationDemand - periodicIO.rotationPosition));
        return error < 4.5;
    }

    public void set10VoltRotationMode(boolean tenVolts){
        if(tenVolts && !tenVoltRotationMode){
            rotationMotor.selectProfileSlot(1, 0);
            rotationMotor.configVoltageCompSaturation(10.0, Constants.kLongCANTimeoutMs);
            tenVoltRotationMode = true;
        }else if(!tenVolts && tenVoltRotationMode){
            rotationMotor.selectProfileSlot(0, 0);
            rotationMotor.configVoltageCompSaturation(7.0, Constants.kLongCANTimeoutMs);
            tenVoltRotationMode = false;
        }
    }

    public void setRotationOpenLoop(double power){
        periodicIO.rotationControlMode = ControlMode.PercentOutput;
        periodicIO.rotationDemand = power;
    }

    /**
     * @param velocity Normalized value
     */
    public void setDriveOpenLoop(double velocity){
		/*double volts = 0.0;
		if(!Util.epsilonEquals(velocity, 0.0, Constants.kEpsilon)){
			velocity *= Constants.kSwerveMaxSpeedInchesPerSecond;
			double m =  Constants.kVoltageVelocityEquations[moduleID][velocity < 0 ? 1 : 0][0];
			double b = Constants.kVoltageVelocityEquations[moduleID][velocity < 0 ? 1 : 0][1];
			volts = (velocity - b) / m;
			volts = Util.deadBand(volts, 1.0);
		}*/

        periodicIO.driveControlMode = ControlMode.PercentOutput;
        //periodicIO.driveDemand = volts / 12.0;
        periodicIO.driveDemand = velocity;
    }

    public void setDrivePositionTarget(double deltaDistanceInches){
        driveMotor.selectProfileSlot(0, 0);
        periodicIO.driveControlMode = ControlMode.MotionMagic;
        periodicIO.driveDemand = periodicIO.drivePosition + inchesToEncUnits(deltaDistanceInches);
        if (name.equals("Module 0 ")){
            System.out.println("drive motion magic");
        }
    }

    public boolean drivePositionOnTarget(){
        if(driveMotor.getControlMode() == ControlMode.MotionMagic)
            return encUnitsToInches((int)Math.abs(periodicIO.driveDemand - periodicIO.drivePosition)) < 2.0;
        return false;
    }
    double minSpeed = 800; // 127 fails to move Cetus
    final double hack = 1;//3.8; // this is a hack. I don't know why it is needed but the robot goes too fast otherwise
    public void setVelocitySetpoint(double inchesPerSecond){
        driveMotor.selectProfileSlot(1, 0);
        periodicIO.driveControlMode = ControlMode.Velocity;
        double vel = inchesPerSecondToEncVelocity(inchesPerSecond)/hack;
        if (Math.abs(vel) < minSpeed){
            if(vel==0){
                periodicIO.driveDemand = 0;
            }
            else if (vel>0){
                periodicIO.driveDemand = minSpeed;
            }
            else{
                periodicIO.driveDemand = -minSpeed;
            }
        }
        else{
            periodicIO.driveDemand = vel;
        }
        // if (name.equals("Module 0 ")){
        // 	// if (periodicIO.driveDemand == vel){
        // 	// 	// System.out.println(inchesPerSecond+" "+(vel*hack));
        // 	// }
        // 	// else{
        // 	// 	// System.out.println(inchesPerSecond+" "+vel+" (min "+minSpeed+")");
        // 	// }
        // }
    }

    private double getDriveDistanceInches(){
        return encUnitsToInches(periodicIO.drivePosition);
    }

    public double encUnitsToInches(double encUnits){
        return encUnits/Constants.kSwerveEncUnitsPerInch;
    }

    public int inchesToEncUnits(double inches){
        return (int) (inches*Constants.kSwerveEncUnitsPerInch);
    }

    public double encVelocityToInchesPerSecond(double encUnitsPer100ms){
        return encUnitsToInches(encUnitsPer100ms) * 10;
    }

    public int inchesPerSecondToEncVelocity(double inchesPerSecond){
        return (int) (inchesToEncUnits(inchesPerSecond / 10.0));
    }

    public int degreesToEncUnits(double degrees){
        return (int) (degrees/360.0*Constants.kSwerveRotationMotorTicksPerRotation);
    }

    public double encUnitsToDegrees(double encUnits){
        return encUnits/Constants.kSwerveRotationMotorTicksPerRotation*360.0;
    }

    public Translation2d getPosition(){
        return position;
    }

    public Pose2d getEstimatedRobotPose(){
        return estimatedRobotPose;
    }

    public synchronized void resetPose(Pose2d robotPose){
        Translation2d modulePosition = robotPose.transformBy(Pose2d.fromTranslation(startingPosition)).getTranslation();
        position = modulePosition;
    }

    public synchronized void resetPose(){
        position = startingPosition;
    }

    public synchronized void resetLastEncoderReading(){
        previousEncDistance = getDriveDistanceInches();
    }

    public synchronized void zeroSensors(Pose2d robotPose) {
        //driveMotor.setSelectedSensorPosition(0, 0, 100); TODO check if this is necessary
        resetPose(robotPose);
        estimatedRobotPose = robotPose;
        previousEncDistance = getDriveDistanceInches();
    }

    public synchronized void disable(){
        setDriveOpenLoop(0.0);
        setRotationOpenLoop(0.0);
    }

    @Override
    public String getLogHeaders() {
        if (mLoggingEnabled){
            cf.clearFaults(driveMotor);
            cf.clearFaults(rotationMotor);
            String shortName ="SwerveDriveModule"+moduleID;

            return  shortName+".rotationPosition,"+
                    shortName+".drivePosition,"+
                    shortName+".rotationControlMode,"+
                    shortName+".driveControlMode,"+
                    shortName+".rotationDemand,"+
                    shortName+".driveDemand,"+
                    shortName+".rotationCurrent,"+
                    shortName+".driveCurrent,"+
                    shortName+".rotationFaults,"+
                    shortName+".driveFaults";
        }
        return null;
    }

    private String generateLogValues(boolean telemetry){
        String values;

        if (telemetry){
            periodicIO.rotationCurrent = rotationMotor.getStatorCurrent();
            periodicIO.driveCurrent    = driveMotor.getStatorCurrent();
            periodicIO.rotationFaults  = cf.getFaults(rotationMotor);
            periodicIO.driveFaults     = cf.getFaults(driveMotor);

            values = ""+periodicIO.rotationPosition+","+
                    periodicIO.drivePosition+","+
                    periodicIO.rotationControlMode+","+
                    periodicIO.driveControlMode+","+
                    periodicIO.rotationDemand+","+
                    periodicIO.driveDemand+","+
                    periodicIO.rotationCurrent+","+
                    periodicIO.driveCurrent+","+
                    periodicIO.rotationFaults+","+
                    periodicIO.driveFaults;
        }
        else{
            values = ""+periodicIO.rotationPosition+","+
                    periodicIO.drivePosition+","+
                    periodicIO.rotationControlMode+","+
                    periodicIO.driveControlMode+","+
                    periodicIO.rotationDemand+","+
                    periodicIO.driveDemand+","+
                    /*periodicIO.rotationCurrent+*/","+
                    /*periodicIO.driveCurrent+*/","+
                    /*cf.getFXFaults(rotationMotor)+*/","
            /*cf.getFXFaults(driveMotor)*/;
        }
        return values;
    }

    @Override
    public String getLogValues(boolean telemetry) {
        if (mLoggingEnabled){
            return generateLogValues(telemetry);
        }
        return null;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        periodicIO.rotationPosition = (int)rotationMotor.getSelectedSensorPosition(0);
        periodicIO.drivePosition = (int)driveMotor.getSelectedSensorPosition(0);

        //periodicIO.velocity = driveMotor.getSelectedSensorVelocity();
        // if(Constants.kDebuggingOutput){
        // 	periodicIO.velocity = driveMotor.getSelectedSensorVelocity();
        // }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        rotationMotor.set(periodicIO.rotationControlMode, periodicIO.rotationDemand);
        driveMotor.set(periodicIO.driveControlMode, periodicIO.driveDemand);
    }

    @Override
    public void outputTelemetry() {
        // SmartDashboard.putNumber(name + "Angle", getModuleAngle().getDegrees());
        // SmartDashboard.putNumber(name + "Inches Driven", getDriveDistanceInches());
        // SmartDashboard.putNumber(name + "Rotation", periodicIO.rotationPosition); // Alex
        // SmartDashboard.putNumber(name + "Rotation velocity", rotationMotor.getSelectedSensorVelocity(0)); // Alex
        // SmartDashboard.putNumber(name + "Rotation (cancoder)", enc.getAbsolutePosition()-cancoderOffsetDegrees); // Alex
        SmartDashboard.putNumber(name + "Rotation (cancoder)", canCoder.getAbsolutePosition()); // Alex
        // SmartDashboard.putNumber(name + "Position", periodicIO.drivePosition); // Alex
        // SmartDashboard.putNumber(name + "Velocity", driveMotor.getSelectedSensorVelocity(0)); // Alex
        //SmartDashboard.putNumber(name + "Velocity", encVelocityToInchesPerSecond(periodicIO.velocity));
        if(Constants.kDebuggingOutput){
            SmartDashboard.putNumber(name + "Pulse Width", rotationMotor.getSelectedSensorPosition(0));
            if(rotationMotor.getControlMode() == ControlMode.MotionMagic)
                SmartDashboard.putNumber(name + "Error", encUnitsToDegrees(rotationMotor.getClosedLoopError(0)));
            //SmartDashboard.putNumber(name + "X", position.x());
            //SmartDashboard.putNumber(name + "Y", position.y());
            SmartDashboard.putNumber(name + "Rotation Speed", rotationMotor.getSelectedSensorVelocity(0));
        }
    }

    public static class PeriodicIO {
        //Inputs
        public int moduleID;
        public int rotationPosition;
        public int drivePosition;
        public double rotationCurrent;
        public double driveCurrent;
        public String rotationFaults;
        public String driveFaults;


        //Outputs
        public ControlMode rotationControlMode = ControlMode.PercentOutput;
        public ControlMode driveControlMode = ControlMode.PercentOutput;
        public double rotationDemand;
        public double driveDemand;
    }
}
