package libraries.madtownlib.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import libraries.cheesylib.geometry.*;

import java.util.ArrayList;
import java.util.List;

public class SwerveInverseKinematics {
	private List<Translation2d> moduleRelativePositions;
	private List<Translation2d> moduleRotationDirections; // Alex

	public SwerveInverseKinematics(List<Translation2d> modulePositions){
		moduleRelativePositions = modulePositions;
		moduleRotationDirections = updateRotationDirections(); // Alex
		setCenterOfRotation(new Translation2d());
	}
	
	private final int kNumberOfModules = 4;
	
	private List<Translation2d> updateRotationDirections(){
		List<Translation2d> directions = new ArrayList<>(kNumberOfModules);
		for(int i = 0; i < kNumberOfModules; i++){
			directions.add(moduleRelativePositions.get(i).rotateBy(Rotation2d.fromDegrees(90)));
		}
		return directions;
	}
	
	public void setCenterOfRotation(Translation2d center){
		List<Translation2d> positions = new ArrayList<>(kNumberOfModules);
		double maxMagnitude = 0.0;
		for(int i = 0; i < kNumberOfModules; i++){
			Translation2d position = moduleRelativePositions.get(i).translateBy(center.inverse());
			positions.add(position);
			double magnitude = position.norm();
			if(magnitude > maxMagnitude){
				maxMagnitude = magnitude;
			}
		}
		for(int i = 0; i < kNumberOfModules; i++){
			Translation2d position = positions.get(i);
			positions.set(i, position.scale(1.0/maxMagnitude));
		}
		moduleRelativePositions = positions;
		moduleRotationDirections = updateRotationDirections();
	}
	
	public List<Translation2d> updateDriveVectors(Translation2d translationalVector, double rotationalMagnitude, 
			Pose2d robotPose, boolean robotCentric){
		SmartDashboard.putNumber("Vector Direction", translationalVector.direction().getDegrees());
		//SmartDashboard.putNumber("Vector Magnitude", translationalVector.norm());
		SmartDashboard.putNumber("Robot Velocity", translationalVector.norm());
		
		if(!robotCentric)
			translationalVector = translationalVector.rotateBy(robotPose.getRotation().inverse());
		List<Translation2d> driveVectors = new ArrayList<>(kNumberOfModules);
		for(int i = 0; i < kNumberOfModules; i++){
			driveVectors.add(translationalVector.translateBy(moduleRotationDirections.get(i).scale(rotationalMagnitude)));
		}
		double maxMagnitude = 1.0;
		for(Translation2d t : driveVectors){
			double magnitude = t.norm();
			if(magnitude > maxMagnitude){
				maxMagnitude = magnitude;
			}
		}
		for(int i = 0; i < kNumberOfModules; i++){
			Translation2d driveVector = driveVectors.get(i);
			driveVectors.set(i, driveVector.scale(1.0/maxMagnitude));
		}
		return driveVectors;
	}
}
