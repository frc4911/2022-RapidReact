package frc.robot.config;

import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Rotation2d;

public class LimelightConfig {

    private final int id;
    private final String name;
    private final String tableName;
    private final double height;
    private final Pose2d shooterToLens;
    private final Rotation2d horizontalPlaneToLens;
    private final Type type;
    private final double horizontalFOV;
    private final double verticalFOV;

    public LimelightConfig(int id, Type type, String name, String tableName, double height, Pose2d
            shooterToLens, Rotation2d horizontalPlaneToLens, double horizontalFOV, double verticalFOV) {
        this.id = id;
        this.type = type;
        this.name = name;
        this.tableName = tableName;
        this.height = height;
        this.shooterToLens = shooterToLens;
        this.horizontalPlaneToLens = horizontalPlaneToLens;
        this.horizontalFOV = horizontalFOV;
        this.verticalFOV = verticalFOV;
    }

    public int getId() {
        return id;
    }

    public String getName() {
        return name;
    }

    public String getTableName() {
        return tableName;
    }

    public double getHeight() {
        return height;
    }

    public Pose2d getShooterToLens() {
        return shooterToLens;
    }

    public Rotation2d getHorizontalPlaneToLens() {
        return horizontalPlaneToLens;
    }

    public Type getType() {
        return type;
    }

    public double getHorizontalFOV() {
        return horizontalFOV;
    }

    public double getVerticalFOV() {
        return verticalFOV;
    }

    public enum Type {
        Shooter
    }
}
