package libraries.cyberlib.utils;

import libraries.cheesylib.geometry.Translation2d;

/**
 * Represents a holonomic drive signal.
 */
public class SwerveDriveSignal {
    private final Translation2d translation;
    private final double rotation;
    private final boolean fieldOriented;

    public SwerveDriveSignal(Translation2d translation, double rotation, boolean fieldOriented) {
        this.translation = translation;
        this.rotation = rotation;
        this.fieldOriented = fieldOriented;
    }

    public Translation2d getTranslation() {
        return translation;
    }

    public double getRotation() {
        return rotation;
    }

    public boolean isFieldOriented() {
        return fieldOriented;
    }
}
