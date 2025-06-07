package org.team340.lib.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Retrieves information about the playing field from an AprilTag field
 * layout. By default, {@link AprilTagFields#kDefaultField} is used.
 */
public final class FieldInfo {

    private FieldInfo() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    private static AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    /**
     * Loads a new AprilTag layout to retrieve field information from.
     * This method should be called before all other user code.
     * @param field The loadable AprilTag field layout.
     */
    public static void setLayout(AprilTagFields field) {
        layout = AprilTagFieldLayout.loadField(field);
    }

    /**
     * Loads a new AprilTag layout to retrieve field information from.
     * This method should be called before all other user code.
     * @param resourcePath The absolute path of the resource.
     */
    public static void setLayout(String resourcePath) {
        try {
            layout = AprilTagFieldLayout.loadFromResource(resourcePath);
        } catch (Exception e) {
            DriverStation.reportError(
                "[AprilTags] Unable to load layout from resource \"" + resourcePath + "\": " + e.getMessage(),
                true
            );
        }
    }

    /**
     * Returns the length of the field in meters.
     */
    public static double length() {
        return layout.getFieldLength();
    }

    /**
     * Returns thr width of the field in meters.
     */
    public static double width() {
        return layout.getFieldWidth();
    }

    /**
     * Returns the current field's AprilTag layout.
     */
    public static AprilTagFieldLayout aprilTags() {
        return layout;
    }
}
