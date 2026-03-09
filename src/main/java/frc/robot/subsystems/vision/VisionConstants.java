package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {
    private VisionConstants() {
    }

    public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Nombres de cámaras (Debe coincidir con PhotonVision)
    public static final String FRONT_CAMERA_NAME = "FrontCam";
    public static final String LEFT_CAMERA_NAME = "LeftCam";
    public static final String RIGHT_CAMERA_NAME = "RightCam";

    // FIXME: Offsets físicos con sufijos de unidad
    public static final Transform3d ROBOT_TO_FRONT = new Transform3d(
            new Translation3d(Units.inchesToMeters(12), 0, Units.inchesToMeters(8)),
            new Rotation3d(0, Units.degreesToRadians(-15), 0));

    public static final Transform3d ROBOT_TO_LEFT = new Transform3d(
            new Translation3d(0, Units.inchesToMeters(10), Units.inchesToMeters(8)),
            new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(90)));

    public static final Transform3d ROBOT_TO_RIGHT = new Transform3d(
            new Translation3d(0, Units.inchesToMeters(-10), Units.inchesToMeters(8)),
            new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-90)));

    // Valores por defecto para filtros
    public static final double DEFAULT_MAX_AMBIGUITY_THRESHOLD = 0.2;
    public static final double DEFAULT_MIN_DISTANCE_METERS = 0.5;
    public static final double DEFAULT_MAX_DISTANCE_METERS = 5.0;

    // Desviaciones Estándar Base con sufijos
    public static final double BASE_XY_STD_DEV_METERS = 0.1;
    public static final double BASE_THETA_STD_DEV_RADS = 0.4;
}