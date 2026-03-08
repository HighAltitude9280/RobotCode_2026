package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {

  private VisionConstants() {}

  // Layout de AprilTags del Field
  public static final AprilTagFieldLayout TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // ==========================
  // NOMBRES DE CAMARAS
  // ==========================

  // Deben coincidir EXACTAMENTE con PhotonVision
  public static final String ARDUCAM_NAME = "Arducam";
  public static final String LIMELIGHT_NAME = "Limelight";

  // ==========================
  // POSICION CAMARAS EN ROBOT
  // ==========================

  // ROBOT -> ARDUCAM
  public static final Transform3d ROBOT_TO_ARDUCAM =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-5.0),
              Units.inchesToMeters(12.0),
              Units.inchesToMeters(8.5)),
          new Rotation3d(
              0.0,
              Units.degreesToRadians(-20.0),
              0.0));

  // ROBOT -> LIMELIGHT
  public static final Transform3d ROBOT_TO_LIMELIGHT =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(10.0),
              Units.inchesToMeters(-6.0),
              Units.inchesToMeters(9.5)),
          new Rotation3d(
              0.0,
              Units.degreesToRadians(-18.0),
              0.0));

  // ==========================
  // FILTROS DE VISION
  // ==========================

  // distancia máxima a tags aceptada
  public static final double MAX_TAG_DISTANCE = 5.5;

  // ambigüedad máxima si solo hay 1 tag
  public static final double MAX_SINGLE_TAG_AMBIGUITY = 0.2;

  // ==========================
  // COVARIANZA BASE
  // ==========================

  public static final double BASE_XY_STD_DEV = 0.08;
  public static final double BASE_THETA_STD_DEV = 0.2;
}