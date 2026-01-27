// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public final class HighAltitudeConstants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  // =============================================================================
  // GLOBAL
  // =============================================================================
  public static final double LOOP_PERIOD_SECS = 0.02;
  public static final double MAX_VOLTAGE = 12.0;

  // =============================================================================
  // CONTROLS (NUEVO)
  // =============================================================================
  public static final class Controls {
    public static final double DEADBAND = 0.1;
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
  }

  // --- PHYSICS SIMULATION CONSTANTS ---
  // Masa del robot (aprox 90lbs)
  public static final double ROBOT_MASS_KG = 40.82;

  // Momentos de Inercia (J)
  // Inercia del mecanismo de giro (masa pequeña rotando)
  public static final double MOI_TURN_KG_M2 = 0.004;
  // Inercia de la transmisión (rueda + carga efectiva del robot sobre la rueda)
  public static final double MOI_DRIVE_KG_M2 = 0.025;

  // =============================================================================
  // SWERVE SUBSYSTEM
  // =============================================================================
  public static final class Swerve {

    // --- Physical Dimensions ---
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(21.0);
    public static final double WHEEL_BASE_METERS = Units.inchesToMeters(23.0);
    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(4.0) / 2.0;

    // --- Gearing & Conversions ---
    // Drive: (50/16) * (16/28) * (45/15) approx 5.357
    public static final double DRIVE_GEAR_RATIO = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
    // Turn: MK4i Standard (150/7) approx 21.428
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

    // Conversion Factors
    public static final double DRIVE_METERS_PER_ROTATION =
        (2.0 * Math.PI * WHEEL_RADIUS_METERS) / DRIVE_GEAR_RATIO;
    public static final double TURN_RADIANS_PER_ROTATION = (2.0 * Math.PI) / TURN_GEAR_RATIO;

    // --- Kinematics ---
    public static final SwerveDriveKinematics KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0), // FL
            new Translation2d(WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0), // FR
            new Translation2d(-WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0), // BL
            new Translation2d(-WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0) // BR
            );

    // --- DRIVE CONTROL (PID + FF) ---
    public static final double DRIVE_KP = 0.125;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KS = 0.0;
    public static final double DRIVE_KV = 2.05;

    // --- TURN CONTROL (Profiled PID) ---
    public static final double TURN_KP = 5.0;
    public static final double TURN_KI = 0.0;
    public static final double TURN_KD = 0.075;

    // Constraints for ProfiledPIDController
    public static final double TURN_MAX_VELOCITY_RAD_S = 6.0;
    public static final double TURN_MAX_ACCELERATION_RAD_S2 = 15.0;

    // --- Limits ---
    public static final double MAX_LINEAR_SPEED_M_S = 4.5;
    public static final double MAX_ANGULAR_SPEED_RAD_S = 5.0;

    // NUEVO: Agregado para SlewRateLimiter en SwerveModule
    public static final double MAX_ACCEL_M_S2 = 15.0;

    // --- Module Configuration ---
    public record ModuleConstants(
        int driveID, int turnID, int cancoderID, Rotation2d offset, boolean driveInverted) {}

    public static final ModuleConstants MOD_FL =
        new ModuleConstants(12, 17, 39, Rotation2d.fromRotations(-0.114746), true);
    public static final ModuleConstants MOD_FR =
        new ModuleConstants(10, 11, 40, Rotation2d.fromRotations(0.084716), false);
    public static final ModuleConstants MOD_BL =
        new ModuleConstants(14, 15, 38, Rotation2d.fromRotations(0.208251), true);
    public static final ModuleConstants MOD_BR =
        new ModuleConstants(16, 13, 37, Rotation2d.fromRotations(0.055419), false);
  }

  // =============================================================================
  // VISION SUBSYSTEM
  // =============================================================================
  public static final class Vision {
    public static final String CAMERA_FRONT_NAME = "ArducamFront2";
    public static final String CAMERA_BACK_NAME = "limelight2plus";

    public static final Transform3d FRONT_CAM_POSE =
        new Transform3d(
            new Translation3d(0.223774, 0.261112, 0.20917466),
            new Rotation3d(0, Math.toRadians(-20), Math.toRadians(-24.97)));

    public static final Transform3d BACK_CAM_POSE =
        new Transform3d(
            new Translation3d(0.202692, -0.27051, 0.21686527),
            new Rotation3d(0, Math.toRadians(-20), Math.toRadians(30)));

    public static final double POSE_AMBIGUITY_CUTOFF = 0.15;
    public static final double MAX_POSE_DISTANCE_METERS = 6.0;
  }
}
