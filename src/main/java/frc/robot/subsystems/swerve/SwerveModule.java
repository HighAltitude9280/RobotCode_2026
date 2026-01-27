package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.HighAltitudeConstants;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    // Controllers
    private final ProfiledPIDController turnPID;
    private final PIDController drivePID;
    private final SimpleMotorFeedforward driveFF;

    // "Powerhouse" feature: SlewRateLimiter en el módulo para proteger mecánica
    // Limita el cambio de voltaje/velocidad (ej. 50% a 100% en X segundos)
    private final SlewRateLimiter driveLimiter = new SlewRateLimiter(HighAltitudeConstants.Swerve.MAX_ACCEL_M_S2);

    public SwerveModule(ModuleIO io, int index) {
        this.io = io;
        this.index = index;

        turnPID = new ProfiledPIDController(
                HighAltitudeConstants.Swerve.TURN_KP,
                HighAltitudeConstants.Swerve.TURN_KI,
                HighAltitudeConstants.Swerve.TURN_KD,
                new TrapezoidProfile.Constraints(
                        HighAltitudeConstants.Swerve.TURN_MAX_VELOCITY_RAD_S,
                        HighAltitudeConstants.Swerve.TURN_MAX_ACCELERATION_RAD_S2));

        turnPID.enableContinuousInput(-Math.PI, Math.PI);

        drivePID = new PIDController(
                HighAltitudeConstants.Swerve.DRIVE_KP,
                HighAltitudeConstants.Swerve.DRIVE_KI,
                HighAltitudeConstants.Swerve.DRIVE_KD);

        driveFF = new SimpleMotorFeedforward(
                HighAltitudeConstants.Swerve.DRIVE_KS, HighAltitudeConstants.Swerve.DRIVE_KV);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + index, inputs);
    }

    public void runSetpoint(SwerveModuleState state) {
        // Optimización manual
        // FIXME: Corregir la optimización manual por la optimización de
        // SwerveDriveKinematics
        Rotation2d currentAngle = new Rotation2d(inputs.turnAbsolutePositionRad);
        var delta = state.angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 90.0) {
            state.speedMetersPerSecond = -state.speedMetersPerSecond;
            state.angle = state.angle.rotateBy(Rotation2d.fromDegrees(180.0));
        }

        // Closed Loop Drive
        double targetSpeed = state.speedMetersPerSecond;
        // Aplicar SlewRate si estamos en Open Loop o Closed Loop muy agresivo
        targetSpeed = driveLimiter.calculate(targetSpeed);
        // si se desea suavizado en hardware level

        double turnVolts = turnPID.calculate(inputs.turnAbsolutePositionRad, state.angle.getRadians());

        double currentVelocity = inputs.driveVelocityRadPerSec * HighAltitudeConstants.Swerve.WHEEL_RADIUS_METERS;
        double driveVolts = driveFF.calculate(targetSpeed) + drivePID.calculate(currentVelocity, targetSpeed);

        turnVolts = MathUtil.clamp(
                turnVolts, -HighAltitudeConstants.MAX_VOLTAGE, HighAltitudeConstants.MAX_VOLTAGE);
        driveVolts = MathUtil.clamp(
                driveVolts, -HighAltitudeConstants.MAX_VOLTAGE, HighAltitudeConstants.MAX_VOLTAGE);

        io.setTurnVoltage(turnVolts);
        io.setDriveVoltage(driveVolts);

        // Log Setpoints
        Logger.recordOutput("Drive/Module" + index + "/SetpointVelocity", state.speedMetersPerSecond);
        Logger.recordOutput("Drive/Module" + index + "/SetpointAngle", state.angle.getRadians());
    }

    // --- GETTERS & SETTERS (Powerhouse Requests) ---

    public SwerveModulePosition getPosition() {
        double distanceMeters = inputs.drivePositionRad * HighAltitudeConstants.Swerve.WHEEL_RADIUS_METERS;
        return new SwerveModulePosition(distanceMeters, new Rotation2d(inputs.turnAbsolutePositionRad));
    }

    public SwerveModuleState getState() {
        double velocityMeters = inputs.driveVelocityRadPerSec * HighAltitudeConstants.Swerve.WHEEL_RADIUS_METERS;
        return new SwerveModuleState(velocityMeters, new Rotation2d(inputs.turnAbsolutePositionRad));
    }

    public double getDriveVelocity() {
        return inputs.driveVelocityRadPerSec * HighAltitudeConstants.Swerve.WHEEL_RADIUS_METERS;
    }

    public Rotation2d getAngle() {
        return new Rotation2d(inputs.turnAbsolutePositionRad);
    }

    public void setBrakeMode(boolean enable) {
        io.setDriveBrakeMode(enable);
        io.setTurnBrakeMode(enable);
    }

    public void stop() {
        io.setDriveVoltage(0.0);
        io.setTurnVoltage(0.0);
    }
}
