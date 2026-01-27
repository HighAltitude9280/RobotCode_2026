package frc.robot.subsystems.swerve.module;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

    @AutoLog
    public static class ModuleIOInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveTempCelcius = 0.0;

        public double turnAbsolutePositionRad = 0.0;
        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
    }

    /** Update the inputs from the hardware/sim */
    default void updateInputs(ModuleIOInputs inputs) {
    }

    /** Open-loop voltage control */
    default void setDriveVoltage(double volts) {
    }

    default void setTurnVoltage(double volts) {
    }

    /** Essential configs */
    default void setDriveBrakeMode(boolean enable) {
    }

    default void setTurnBrakeMode(boolean enable) {
    }
}
