package frc.robot.subsystems.swerve.gyro;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.util.Units;

public class GyroIONavX implements GyroIO {
  private final AHRS navx;

  public GyroIONavX() {
    // navx = new AHRS(SPI.Port.kMXP);
    navx = new AHRS(NavXComType.kMXP_SPI);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navx.isConnected();
    // NavX is CW positive by default, WPILib/AK expects CCW positive (NWU).
    // Check standard NavX usage, usually requires negation.
    inputs.yawPositionRad = Units.degreesToRadians(-navx.getAngle());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navx.getRate());
  }

  @Override
  public void reset() {
    navx.reset();
  }
}
