package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.LoggedTunableNumber;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final SwerveDrive swerve;
  private final VisionIO[] ios;
  private final VisionIOInputsAutoLogged[] inputs;
  private final String[] cameraNames;

  private final List<LoggedTunableNumber> maxAmbiguityTunables = new ArrayList<>();
  private final List<LoggedTunableNumber> minDistanceTunables = new ArrayList<>();
  private final List<LoggedTunableNumber> maxDistanceTunables = new ArrayList<>();

  public Vision(SwerveDrive swerve, String[] cameraNames, VisionIO... ios) {
    this.swerve = swerve;

    // Fallback Resiliente
    if (ios == null || cameraNames == null || ios.length != cameraNames.length) {
      DriverStation.reportError("CRÍTICO: Visión desactivada. Los arrays IO y nombres no coinciden o son nulos.",
          false);

      // Inicialización vacía para evitar NPEs en el periodic y sobrevivir al error
      this.ios = new VisionIO[0];
      this.cameraNames = new String[0];
      this.inputs = new VisionIOInputsAutoLogged[0];
      return;
    }

    this.cameraNames = cameraNames;
    this.ios = ios;
    this.inputs = new VisionIOInputsAutoLogged[ios.length];

    for (int i = 0; i < ios.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();

      maxAmbiguityTunables
          .add(new LoggedTunableNumber("Vision/" + cameraNames[i] + "/MaxAmbiguity", DEFAULT_MAX_AMBIGUITY_THRESHOLD));
      minDistanceTunables
          .add(new LoggedTunableNumber("Vision/" + cameraNames[i] + "/MinDistance", DEFAULT_MIN_DISTANCE_METERS));
      maxDistanceTunables
          .add(new LoggedTunableNumber("Vision/" + cameraNames[i] + "/MaxDistance", DEFAULT_MAX_DISTANCE_METERS));
    }
  }

  @Override
  public void periodic() {
    // Si la inicialización falló y los arrays están vacíos, este bucle simplemente
    // no hace nada,
    // evitando crasheos catastróficos en pleno match.
    for (int i = 0; i < ios.length; i++) {
      ios[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/" + cameraNames[i], inputs[i]);

      if (isValid(i)) {
        double xyStdDev = BASE_XY_STD_DEV_METERS * Math.pow(inputs[i].averageTagDistance, 2);
        double thetaStdDev;

        if (inputs[i].targetCount > 1) {
          thetaStdDev = BASE_THETA_STD_DEV_RADS * Math.pow(inputs[i].averageTagDistance, 2);
        } else {
          thetaStdDev = Double.MAX_VALUE;
        }

        swerve.addVisionMeasurement(
            inputs[i].estimatedPose,
            inputs[i].timestamp,
            VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));

        Logger.recordOutput("Vision/" + cameraNames[i] + "/AcceptedPose", inputs[i].estimatedPose);
      }
    }
  }

  private boolean isValid(int index) {
    var input = inputs[index];
    if (!input.connected || input.targetCount == 0)
      return false;

    if (input.averageTagDistance < minDistanceTunables.get(index).get() ||
        input.averageTagDistance > maxDistanceTunables.get(index).get())
      return false;

    if (input.maxAmbiguity > maxAmbiguityTunables.get(index).get())
      return false;

    Pose2d p = input.estimatedPose;
    return p.getX() >= 0 && p.getX() <= TAG_LAYOUT.getFieldLength() &&
        p.getY() >= 0 && p.getY() <= TAG_LAYOUT.getFieldWidth();
  }
}