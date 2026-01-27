package frc.robot.util;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/** Hilo "PowerHouse" que corre a 250Hz para capturar cada movimiento de rueda. */
public class PhoenixOdometryThread extends Thread {
  private final Lock signalsLock = new ReentrantLock();
  // Guardamos las señales crudas (BaseStatusSignal) para el waitForAll
  private final List<BaseStatusSignal> allSignals = new ArrayList<>();
  // Guardamos referencias tipadas para leer los valores
  private final List<StatusSignal<Angle>> driveSignals = new ArrayList<>();
  private final List<StatusSignal<Angle>> turnSignals = new ArrayList<>();

  private final Queue<OdometrySample> odometryQueue = new ArrayBlockingQueue<>(100);
  private boolean isRunning = true;

  private static PhoenixOdometryThread instance = null;

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  private PhoenixOdometryThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true); // Se muere si el robot se apaga
    start();
  }

  public record OdometrySample(
      double timestamp, double[] drivePositionsRad, double[] turnPositionsRad) {}

  /** Registra las señales de un módulo. Llamar esto en el constructor de ModuleIOTalonSpark. */
  public void registerSignals(StatusSignal<Angle> drivePos, StatusSignal<Angle> turnPos) {
    signalsLock.lock();
    try {
      driveSignals.add(drivePos);
      turnSignals.add(turnPos);
      allSignals.add(drivePos);
      allSignals.add(turnPos);
    } finally {
      signalsLock.unlock();
    }
  }

  @Override
  public void run() {
    while (isRunning) {
      // Si no hay señales registradas, dormir un poco y reintentar
      if (allSignals.isEmpty()) {
        try {
          Thread.sleep(100);
        } catch (InterruptedException e) {
        }
        continue;
      }

      signalsLock.lock();
      try {
        // MAGIA: Esperar síncronamente a que lleguen datos nuevos del CANivore/RIO
        // 250Hz = 0.004s. Ponemos un timeout un poco mayor (0.02) por seguridad.
        BaseStatusSignal.waitForAll(0.02, allSignals.toArray(new BaseStatusSignal[0]));

        double[] drives = new double[driveSignals.size()];
        double[] turns = new double[turnSignals.size()];
        double timestamp = 0.0; // FPGA Timestamp

        // Leer valores. Phoenix 6 retorna Rotations por defecto en getValue().
        for (int i = 0; i < driveSignals.size(); i++) {
          drives[i] = Units.rotationsToRadians(driveSignals.get(i).getValue().in(Rotations));
          turns[i] = Units.rotationsToRadians(turnSignals.get(i).getValue().in(Rotations));

          // Usamos el timestamp de la primera señal
          if (i == 0) timestamp = driveSignals.get(i).getTimestamp().getTime();
        }

        // Guardar en la cola (offer no lanza excepción si está llena, solo descarta la
        // vieja)
        odometryQueue.offer(new OdometrySample(timestamp, drives, turns));

      } catch (Exception e) {
        e.printStackTrace();
      } finally {
        signalsLock.unlock();
      }
    }
  }

  public Queue<OdometrySample> getQueue() {
    return odometryQueue;
  }
}
