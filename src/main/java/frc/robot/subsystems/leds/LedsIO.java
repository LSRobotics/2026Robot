package frc.robot.subsystems.leds;

import java.security.PublicKey;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public interface LedsIO {
    @AutoLog
    public static class LedsIOInputs {
        public double colorCode = 0;
        public double pwmSignal = 0;
        public boolean pwmInverted = false;
    }

  public static class LedsIOOutputs {
    public byte[] buffer = new byte[0];
  }

  public default void updateInputs(LedsIOInputs inputs) {}

  public default void applyOutputs(LedsIOOutputs outputs) {}


    public static void setColor(double color) {}

    public static void setPWM(double pwm) {}
}