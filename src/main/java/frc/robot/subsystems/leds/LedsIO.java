package frc.robot.subsystems.leds;


import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public interface LedsIO {
    @AutoLog
    public static class LedsIOInputs {
        public double pwmSignal = 0;
    }

  public static class LedsIOOutputs {
  }

  public default void updateInputs(LedsIOInputs inputs) {}

  public default void applyOutputs(LedsIOOutputs outputs) {}


    public default void setColor(double color) {}

    public default void setPWM(double pwm) {}
}