package frc.robot.subsystems.leds;


import org.littletonrobotics.junction.AutoLog;

import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;

public interface LedsIO {
    @AutoLog
    public static class LedsIOInputs {
        public double pwmSignal = 0;
        public boolean pwmInvereted = false;
    }

  public default void updateInputs(LedsIOInputs inputs) {}

  public default void setColor(double color) {}

  public default void setPWM(double pwm) {}
}