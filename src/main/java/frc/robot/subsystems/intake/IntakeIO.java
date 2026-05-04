package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RPM;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;
import static org.wpilib.units.Units.Volts;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {
        public Current intakeRollerMotorCurrent = Amps.of(0);
        public AngularVelocity intakeRollerSpeed = RPM.of(0);
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setRollerSpeed(double speed) {}

    public default void setVoltage(Voltage voltage) {}

    public default void stop() {}
}
