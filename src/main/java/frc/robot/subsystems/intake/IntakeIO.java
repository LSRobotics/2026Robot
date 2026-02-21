package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Volts;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {
        public Current intakeRollerMotorCurrent = Amps.of(0);
        public boolean hasGamePiece = false;
        public AngularVelocity intakeRollerSpeed = RPM.of(0);
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setRollerSpeed(double speed) {}

    public default void setVoltage(Voltage voltage) {}

    public default void stop() {}
}
