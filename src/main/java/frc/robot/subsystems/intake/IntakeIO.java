package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import static edu.wpi.first.units.Units.Amps;
import edu.wpi.first.units.measure.Current;


public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {
        public Current intakeRollerMotorCurrent = Amps.of(0);
        public Current intakeArmMotorCurrent = Amps.of(0);
        public boolean hasGamePiece = false;
        public boolean intakeArmOut = false;
        public double intakeRollerSpeed = 0.0;
        public double intakeArmSpeed = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setRollerSpeed(double speed) {}

    public default void setArmSpeed(double speed) {}

    public default void stop() {}
}
