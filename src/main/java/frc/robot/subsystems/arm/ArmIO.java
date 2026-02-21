package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Volts;
import frc.robot.subsystems.arm.ArmConstants.ArmMotorConstants;

public interface ArmIO {

    @AutoLog
    public static class ArmIOInputs {
        public Current armMotorCurrent = Amps.of(0);
        public boolean armOut = false;
        public Angle armAngle = ArmMotorConstants.ARM_REST_ANGLE;
        public AngularVelocity armSpeed = RPM.of(0);
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setArmSpeed(double speed) {}

    public default void setVoltage(Voltage voltage) {}

    public default void stop() {}
}
