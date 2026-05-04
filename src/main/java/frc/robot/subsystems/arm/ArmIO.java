package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RPM;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;
import static org.wpilib.units.Units.Volts;
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

    public default void setArmAngle(Angle angle) {}

    public default void setBrakeOnNeutral(boolean brake){}

    public default void stop() {}
}
