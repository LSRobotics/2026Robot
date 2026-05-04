package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.hardware.discrete.DigitalInput;

public interface ArmLimitSwitchIO {
    
    @AutoLog
    public static class ArmLimitSwitchIOInputs {
        public boolean activity = false;
    }

    public default void updateInputs(ArmLimitSwitchIOInputs inputs) {}
}
