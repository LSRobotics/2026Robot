package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.arm.ArmConstants.ArmLimitSwitchConstants;

public class ArmLimitSwitchIOLimitSwitch implements ArmLimitSwitchIO {
    private final DigitalInput limitSwitch = new DigitalInput(ArmLimitSwitchConstants.LIMIT_SWITCH_ID);

    public ArmLimitSwitchIOLimitSwitch() {}

    public void updateInputs(ArmLimitSwitchIOInputs inputs) {
        inputs.activity = limitSwitch.get();
    }
}
