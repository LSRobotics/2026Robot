package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmIO;

public class ArmSubsystem extends SubsystemBase {
    private ArmIO armIO;
    private ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
    private ArmLimitSwitchIO limitSwitchIO;
    private ArmLimitSwitchIOInputsAutoLogged limitSwitchInputs = new ArmLimitSwitchIOInputsAutoLogged();

    public ArmSubsystem(ArmIO armIO, ArmLimitSwitchIO limitSwitchIO) {
        this.armIO = armIO;
        this.limitSwitchIO = limitSwitchIO;
    }

    public void runArm(double speed) {
        armIO.setArmSpeed(speed);
    }

    public void setAngle(Angle angle) {
        armIO.setArmAngle(angle);
    }

    @Override
    public void periodic() {
        armIO.updateInputs(armInputs);
        limitSwitchIO.updateInputs(limitSwitchInputs);
        Logger.processInputs("Arm", armInputs);
        Logger.processInputs("LimitSwitch", limitSwitchInputs);
    }

    public AngularVelocity getArmSpeed() {
        return armInputs.armSpeed;
    }

    public Angle getArmEncoder() {
        return armInputs.armAngle;
    }

    public boolean getLimitSwitch() {
        return limitSwitchInputs.activity;
    }
}
