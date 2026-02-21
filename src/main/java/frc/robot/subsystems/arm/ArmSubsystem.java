package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmIO;

public class ArmSubsystem extends SubsystemBase {
    private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public ArmSubsystem(ArmIO io) {
        this.io = io;
    }

    public void runArm(double speed) {
        io.setArmSpeed(speed);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }

    public AngularVelocity getArmSpeed() {
        return inputs.armSpeed;
    }

    public Angle getArmEncoder() {
        return inputs.armAngle;
    }
}
