package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLog;
import frc.robot.subsystems.intake.IntakeIO;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    public void runIntake() {
        io.setRollerSpeed(IntakeConstants.INTAKE_SPEED);
    }

    public void runArm() {
        io.setArmSpeed(IntakeConstants.INTAKE_ARM_SPEED);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public double getRollerSpeed() {
        return inputs.intakeRollerSpeed;
    }
}