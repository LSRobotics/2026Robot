package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLog;
import frc.robot.subsystems.intake.IntakeIO;
import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    public void runIntake(double speed) {
        io.setRollerSpeed(speed);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public AngularVelocity getRollerSpeed() {
        return inputs.intakeRollerSpeed;
    }
}