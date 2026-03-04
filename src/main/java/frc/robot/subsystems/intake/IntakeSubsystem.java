package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RPM;

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

        if (inputs.intakeRollerSpeed.lte(RPM.zero())){
            Logger.recordOutput("Intake/Color", "#2ba02b"); //INtaking
        }
        else if (inputs.intakeRollerSpeed.gte(RPM.zero())){
            Logger.recordOutput("Intake/Color", "#fffb00"); //Outatking
        }
        else {
            Logger.recordOutput("Intake/Color", "#ff6600ff");
        }
    }

    public AngularVelocity getRollerSpeed() {
        return inputs.intakeRollerSpeed;
    }
}