package frc.robot.subsystems.intake;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.littletonrobotics.junction.Logger;

import static org.wpilib.units.Units.RPM;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.subsystems.intake.IntakeIO;
import org.wpilib.units.measure.AngularVelocity;

public class IntakeMechanism extends Mechanism {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public IntakeMechanism(IntakeIO io) {
        this.io = io;
        Scheduler.getDefault().addPeriodic(() -> {
            io.updateInputs(inputs);
            Logger.processInputs("Intake", inputs);
        });
    }

    public void runIntake(double speed) {
        io.setRollerSpeed(speed);
    }

    // @Override
    // public void periodic() {
    //     io.updateInputs(inputs);
    //     Logger.processInputs("Intake", inputs);

    //     if (inputs.intakeRollerSpeed.lte(RPM.zero())){
    //         Logger.recordOutput("Intake/Color", "#2ba02b"); //INtaking
    //     }
    //     else if (inputs.intakeRollerSpeed.gte(RPM.zero())){
    //         Logger.recordOutput("Intake/Color", "#fffb00"); //Outatking
    //     }
    //     else {
    //         Logger.recordOutput("Intake/Color", "#ff6600ff");
    //     }
    // }

    public AngularVelocity getRollerSpeed() {
        return inputs.intakeRollerSpeed;
    }
}