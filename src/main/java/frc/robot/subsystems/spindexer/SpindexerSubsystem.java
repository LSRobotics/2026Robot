package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLog;
import frc.robot.subsystems.spindexer.SpindexerIO;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class SpindexerSubsystem extends SubsystemBase {
    private final SpindexerIO io;
    private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    public SpindexerSubsystem(SpindexerIO io) {
        this.io = io;
    }

    public void runSpindexer(double speed) {
        io.setSpindexerSpeed(speed);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Spindexer", inputs);
    }

    public AngularVelocity getSpindexerSpeed() {
        return inputs.spindexerSpeed;
    }

    public Current getSpindexerCurrent() {
        return inputs.spindexerMotorCurrent;
    }
}