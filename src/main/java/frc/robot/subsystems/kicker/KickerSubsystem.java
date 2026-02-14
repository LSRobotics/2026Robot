package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLog;
import frc.robot.subsystems.kicker.KickerIO;
import edu.wpi.first.units.measure.AngularVelocity;

public class KickerSubsystem extends SubsystemBase {
    private final KickerIO io;
    private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

    public KickerSubsystem(KickerIO io) {
        this.io = io;
    }

    public void runKicker(double speed) {
        io.setKickerSpeed(speed);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Kicker", inputs);
    }

    public AngularVelocity getKickerSpeed() {
        return inputs.kickerSpeed;
    }
}