package frc.robot.subsystems.kicker;

import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.AngularVelocity;
import frc.robot.subsystems.kicker.KickerIO;

public class KickerMechanism extends Mechanism {
    private final KickerIO io;
    private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

    public KickerMechanism(KickerIO io) {
        this.io = io;
        Scheduler.getDefault().addPeriodic(() -> {
            io.updateInputs(inputs);
            Logger.processInputs("Kicker", inputs);
        });
    }

    public void runKicker(double speed) {
        io.setKickerSpeed(speed);
    }
      public void runKicker(DoubleSupplier speed) {
        io.setKickerSpeed(speed.getAsDouble());
    }

    public AngularVelocity getKickerVelocity() {
        return inputs.kickerVelocity;
    }

    public double getKickerSpeed() {
        return inputs.kickerSpeed;
    }
}