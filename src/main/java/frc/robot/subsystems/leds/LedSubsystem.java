package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase{
    private final LedsIO io;
    private LedsIOInputsAutoLogged inputs = new LedsIOInputsAutoLogged();
    public LedSubsystem(LedsIO IO){
        this.io = IO;
    }

    public void setColor(double color) {
        io.setPWM(color);
        io.updateInputs(inputs);
    }
}
