package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.Logger;

import org.wpilib.command2.Subsystem;
import org.wpilib.command2.SubsystemBase;

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
