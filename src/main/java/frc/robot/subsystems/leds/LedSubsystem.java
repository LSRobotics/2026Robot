package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase{
    private final LedsIO io;
    public LedSubsystem(LedsIO IO){
        this.io = IO;
    }

    public void setColor(double color) {
        LedsIO.setPWM(color);
    
    }

}
