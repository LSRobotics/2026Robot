package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class LedSubsystem {
    private final LedsIO io;
    public LedSubsystem(LedsIO IO){
        this.io = IO;
    }

    public void setColor(double color) {
        LedsIO.setPWM(color);
    
    }

}
