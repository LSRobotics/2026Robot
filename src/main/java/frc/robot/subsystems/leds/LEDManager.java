package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.Logger;

//singleton
public class LEDManager {
    private static LEDMechanism ledSubsystem;

    public static void init(LEDMechanism ledSubsystem1) {
        LEDManager.ledSubsystem = ledSubsystem1;
    }

    public static void setColor(double color) {
        if (ledSubsystem == null) {
            throw new IllegalStateException("LEDManager not initialized");
        }
        ledSubsystem.setColor(color);
        Logger.recordOutput("LED", color);
    }

    public static void setDefault(){
        if (ledSubsystem == null) {
            throw new IllegalStateException("LEDManager not initialized");
        }
        LEDManager.ledSubsystem.setColor(LEDConstants.defaultColor);
        Logger.recordOutput("LED", LEDConstants.defaultColor);
    }

    private LEDManager() {
        throw new UnsupportedOperationException("LEDmanager is a  static clas dont instantiate ");
    }

    
}
