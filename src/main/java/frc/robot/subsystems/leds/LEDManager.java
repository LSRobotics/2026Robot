package frc.robot.subsystems.leds;
//Static class wrapping led subssyetm
public class LEDManager {
    private static LedSubsystem ledSubsystem;

    public static void init(LedSubsystem ledSubsystem) {
        LEDManager.ledSubsystem = ledSubsystem;
    }

    public static void setColor(double color) {
        if (ledSubsystem == null) {
            throw new IllegalStateException("LEDManager not initialized");
        }
        ledSubsystem.setColor(color);
    }

    public static void setDefault(){
        ledSubsystem.setColor(LEDConstants.defaultColor);
    }

    private LEDManager() {
        throw new UnsupportedOperationException("LEDmanager is a  static clas dont instantiate ");
    }

    
}
