package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LedsIOBlinkin implements LedsIO {

    // private final Spark ledController = 
    //     new Spark(LEDConstants.LED_CONTROLLER_ID);

    public LedsIOBlinkin() {}

    public void updateInputs(LedsIOInputs inputs) {
        // inputs.pwmSignal = ledController.get();
        // inputs.pwmInverted = ledController.getInverted();
    }

    public void setPWM(double pwm) {
        // ledController.set(pwm);
    }

    public void setColor(double color) {
        // setPWM(color);
    }
}
