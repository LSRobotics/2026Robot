package frc.robot.subsystems.leds;

import org.wpilib.hardware.motor.Spark;
import org.wpilib.smartdashboard.SmartDashboard;

public class LedsIOBlinkin implements LedsIO {

     private final Spark ledController = 
         new Spark(LEDConstants.LED_CONTROLLER_PWM_PORT);

    public LedsIOBlinkin() {}

    public void updateInputs(LedsIOInputs inputs) {
        inputs.pwmSignal = ledController.get();
        
    }

    @Override
    public void setPWM(double pwm) {
         ledController.set(pwm);
    }
    @Override
    public void setColor(double color) {
         setPWM(color);
    }
}
