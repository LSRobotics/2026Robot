package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.Logger;


import org.wpilib.command3.Mechanism;

public class LEDMechanism extends Mechanism{
    private final LedsIO io;
    private LedsIOInputsAutoLogged inputs = new LedsIOInputsAutoLogged();
    public LEDMechanism(LedsIO IO){
        this.io = IO;
    }

    public void setColor(double color) {
        io.setPWM(color);
        io.updateInputs(inputs);
    }
}
