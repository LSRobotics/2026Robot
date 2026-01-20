package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase{

    private final TurretIO io;
    private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    public TurretSubsystem(TurretIO io){
        this.io = io;
    }

    public void periodic(){
        io.updateInputs(inputs);
    }

    public void setVoltage(Voltage voltage){
        io.setTurretVoltage(voltage);
    }

    public void setSpeed(double speed){
        io.setTurretSpeed(speed);
    }

    public void pointAtAngle(Angle angle){
        PIDController pid = new PIDController(0.1, 0, 0.01);
        double speed = pid.calculate(inputs.turretAngle.in(Degrees), angle.in(Degrees));
        setSpeed(speed);
        pid.close();
    }

    public Angle getAngle(){
        return inputs.turretAngle;
    }

    
}
