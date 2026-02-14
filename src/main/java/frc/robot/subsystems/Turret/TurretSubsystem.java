package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;
import org.xml.sax.ext.DeclHandler;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MathUtils;

public class TurretSubsystem extends SubsystemBase{

    private final TurretIO io;
    public TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged(); //TODO: Make private when done debugging


    public TurretSubsystem(TurretIO io){
        this.io = io;
    }

    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
    }

    public void setVoltage(Voltage voltage){
        io.setTurretVoltage(voltage);
    }

    public void setSpeed(double speed){
        io.setTurretSpeed(speed);
    }

    public Angle getAngle(){
        return inputs.turretAngle;
    }

    public void zeroEncoder(){
        io.zeroEncoder();
    }

    
}
