package frc.robot.subsystems.Turret;

import static org.wpilib.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;
import org.xml.sax.ext.DeclHandler;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.controller.PIDController;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Voltage;
import org.wpilib.smartdashboard.SmartDashboard;
import frc.robot.util.MathUtils;

public class TurretMechanism extends Mechanism{

    private final TurretIO io;
    private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged(); 

    public TurretMechanism(TurretIO io){
        this.io = io;
        io.zeroEncoder();
        Scheduler.getDefault().addPeriodic(() -> {
            io.updateInputs(inputs);
            Logger.processInputs("Turret", inputs);
        });
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
