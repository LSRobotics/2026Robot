package frc.robot.subsystems.arm;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;
import frc.robot.subsystems.arm.ArmConstants.ArmMotorConstants;;

public class ArmIOSparkMax implements ArmIO {
    private final SparkMax armMotor = new SparkMax(0, ArmMotorConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    private final SparkBaseConfig leaderConfig = new SparkMaxConfig().smartCurrentLimit((int) ArmMotorConstants.STATOR_CURRENT_LIMIT.in(Amps)).secondaryCurrentLimit(ArmMotorConstants.SUPPLY_CURRENT_LIMIT.in(Amps)).idleMode(IdleMode.kBrake);
    //private final SparkMax armMotorFollower = new SparkMax(ArmMotorConstants.ARM_MOTOR_FOLLOWER_ID, MotorType.kBrushless);
    private final SparkBaseConfig followerConfig = new SparkMaxConfig().smartCurrentLimit((int) ArmMotorConstants.STATOR_CURRENT_LIMIT.in(Amps)).secondaryCurrentLimit(ArmMotorConstants.SUPPLY_CURRENT_LIMIT.in(Amps)).idleMode(IdleMode.kBrake);


    public ArmIOSparkMax() {
        SparkBaseConfig config = new SparkMaxConfig();
        config = config
            .smartCurrentLimit((int) ArmMotorConstants.STATOR_CURRENT_LIMIT.in(Amps))
            .secondaryCurrentLimit(ArmMotorConstants.SUPPLY_CURRENT_LIMIT.in(Amps))
            .idleMode(IdleMode.kBrake);

        armMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        //config.follow(ArmMotorConstants.ARM_MOTOR_ID);

        //armMotorFollower.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armMotorCurrent = Amps.of(armMotor.getOutputCurrent());
        inputs.armAngle = Angle.ofBaseUnits(armMotor.getEncoder().getPosition(), Degrees);
        inputs.armOut = inputs.armAngle.equals(ArmMotorConstants.ARM_DEPLOY_ANGLE);
        inputs.armSpeed = AngularVelocity.ofBaseUnits(armMotor.getEncoder().getVelocity(), RPM);
    }

    @Override
    public void setArmSpeed(double speed) {
        armMotor.set(speed);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        armMotor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void setArmAngle(Angle angle) {
        armMotor.getEncoder().setPosition(angle.in(Degrees));
    }

    @Override
    public void stop() {
        armMotor.stopMotor();
    }

    // @Override 
    // public void setBrakeOnNeutral(boolean brake){
    //     if (brake){
    //         armMotor.configure(leaderConfig.idleMode(IdleMode.kBrake),ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    //         armMotorFollower.configure(followerConfig.idleMode(IdleMode.kBrake),ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    //     }
    //     else{
    //         armMotor.configure(leaderConfig.idleMode(IdleMode.kBrake),ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    //         armMotorFollower.configure(followerConfig.idleMode(IdleMode.kBrake),ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    //     }
    // }
    
    // public AngularVelocity getArmSpeed() {
    //     return RPM.of(armMotor.getRotorVelocity().getValueAsDouble());
    // }
    
}
