// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import static org.wpilib.units.Units.Degrees;

// import java.util.function.Supplier;

// import org.littletonrobotics.junction.Logger;

// import org.wpilib.units.measure.Angle;
// import org.wpilib.command2.Command;
// import frc.robot.subsystems.arm.ArmConstants.ArmMotorConstants;
// import frc.robot.subsystems.arm.ArmConstants;
// import frc.robot.subsystems.arm.ArmSubsystem;

// /** An example command that uses an example subsystem. */
// public class ArmOutCommand extends Command {
//   @SuppressWarnings("PMD.UnusedPrivateField")
//   private final ArmSubsystem m_arm;
//   private final Supplier<Angle> targetAngle;
//   private final Supplier<Double> targetArmDegrees;
//   private final Supplier<Double> targetMotorDegrees;
//   private final Supplier<Double> speed;

//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */

//   public ArmOutCommand(ArmSubsystem arm, Supplier<Angle> angle) {
//     m_arm = arm;
//     this.targetAngle = angle;
//     this.targetArmDegrees = ()->angle.get().in(Degrees);
//     this.targetMotorDegrees = ()->targetArmDegrees.get() * ArmMotorConstants.gearRatio;
//     this.speed = ()->ArmMotorConstants.ARM_SPEED;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(arm);
//   }

//   public ArmOutCommand(ArmSubsystem arm, Supplier<Angle> angle, Supplier<Double> speed) {
//     m_arm = arm;
//     this.targetAngle = angle;
//     this.targetArmDegrees = ()->angle.get().in(Degrees);
//     this.targetMotorDegrees = ()->targetArmDegrees.get() * ArmMotorConstants.gearRatio;
//     this.speed = speed;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(arm);
//   }
//   public ArmOutCommand(ArmSubsystem arm, Angle angle) {
//     m_arm = arm;
//     this.targetAngle = ()->angle;
//     this.targetArmDegrees = ()->angle.in(Degrees);
//     this.targetMotorDegrees = ()->targetArmDegrees.get() * ArmMotorConstants.gearRatio;
//     this.speed = ()->ArmMotorConstants.ARM_SPEED;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(arm);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     Logger.recordOutput("Arm/TargetArmDegrees", targetArmDegrees.get());
//     Logger.recordOutput("Arm/TargetMotorDegrees", targetMotorDegrees.get());
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double motorDegrees = m_arm.getArmEncoder().in(Degrees);
//     double armDegrees = motorDegrees / ArmMotorConstants.gearRatio;
//     double armErrorDegrees = targetArmDegrees.get() - armDegrees;
//     double motorErrorDegrees = targetMotorDegrees.get() - motorDegrees;

//     Logger.recordOutput("Arm/MotorAngleDegrees", motorDegrees);
//     Logger.recordOutput("Arm/ArmAngleDegrees", armDegrees);
//     Logger.recordOutput("Arm/TargetArmDegrees", targetArmDegrees.get());
//     Logger.recordOutput("Arm/TargetMotorDegrees", targetMotorDegrees.get());
//     Logger.recordOutput("Arm/ArmErrorDegrees", armErrorDegrees);
//     Logger.recordOutput("Arm/MotorErrorDegrees", motorErrorDegrees);

//     if (Math.abs(armErrorDegrees) <= ArmMotorConstants.ARM_TOLERANCE) {
//       m_arm.runArm(0);
//       return;
//     }

//     double direction = Math.signum(motorErrorDegrees);

//     // if (direction > 0 && m_arm.getLimitSwitch()) {
//     // m_arm.runArm(0);
//     // return;
//     // }

//     if (armDegrees>ArmConstants.ArmMotorConstants.ARM_REST_ANGLE.in(Degrees) && direction==1d){
//       direction = 0;
//     }

//     else if (armDegrees<ArmConstants.ArmMotorConstants.ARM_DEPLOY_ANGLE.in(Degrees) && direction==-1d){
//       direction = 0;
//     }

//     m_arm.runArm(direction * speed.get());
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_arm.runArm(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     double armDegrees = m_arm.getArmEncoder().in(Degrees) / ArmMotorConstants.gearRatio;
//     return Math.abs(targetArmDegrees.get() - armDegrees) <= ArmMotorConstants.ARM_TOLERANCE;
//   }
// }
