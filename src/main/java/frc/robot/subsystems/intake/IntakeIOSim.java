package frc.robot.subsystems.intake;
import frc.robot.subsystems.intake.IntakeIO;
import org.ironmaple.simulation.IntakeSimulation;

public class IntakeIOSim implements IntakeIO {
    private final IntakeSimulation intakeSimulation;

    public IntakeIOSim(IntakeSimulation intakeSimulation) {
        this.intakeSimulation = intakeSimulation;
    }

    @Override // Defined by IntakeIO
    public void setRollerSpeed(double speed) {
        if (speed != 0)
            intakeSimulation.startIntake(); 
        else
            intakeSimulation.stopIntake();
    }

    // Defined by IntakeIO
    public boolean isFuelInsideIntake() {
        return intakeSimulation.getGamePiecesAmount() != 0; // True if there is a game piece in the intake
    }

    public boolean launchFuel() {
        // if there is a note in the intake, it will be removed and return true; otherwise, returns false
        return intakeSimulation.obtainGamePieceFromIntake();
 
    }
}