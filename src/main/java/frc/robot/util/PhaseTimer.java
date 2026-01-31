package frc.robot.util;

import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

/**
 * PhaseTimer is a utility class to manage timed phases in a robot match.
 * 
 * Usage:
 * - Call PhaseTimer.initialize() to set up the timer based on alliance and game data.
 * - Call PhaseTimer.start() to begin the timer.
 * - Periodically call PhaseTimer.update() from Robot.periodic() to handle phase transitions.
 * - Use PhaseTimer.isHubActive() to check if the hub is active.
 * - Use PhaseTimer.getTime() to get the elapsed time.
 */
public final class PhaseTimer {
    private static Optional<Alliance> alliance = DriverStation.getAlliance();
    private static String whoIsInactiveFirst = DriverStation.getGameSpecificMessage();
    private static String currentPhase;
    private static Timer phaseTimer = new Timer();
    private static boolean amIActive;
    private static boolean transitionPhaseDone = false;
    private static boolean firstPhaseDone = false;
    private static boolean secondPhaseDone = false;
    private static boolean thirdPhaseDone = false;
    private static boolean fourthPhaseDone = false;
    private static double elapsedTime = 0;

    public static boolean initialize() {
        if (alliance.isPresent()) {
            if (alliance.get().equals(Alliance.Red) && whoIsInactiveFirst.equals("R")) {
                amIActive = true;
                start();
                return true;
            } else if (alliance.get().equals(Alliance.Blue) && whoIsInactiveFirst.equals("B")) {
                amIActive = true;
                start();
                return true;
            } else {
                amIActive = false;
                start();
                return true;
            }
        }
        start();
        return false;
    }

    public static void start() {
        phaseTimer.reset();
        phaseTimer.start();
        resetPhases();
    }

    public static void stop() {
        phaseTimer.stop();
    }

    public static void update() {
        elapsedTime = phaseTimer.get();

        if (elapsedTime >= 10 && !transitionPhaseDone) {
            transitionPhaseDone = true;
            amIActive = !amIActive;
            phaseTimer.reset(); // Reset after transition phase
        } else if (elapsedTime >= 25 && !firstPhaseDone) {
            firstPhaseDone = true;
            amIActive = !amIActive;
            phaseTimer.reset(); // Reset after first phase
        } else if (elapsedTime >= 25 && !secondPhaseDone) {
            secondPhaseDone = true;
            amIActive = !amIActive;
            phaseTimer.reset(); // Reset after second phase
        } else if (elapsedTime >= 25 && !thirdPhaseDone) {
            thirdPhaseDone = true;
            amIActive = !amIActive;
            phaseTimer.reset(); // Reset after third phase
        } else if (elapsedTime >= 25 && !fourthPhaseDone) {
            fourthPhaseDone = true;
            amIActive = true;
            phaseTimer.reset(); // Reset after fourth phase
        }

        if (fourthPhaseDone) {
            currentPhase = "Endgame";
        } else if (thirdPhaseDone) {
            currentPhase = "Phase 4";
        } else if (secondPhaseDone) {
            currentPhase = "Phase 3";
        } else if (firstPhaseDone) {
            currentPhase = "Phase 2";
        } else if (transitionPhaseDone) {
            currentPhase = "Phase 1";
        } else {
            currentPhase = "Transition Phase";
        }
    }

    public static boolean isHubActive() {
        return amIActive;
    }

    public static double getTime() {
        return phaseTimer.get();
    }

    public static String getCurrentPhase() {
        return currentPhase;
    }

    private static void resetPhases() {
        transitionPhaseDone = false;
        firstPhaseDone = false;
        secondPhaseDone = false;
        thirdPhaseDone = false;
        fourthPhaseDone = false;
    }
}
