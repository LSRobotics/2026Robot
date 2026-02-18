package frc.robot.util;
import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public final class PhaseTimer {
    private static Optional<Alliance> alliance;
    private static String whoIsInactiveFirst;
    private static String currentPhase;
    private static Timer phaseTimer = new Timer();
    private static boolean amIActive;
    private static boolean amIInactiveFirst;
    private static boolean transitionPhaseDone = false;
    private static boolean firstPhaseDone = false;
    private static boolean secondPhaseDone = false;
    private static boolean thirdPhaseDone = false;
    private static boolean fourthPhaseDone = false;
    private static double elapsedTime = 0;
    private static double timeLimit = 10;
    
    public static boolean initialize() {
        alliance = DriverStation.getAlliance();
        whoIsInactiveFirst = DriverStation.getGameSpecificMessage();
        
        if (alliance.isPresent()) {
            if ((alliance.get().equals(Alliance.Red) && whoIsInactiveFirst.equals("R")) || 
            (alliance.get().equals(Alliance.Blue) && whoIsInactiveFirst.equals("B"))) {
                amIInactiveFirst = true;
                start();
                return true;
            } else {
                amIInactiveFirst = false;
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

        if (!transitionPhaseDone) {
            amIActive = false;
        }
        
        if (elapsedTime >= timeLimit && !transitionPhaseDone) {
            transitionPhaseDone = true;
            amIActive = !amIInactiveFirst;
            timeLimit = 25;
            phaseTimer.reset();
            // alliance = DriverStation.getAlliance();
            // whoIsInactiveFirst = DriverStation.getGameSpecificMessage();
            // if (alliance.isPresent()) {
            //     if ((alliance.get().equals(Alliance.Red) && whoIsInactiveFirst.equals("R")) ||
            //         (alliance.get().equals(Alliance.Blue) && whoIsInactiveFirst.equals("B"))) {
            //         amIActive = false;
            //     } else {
            //         amIActive = true;
            //     }
            // }
        } else if (elapsedTime >= timeLimit && !firstPhaseDone) {
            firstPhaseDone = true;
            amIActive = !amIActive;
            phaseTimer.reset();
        } else if (elapsedTime >= timeLimit && !secondPhaseDone) {
            secondPhaseDone = true;
            amIActive = !amIActive;
            phaseTimer.reset();
        } else if (elapsedTime >= timeLimit && !thirdPhaseDone) {
            thirdPhaseDone = true;
            amIActive = !amIActive;
            phaseTimer.reset();
        } else if (elapsedTime >= timeLimit && !fourthPhaseDone) {
            fourthPhaseDone = true;
            amIActive = true;
            timeLimit = 30;
            phaseTimer.reset();
        }
        
        if (fourthPhaseDone) {
            currentPhase = "Endgame";
        } else if (thirdPhaseDone) {
            currentPhase = "Shift 4";
        } else if (secondPhaseDone) {
            currentPhase = "Shift 3";
        } else if (firstPhaseDone) {
            currentPhase = "Shift 2";
        } else if (transitionPhaseDone) {
            currentPhase = "Shift 1";
        } else {
            currentPhase = "Transition Phase";
        }
    }
    
    public static boolean isHubActive() {
        return amIActive;
    }
    
    public static double getTime() {
        return timeLimit - phaseTimer.get();
    }
    
    public static String getCurrentPhase() {
        return currentPhase;
    }
    
    private static void resetPhases() {
        timeLimit = 10;
        transitionPhaseDone = false;
        firstPhaseDone = false;
        secondPhaseDone = false;
        thirdPhaseDone = false;
        fourthPhaseDone = false;
    }
}