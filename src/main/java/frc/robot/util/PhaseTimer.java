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
    private static boolean transitionPhaseDone = false;
    private static boolean firstPhaseDone = false;
    private static boolean secondPhaseDone = false;
    private static boolean thirdPhaseDone = false;
    private static boolean fourthPhaseDone = false;
    private static double elapsedTime = 0;
    
    public static boolean initialize() {
        alliance = DriverStation.getAlliance();
        whoIsInactiveFirst = DriverStation.getGameSpecificMessage();
        
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
                amIActive = true;
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
        amIActive = true;
    }
    
    public static void stop() {
        phaseTimer.stop();
    }
    
    public static void update() {
        elapsedTime = phaseTimer.get();
        
        if (elapsedTime >= 10 && !transitionPhaseDone) {
            transitionPhaseDone = true;
            alliance = DriverStation.getAlliance();
            whoIsInactiveFirst = DriverStation.getGameSpecificMessage();
            if (alliance.isPresent()) {
                if ((alliance.get().equals(Alliance.Red) && whoIsInactiveFirst.equals("R")) ||
                    (alliance.get().equals(Alliance.Blue) && whoIsInactiveFirst.equals("B"))) {
                    amIActive = false;
                } else {
                    amIActive = true;
                }
            }
        } else if (elapsedTime >= 35 && !firstPhaseDone) {
            firstPhaseDone = true;
            amIActive = !amIActive;
        } else if (elapsedTime >= 60 && !secondPhaseDone) {
            secondPhaseDone = true;
            amIActive = !amIActive;
        } else if (elapsedTime >= 85 && !thirdPhaseDone) {
            thirdPhaseDone = true;
            amIActive = !amIActive;
        } else if (elapsedTime >= 110 && !fourthPhaseDone) {
            fourthPhaseDone = true;
            amIActive = true;
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