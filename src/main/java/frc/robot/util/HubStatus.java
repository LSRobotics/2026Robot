package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

public class HubStatus {

    private HubStatus() {
        throw new UnsupportedOperationException("Static utility clas");
    }

    public static char getFirstInactiveAlliance() {
        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.length() > 0) {
            char alliance = gameData.charAt(0);
            if (alliance == 'R' || alliance == 'B') {
                return alliance;
            }
        }

        return ' ';
    }

    public static boolean isActive() {
        var allianceOptional = DriverStation.getAlliance();
        if (allianceOptional.isEmpty()) {
            return false;
        }

        Alliance myAlliance = allianceOptional.get();
        char myAllianceChar = (myAlliance == Alliance.Red) ? 'R' : 'B';

        char activeHub = getActiveHub();

        if (activeHub == ' ') {
            return false;
        }


        return myAllianceChar == activeHub || activeHub == 'C';
    }

    public static char getActiveHub() {
        double matchTime = DriverStation.getMatchTime();
        double time = Timer.getMatchTime();
        char firstInactive = getFirstInactiveAlliance();

        if (firstInactive == ' ') {
            return ' ';
        }

        char secondInactive = (firstInactive == 'R') ? 'B' : 'R';

        if (!DriverStation.isTeleopEnabled()) {
            return 'C'; // both active in auto
        }

        if (matchTime > 130) {// Transition shift
            return 'C';
        } 
        else if (matchTime > 105) {// shift 1
            return secondInactive;
        } 
        else if (matchTime > 80) { // Shift 2
            return firstInactive;
        } 
        else if (matchTime > 55) { // Shift 3
            return secondInactive;
        } 
        else if (matchTime > 30) { // Shift 4
            return firstInactive;
        } 
        else { // endgame
            return 'C';
        }
    }
}