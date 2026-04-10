package frc.robot.util;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandLogger {
    private static LinkedHashSet<Command> runningCommands = new LinkedHashSet<>();
    private CommandLogger() {
        throw new UnsupportedOperationException("Static class dont instantiate");
    }

    public static void logCommandStart(Command command) {
        runningCommands.add(command);
        logState();
    }

    public static void logCommandEnd(Command command) {
        runningCommands.remove(command);
        logState();
    }

    public static List<String> getRunningCommandNames() {
        return runningCommands.stream().map(Command::getName).toList();
    }

    private static void logState() {
        Logger.recordOutput("LoggedCommands/runningCommandNames", runningCommands.stream().map(Command::getName).toArray(String[]::new));
    }

    public static ArrayList<Command> getRunningCommands() {
        return new ArrayList<>(runningCommands);
    }


}
