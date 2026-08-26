package frc.robot.commands.systemtest;

public record StageResult(boolean passed, String message) {
    public static StageResult pass(String message) {
        return new StageResult(true, message);
    }

    public static StageResult fail(String message) {
        return new StageResult(false, message);
    }
}
