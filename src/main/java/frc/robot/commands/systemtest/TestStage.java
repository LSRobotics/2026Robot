package frc.robot.commands.systemtest;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public final class TestStage {
    public final String name;
    public final Command action;
    public final Runnable monitor;
    public final Supplier<StageResult> check;
    public final Runnable stop;

    public TestStage(String name, Command action, Runnable monitor,
            Supplier<StageResult> check, Runnable stop) {
        this.name = name;
        this.action = action;
        this.monitor = monitor;
        this.check = check;
        this.stop = stop;
    }
}
