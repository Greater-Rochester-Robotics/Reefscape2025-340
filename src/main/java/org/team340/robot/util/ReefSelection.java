package org.team340.robot.util;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.team340.lib.util.command.DummySubsystem;

@Logged
public final class ReefSelection {

    @NotLogged
    private final Subsystem scoringMutex = new DummySubsystem();

    private int level = 1;
    private boolean left = false;
    private boolean scoring = false;

    @NotLogged
    public int getLevel() {
        return level;
    }

    @NotLogged
    public boolean isL1() {
        return level == 1;
    }

    @NotLogged
    public boolean isLeft() {
        return left;
    }

    @NotLogged
    public boolean isScoring() {
        return scoring;
    }

    public Command selectLevel(int level) {
        return Commands.runOnce(() -> this.level = level)
            .ignoringDisable(true)
            .withName("ReefSelection.incrementLevel()");
    }

    public Command incrementLevel() {
        return Commands.runOnce(() -> level = (int) MathUtil.inputModulus(level + 1, 0.5, 4.5))
            .ignoringDisable(true)
            .withName("ReefSelection.incrementLevel()");
    }

    public Command decrementLevel() {
        return Commands.runOnce(() -> level = (int) MathUtil.inputModulus(level - 1, 0.5, 4.5))
            .ignoringDisable(true)
            .withName("ReefSelection.decrementLevel()");
    }

    public Command setLeft() {
        return Commands.runOnce(() -> left = true).ignoringDisable(true).withName("ReefSelection.setLeft()");
    }

    public Command setRight() {
        return Commands.runOnce(() -> left = false).ignoringDisable(true).withName("ReefSelection.setRight()");
    }

    public Command whileScoring() {
        return scoringMutex
            .startEnd(() -> scoring = true, () -> scoring = false)
            .withName("ReefSelection.whileScoring()");
    }
}
