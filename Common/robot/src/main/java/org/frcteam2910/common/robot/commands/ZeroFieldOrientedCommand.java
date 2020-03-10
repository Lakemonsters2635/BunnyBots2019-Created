package org.frcteam2910.common.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.subsystems.HolonomicDrivetrain;

public class ZeroFieldOrientedCommand extends Command {
    private final HolonomicDrivetrain drivetrain;
    private final double offset;

    public ZeroFieldOrientedCommand(HolonomicDrivetrain drivetrain) {
        this(drivetrain, 0);
    }

    public ZeroFieldOrientedCommand(HolonomicDrivetrain drivetrain, double offset) {
        this.drivetrain = drivetrain;
        this.offset = offset;
    }

    @Override
    protected void initialize() {
        drivetrain.getGyroscope().setAdjustmentAngle(Rotation2.fromDegrees(offset + drivetrain.getGyroscope().getUnadjustedAngle().toDegrees()));
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
