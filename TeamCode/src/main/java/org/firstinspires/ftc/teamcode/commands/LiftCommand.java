package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class LiftCommand extends CommandBase {
    private LiftSubsystem lift;
    private int length;

    public LiftCommand (LiftSubsystem lift, int length) {
        this.lift = lift;
        this.length = length;
    }

    @Override
    public void initialize () {
        lift.setTarget(length);
    }

    @Override
    public boolean isFinished () {
        return lift.isAtTarget();
    }
}
