package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem;

public class ExtendoCommand extends CommandBase {
    private ExtendoSubsystem extendo;
    private int length;

    public ExtendoCommand (ExtendoSubsystem extendo, int length) {
        this.extendo = extendo;
        this.length = length;
    }

    @Override
    public void initialize () {
        extendo.setTarget(length);
    }

    @Override
    public boolean isFinished () {
        return extendo.isAtTarget();
    }
}
