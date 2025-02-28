package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;

public class PivotCommand extends CommandBase {
    private PivotSubsystem pivot;
    private double target = .95;

    public PivotCommand (PivotSubsystem pivot, double target) {
        this.pivot = pivot;
        this.target = target;
    }

    @Override
    public void initialize () {
        pivot.setTarget(target);
    }

    @Override
    public boolean isFinished () {
        return true;
    }
}
