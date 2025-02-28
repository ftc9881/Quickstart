package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SweeperSubsystem;

public class SweeperCommand extends CommandBase {
    private SweeperSubsystem sweeper;
    private double target = 0;

    public SweeperCommand (SweeperSubsystem sweeper, double target) {
        this.sweeper = sweeper;
        this.target = target;
    }

    @Override
    public void initialize () {
        sweeper.setTarget(target);
    }

    @Override
    public boolean isFinished () {
        return true;
    }
}
