package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem;

public class ClawCommand extends CommandBase {
    private ClawSubsystem claw;
    private double target = 0;

    public ClawCommand (ClawSubsystem claw, double target) {
        this.claw = claw;
        this.target = target;
    }

    @Override
    public void initialize () {
        claw.setTarget(target);
    }

    @Override
    public boolean isFinished () {
        return true;
    }
}
