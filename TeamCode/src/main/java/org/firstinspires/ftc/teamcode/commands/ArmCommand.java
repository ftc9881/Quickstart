package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem;

public class ArmCommand extends CommandBase {
    private ArmSubsystem arm;
    private double target = 0;


    public ArmCommand (ArmSubsystem arm, double target) {
        this.arm = arm;
        this.target = target;
    }

    @Override
    public void initialize () {
        arm.setTarget(target);
    }

    @Override
    public boolean isFinished () {
        return true;
    }
}
