package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private IntakeSubsystem intake;
    private double power = 0;

    public IntakeCommand (IntakeSubsystem intake, double power) {
        this.intake = intake;
        this.power = power;
    }

    @Override
    public void initialize () {
        intake.setTarget(power);
    }

    @Override
    public boolean isFinished () {
        return true;
    }
}
