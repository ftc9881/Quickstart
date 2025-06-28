package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;

public class SensorCommand extends CommandBase {
    private SensorSubsystem sensor;
    private String color;

    public SensorCommand(SensorSubsystem sensor, String color) {
        this.sensor = sensor;
        this.color = color;
        addRequirements(sensor);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        if (color.equals("yellow")) {
            return sensor.isYellow(); // Stop when red/yellow is detected
        }
        if (color.equals("red")) {
            return sensor.isRed(); // Stop when red/yellow is detected
        }
        if (color.equals("blue")) {
            return sensor.isBlue(); // Stop when red/yellow is detected
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
