package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;

public class SensorCommand extends CommandBase {
    private IntakeSubsystem intake;
    private ExtendoSubsystem extendo;
    private SensorSubsystem sensor;
    private String color;

    public SensorCommand(IntakeSubsystem intake, ExtendoSubsystem extendo, SensorSubsystem sensor, String color) {
        this.intake = intake;
        this.extendo = extendo;
        this.sensor = sensor;
        this.color = color;
        addRequirements(intake, extendo, sensor);
    }

    @Override
    public void initialize() {
        intake.setTarget(-1);
        extendo.setTarget(240);
    }

    @Override
    public void execute() {
        if (color.equals("yellow")) {
            while (!sensor.isYellow()) {
                if (sensor.isRed() || sensor.isBlue()) {
                    intake.setTarget(1);
                    extendo.setTarget(420);
                }
                else {
                    intake.setTarget(-1);
                    extendo.setTarget(400);
                }
            }
        }
        if (color.equals("red")) {
            while (!sensor.isRed()) {
                if (sensor.isYellow() || sensor.isBlue()) {
                    intake.setTarget(1);
                    extendo.setTarget(420);
                }
                else {
                    intake.setTarget(-1);
                    extendo.setTarget(400);
                }
            }
        }
        if (color.equals("blue")) {
            while (!sensor.isBlue()) {
                if (sensor.isRed() || sensor.isYellow()) {
                    intake.setTarget(1);
                    extendo.setTarget(420);
                }
                else {
                    intake.setTarget(-1);
                    extendo.setTarget(400);
                }
            }
        }
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
        intake.setTarget(0); // Stop intake
        extendo.setTarget(0); // Retract extendo
    }
}
