package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import java.util.ArrayList;

public class IntakeUntilCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final ExtendoSubsystem extendo;
    private final SensorSubsystem sensor;
    private final Follower follower; // For path following
    private String color; // color of wanted sample
    private final int initialExtendoPosition = 80;
    private final int maxExtendoPosition = 160;
    private boolean sampleDetected = false;
    private boolean isGoodSample = false;
    private Pose currentPosition;
    private Pose nextPosition;

    public IntakeUntilCommand(IntakeSubsystem intake, ExtendoSubsystem extendo,
                              SensorSubsystem colorSensor, Follower follower,
                              Pose currentPosition, Pose nextPosition, String color) {
        this.intake = intake;
        this.extendo = extendo;
        this.sensor = colorSensor;
        this.follower = follower;
        this.currentPosition = currentPosition;
        this.nextPosition = nextPosition;
        this.color = color;
        addRequirements(intake, extendo, colorSensor); // No follower requirement (handled by PedroCommand)
    }

    @Override
    public void initialize() {
        intake.setTarget(-1);
        extendo.setTarget(initialExtendoPosition);
        sampleDetected = false;
        isGoodSample = false;
    }

    @Override
    public void execute() {
        // Extend until a sample is close or max position is reached
        if (!sampleDetected && extendo.getPos() < maxExtendoPosition) {
            extendo.setTarget(extendo.getPos() + 10);
        }

        // Check proximity to detect a sample
        if (sensor.isBlue() || sensor.isRed() || sensor.isYellow()) { // Sample is in
            sampleDetected = true;
            if (color.equals("yellow")) {
                isGoodSample = sensor.isYellow();
            }
            else if (color.equals("blue")) {
                isGoodSample = sensor.isBlue();
            }
            else {
                isGoodSample = sensor.isRed();
            }
        }
    }

    @Override
    public boolean isFinished() {
        // Finish if a yellow sample is detected or we've checked and need to move
        return sampleDetected && (isGoodSample || extendo.getPos() >= maxExtendoPosition);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setTarget(0);
        extendo.setTarget(0);
        PathChain relocatePath = new PathBuilder()
                .addPath(
                        new BezierLine(new Point(currentPosition), new Point(new Pose(currentPosition.getX() + 3, currentPosition.getY())))
                ).setLinearHeadingInterpolation(currentPosition.getHeading(), currentPosition.getHeading())
                .build();

        // If the sample isn’t yellow, move to the next position
        if (sampleDetected && !isGoodSample && currentPosition.getX() < 80) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new IntakeCommand(intake, 1),
                            new WaitCommand(200), // Give time to settle
                            new IntakeCommand(intake, 0),
                            new PedroCommand(follower, relocatePath), // Move to next sample position
                            new IntakeUntilCommand(intake, extendo, sensor, follower, currentPosition, nextPosition, color) // Try again
                    )
            );
        }
    }
}