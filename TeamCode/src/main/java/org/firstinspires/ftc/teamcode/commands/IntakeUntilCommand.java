package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import static org.firstinspires.ftc.teamcode.RobotTeleopLotus.PIVOT_DOWN_POSITION;
import static org.firstinspires.ftc.teamcode.RobotTeleopLotus.PIVOT_MID_POSITION;
import static org.firstinspires.ftc.teamcode.RobotTeleopLotus.PIVOT_UP_POSITION;
import org.firstinspires.ftc.teamcode.TheOneAndOnlyGoatedAuto.*;

import java.util.ArrayList;

public class IntakeUntilCommand extends CommandBase {
    private final PivotSubsystem pivot;
    private final IntakeSubsystem intake;
    private final ExtendoSubsystem extendo;
    private final SensorSubsystem sensor;
    private final Follower follower; // For path following
    private final ArrayList<PathChain> subs;
    private Index index;
    private String color; // color of wanted sample
    private final int initialExtendoPosition = 80;
    private final int maxExtendoPosition = 250;
    private boolean sampleDetected = false;
    private boolean isGoodSample = false;

    public IntakeUntilCommand(PivotSubsystem pivot, IntakeSubsystem intake, ExtendoSubsystem extendo,
                              SensorSubsystem colorSensor, Follower follower,
                              ArrayList<PathChain> subs, Index index, String color) {
        this.pivot = pivot;
        this.intake = intake;
        this.extendo = extendo;
        this.sensor = colorSensor;
        this.follower = follower;
        this.subs = subs;
        this.index = index;
        this.color = color;
        addRequirements(pivot, intake, extendo, colorSensor); // No follower requirement (handled by PedroCommand)
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
            extendo.setTarget(extendo.getPos() + 100);
        }

        // Check proximity to detect a sample

        if (sensor.isBlue() || sensor.isRed() || sensor.isYellow()) { // Sample is in
            sampleDetected = true;

            if (color.equals("blue")) {
                isGoodSample = sensor.isYellow() || sensor.isBlue();
            }
            else {
                isGoodSample = sensor.isYellow() || sensor.isRed();
            }
        }
    }

    @Override
    public boolean isFinished() {
        // Finish if a yellow sample is detected or we've checked and need to move
        return sampleDetected || extendo.getPos() >= maxExtendoPosition;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setTarget(0);

        // If the sample isn’t yellow, move to the next position
        if (!isGoodSample && index.getValue() < subs.size()) {
            sampleDetected = false;
            index.plus();
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new ExtendoCommand(extendo, 80),
                                    new IntakeCommand(intake, 1)
                            ),
                            new WaitCommand(20), // Give time to settle
                            new IntakeCommand(intake, 0),
                            new PedroCommand(follower, subs.get(index.getValue() - 1)), // Move to next sample position
                            new IntakeUntilCommand(pivot, intake, extendo, sensor, follower, subs, index, color) // Try again
                    )
            );
        }
        else if (isGoodSample) {
            CommandScheduler.getInstance().schedule(
                    new PivotCommand(pivot, PIVOT_UP_POSITION)
            );
        }
    }
}