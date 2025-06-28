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

import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.RobotTeleopLotus.INTAKE_SLOW_SPEED;
import static org.firstinspires.ftc.teamcode.RobotTeleopLotus.PIVOT_DOWN_POSITION;
import static org.firstinspires.ftc.teamcode.RobotTeleopLotus.PIVOT_MID_POSITION;
import static org.firstinspires.ftc.teamcode.RobotTeleopLotus.PIVOT_UP_POSITION;
import org.firstinspires.ftc.teamcode.SevenSampGoated.*;

import java.util.ArrayList;

public class IntakeUntilSevenCommand extends CommandBase {
    private final PivotSubsystem pivot;
    private final IntakeSubsystem intake;
    private final ExtendoSubsystem extendo;
    private final SensorSubsystem sensor;
    private final Follower follower; // For path following
    private final ArrayList<PathChain> subs;
    private Index index;
    private String color; // color of wanted sample
    private final int initialExtendoPosition = 130;
    private final int maxExtendoPosition = 360;
    private boolean sampleDetected = false;
    private boolean isGoodSample = false;
    private ElapsedTime runtime = new ElapsedTime();

    public IntakeUntilSevenCommand(PivotSubsystem pivot, IntakeSubsystem intake, ExtendoSubsystem extendo,
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
        runtime.reset(); // Reset the timer to 0

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new PivotCommand(pivot, PIVOT_DOWN_POSITION)
                )
        );
    }

    @Override
    public void execute() {
        if (sensor.isBlue() || sensor.isRed() || sensor.isYellow()) { // Sample is in
            sampleDetected = true;

            if (color.equals("blue")) {
                isGoodSample = (sensor.isYellow() || sensor.isBlue());
            }
            else {
                isGoodSample = (sensor.isYellow() || sensor.isRed());
            }
        }
        // Extend until a sample is close or max position is reached
        if (!sampleDetected && extendo.getPos() < maxExtendoPosition) {
            extendo.setTarget(extendo.getPos() + 35);
        }

        // Check proximity to detect a sample

    }

    @Override
    public boolean isFinished() {
        // Finish if a yellow sample is detected or we've checked and need to move
        return sampleDetected || extendo.getPos() > maxExtendoPosition;
    }

    @Override
    public void end(boolean interrupted) {

        if (isGoodSample) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new PivotCommand(pivot, PIVOT_UP_POSITION),
                            new ParallelCommandGroup(
                                    new ExtendoCommand(extendo, 130), // Retract extendo
                                    new IntakeCommand(intake, INTAKE_SLOW_SPEED) // Ensure intake is stopped
                            )
                    )
            );
        } else if (index.getValue() == 4) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new PivotCommand(pivot, PIVOT_UP_POSITION),
                            new ParallelCommandGroup(
                                    new ExtendoCommand(extendo, 130), // Retract extendo
                                    new IntakeCommand(intake, INTAKE_SLOW_SPEED) // Ensure intake is stopped
                            )
                    )
            );
        }else {
            sampleDetected = false;
            index.plus();
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new PivotCommand(pivot, PIVOT_MID_POSITION),
                                    new ExtendoCommand(extendo, 130),
                                    new IntakeCommand(intake, 1)
                            ),
                            new WaitCommand(150), // Give time to settle
                            new PedroCommand(follower, subs.get(index.getValue() - 1)), // Move to next sample position
                            new IntakeUntilSevenCommand(pivot, intake, extendo, sensor, follower, subs, index, color).withTimeout(2000) // Try again
                    )
            );
        }
    }
}