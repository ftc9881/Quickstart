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

public class SubGrabCommand extends CommandBase {
    private final PivotSubsystem pivot;
    private final IntakeSubsystem intake;
    private final ExtendoSubsystem extendo;
    private final SensorSubsystem sensor;
    private final Follower follower; // For path following
    private final ArrayList<PathChain> subs;
    private double distanceTraveled;
    private String color; // color of wanted sample
    private final int initialExtendoPosition = 130;
    private final int maxExtendoPosition = 420;
    private double increment = 3.3;
    private final int maxDistance = 10;
    private boolean sampleDetected = false;
    private boolean validSample = false;
    private final Pose currentPosition;
    private final Pose submersiblePosition = new Pose(62, 99, Math.toRadians(-90));
    private ElapsedTime runtime = new ElapsedTime();

    public SubGrabCommand(PivotSubsystem pivot, IntakeSubsystem intake, ExtendoSubsystem extendo,
                          SensorSubsystem colorSensor, Follower follower,
                          ArrayList<PathChain> subs, double distanceTraveled, String color, Pose currentPosition) {
        this.pivot = pivot;
        this.intake = intake;
        this.extendo = extendo;
        this.sensor = colorSensor;
        this.follower = follower;
        this.subs = subs;
        this.distanceTraveled = distanceTraveled;
        this.color = color;
        this.currentPosition = currentPosition;
        addRequirements(pivot, intake, extendo, colorSensor); // No follower requirement (handled by PedroCommand)
    }

    @Override
    public void initialize() {
        intake.setTarget(-1);
        extendo.setTarget(initialExtendoPosition);
        sampleDetected = false;
        validSample = false;
        runtime.reset(); // Reset the timer to 0

        if (increment >= maxDistance) {
            increment = 3.3;
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new PivotCommand(pivot, PIVOT_DOWN_POSITION)
                )
        );
    }


    @Override
    public void execute() {
        if (sensor.isYellow()) {
            sampleDetected = true;
            validSample = true;
        } else if (color.equals("blue") && sensor.isBlue()) {
            sampleDetected = true;
            validSample = true;
        } else if (color.equals("red") && sensor.isRed() ) {
            sampleDetected = true;
            validSample = true;
        }

        if (!sampleDetected && extendo.getPos() < maxExtendoPosition) {
            extendo.setTarget(extendo.getPos() + 20);
        }

    }

    @Override
    public boolean isFinished() {
        return sampleDetected || extendo.getPos() > maxExtendoPosition;
    }

    @Override
    public void end(boolean interrupted) {
        if (validSample) {
            //retract and end sequence
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new PivotCommand(pivot, PIVOT_UP_POSITION),
                            new WaitCommand(50),
                            new ParallelCommandGroup(
                                    new PedroCommand(follower, new PathBuilder()
                                            .addPath(
                                                    // Line 0
                                                    new BezierCurve(
                                                            new Point(currentPosition),
                                                            new Point(51.6, 130, Point.CARTESIAN),
                                                            new Point(submersiblePosition)
                                                    )
                                            )
                                            .setLinearHeadingInterpolation(currentPosition.getHeading(), currentPosition.getHeading())
                                            .build()),
                                    new ExtendoCommand(extendo, 0), // Retract extendo
                                    new IntakeCommand(intake, INTAKE_SLOW_SPEED) // Ensure sample stays in
                            )
                    )
            );

        } else {
            //retract and retry sequence
            sampleDetected = false;
            distanceTraveled += increment;

            if (distanceTraveled >= maxDistance) {
                increment = -maxDistance;
            }

            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new PedroCommand(follower, new PathBuilder()
                                            .addPath(
                                                    // Line 0
                                                    new BezierLine(
                                                            new Point(currentPosition),
                                                            new Point(currentPosition.getX() + increment, currentPosition.getY())
                                                    )
                                            )
                                            .setLinearHeadingInterpolation(currentPosition.getHeading(), currentPosition.getHeading())
                                            .build()),
                                    new PivotCommand(pivot, PIVOT_MID_POSITION),
                                    new ExtendoCommand(extendo, initialExtendoPosition),
                                    new IntakeCommand(intake, 1)
                            ),
                            new ParallelCommandGroup(
                                    new IntakeCommand(intake, -1),
                                    new SequentialCommandGroup(
                                            new PivotCommand(pivot, PIVOT_DOWN_POSITION),
                                            new WaitCommand(100),
                                            new SubGrabCommand(pivot,intake,extendo,sensor,follower,subs,distanceTraveled,color,new Pose(currentPosition.getX() + increment, currentPosition.getY())))
                                    )
                            )
            );



        }
    }

}