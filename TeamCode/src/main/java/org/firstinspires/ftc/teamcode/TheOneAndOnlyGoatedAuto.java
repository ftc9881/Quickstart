package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotTeleopLotus.ARM_DOWN_POSITION;
import static org.firstinspires.ftc.teamcode.RobotTeleopLotus.ARM_UP_POSITION;
import static org.firstinspires.ftc.teamcode.RobotTeleopLotus.CLAW_CLOSED_POSITION;
import static org.firstinspires.ftc.teamcode.RobotTeleopLotus.CLAW_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.RobotTeleopLotus.INTAKE_SLOW_SPEED;
import static org.firstinspires.ftc.teamcode.RobotTeleopLotus.PIVOT_DOWN_POSITION;
import static org.firstinspires.ftc.teamcode.RobotTeleopLotus.PIVOT_MID_POSITION;
import static org.firstinspires.ftc.teamcode.RobotTeleopLotus.PIVOT_UP_POSITION;
import static org.firstinspires.ftc.teamcode.RobotTeleopLotus.SWEEPER_IN_POSITION;
import static org.firstinspires.ftc.teamcode.RobotTeleopLotus.SWEEPER_OUT_POSITION;


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.commands.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendoCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeUntilCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeUntilSevenCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommand;
import org.firstinspires.ftc.teamcode.commands.PedroCommand;
import org.firstinspires.ftc.teamcode.commands.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.SubGrabCommand;
import org.firstinspires.ftc.teamcode.commands.SweeperCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SweeperSubsystem;

import java.util.ArrayList;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;




@Autonomous(name = "TheOneAndOnlyGoatedAuto")
public class TheOneAndOnlyGoatedAuto extends OpMode {
    public static class Index {
        int value;
        Index(int value) {
            this.value = value;
        }
        public void plus() {
            value++;
        }
        public int getValue() {
            return value;
        }
    }

    private String allianceColor = "blue";

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    CommandScheduler adamIsTheGoat;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private boolean pLoopLB = false;
    private boolean pLoopRB = false;
    private int xOffset = 0;

    public ArmSubsystem arm;
    public ClawSubsystem claw ;
    public ExtendoSubsystem extendo;
    public IntakeSubsystem intake;
    public LiftSubsystem lift;
    public PivotSubsystem pivot;
    public SensorSubsystem sensor;
    public SweeperSubsystem sweeper;

    /** width: 14  //  height: 13
    /** Start Position */
    private final Pose startingPosition = new Pose(6.75, 112, Math.toRadians(270));
//    private final Pose startingPosition = new Pose(61, 94.5, Math.toRadians(-90));

    private final Pose dropPreloadPosition = new Pose(10.5, 128, Math.toRadians(315));

    /** Scoring Position (The Buckets) */
    private final Pose bucketPosition = new Pose(13, 131.2, Math.toRadians(315));

    /** First Yellow Sample */
    private final Pose sampleOnePosition = new Pose(14, 130.5, Math.toRadians(-14));

    private final Pose sampleTwoPosition = new Pose(14, 131, Math.toRadians(6));

    /** Third Yellow Sample */
    private final Pose sampleThreePosition = new Pose(14, 130.5, Math.toRadians(25));

    /** Parking Position */
    private final Pose subBucketPosition = new Pose(13.6, 132, Math.toRadians(315));

    /** Sub position */
    private final Pose submersiblePosition = new Pose(62, 99, Math.toRadians(-90));

    private final Pose startingAcquisitionPosition = new Pose(58, 99, Math.toRadians(-90));

    /** These are our Paths and PathChains that we will define in buildPaths() */
//    private Path scorePreload, getSampleOne;
//    private Path scoreSampleOne, getSampleTwo;
//    private Path scoreSampleTwo, getSampleThree;
//    private Path scoreSampleThree, getTeammateSample;
//    private Path scoreTeammateSample, park;

//    public PathChain preload, pick1, drop1, pick2, drop2, pick3, drop3, pick4, drop4, park;

    private final ArrayList<PathChain> paths = new ArrayList<>();
    private ArrayList<PathChain> subs = new ArrayList<>();
    private ArrayList<PathChain> subscores = new ArrayList<>();

    public void buildPaths() {

        paths.add(new PathBuilder()
                .addPath(
                        // Line 0
                        new BezierLine(
                                new Point(startingPosition), new Point(dropPreloadPosition)
                        )
                ).setLinearHeadingInterpolation(startingPosition.getHeading(), dropPreloadPosition.getHeading())
                .build());

        paths.add(new PathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(dropPreloadPosition), new Point(sampleOnePosition)
                        )
                )
                .setLinearHeadingInterpolation(dropPreloadPosition.getHeading(), sampleOnePosition.getHeading())
                .build());

        paths.add(new PathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(sampleOnePosition), new Point(bucketPosition)
                        )
                )
                .setLinearHeadingInterpolation(sampleOnePosition.getHeading(), bucketPosition.getHeading())
                .build());

        paths.add(new PathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(bucketPosition), new Point(sampleTwoPosition)
                        )
                )
                .setLinearHeadingInterpolation(bucketPosition.getHeading(), sampleTwoPosition.getHeading())
                .build());

        paths.add(new PathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(sampleTwoPosition), new Point(bucketPosition)
                        )
                )
                .setLinearHeadingInterpolation(sampleTwoPosition.getHeading(), bucketPosition.getHeading())
                .build());

        paths.add(new PathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(bucketPosition), new Point(sampleThreePosition)
                        )
                )
                .setLinearHeadingInterpolation(bucketPosition.getHeading(), sampleThreePosition.getHeading())
                .build());

        paths.add(new PathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(sampleThreePosition), new Point(subBucketPosition)
                        )
                )
                .setLinearHeadingInterpolation(sampleThreePosition.getHeading(), subBucketPosition.getHeading())
                .build());



        paths.add(new PathBuilder()
                .addPath(
                        // Line 7
                        new BezierCurve(
                                new Point(subBucketPosition),
                                new Point(56, 133, Point.CARTESIAN),
                                new Point(submersiblePosition)
                        )
                )
                .setLinearHeadingInterpolation(subBucketPosition.getHeading(), submersiblePosition.getHeading())
                .build());

        paths.add(new PathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(submersiblePosition),
                                new Point(51.6, 130, Point.CARTESIAN),
                                new Point(subBucketPosition)
                        )
                )
                .setLinearHeadingInterpolation(submersiblePosition.getHeading(), subBucketPosition.getHeading())
                .build());

        paths.add(new PathBuilder()
                .addPath(
                        // line 9, strafe to preinput
                        new BezierLine(
                                new Point(submersiblePosition),
                                new Point(startingAcquisitionPosition)
                        )
                )
                .setLinearHeadingInterpolation(submersiblePosition.getHeading(), startingAcquisitionPosition.getHeading())
                .build());

        paths.add(new PathBuilder()
                .addPath(
                        // line 10, bucket to acquisition
                        new BezierLine(
                                new Point(subBucketPosition),
                                new Point(startingAcquisitionPosition)
                        )
                )
                .setLinearHeadingInterpolation(subBucketPosition.getHeading(), startingAcquisitionPosition.getHeading())
                .build());

        paths.add(new PathBuilder()
                .addPath(
                        // line 11, acquisition to bucket
                        new BezierCurve(
                                new Point(startingAcquisitionPosition),
                                new Point(53, 131, Point.CARTESIAN),
                                new Point(subBucketPosition)
                        )
                )
                .setLinearHeadingInterpolation(startingAcquisitionPosition.getHeading(), subBucketPosition.getHeading())
                .build());

// END NORMAL PATHS
/*
        subs.add(new PathBuilder()
                .addPath(
                        // Line 0
                        new BezierCurve(
                                new Point(submersiblePosition),
                                new Point(new Pose(submersiblePosition.getX() + 2.5, submersiblePosition.getY()))
                        )
                )
                .setLinearHeadingInterpolation(submersiblePosition.getHeading(), submersiblePosition.getHeading())
                .build());

        subs.add(new PathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(new Pose(submersiblePosition.getX() + 2.5, submersiblePosition.getY())),
                                new Point(new Pose(submersiblePosition.getX() + 5, submersiblePosition.getY()))
                        )
                )
                .setLinearHeadingInterpolation(submersiblePosition.getHeading(), submersiblePosition.getHeading())
                .build());

        subs.add(new PathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(new Pose(submersiblePosition.getX() + 5, submersiblePosition.getY())),
                                new Point(new Pose(submersiblePosition.getX() + 7.5, submersiblePosition.getY()))
                        )
                )
                .setLinearHeadingInterpolation(submersiblePosition.getHeading(), submersiblePosition.getHeading())
                .build());

        subs.add(new PathBuilder()
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(new Pose(submersiblePosition.getX() + 7.5, submersiblePosition.getY())),
                                new Point(new Pose(submersiblePosition.getX() + 10, submersiblePosition.getY()))
                        )
                )
                .setLinearHeadingInterpolation(submersiblePosition.getHeading(), submersiblePosition.getHeading())
                .build());

        subs.add(new PathBuilder()
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(new Pose(submersiblePosition.getX() + 10, submersiblePosition.getY())),
                                new Point(new Pose(submersiblePosition.getX() + 12.5, submersiblePosition.getY()))
                        )
                )
                .setLinearHeadingInterpolation(submersiblePosition.getHeading(), submersiblePosition.getHeading())
                .build());


        subscores.add(new PathBuilder()
                .addPath(
                        // Line 0
                        new BezierCurve(
                                new Point(submersiblePosition),
                                new Point(60.75, 120.11538461538461, Point.CARTESIAN),
                                new Point(30.115384615384617, 115.61538461538461, Point.CARTESIAN),
                                new Point(bucketPosition)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build());

        subscores.add(new PathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(new Pose(submersiblePosition.getX() + 2.5, submersiblePosition.getY())),
                                new Point(60.75, 120.11538461538461, Point.CARTESIAN),
                                new Point(30.115384615384617, 115.61538461538461, Point.CARTESIAN),
                                new Point(bucketPosition)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build());

        subscores.add(new PathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(new Pose(submersiblePosition.getX() + 5, submersiblePosition.getY())),
                                new Point(60.75, 120.11538461538461, Point.CARTESIAN),
                                new Point(30.115384615384617, 115.61538461538461, Point.CARTESIAN),
                                new Point(bucketPosition)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build());

        subscores.add(new PathBuilder()
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(new Pose(submersiblePosition.getX() + 7.5, submersiblePosition.getY())),
                                new Point(60.75, 120.11538461538461, Point.CARTESIAN),
                                new Point(30.115384615384617, 115.61538461538461, Point.CARTESIAN),
                                new Point(bucketPosition)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build());

        subscores.add(new PathBuilder()
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(new Pose(submersiblePosition.getX() + 10, submersiblePosition.getY())),
                                new Point(60.75, 120.11538461538461, Point.CARTESIAN),
                                new Point(30.115384615384617, 115.61538461538461, Point.CARTESIAN),
                                new Point(bucketPosition)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build());
*/

        //obsolete default paths

        // Path chain version
//        PathChain path = follower.pathBuilder()
//
//                .addPath(new BezierLine(new Point(startingPosition), new Point(bucketPosition)))
//                .setLinearHeadingInterpolation(startingPosition.getHeading(), bucketPosition.getHeading())
//
//                .addPath(new BezierLine(new Point(bucketPosition), new Point(sampleOnePosition)))
//                .setLinearHeadingInterpolation(bucketPosition.getHeading(), sampleOnePosition.getHeading())
//
//                .addPath(new BezierLine(new Point(sampleOnePosition), new Point(bucketPosition)))
//                .setLinearHeadingInterpolation(sampleOnePosition.getHeading(), bucketPosition.getHeading())
//
//                .addPath(new BezierLine(new Point(bucketPosition), new Point(sampleTwoPosition)))
//                .setLinearHeadingInterpolation(bucketPosition.getHeading(), sampleTwoPosition.getHeading())
//
//                .addPath(new BezierLine(new Point(sampleTwoPosition), new Point(bucketPosition)))
//                .setLinearHeadingInterpolation(sampleTwoPosition.getHeading(), bucketPosition.getHeading())
//
//                .addPath(new BezierLine(new Point(bucketPosition), new Point(sampleThreePosition)))
//                .setLinearHeadingInterpolation(bucketPosition.getHeading(), sampleThreePosition.getHeading())
//
//                .addPath(new BezierLine(new Point(sampleThreePosition), new Point(bucketPosition)))
//                .setLinearHeadingInterpolation(sampleThreePosition.getHeading(), bucketPosition.getHeading())
//
//                .addPath(new BezierLine(new Point(bucketPosition), new Point(teammateSamplePosition)))
//                .setLinearHeadingInterpolation(bucketPosition.getHeading(), teammateSamplePosition.getHeading())
//
//                .addPath(new BezierLine(new Point(teammateSamplePosition), new Point(bucketPosition)))
//                .setLinearHeadingInterpolation(teammateSamplePosition.getHeading(), bucketPosition.getHeading())
//
//                .addPath(new BezierLine(new Point(bucketPosition), new Point(parkingPosition)))
//                .setLinearHeadingInterpolation(bucketPosition.getHeading(), parkingPosition.getHeading())
//
//                .build();
        /*
        scorePreload = new Path(new BezierLine(new Point(startingPosition), new Point(bucketPosition)));
        scorePreload.setLinearHeadingInterpolation(startingPosition.getHeading(), bucketPosition.getHeading());



        getSampleOne = new Path(new BezierLine(new Point(bucketPosition), new Point(sampleOnePosition)));
        getSampleOne.setLinearHeadingInterpolation(bucketPosition.getHeading(), sampleOnePosition.getHeading());

        scoreSampleOne = new Path(new BezierLine(new Point(sampleOnePosition), new Point(bucketPosition)));
        scoreSampleOne.setLinearHeadingInterpolation(sampleOnePosition.getHeading(), bucketPosition.getHeading());


        getSampleTwo = new Path(new BezierLine(new Point(bucketPosition), new Point(sampleTwoPosition)));
        getSampleTwo.setLinearHeadingInterpolation(bucketPosition.getHeading(), sampleTwoPosition.getHeading());

        scoreSampleTwo = new Path(new BezierLine(new Point(sampleTwoPosition), new Point(bucketPosition)));
        scoreSampleTwo.setLinearHeadingInterpolation(sampleTwoPosition.getHeading(), bucketPosition.getHeading());


        getSampleThree = new Path(new BezierLine(new Point(bucketPosition), new Point(sampleThreePosition)));
        getSampleThree.setLinearHeadingInterpolation(bucketPosition.getHeading(), sampleThreePosition.getHeading());

        scoreSampleThree = new Path(new BezierLine(new Point(sampleThreePosition), new Point(bucketPosition)));
        scoreSampleThree.setLinearHeadingInterpolation(sampleThreePosition.getHeading(), bucketPosition.getHeading());


        getTeammateSample = new Path(new BezierLine(new Point(bucketPosition), new Point(teammateSamplePosition)));
        getTeammateSample.setLinearHeadingInterpolation(bucketPosition.getHeading(), teammateSamplePosition.getHeading());

        scoreTeammateSample = new Path(new BezierLine(new Point(teammateSamplePosition), new Point(bucketPosition)));
        scoreTeammateSample.setLinearHeadingInterpolation(teammateSamplePosition.getHeading(), bucketPosition.getHeading());


        park = new Path(new BezierCurve(new Point(bucketPosition), new Point(parkingControlPoint), new Point(parkingPosition)));
        park.setLinearHeadingInterpolation(bucketPosition.getHeading(), parkingPosition.getHeading());
        */

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */



    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        adamIsTheGoat.run();

        // These loop the movements of the robot
        follower.update();


        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("blue", sensor.isBlue());
        telemetry.addData("red", sensor.isRed());
        telemetry.addData("yellow", sensor.isYellow());

        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startingPosition);

        arm = new ArmSubsystem(hardwareMap);
        claw = new ClawSubsystem(hardwareMap);
        extendo = new ExtendoSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        lift = new LiftSubsystem(hardwareMap);
        pivot = new PivotSubsystem(hardwareMap);
        sensor = new SensorSubsystem(hardwareMap);
        sweeper = new SweeperSubsystem(hardwareMap);

        // COACH_DAVE: If the B button is pressed while init is performed then
        //  allianceColor is "red" otherwise "blue"

        double totalDistanceTraveled = 0;
        buildPaths();

        follower.setMaxPower(1.0);

        adamIsTheGoat = CommandScheduler.getInstance();

        adamIsTheGoat.schedule(
                new SequentialCommandGroup(
                        //Initial Conditions
                        new ClawCommand(claw, CLAW_CLOSED_POSITION),
                        new SweeperCommand(sweeper, SWEEPER_OUT_POSITION),
                        new ArmCommand(arm, ARM_UP_POSITION),



//                        new ClawCommand(claw, CLAW_OPEN_POSITION),
//                        new ExtendoCommand(extendo, 80),
//                        new PivotCommand(pivot, PIVOT_DOWN_POSITION),
//                        new IntakeUntilCommand(pivot, intake, extendo, sensor, follower, subs, 0, "yellow"),
//                        new ExtendoCommand(extendo, 0),
//                        new ClawCommand(claw, CLAW_CLOSED_POSITION),

                        //Preload raise lift
                        new ParallelCommandGroup(
                                new PedroCommand(follower, paths.get(0)),
                                new LiftCommand(lift, 870),
                                new SequentialCommandGroup(
                                        new WaitCommand(800),
                                        //preextend for sample 1
                                        new InstantCommand(() -> extendo.setTarget(380)),
                                        new ParallelCommandGroup(
                                                new ArmCommand(arm, ARM_UP_POSITION)
                                        )

                                )
                        ),

                        //preload drop sample
                        new ClawCommand(claw, CLAW_OPEN_POSITION),
                        new WaitCommand(50),

                        //sample 0 retract
                        new ParallelCommandGroup(
                                new ArmCommand(arm, ARM_DOWN_POSITION),
                                new LiftCommand(lift, 0),
                                new PivotCommand(pivot, PIVOT_DOWN_POSITION),
                                new IntakeCommand(intake, -1),
                                new PedroCommand(follower, paths.get(1))
                        ),

                        //sample 1 extend
//                        new PivotCommand(pivot, PIVOT_DOWN_POSITION),
                        new WaitCommand(50),


                        //sample 1 retract extendo
                        new ParallelCommandGroup(
                                new PivotCommand(pivot, PIVOT_UP_POSITION),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new IntakeCommand(intake, INTAKE_SLOW_SPEED)
                                        ),
                                new PedroCommand(follower, paths.get(2)),
                                new SequentialCommandGroup(
                                        new ExtendoCommand(extendo, 0),
                                        new WaitCommand(200),
                                        new ClawCommand(claw, CLAW_CLOSED_POSITION),
                                        new WaitCommand(75),
                                        //sample 1 lift extend
                                        new ParallelCommandGroup(
                                                new LiftCommand(lift, 840),

                                                new SequentialCommandGroup(
                                                        new WaitCommand(250),
                                                        //sample 2 extend
                                                        new InstantCommand(() -> extendo.setTarget(358)),
                                                        new ParallelCommandGroup(
                                                                new ArmCommand(arm, ARM_UP_POSITION)

                                                        )
                                                )
                                        )
                                )


                        ),

                        //sample 1 score
                        new WaitCommand(150),
                        new ClawCommand(claw, CLAW_OPEN_POSITION),
                        new WaitCommand(50),

                        //sample 1 retract
                        new ParallelCommandGroup(
                                new ArmCommand(arm, ARM_DOWN_POSITION),
                                new LiftCommand(lift, 0),
                                new PivotCommand(pivot, PIVOT_DOWN_POSITION),
                                new IntakeCommand(intake, -1),
                                new PedroCommand(follower, paths.get(3))

                        ),

//                        new PivotCommand(pivot, PIVOT_DOWN_POSITION),
                        new WaitCommand(50),




                        //sample 2 retract extendo
                        new ParallelCommandGroup(
                                new PivotCommand(pivot, PIVOT_UP_POSITION),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new IntakeCommand(intake, INTAKE_SLOW_SPEED)
                                ),
                                new PedroCommand(follower, paths.get(4)),
                                new SequentialCommandGroup(
                                        new ExtendoCommand(extendo, 0),
                                        new WaitCommand(200),
                                        new ClawCommand(claw, CLAW_CLOSED_POSITION),
                                        new WaitCommand(70),
                                        //sample 2 lift extend
                                        new ParallelCommandGroup(
                                                new LiftCommand(lift, 840),

                                                new SequentialCommandGroup(
                                                        new WaitCommand(250),
                                                        //sample 3 preextend
                                                        new InstantCommand(() -> extendo.setTarget(410)),
                                                        new ParallelCommandGroup(
                                                                new ArmCommand(arm, ARM_UP_POSITION)

                                                        )
                                                )
                                        )
                                )


                        ),

                        //sample 2 score
                        new WaitCommand(150),
                        new ClawCommand(claw, CLAW_OPEN_POSITION),
                        new WaitCommand(50),

                        //sample 2-3 retract and pre-extend
                        new ParallelCommandGroup(
                                new ArmCommand(arm, ARM_DOWN_POSITION),
                                new LiftCommand(lift, 0),
                                new PivotCommand(pivot, PIVOT_DOWN_POSITION),
                                new IntakeCommand(intake, -1),
                                new PedroCommand(follower, paths.get(5))
                        ),

                        //sample 3 grab

//                        new PivotCommand(pivot, PIVOT_DOWN_POSITION),
                        new WaitCommand(200),

                        //sample 3 retract extendo
                        new ParallelCommandGroup(
                                new PivotCommand(pivot, PIVOT_UP_POSITION),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new IntakeCommand(intake, INTAKE_SLOW_SPEED)
                                ),
                                new PedroCommand(follower, paths.get(6)),
                                new SequentialCommandGroup(
                                        new ExtendoCommand(extendo, 0),
                                        new WaitCommand(200),
                                        new ClawCommand(claw, CLAW_CLOSED_POSITION),
                                        new WaitCommand(70),
                                        //sample 3 lift extend
                                        new ParallelCommandGroup(
                                                new LiftCommand(lift, 840),

                                                new SequentialCommandGroup(
                                                        new WaitCommand(250),
                                                        new ParallelCommandGroup(
                                                                new ArmCommand(arm, ARM_UP_POSITION)

                                                        )

                                                )
                                        )
                                )


                        ),


                        //sample 3 score
                        new WaitCommand(150),
                        new ClawCommand(claw, CLAW_OPEN_POSITION),
                        new WaitCommand(50),



                        //SUB CYCLE 1
                        new ParallelCommandGroup(
                                new ArmCommand(arm, ARM_DOWN_POSITION),
                                new LiftCommand(lift, 0),
                                new PivotCommand(pivot, PIVOT_MID_POSITION),
                                new PedroCommand(follower, paths.get(7)),
                                new InstantCommand(() -> extendo.setTarget(130)),
                                new SweeperCommand(sweeper, 0.53) // artifically shorter for sussy auto
                        ),


                        new SweeperCommand(sweeper, SWEEPER_OUT_POSITION),
                        new IntakeCommand(intake, -1),
                        new WaitCommand(50),
                        new SubGrabCommand(pivot, intake, extendo, sensor, follower, subs, totalDistanceTraveled, allianceColor, submersiblePosition),

                        //retraction -> lift and drive, sub cycle 1
                        new ParallelCommandGroup(
                                new PivotCommand(pivot, PIVOT_UP_POSITION),
                                new PedroCommand(follower, paths.get(8)),
                                //retraction intake sequence
                                new SequentialCommandGroup(
                                        new ExtendoCommand(extendo,0),
                                        new WaitCommand(100),
                                        new ClawCommand(claw, CLAW_CLOSED_POSITION),
                                        new WaitCommand(70),
                                        new ParallelCommandGroup(
                                                new LiftCommand(lift, 850),
                                                new ArmCommand(arm, ARM_UP_POSITION)
                                        )

                                )


                        ),

                        new ClawCommand(claw, CLAW_OPEN_POSITION),
                        new WaitCommand(70),


                        //SUB CYCLE 2, NO SWEEP
                        new ParallelCommandGroup(
                                new ArmCommand(arm, ARM_DOWN_POSITION),
                                new LiftCommand(lift, 0),
                                new PivotCommand(pivot, PIVOT_MID_POSITION),
                                new PedroCommand(follower, paths.get(7)),
                                new IntakeCommand(intake, -1),
                                new ExtendoCommand(extendo, 130)

                        ),


                        new SubGrabCommand(pivot, intake, extendo, sensor, follower, subs, totalDistanceTraveled, allianceColor, submersiblePosition),
                        
                        //retraction -> lift and drive, sub cycle 2
                        new ParallelCommandGroup(
                                new PivotCommand(pivot, PIVOT_UP_POSITION),
                                new PedroCommand(follower, paths.get(8)),
                                //retraction intake sequence
                                new SequentialCommandGroup(
                                        new WaitCommand(100),
                                        new ClawCommand(claw, CLAW_CLOSED_POSITION),
                                        new WaitCommand(70),
                                        new ParallelCommandGroup(
                                                new LiftCommand(lift, 850),
                                                new ArmCommand(arm, ARM_UP_POSITION)
                                        )

                                )


                        ),

                        new ClawCommand(claw, CLAW_OPEN_POSITION),
                        new WaitCommand(70),


                        //SUB CYCLE 3, NO SWEEP
                        new ParallelCommandGroup(
                                new ArmCommand(arm, ARM_DOWN_POSITION),
                                new LiftCommand(lift, 0),
                                new PivotCommand(pivot, PIVOT_MID_POSITION),
                                new PedroCommand(follower, paths.get(10)),
                                new IntakeCommand(intake, -1),
                                new ExtendoCommand(extendo, 130)

                        ),

                        new SubGrabCommand(pivot, intake, extendo, sensor, follower, subs, totalDistanceTraveled, allianceColor, startingAcquisitionPosition),


                        //retraction -> lift and drive, sub cycle 3
                        new ParallelCommandGroup(
                                new PivotCommand(pivot, PIVOT_UP_POSITION),
                                new PedroCommand(follower, paths.get(11)),
                                //retraction intake sequence
                                new SequentialCommandGroup(
                                        new WaitCommand(100),
                                        new ClawCommand(claw, CLAW_CLOSED_POSITION),
                                        new WaitCommand(70),
                                        new ParallelCommandGroup(
                                                new LiftCommand(lift, 850),
                                                new ArmCommand(arm, ARM_UP_POSITION)
                                        )

                                )


                        ),

                        new ClawCommand(claw, CLAW_OPEN_POSITION),
                        new WaitCommand(70),

                        //park
                        new ParallelCommandGroup(
                                new ArmCommand(arm, ARM_DOWN_POSITION),
                                new LiftCommand(lift, 0),
                                new PedroCommand(follower, paths.get(7))

                        )








                        )
        );


    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        //color selection
        if (gamepad1.a) {
            allianceColor = "red";
        } else if (gamepad1.b) {
            allianceColor = "blue";
        }

        //preinput shenanigans

        if (gamepad1.left_bumper && !pLoopLB) {
            xOffset += 1;
            pLoopLB = true;
        }
        if (!gamepad1.left_bumper) {
            pLoopLB = false;
        }

        if (gamepad1.right_bumper && !pLoopRB) {
            xOffset -= 1;
            pLoopRB = true;
        }
        if (!gamepad1.right_bumper) {
            pLoopRB = false;
        }




        // Feedback to Driver Hub
        telemetry.addData("allianceColor", allianceColor);
        telemetry.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
    }
}