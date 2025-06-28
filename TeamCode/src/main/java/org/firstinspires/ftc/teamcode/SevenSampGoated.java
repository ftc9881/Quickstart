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
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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




@Autonomous(name = "SevenSampGoated")
public class SevenSampGoated extends OpMode {
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
    private final Pose startingPosition = new Pose(6.60, 113.5, Math.toRadians(270));
//    private final Pose startingPosition = new Pose(61, 94.5, Math.toRadians(-90));

    private final Pose dropPreloadPosition = new Pose(12.6, 132.0, Math.toRadians(315));

    /** Scoring Position (The Buckets) */
    private final Pose bucketPosition = new Pose(13.2, 132.9, Math.toRadians(315));

    /** First Yellow Sample */
    private final Pose sampleOnePosition = new Pose(16.80, 128.5, Math.toRadians(-3));

    /** Second Yellow Sample */
    private final Pose sampleTwoPosition = new Pose(16.4, 135.6, Math.toRadians(0));

    /** Third Yellow Sample */
    private final Pose sampleThreePosition = new Pose(16.44, 131.4, Math.toRadians(32));

    /** Teammate's Sample */
    private final Pose teammateSamplePosition = new Pose(11.00, 110.0, Math.toRadians(270));

    /** Parking Position */
    private final Pose ascentPosition = new Pose(61, 110, Math.toRadians(-90));

    /** Control Point for Parking */
    private final Pose submersiblePosition = new Pose(61, 98, Math.toRadians(-90));

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
                                new Point(sampleThreePosition), new Point(bucketPosition)
                        )
                )
                .setLinearHeadingInterpolation(sampleThreePosition.getHeading(), bucketPosition.getHeading())
                .build());

        paths.add(new PathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(bucketPosition), new Point(teammateSamplePosition)
                        )
                )
                .setLinearHeadingInterpolation(bucketPosition.getHeading(), teammateSamplePosition.getHeading())
                .build());

        paths.add(new PathBuilder()
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(teammateSamplePosition), new Point(bucketPosition)
                        )
                )
                .setLinearHeadingInterpolation(teammateSamplePosition.getHeading(), bucketPosition.getHeading())
                .build());

        paths.add(new PathBuilder()
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(bucketPosition),
                                new Point(25.720534629404618, 121.42891859052247, Point.CARTESIAN),
                                new Point(62.1142162818955, 134, Point.CARTESIAN),
                                new Point(submersiblePosition)
                        )
                )
                .setTangentHeadingInterpolation()
                .build());

        paths.add(new PathBuilder()
                .addPath(
                        // Line 10
                        new BezierCurve(
                                new Point(submersiblePosition),
                                new Point(62.1142162818955, 134, Point.CARTESIAN),
                                new Point(25.720534629404618, 121.42891859052247, Point.CARTESIAN),
                                new Point(bucketPosition)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build());
        paths.add(new PathBuilder()
                .addPath(
                        // Line 11
                        new BezierLine(
                                new Point(bucketPosition), new Point(teammateSamplePosition)
                        )
                )
                .setLinearHeadingInterpolation(bucketPosition.getHeading(), teammateSamplePosition.getHeading())
                .build());

        paths.add(new PathBuilder()
                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(teammateSamplePosition), new Point(bucketPosition)
                        )
                )
                .setLinearHeadingInterpolation(teammateSamplePosition.getHeading(), bucketPosition.getHeading())
                .build());

/*
        paths.add(new PathBuilder()
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(bucketPosition), new Point(52.5, 132, Point.CARTESIAN), new Point(ascentPosition)
                        )
                )
                .setLinearHeadingInterpolation(bucketPosition.getHeading(), ascentPosition.getHeading())
                .build());

        paths.add(new PathBuilder()
                .addPath(
                        // Line 10
                        new BezierCurve(
                                new Point(ascentPosition), new Point(submersibleRamPosition)
                        )
                )
                .setLinearHeadingInterpolation(ascentPosition.getHeading(), submersibleRamPosition.getHeading())
                .build());

        paths.add(new PathBuilder()
                .addPath(
                        // Line 11
                        new BezierCurve(
                                new Point(submersibleRamPosition), new Point(ascentPosition)
                        )
                )
                .setLinearHeadingInterpolation(submersibleRamPosition.getHeading(), ascentPosition.getHeading())
                .build());

        paths.add(new PathBuilder()
                .addPath(
                        // Line 12
                        new BezierCurve(
                                new Point(ascentPosition), new Point(52.5, 132, Point.CARTESIAN), new Point(bucketPosition)
                        )
                )
                .setLinearHeadingInterpolation(ascentPosition.getHeading(), bucketPosition.getHeading())
                .build());
        */


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

        allianceColor = gamepad1.b ? "red" : "blue";

        Index currentSubIndex = new Index(0);
        buildPaths();

        follower.setMaxPower(.75);

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
                                new LiftCommand(lift, 850),
                                new SequentialCommandGroup(
                                        new WaitCommand(650),
                                        new ParallelCommandGroup(
                                                new ArmCommand(arm, ARM_UP_POSITION),
                                                new PivotCommand(pivot, PIVOT_MID_POSITION)
                                        )
                                )
                        ),

                        //preload drop sample
                        new WaitCommand(200),
                        new ClawCommand(claw, CLAW_OPEN_POSITION),
                        new WaitCommand(20),

                        //sample 0-1 retract + pre-extend
                        new ParallelCommandGroup(
                                new ArmCommand(arm, ARM_DOWN_POSITION),
                                new LiftCommand(lift, 0),
                                new PivotCommand(pivot, PIVOT_DOWN_POSITION),
                                new IntakeCommand(intake, -1),
                                new SequentialCommandGroup(
                                        new PedroCommand(follower, paths.get(1)),
                                        new ExtendoCommand(extendo, 400)
                                )
                        ),

                        new WaitCommand(500),
                        //sample 1 retract extendo
                        new ParallelCommandGroup(
                                new PivotCommand(pivot, PIVOT_UP_POSITION),
                                new IntakeCommand(intake, INTAKE_SLOW_SPEED),
                                new ExtendoCommand(extendo, 0)
                        ),

                        new WaitCommand(250),
                        new ClawCommand(claw, CLAW_CLOSED_POSITION),
                        new WaitCommand(50),

                        //sample 1 raise lift
                        new ParallelCommandGroup(
                                new PedroCommand(follower, paths.get(2)),
                                new LiftCommand(lift, 850),
                                new SequentialCommandGroup(
                                        new WaitCommand(450),
                                        new ParallelCommandGroup(
                                                new PivotCommand(pivot, PIVOT_MID_POSITION),
                                                new ArmCommand(arm, ARM_UP_POSITION)
                                        )
                                )
                        ),

                        //sample 1 score
                        new WaitCommand(350),
                        new ClawCommand(claw, CLAW_OPEN_POSITION),
                        new WaitCommand(50),

                        //sample 1-2 retract + pre-extend
                        new ParallelCommandGroup(
                                new ArmCommand(arm, ARM_DOWN_POSITION),
                                new LiftCommand(lift, 0),
                                new PivotCommand(pivot, PIVOT_DOWN_POSITION),
                                new IntakeCommand(intake, -1),
                                new SequentialCommandGroup(
                                        new PedroCommand(follower, paths.get(3)),
                                        new ExtendoCommand(extendo, 380)
                                )
                        ),

                        new WaitCommand(500),


                        //sample 2 retract extendo
                        new ParallelCommandGroup(
                                new PivotCommand(pivot, PIVOT_UP_POSITION),
                                new IntakeCommand(intake, INTAKE_SLOW_SPEED + .1),
                                new ExtendoCommand(extendo, 0)
                        ),

                        new WaitCommand(600),
                        new ClawCommand(claw, CLAW_CLOSED_POSITION),
                        new WaitCommand(50),

                        //sample 2 extend lift
                        new ParallelCommandGroup(
                                new PedroCommand(follower, paths.get(4)),
                                new LiftCommand(lift, 850),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new ParallelCommandGroup(
                                                new PivotCommand(pivot, PIVOT_MID_POSITION),
                                                new ArmCommand(arm, ARM_UP_POSITION)
                                        )
                                )

                        ),

                        //sample 2 score
                        new WaitCommand(200),
                        new ClawCommand(claw, CLAW_OPEN_POSITION),
                        new WaitCommand(50),

                        //sample 2-3 retract and pre-extend
                        new ParallelCommandGroup(
                                new ArmCommand(arm, ARM_DOWN_POSITION),
                                new LiftCommand(lift, 0),
                                new PivotCommand(pivot, PIVOT_DOWN_POSITION),
                                new SequentialCommandGroup(
                                        new PedroCommand(follower, paths.get(5)),
                                        new ExtendoCommand(extendo, 410)
                                ),
                                new IntakeCommand(intake, -8)

                        ),

                        //sample 3 grab

                        new WaitCommand(500),


                        //sample 3 retract extendo
                        new ParallelCommandGroup(
                                new PivotCommand(pivot, PIVOT_UP_POSITION),
                                new IntakeCommand(intake, INTAKE_SLOW_SPEED),
                                new ExtendoCommand(extendo, 0)
                        ),

                        new WaitCommand(305),
                        new ClawCommand(claw, CLAW_CLOSED_POSITION),
                        new WaitCommand(50),

                        //sample 3 lift extend
                        new ParallelCommandGroup(
                                new PedroCommand(follower, paths.get(6)),
                                new LiftCommand(lift, 850),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new ParallelCommandGroup(
                                                new ArmCommand(arm, ARM_UP_POSITION)
                                        )
                                )
                        ),

                        //sample 3 score
                        new WaitCommand(100),
                        new ClawCommand(claw, CLAW_OPEN_POSITION),
                        new WaitCommand(50),

                        new ParallelCommandGroup(
                                new ArmCommand(arm, ARM_DOWN_POSITION),
                                new LiftCommand(lift, 0),
                                new PedroCommand(follower, paths.get(9), false),
                                new ExtendoCommand(extendo, 130)
                        ),

                        //sussy 6 samp
                        new SweeperCommand(sweeper, SWEEPER_IN_POSITION),
                        new PivotCommand(pivot, PIVOT_MID_POSITION),
                        new WaitCommand(120),
                        new SweeperCommand(sweeper, SWEEPER_OUT_POSITION),
                        new WaitCommand(50),
                        new PivotCommand(pivot, PIVOT_DOWN_POSITION),

                        new IntakeUntilSevenCommand(pivot, intake, extendo, sensor, follower, subs, currentSubIndex, allianceColor).withTimeout(4000),

                        new PivotCommand(pivot, PIVOT_UP_POSITION),
                        new IntakeCommand(intake, INTAKE_SLOW_SPEED),


                        new ParallelCommandGroup(
                                new PedroCommand(follower, subscores.get(currentSubIndex.getValue())),
                                new SequentialCommandGroup(
                                        new ExtendoCommand(extendo, 0),
                                        new ClawCommand(claw, CLAW_CLOSED_POSITION),
                                        new WaitCommand(100),
                                        new ParallelCommandGroup(
                                                new LiftCommand(lift, 850),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(500),
                                                        new ParallelCommandGroup(
                                                                new ArmCommand(arm, ARM_UP_POSITION),
                                                                new PivotCommand(pivot, PIVOT_MID_POSITION)
                                                        )
                                                )
                                        )


                                )

                        ),

                        new WaitCommand(100),
                        new ClawCommand(claw, CLAW_OPEN_POSITION),
                        new WaitCommand(50),

                        /*new ParallelCommandGroup(
                                new PedroCommand(follower, paths.get(9), false),
                                new LiftCommand(lift, 0),
                                new ArmCommand(arm, ARM_DOWN_POSITION),
                                new IntakeCommand(intake, -1)
                        ),

                        new ExtendoCommand(extendo,150),
                        new PivotCommand(pivot, PIVOT_DOWN_POSITION),
                        new IntakeUntilSevenCommand(pivot, intake, extendo, sensor, follower, subs, currentSubIndex, allianceColor).withTimeout(5000),

                        new IntakeCommand(intake, INTAKE_SLOW_SPEED),
                        new PivotCommand(pivot, PIVOT_UP_POSITION),

                        new ParallelCommandGroup(
                                new PedroCommand(follower, subscores.get(currentSubIndex.getValue()), false),
                                new SequentialCommandGroup(
                                        new ExtendoCommand(extendo, 0),
                                        new ClawCommand(claw, CLAW_CLOSED_POSITION),
                                        new WaitCommand(100),
                                        new ParallelCommandGroup(
                                                new LiftCommand(lift, 850),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(400),
                                                        new ParallelCommandGroup(
                                                                new ArmCommand(arm, ARM_UP_POSITION),
                                                                new PivotCommand(pivot, PIVOT_MID_POSITION)
                                                        )
                                                )
                                        )


                                )

                        ),

                        new WaitCommand(50),
                        new ClawCommand(claw, CLAW_OPEN_POSITION),
                        new WaitCommand(50),
                        */


                        new ParallelCommandGroup(
                                new PedroCommand(follower, paths.get(11)),
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new ParallelCommandGroup(
                                                new ArmCommand(arm, ARM_DOWN_POSITION),
                                                new LiftCommand(lift, 0),
                                                new ExtendoCommand(extendo, 180),
                                                new IntakeCommand(intake, -1),
                                                new PivotCommand(pivot, PIVOT_DOWN_POSITION)
                                        )

                                )

                        ),
                        new ExtendoCommand(extendo, 380),
                        new ParallelCommandGroup(
                                new PivotCommand(pivot, PIVOT_UP_POSITION),
                                new SequentialCommandGroup(
                                        new ExtendoCommand(extendo, 0),
                                        new ClawCommand(claw, CLAW_CLOSED_POSITION),
                                        new WaitCommand(80),
                                        new ParallelCommandGroup(
                                                new LiftCommand(lift,850),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(500),
                                                        new ArmCommand(arm, ARM_UP_POSITION)
                                                )

                                                )

                                        ),
                                new PedroCommand(follower, paths.get(12))

                        ),

                        new ClawCommand(claw, CLAW_OPEN_POSITION),
                        new WaitCommand(100),
                        new ArmCommand(arm, ARM_DOWN_POSITION),
                        new PivotCommand(pivot, PIVOT_MID_POSITION),
                        new LiftCommand(lift, 0)







                )
        );


    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
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