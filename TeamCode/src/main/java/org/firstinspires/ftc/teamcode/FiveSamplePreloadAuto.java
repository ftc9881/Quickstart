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
import org.firstinspires.ftc.teamcode.subsystems.SweeperSubsystem;

import java.util.ArrayList;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "FiveSamplePreloadAuto")
public class FiveSamplePreloadAuto extends OpMode {

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
    public SweeperSubsystem sweeper;

    /** width: 14  //  height: 13
     /** Start Position */
    private final Pose startingPosition = new Pose(6.60, 112.6, Math.toRadians(270));

    /** Scoring Position (The Buckets) */
    private final Pose bucketPosition = new Pose(14, 131.75, Math.toRadians(315));

    /** First Yellow Sample */
    private final Pose sampleOnePosition = new Pose(16.80, 129.0, Math.toRadians(-10));

    /** Second Yellow Sample */
    private final Pose sampleTwoPosition = new Pose(16.4, 132.38, Math.toRadians(0));

    /** Third Yellow Sample */
    private final Pose sampleThreePosition = new Pose(16.44, 130.63, Math.toRadians(30));

    /** Teammate's Sample */
    private final Pose teammateSamplePosition = new Pose(12.00, 96.11, Math.toRadians(270));

    /** Parking Position */
    private final Pose ascentPosition = new Pose(61, 110, Math.toRadians(-90));



    /** These are our Paths and PathChains that we will define in buildPaths() */
//    private Path scorePreload, getSampleOne;
//    private Path scoreSampleOne, getSampleTwo;
//    private Path scoreSampleTwo, getSampleThree;
//    private Path scoreSampleThree, getTeammateSample;
//    private Path scoreTeammateSample, park;

//    public PathChain preload, pick1, drop1, pick2, drop2, pick3, drop3, pick4, drop4, park;

    private final ArrayList<PathChain> paths = new ArrayList<>();

    public void buildPaths() {

        paths.add(new PathBuilder()
                .addPath(
                        // Line 0
                        new BezierLine(
                                new Point(startingPosition), new Point(bucketPosition)
                        )
                ).setLinearHeadingInterpolation(startingPosition.getHeading(), bucketPosition.getHeading())
                .build());

        paths.add(new PathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(bucketPosition), new Point(sampleOnePosition)
                        )
                )
                .setLinearHeadingInterpolation(bucketPosition.getHeading(), sampleOnePosition.getHeading())
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
                                new Point(bucketPosition), new Point(52.5, 132, Point.CARTESIAN), new Point(ascentPosition)
                        )
                )
                .setLinearHeadingInterpolation(bucketPosition.getHeading(), ascentPosition.getHeading())
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
        sweeper = new SweeperSubsystem(hardwareMap);

        buildPaths();

        follower.setMaxPower(.8);


        adamIsTheGoat = CommandScheduler.getInstance();

        adamIsTheGoat.schedule(
                new SequentialCommandGroup(
                        //Initial Conditions
                        new ClawCommand(claw, CLAW_CLOSED_POSITION),
                        new SweeperCommand(sweeper, SWEEPER_OUT_POSITION),
                        new ArmCommand(arm, ARM_UP_POSITION),


                        //Preload raise lift
                        new WaitCommand(50),
                        new ParallelCommandGroup(
                                new PedroCommand(follower, paths.get(0)),
                                new LiftCommand(lift, 860),
                                new ExtendoCommand(extendo, 240),
                                new ArmCommand(arm, ARM_UP_POSITION)

                        ),

                        //preload drop sample
                        new ClawCommand(claw, CLAW_OPEN_POSITION),
                        new WaitCommand(80),

                        //sample 0-1 retract + pre-extend
                        new ParallelCommandGroup(
                                new ArmCommand(arm, ARM_DOWN_POSITION),
                                new LiftCommand(lift, 0),
                                new PivotCommand(pivot, PIVOT_DOWN_POSITION),
                                new PedroCommand(follower, paths.get(1)),
                                new IntakeCommand(intake, -1)


                        ),

                        //sample 1 intaking sequence

                        new ExtendoCommand(extendo, 420),

                        //sample 1 retract extendo
                        new ParallelCommandGroup(
                                new PivotCommand(pivot, PIVOT_UP_POSITION),
                                new IntakeCommand(intake, INTAKE_SLOW_SPEED),
                                new ExtendoCommand(extendo, 0)
                        ),
                        new ClawCommand(claw, CLAW_CLOSED_POSITION),
                        new WaitCommand(100),

                        //sample 1 raise lift
                        new ParallelCommandGroup(
                                new LiftCommand(lift, 860),
                                new ArmCommand(arm, ARM_UP_POSITION),
                                new SequentialCommandGroup(
                                        new PedroCommand(follower, paths.get(2)),
                                        new ExtendoCommand(extendo, 240)
                                )
                        ),

                        //sample 1 score

                        new ClawCommand(claw, CLAW_OPEN_POSITION),
                        new WaitCommand(80),

                        //sample 1-2 retract + pre-extend
                        new ParallelCommandGroup(
                                new ArmCommand(arm, ARM_DOWN_POSITION),
                                new LiftCommand(lift, 0),
                                new PivotCommand(pivot, PIVOT_DOWN_POSITION),
                                new PedroCommand(follower, paths.get(3)),
                                new IntakeCommand(intake, -1)
                        ),

                        //sample 2 grab

                        new ExtendoCommand(extendo, 420),

                        //sample 2 retract extendo
                        new ParallelCommandGroup(
                                new PivotCommand(pivot, PIVOT_UP_POSITION),
                                new IntakeCommand(intake, INTAKE_SLOW_SPEED),
                                new ExtendoCommand(extendo, 0)
                        ),

                        new WaitCommand(50),
                        new ClawCommand(claw, CLAW_CLOSED_POSITION),
                        new WaitCommand(50),

                        //sample 2 extend lift
                        new ParallelCommandGroup(
                                new LiftCommand(lift, 860),
                                new ArmCommand(arm, ARM_UP_POSITION),
                                new SequentialCommandGroup(
                                        new PedroCommand(follower, paths.get(4)),
                                        new ExtendoCommand(extendo, 240)
                                )


                        ),

                        //sample 2 score
                        new ClawCommand(claw, CLAW_OPEN_POSITION),

                        //sample 2-3 retract and pre-extend
                        new ParallelCommandGroup(
                                new ArmCommand(arm, ARM_DOWN_POSITION),
                                new LiftCommand(lift, 0),
                                new PivotCommand(pivot, PIVOT_DOWN_POSITION),
                                new PedroCommand(follower, paths.get(5)),
                                new IntakeCommand(intake, -1)

                        ),

                        //sample 3 grab

                        new ExtendoCommand(extendo, 420),

                        //sample 3 retract extendo
                        new ParallelCommandGroup(
                                new PivotCommand(pivot, PIVOT_UP_POSITION),
                                new IntakeCommand(intake, -.3),
                                new ExtendoCommand(extendo, 0)
                        ),

                        new WaitCommand(50),
                        new ClawCommand(claw, CLAW_CLOSED_POSITION),
                        new WaitCommand(100),

                        //sample 3 lift extend
                        new ParallelCommandGroup(
                                new LiftCommand(lift, 860),
                                new PedroCommand(follower, paths.get(6)),
                                new ArmCommand(arm, ARM_UP_POSITION)
                        ),

                        //sample 3 score

                        new ClawCommand(claw, CLAW_OPEN_POSITION),
                        new WaitCommand(100),

                        //sample 3-4 retract and pre-extend
                        new ParallelCommandGroup(
                                new ArmCommand(arm, ARM_DOWN_POSITION),
                                new LiftCommand(lift, 0),
                                new ExtendoCommand(extendo, 160),
                                new PivotCommand(pivot, PIVOT_MID_POSITION),
                                new PedroCommand(follower, paths.get(7)),
                                new IntakeCommand(intake, -1)
                        ),

                        //sample 4 grab

                        new PivotCommand(pivot, PIVOT_DOWN_POSITION),
                        new WaitCommand(100),
                        new ExtendoCommand(extendo, 450),
                        new WaitCommand(100),

                        //sample 4 retract
                        new ParallelCommandGroup(
                                new PivotCommand(pivot, PIVOT_UP_POSITION),
                                new IntakeCommand(intake, INTAKE_SLOW_SPEED),
                                new ExtendoCommand(extendo, 0)
                        ),
                        new WaitCommand(150),
                        new ClawCommand(claw, CLAW_CLOSED_POSITION),
                        new WaitCommand(100),

                        //sample 4 lift raise + score
                        new ParallelCommandGroup(
                                new LiftCommand(lift, 860),
                                new ArmCommand(arm, ARM_UP_POSITION),
                                new PedroCommand(follower, paths.get(8))
                        ),

                        //drop
                        new ClawCommand(claw, CLAW_OPEN_POSITION),
                        new WaitCommand(200),

                        new ParallelCommandGroup(
                                new ArmCommand(arm, ARM_DOWN_POSITION),
                                new LiftCommand(lift, 0),
                                new PedroCommand(follower, paths.get(9), false),
                                new PivotCommand(pivot, PIVOT_MID_POSITION),
                                new ExtendoCommand(extendo, 0)


                        ),


                        new ExtendoCommand(extendo, -50)



                )
        );

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {

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
