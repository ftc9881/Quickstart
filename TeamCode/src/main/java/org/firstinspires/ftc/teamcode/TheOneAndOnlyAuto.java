package org.firstinspires.ftc.teamcode;

import android.os.Build;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.nio.file.Paths;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "TheOneAndOnlyAuto")
public class TheOneAndOnlyAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    public Servo arm = null;
    public Servo pto = null;
    public Servo sweeper = null;
    public CRServo intake = null;
    public Servo claw = null;
    public Servo pivot = null;
    // ---------------------------------- //
    public DcMotor extendo = null;
    public DcMotor leftLift = null;
    public DcMotor rightLift = null;


    /** Start Position */
    private final Pose startingPosition = new Pose(6.60, 112.65, Math.toRadians(270));

    /** Scoring Position (The Buckets) */
    private final Pose bucketPosition = new Pose(14.4, 129.09, Math.toRadians(315));

    /** First Yellow Sample */
    private final Pose sampleOnePosition = new Pose(16.80, 114.40, Math.toRadians(15));

    /** Second Yellow Sample */
    private final Pose sampleTwoPosition = new Pose(16.4, 131.38, Math.toRadians(0));

    /** Third Yellow Sample */
    private final Pose sampleThreePosition = new Pose(16.44, 130.63, Math.toRadians(20));

    /** Teammate's Sample */
    private final Pose teammateSamplePosition = new Pose(12.00, 96.11, Math.toRadians(270));

    /** Parking Position */
    private final Pose parkingPosition = new Pose(10.29, 29.49, Math.toRadians(45));

    /** Control Point for Parking */
    private final Pose parkingControlPoint = new Pose(9.6, 81.6);

    /** These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, getSampleOne;
    private Path scoreSampleOne, getSampleTwo;
    private Path scoreSampleTwo, getSampleThree;
    private Path scoreSampleThree, getTeammateSample;
    private Path scoreTeammateSample, park;



    public void buildPaths() {


        /** Path chain version
        PathChain all = follower.pathBuilder()

                .addPath(new BezierLine(new Point(startingPosition), new Point(bucketPosition)))
                .setLinearHeadingInterpolation(startingPosition.getHeading(), bucketPosition.getHeading())

                .addPath(new BezierLine(new Point(bucketPosition), new Point(sampleOnePosition)))
                .setLinearHeadingInterpolation(bucketPosition.getHeading(), sampleOnePosition.getHeading())

                .addPath(new BezierLine(new Point(sampleOnePosition), new Point(bucketPosition)))
                .setLinearHeadingInterpolation(sampleOnePosition.getHeading(), bucketPosition.getHeading())

                .addPath(new BezierLine(new Point(bucketPosition), new Point(sampleTwoPosition)))
                .setLinearHeadingInterpolation(bucketPosition.getHeading(), sampleTwoPosition.getHeading())

                .addPath(new BezierLine(new Point(sampleTwoPosition), new Point(bucketPosition)))
                .setLinearHeadingInterpolation(sampleTwoPosition.getHeading(), bucketPosition.getHeading())

                .addPath(new BezierLine(new Point(bucketPosition), new Point(sampleThreePosition)))
                .setLinearHeadingInterpolation(bucketPosition.getHeading(), sampleThreePosition.getHeading())

                .addPath(new BezierLine(new Point(sampleThreePosition), new Point(bucketPosition)))
                .setLinearHeadingInterpolation(sampleThreePosition.getHeading(), bucketPosition.getHeading())

                .addPath(new BezierLine(new Point(bucketPosition), new Point(teammateSamplePosition)))
                .setLinearHeadingInterpolation(bucketPosition.getHeading(), teammateSamplePosition.getHeading())

                .addPath(new BezierLine(new Point(teammateSamplePosition), new Point(bucketPosition)))
                .setLinearHeadingInterpolation(teammateSamplePosition.getHeading(), bucketPosition.getHeading())

                .addPath(new BezierLine(new Point(bucketPosition), new Point(parkingPosition)))
                .setLinearHeadingInterpolation(bucketPosition.getHeading(), parkingPosition.getHeading())

                .build();
            */
    /** Regular Path Version */
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

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate(){
        switch(pathState){
            case 0:
                follower.followPath(scorePreload);
                clawClose();
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                if(!follower.isBusy()){
                    follower.followPath(getSampleOne,true);
                    liftUp();
                    setPathState(2);
                }
                break;
            case 2:
                clawOpen();
                if(!follower.isBusy()){
                    follower.followPath(scoreSampleOne,true);
                    liftDown();
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    follower.followPath(getSampleTwo,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    follower.followPath(scoreSampleTwo,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    follower.followPath(getSampleThree,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    follower.followPath(scoreSampleThree, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    follower.followPath(getTeammateSample, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    follower.followPath(scoreTeammateSample ,true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    follower.followPath(scoreTeammateSample ,true);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()){
                    follower.followPath(park, true);
                    setPathState(-1);
                }
        }
    }

    public void liftUp () {
        leftLift.setTargetPosition(890);
        rightLift.setTargetPosition(890);
    }

    public void liftDown () {
        leftLift.setTargetPosition(0);
        rightLift.setTargetPosition(0);
    }

    public void setExtendo (int pos) {//250 short 470 long
        extendo.setTargetPosition(pos);
    }

    public void armDown () {
        arm.setPosition(.93);
    }

    public void armUp () {
        arm.setPosition(.21);
    }

    public void sweeperOut () {
        sweeper.setPosition(.1639);
    }

    public void sweeperIn () {
        sweeper.setPosition(.4867);
    }

    public void clawOpen () {
        claw.setPosition(.37);
    }

    public void clawClose () {
        claw.setPosition(.74);
    }

    public void pivotUp () {
        pivot.setPosition(.96);
    }

    public void pivotDown () {
        pivot.setPosition(.4);
    }

    public void intakeOn () {
        intake.setPower(-.8);
    }

    public void intakeOff () {
        intake.setPower(0);
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

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
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

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
    }
}