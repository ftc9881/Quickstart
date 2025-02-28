package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Robot: Teleop Lotus", group="Robot")
//@Disabled
public class RobotTeleopLotus extends LinearOpMode {
    /* Copyright (c) 2017 FIRST. All rights reserved.
     *
     * Redistribution and use in source and binary forms, with or without modification,
     * are permitted (subject to the limitations in the disclaimer below) provided that
     * the following conditions are met:
     *
     * Redistributions of source code must retain the above copyright notice, this list
     * of conditions and the following disclaimer.
     *
     * Redistributions in binary form must reproduce the above copyright notice, this
     * list of conditions and the following disclaimer in the documentation and/or
     * other materials provided with the distribution.
     *
     * Neither the name of FIRST nor the names of its contributors may be used to endorse or
     * promote products derived from this software without specific prior written permission.
     *
     * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
     * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
     * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
     * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
     * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
     * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
     * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
     * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
     * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
     * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
     * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
     */



    /* Declare OpMode members. */
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor rightRear = null;
    public DcMotor leftRear = null;
    public DcMotor extendo = null;
    // ---------------------------------- //
    public Servo arm = null;
    public Servo pto = null;
    public Servo sweeper = null;
    public CRServo intake = null;
    public Servo claw = null;
    public Servo pivot = null;
    // ---------------------------------- //
    public DcMotor leftLift = null;
    public DcMotor rightLift = null;
    // ---------------------------------- //
    public static double ARM_DOWN_POSITION = .93;
    public static double ARM_UP_POSITION = .21;
    // ---------------------------------- //
    public static double PTO_DEFAULT_POSITION = 0.01;
    public static double PTO_ENGAGE_POSITION = 0.58;
    // ---------------------------------- //
    public static double SWEEPER_OUT_POSITION = 0.15;
    public static double SWEEPER_IN_POSITION = .57;
    // ---------------------------------- //
    public static double CLAW_OPEN_POSITION = .25;
    public static double CLAW_CLOSED_POSITION = .74;
    // ---------------------------------- //
    public static double PIVOT_DOWN_POSITION = 0.335;
    public static double PIVOT_MID_POSITION = 0.56;
    public static double PIVOT_UP_POSITION = .95;
    // ---------------------------------- //

    // ---------------------------------- //
    public static int LIFT_HEIGHTS[] = {
            0,
            380,
            860
    };
    // ---------------------------------- //
    public static int EXTENDO_LENGTHS[] = {
            0,
            250,
            450
    };
    // ---------------------------------- //


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // Define and Initialize Motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        extendo = hardwareMap.get(DcMotor.class, "extendo");

        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");

        arm = hardwareMap.get(Servo.class, "arm"); //0
        pto = hardwareMap.get(Servo.class, "pto"); //1
        sweeper = hardwareMap.get(Servo.class, "sweeper"); //2
        intake = hardwareMap.get(CRServo.class, "intake"); //3
        claw = hardwareMap.get(Servo.class, "claw"); //4
        pivot = hardwareMap.get(Servo.class, "pivot"); //5

        // Drive

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Extendo
        boolean pLoopA = false;
        boolean extendoIn = true;

        // Arm
        boolean pLoopY = false;
        boolean pLoopX = false;
        boolean armDown = true;
        double armPosition = ARM_DOWN_POSITION;

        // Sweeper
        boolean pLoopDUp = false;
        boolean sweeperIn = false;
        double sweeperPosition = SWEEPER_OUT_POSITION;

        // Intake
        double intakePower = 0;
        boolean passiveIntakeOn = false;


        // Claw
        boolean pLoopRightBumper = false;
        boolean clawOpen = true;
        double clawPosition = CLAW_OPEN_POSITION;

        // Pivot
        boolean pLoopB = false;
        boolean pivotUp = true;
        boolean pivotMid = false;
        double pivotPosition = PIVOT_UP_POSITION;

        // Lift

        int liftLevel = 0;

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setTargetPosition(0);
        rightLift.setTargetPosition(0);

        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.REVERSE);

        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftLift.setPower(1);
        rightLift.setPower(1);

        leftLift.setTargetPosition(0);
        rightLift.setTargetPosition(0);

        // Extendo

        int extendoLevel = 0;

        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setTargetPosition(0);
        extendo.setDirection(DcMotor.Direction.FORWARD);
        extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendo.setPower(1);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.05; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            int extendoPos = extendo.getCurrentPosition();
            int leftLiftPos = leftLift.getCurrentPosition();

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (gamepad1.left_bumper) {
                frontLeftPower *= .3;
                backLeftPower *= .3;
                frontRightPower *= .3;
                backRightPower *= .3;
            }

            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);

            // ----------------LIFT----------------- //

                //this stuff is done in arm section

            // ---------------EXTENDO--------------- //

            if(gamepad1.a && !pLoopA && extendoLevel > 0) {
                extendoLevel = 0;
                clawOpen = true;
                passiveIntakeOn = true;
            } else if(gamepad1.a && !pLoopA && extendoLevel == 0) {
                extendoLevel = 1;
                pivotMid = true;
            }

            if (extendoLevel == 1 && gamepad1.right_bumper) {
                extendoLevel = 2;
            }

            if (extendoPos < 10 && (leftLiftPos > 100 || clawOpen)) {
                passiveIntakeOn = false;
            }



            pLoopA = gamepad1.a;

            // -----------------ARM & LIFT----------------- // high bucket done, needs low bucket on x

            if(gamepad1.y && !pLoopY && liftLevel > 0) {
                armDown = true;
                armPosition = ARM_DOWN_POSITION;
                liftLevel = 0;
            }

            else if(gamepad1.y && !pLoopY && liftLevel == 0) {
                armDown = false;
                armPosition = ARM_UP_POSITION;
                liftLevel = 2;
            }

            pLoopY = gamepad1.y;

            if(gamepad1.x && !pLoopX && liftLevel == 0) {
                armDown = false;
                armPosition = ARM_UP_POSITION;
                liftLevel = 1;
            } else if (gamepad1.x && !pLoopX && liftLevel > 0) {
                armDown = true;
                armPosition = ARM_DOWN_POSITION;
                liftLevel = 0;
            }

            pLoopX = gamepad1.x;

            // -----------------PTO---------------- // engage done, for some reason the disengage doesnt work, probably a servo issue

//            if (gamepad1.dpad_down) {
//                pto.setPosition(PTO_ENGAGE_POSITION);
//            } else if (gamepad1.dpad_right) {
//                pto.setPosition(PTO_DEFAULT_POSITION);
//            }

            //sweeper out + pivot

            // ----------------SWEEPER----------------- // done

            if (gamepad1.dpad_up) {
                sweeperIn = true;
                extendoLevel = 1;
                pivotMid = true;
            }
            if (gamepad1.dpad_left) {
                sweeperIn = false;
            }

            if(sweeperIn) {
                sweeperPosition = SWEEPER_IN_POSITION;
            } else {
                sweeperPosition = SWEEPER_OUT_POSITION;
            }

            // ----------------INTAKE------------------ //

            intakePower = gamepad1.left_trigger - gamepad1.right_trigger;

            if (passiveIntakeOn) {
                intakePower = -.5;
            }

            // ----------------CLAW------------------ //

            if (extendoLevel == 0) {
                if (gamepad1.right_bumper && !pLoopRightBumper) {
                    clawOpen = !clawOpen;
                }

                pLoopRightBumper = gamepad1.right_bumper;

                if (clawOpen) {
                    clawPosition = CLAW_OPEN_POSITION;
                } else {
                    passiveIntakeOn = true;
                    clawPosition = CLAW_CLOSED_POSITION;
                }
            }

            // ----------------PIVOT------------------ //

            if (gamepad1.b && !pLoopB) {
                pivotMid = !pivotMid;
            }

            pLoopB = gamepad1.b;

            if(pivotMid) {
                pivotPosition = PIVOT_MID_POSITION;
            } else {
                pivotPosition = PIVOT_DOWN_POSITION;
            }

            if (extendoLevel == 0) {
                pivotPosition = PIVOT_UP_POSITION;
            }

            leftLift.setTargetPosition(LIFT_HEIGHTS[liftLevel]);
            rightLift.setTargetPosition(LIFT_HEIGHTS[liftLevel]);

            extendo.setTargetPosition(EXTENDO_LENGTHS[extendoLevel]);

            arm.setPosition(armPosition);

            // no PTO setPosition here so we can conserve power with pwm.disable()

            sweeper.setPosition(sweeperPosition);

            intake.setPower(intakePower);

            claw.setPosition(clawPosition);

            pivot.setPosition(pivotPosition);

            telemetry.addData("Extendo Position", extendo.getCurrentPosition());
            telemetry.addData("Left Lift Position", leftLift.getCurrentPosition());
            telemetry.addData("Right Lift Position: ", rightLift.getCurrentPosition());

            telemetry.addData("armPosition: ", armPosition);
            telemetry.addData("sweeperPosition: ", sweeperPosition);
            telemetry.addData("intakePower: ", intakePower);
            telemetry.addData("clawPosition: ", clawPosition);
            telemetry.addData("pivotPosition: ", pivotPosition);

            telemetry.addData("armDown: ", armDown);
            telemetry.addData("sweeperIn: ", sweeperIn);
            telemetry.addData("clawOpen: ", clawOpen);
            telemetry.addData("pivotUp: ", pivotUp);


            telemetry.update();



        }
    }
}
