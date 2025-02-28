package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSubsystem extends SubsystemBase  {
    private DcMotor leftLiftMotor;
    private DcMotor rightLiftMotor;

    private int target;
    private int tolerance = 5;

    public LiftSubsystem (HardwareMap hMap) {
        leftLiftMotor = hMap.get(DcMotor.class, "leftLift");
        rightLiftMotor = hMap.get(DcMotor.class, "rightLift");


        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLiftMotor.setTargetPosition(0);
        rightLiftMotor.setTargetPosition(0);

        leftLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightLiftMotor.setDirection(DcMotor.Direction.REVERSE);

        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftLiftMotor.setPower(1);
        rightLiftMotor.setPower(1);
    }

    @Override
    public void periodic(){
        leftLiftMotor.setTargetPosition(target);
        rightLiftMotor.setTargetPosition(target);
    }

    public void setTarget(int target) {
        this.target = target;
    }

    public void liftDown() {
        this.target = 0;
    }

    public void liftUp() {
        this.target = 860;
    }

    public boolean isAtTarget() {
        int posAvg = (leftLiftMotor.getCurrentPosition() + rightLiftMotor.getCurrentPosition()) / 2;
        return (posAvg > target - tolerance && posAvg < target + tolerance);
    }

}
