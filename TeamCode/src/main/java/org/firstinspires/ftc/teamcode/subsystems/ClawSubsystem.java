package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ClawSubsystem extends SubsystemBase  {
    private Servo claw;
    private double target;

    public ClawSubsystem (HardwareMap hMap) {
        claw = hMap.get(Servo.class, "claw");

    }

    @Override
    public void periodic(){
        claw.setPosition(target);
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void clawClose () {
        target = .74;
    }

    public void clawOpen () {
        target = .25;
    }

    public boolean isAtTarget() {
        return true;
    }

}
