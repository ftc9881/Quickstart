package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PivotSubsystem extends SubsystemBase  {
    private Servo pivot;
    private double target = .95;

    public PivotSubsystem (HardwareMap hMap) {
        pivot = hMap.get(Servo.class, "pivot");

    }

    @Override
    public void periodic(){
        pivot.setPosition(target);
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void pivotDown () {
        target = .342;
    }

    public void pivotUp () {
        target = .95;
    }

    public boolean isAtTarget() {
        return true;
    }

}
