package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem extends SubsystemBase  {
    private Servo arm;
    private double target = .93;

    public ArmSubsystem (HardwareMap hMap) {
        arm = hMap.get(Servo.class, "arm");

    }

    @Override
    public void periodic(){
        arm.setPosition(target);
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void armUp () {
        this.target = .21;
    }

    public void armDown() {
        this.target = .93;
    }

    public boolean isAtTarget() {
        return true;
    }

}
