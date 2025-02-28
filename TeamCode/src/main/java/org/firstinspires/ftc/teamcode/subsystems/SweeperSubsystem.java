package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SweeperSubsystem extends SubsystemBase  {
    private Servo sweeper;
    private double target = .4867;

    public SweeperSubsystem (HardwareMap hMap) {
        sweeper = hMap.get(Servo.class, "sweeper");

    }

    @Override
    public void periodic(){
        sweeper.setPosition(target);
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void sweeperOut () {
        target = .1639;
    }

    public void sweeperIn () {
        target = .4867;
    }

    public boolean isAtTarget() {
        return true;
    }

}
