package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase  {
    private CRServo intake;
    private double power;

    public IntakeSubsystem (HardwareMap hMap) {
        intake = hMap.get(CRServo.class, "intake");

    }

    @Override
    public void periodic(){
        intake.setPower(power);
    }

    public void setTarget(double power) {
        this.power = power;
    }

    public boolean isAtPower() {
        return true;
    }

}
