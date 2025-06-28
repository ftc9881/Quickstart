package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ExtendoSubsystem extends SubsystemBase  {
    private DcMotor extendoMotor;
    private int target;
    private int tolerance = 10;

    public ExtendoSubsystem (HardwareMap hMap) {
        extendoMotor = hMap.get(DcMotor.class, "extendo");
        extendoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoMotor.setTargetPosition(0);
        extendoMotor.setDirection(DcMotor.Direction.FORWARD);
        extendoMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendoMotor.setPower(1);
    }

    @Override
    public void periodic(){
        extendoMotor.setTargetPosition(target);
    }

    public void setTarget(int target) {
        this.target = target;
    }

    public int getPos() {
        return extendoMotor.getCurrentPosition();
    }

    public boolean isAtTarget() {
        return (extendoMotor.getCurrentPosition() + tolerance > target && extendoMotor.getCurrentPosition() - tolerance < target);
    }

}
