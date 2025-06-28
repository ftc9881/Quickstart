package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorSubsystem extends SubsystemBase {
    private DigitalChannel pin0;
    private DigitalChannel pin1;

    public SensorSubsystem(HardwareMap hardwareMap) {
        this.pin0 = hardwareMap.digitalChannel.get("digital0");
        this.pin1 = hardwareMap.digitalChannel.get("digital1");
        // Optional: Adjust LED brightness (0-100, nonlinear)
        // sensor.write8(0x03, 50); // Example: Set LED to 50% brightness (adjust per docs)
    }

    // Get RGB values
    public boolean getPin0() {
        return pin0.getState();
    }

    public boolean getPin1() {
        return pin1.getState();
    }

    public boolean isYellow() {
        return pin0.getState() && pin1.getState();
    }

    public boolean isBlue() {
        return pin0.getState() && !pin1.getState();
    }

    public boolean isRed() {
        return !pin0.getState() && pin1.getState();
    }

}

