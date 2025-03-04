package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorSubsystem extends SubsystemBase {
    private ColorSensor sensor; // Brushland sensor is compatible with this class

    public SensorSubsystem(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(ColorSensor.class, "sensor");
        // Optional: Adjust LED brightness (0-100, nonlinear)
        // sensor.write8(0x03, 50); // Example: Set LED to 50% brightness (adjust per docs)
    }

    // Get RGB values
    public int getRed() {
        return sensor.red();
    }

    public int getGreen() {
        return sensor.green();
    }

    public int getBlue() {
        return sensor.blue();
    }

    public boolean isRed() {
        return getRed() > 1500;
    }

    public boolean isBlue() {
        return getBlue() > 1000;
    }

    public boolean isYellow() {
        return getGreen() > 2000;
    }

//    public int getDistance() {
//        return pin0.getVoltage() / 3.3 * 100;
//    }
}

