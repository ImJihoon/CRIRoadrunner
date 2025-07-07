package org.firstinspires.ftc.teamcode.RobotSystem.subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotSystem.DeviceConfig;
import org.firstinspires.ftc.teamcode.RobotSystem.TomaOpMode;

public class Color_Sensor extends TomaOpMode {

    public enum State {
        RED, YELLOW, BLUE, NOTHING
    }


    public Color_Sensor(OpMode opMode) {
        super(opMode);
    }

    private float[] getHSV() {
        float[] hsv = new float[3];

        // Get RGB values
        int r = DeviceConfig.colorSensor.red();
        int g = DeviceConfig.colorSensor.green();
        int b = DeviceConfig.colorSensor.blue();

        // Convert to HSV
        Color.RGBToHSV(r, g, b, hsv);

        return hsv;
    }

    private boolean isRed() {
        float hue = getHSV()[0];
        return (hue <= 40 || hue > 330);  // 34 // Red hue is around 0° or ≥ 330°
    }

    private boolean isYellow() {
        float hue = getHSV()[0];
        return (hue >= 50 && hue <= 85);  // Yellow hue range
    }

    private boolean isBlue() {
        float hue = getHSV()[0];
        return (hue >= 180 && hue <= 250);  // Blue hue range
    }


    private boolean isThere() {
        if (DeviceConfig.colorSensor.getDistance(DistanceUnit.INCH) <= 2.61) {
            return true;
        }

        return false;
    }



    public void colorTelementry() {

        float[] hsv = getHSV();
        opMode.telemetry.addData("STATE", getState());
        opMode.telemetry.addData("Hue", hsv[0]);
//        opMode.telemetry.addData("Saturation", hsv[1]);
//        opMode.telemetry.addData("Value", hsv[2]);
        opMode.telemetry.update();
    }

    public State getState() {

        if (isThere()) {
            if (isRed()) return State.RED;
            else if (isBlue()) return State.BLUE;
            else if (isYellow()) return State.YELLOW;
        }

        return State.NOTHING;

    }



}
