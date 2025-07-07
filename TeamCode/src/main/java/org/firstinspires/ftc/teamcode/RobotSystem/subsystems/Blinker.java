package org.firstinspires.ftc.teamcode.RobotSystem.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotSystem.DeviceConfig;
import org.firstinspires.ftc.teamcode.RobotSystem.TomaOpMode;

@Config
public class Blinker extends TomaOpMode {


    public static double OFF = 0.5;
    public static double YELLOW = 2000;
    public static double BLUE = 0;
    public static double RED = 0;

    public Blinker(OpMode opMode) {
        super(opMode);
    }

    public void init() {
//        DeviceConfig.blinker.pwmEnable();


    }

//    public void test() {
//        DeviceConfig.blinker.setPosition(0.279);
////        ((LinearOpMode) opMode).sleep(100);
////        DeviceConfig.blinker.setPosition(.388);
//
//    }




}