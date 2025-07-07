package org.firstinspires.ftc.teamcode.RobotSystem.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotSystem.DeviceConfig;
import org.firstinspires.ftc.teamcode.RobotSystem.TomaOpMode;

public class Hang extends TomaOpMode {

    public State currentState;
    public static boolean HANG_MODE = false;

    public Hang(OpMode opMode) {super(opMode); }


    public void setState(State state) {

        switch (state) {

            case REVERSE:
                DeviceConfig.hangLeft.setPower(-1);
                DeviceConfig.hangRight.setPower(-1);
                break;

            case STOP:
                DeviceConfig.hangLeft.setPower(0);
                DeviceConfig.hangRight.setPower(0);
                break;

            case HANG:
                DeviceConfig.hangLeft.setPower(1);
                DeviceConfig.hangRight.setPower(1);
                break;

        }

        currentState = state;

    }



    public enum State {
        HANG,
        REVERSE,
        STOP
    }
}
