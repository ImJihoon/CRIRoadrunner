package org.firstinspires.ftc.teamcode.RobotSystem.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotSystem.DeviceConfig;
import org.firstinspires.ftc.teamcode.RobotSystem.TomaOpMode;

@Config
public class Blocker extends TomaOpMode {

    public Blocker.State currentState = State.NOT_BLOCK;

    public static double blocked = 0.5;
    public static double not_blocked = 0;

    public Blocker(OpMode opMode) {
        super(opMode);
    }

    public void init() {
        setState(currentState);
    }


    public void setState(State state) {
        switch (state) {
            case BLOCK:
                 DeviceConfig.blocker.setPosition(blocked);
                break;

            case NOT_BLOCK:
                 DeviceConfig.blocker.setPosition(not_blocked);
                break;
        }

        currentState = state;
    }

    public enum State {
        BLOCK,
        NOT_BLOCK
    }


}