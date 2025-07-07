package org.firstinspires.ftc.teamcode.RobotSystem.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotSystem.DeviceConfig;
import org.firstinspires.ftc.teamcode.RobotSystem.TomaOpMode;

@Config
public class VirtualBar extends TomaOpMode {

    public State currentState = State.REST;

    public static double REST_POS = 0.1;

    public static double AUTO_BEFORE_DEPOSIT_POS = 0.46;

    public static double DEPOSIT_POS = 0.55;

    public static double SPEC_PICKUP_POS = 0.8;

    public static double SPEC_HANG_POS = 0.05;

    public static double MID = 0.25;



    public VirtualBar(OpMode opMode) {
        super(opMode);
    }


    public void init() {
        setState(State.REST);
    }


    public void setState(State state) {
        switch (state) {
            case REST:
                DeviceConfig.armLeft.setPosition(REST_POS);
                DeviceConfig.armRight.setPosition(REST_POS);
                break;


            case AUTO_BEFORE_DEPOSIT:
                DeviceConfig.armLeft.setPosition(AUTO_BEFORE_DEPOSIT_POS);
                DeviceConfig.armRight.setPosition(AUTO_BEFORE_DEPOSIT_POS);
                break;

            case DEPOSIT:
                DeviceConfig.armLeft.setPosition(DEPOSIT_POS);
                DeviceConfig.armRight.setPosition(DEPOSIT_POS);
                break;

            case PIVOT:
                DeviceConfig.armLeft.setPosition(SPEC_HANG_POS);
                DeviceConfig.armRight.setPosition(SPEC_HANG_POS);
                break;


            case SPEC_PICKUP:
                DeviceConfig.armLeft.setPosition(SPEC_PICKUP_POS);
                DeviceConfig.armRight.setPosition(SPEC_PICKUP_POS);
                break;

            case MID:
                DeviceConfig.armLeft.setPosition(MID);
                DeviceConfig.armRight.setPosition(MID);
                break;

        }
        currentState = state;
    }


    public enum State {
        REST,
        AUTO_BEFORE_DEPOSIT,
        DEPOSIT,
        PIVOT,
        SPEC_PICKUP,
        MID
    }
}
