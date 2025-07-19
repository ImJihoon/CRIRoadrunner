package org.firstinspires.ftc.teamcode.RobotSystem.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotSystem.DeviceConfig;
import org.firstinspires.ftc.teamcode.RobotSystem.TomaOpMode;

@Config
public class Wrist extends TomaOpMode {

    public State currentState = State.REST;

    public static double REST_POS = 0.12;
    public static double AUTO_BEFORE_DEPOSIT_POS = .39;
    public static double AUTO_DEPOSIT_POS = .55;
    public static double DEPOSIT_POS = 0.49;
    public static double GET_SPEC_POS = 0.27;
    public static double SPEC_POS = 0.40;

    public static double DOWN = 0.30;


    public Wrist(OpMode opMode) {
        super(opMode);
    }


    public void init() {
        setState(State.DEPOSIT);
    }


    public void setState(State state) {
        switch (state) {
            case REST:
                DeviceConfig.wrist.setPosition(REST_POS);
                break;
            case AUTO_BEFORE_DEPOSIT:
                DeviceConfig.wrist.setPosition(AUTO_BEFORE_DEPOSIT_POS);
                break;
            case AUTO_DEPOSIT:
                DeviceConfig.wrist.setPosition(AUTO_DEPOSIT_POS);
                break;
            case DEPOSIT:
                DeviceConfig.wrist.setPosition(DEPOSIT_POS);
                break;
            case GET_SPEC:
                DeviceConfig.wrist.setPosition(GET_SPEC_POS);
                break;
            case SPEC:
                DeviceConfig.wrist.setPosition(SPEC_POS);
                break;
            case DOWN:
                DeviceConfig.wrist.setPosition(DOWN);
                break;
        }
        currentState = state;
    }


    public enum State {
        REST,
        AUTO_BEFORE_DEPOSIT,
        AUTO_DEPOSIT,
        DEPOSIT,
        GET_SPEC,
        SPEC,
        DOWN
    }
}
