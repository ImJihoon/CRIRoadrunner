package org.firstinspires.ftc.teamcode.RobotSystem.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotSystem.DeviceConfig;
import org.firstinspires.ftc.teamcode.RobotSystem.TomaOpMode;

@Config
public class Claw extends TomaOpMode{

    public static double openPos = 0.40;
    public static double closPos = 0.67;

    public State currentState = State.CLOSED;

    public Claw(OpMode opMode) {
        super(opMode);
    }


    public void init() {
        setState(State.OPEN);
    }


    public void setState(State state) {
        switch (state) {
            case OPEN:
                DeviceConfig.claw.setPosition(openPos);
                break;


            case CLOSED:
                DeviceConfig.claw.setPosition(closPos);
                break;
        }
        currentState = state;
    }


    public enum State {
        OPEN,
        CLOSED
    }
}
