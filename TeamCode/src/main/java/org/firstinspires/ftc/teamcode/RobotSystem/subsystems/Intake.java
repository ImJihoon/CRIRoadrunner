package org.firstinspires.ftc.teamcode.RobotSystem.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotSystem.DeviceConfig;
import org.firstinspires.ftc.teamcode.RobotSystem.TomaOpMode;

@Config
public class Intake extends TomaOpMode {

    public State currentState = State.UP;

    public static double SEMI_POS = .84;
    public static double DOWN_POS = .92;
    public static double UP_POS = 0.8;
    public static double TRANSFER_POS = 0.74;


    public Intake(OpMode opMode) {
        super(opMode);
    }


    public void init() {
        setState(State.UP);
    }


    public void setState(State state) {
        switch (state) {
            case DOWN:
                DeviceConfig.intake.setPosition(DOWN_POS);
                break;

            case SEMI:
                DeviceConfig.intake.setPosition(SEMI_POS);
                break;

            case UP:
                DeviceConfig.intake.setPosition(UP_POS);
                break;

            case TRANSFER:
                DeviceConfig.intake.setPosition(TRANSFER_POS);
        }
        currentState = state;
    }


    public void setRollerPower(double power) {
        DeviceConfig.intakeRollers.setPower(power);
    }


    public enum State {
        DOWN,
        SEMI,
        UP,
        TRANSFER
    }
}
