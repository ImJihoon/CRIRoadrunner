package org.firstinspires.ftc.teamcode.RobotSystem.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotSystem.DeviceConfig;
import org.firstinspires.ftc.teamcode.RobotSystem.TomaOpMode;

@Config
public class Extendo extends TomaOpMode {
    public State currentState = State.RETRACTED;

    public static double POWER = 1;

    public static int EXTENDED_POS = 550;
    public static int LESS_EXTENDED_POS = 450;
    public static int MAXEXTENDED_POS = 650;
    public static int SEMI_POS = (EXTENDED_POS / 2) + 50;
    public static int MIN_INTAKE_POS = 300; //TODO



    public Extendo(OpMode opMode) {
        super(opMode);
    }


    public void init() {

        setState(State.RETRACTED);
    }


    public void setState(State state) {
        switch (state) {
            case RETRACTED:
                DeviceConfig.extendo.setTargetPosition(0);

                DeviceConfig.extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                DeviceConfig.extendo.setPower(POWER);
                break;


            case EXTENDED:
                DeviceConfig.extendo.setTargetPosition(EXTENDED_POS);

                DeviceConfig.extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                DeviceConfig.extendo.setPower(POWER);
                break;

            case MAXEXTENDED:
                DeviceConfig.extendo.setTargetPosition(MAXEXTENDED_POS);

                DeviceConfig.extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                DeviceConfig.extendo.setPower(POWER);
                break;

            case SEMIEXTENDED:
                DeviceConfig.extendo.setTargetPosition(SEMI_POS);

                DeviceConfig.extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                DeviceConfig.extendo.setPower(POWER);
                break;

            case LESSEXTEND:
                DeviceConfig.extendo.setTargetPosition(LESS_EXTENDED_POS);

                DeviceConfig.extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                DeviceConfig.extendo.setPower(POWER);
                break;
        }
        currentState = state;
    }

    public double getSlidePos() {

        return ( DeviceConfig.extendo.getCurrentPosition() );

    }


    public enum State {
        RETRACTED,
        EXTENDED,
        MAXEXTENDED,
        SEMIEXTENDED,
        LESSEXTEND
    }
}
