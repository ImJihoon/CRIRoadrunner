package org.firstinspires.ftc.teamcode.RobotSystem.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.RobotSystem.DeviceConfig;
import org.firstinspires.ftc.teamcode.RobotSystem.TomaOpMode;

@Config
public class Slides extends TomaOpMode {

    public State currentState = State.REST;

    public static int REST_POS = 0;

    public static int BOTTOMBASKET_POS = 330;
    public static int TOPBASKET_POS = 900;
    public static int TOPBASKET_MAX_POS = 985;

    public static int WALL_RAISE_POS = 125; //TODO
    public static int BOTTOMSPEC_POS = 400; //TODO
    public static int TOPSPEC_POS = 200; // + 10 TODO
    public static int TOPSPEC_UP_POS = 530;


    public static boolean MAX_HEIGHT = false;




    private static final double REST_POWER = 1; // slide powers should be + (0.7 previously)
    private static final double REST_SLOW_POWER = .5; // slide powers should be +

    public static final double LIFT_POWER = 1;


    public Slides(OpMode opMode) {
        super(opMode);
    }


    public void init() {
        setState(State.REST);
    }


    public void setState(State state) {
        switch (state) {
            case REST:
                setSlideTarget(REST_POS, REST_POWER);
                break;

            case RESTSLOW:
                setSlideTarget(REST_POS, REST_SLOW_POWER);
                break;

            case BOTTOM_BASKET:
                setSlideTarget(BOTTOMBASKET_POS, LIFT_POWER);
                break;


            case TOP_BASKET:

                if (MAX_HEIGHT) setSlideTarget(TOPBASKET_MAX_POS, LIFT_POWER);
                else setSlideTarget(TOPBASKET_POS, LIFT_POWER);
                break;


            case WALLRAISE:
                setSlideTarget(WALL_RAISE_POS, LIFT_POWER);
                break;


            case BOTTOMSPEC:
                setSlideTarget(BOTTOMSPEC_POS, LIFT_POWER);
                break;


            case TOPSPEC:
                setSlideTarget(TOPSPEC_POS, LIFT_POWER);
                break;

            case TOPSPEC_UP:
                setSlideTarget(TOPSPEC_UP_POS, LIFT_POWER);

        }
        currentState = state;
    }

    public double getSlidePos() {

        return ( DeviceConfig.slideLeft.getCurrentPosition() + DeviceConfig.slideRight.getCurrentPosition() ) / 2.0;

    }
    private void setSlideTarget(int targetPosition, double power) {


        DeviceConfig.slideLeft.setTargetPosition(targetPosition);
        DeviceConfig.slideRight.setTargetPosition(targetPosition);

        DeviceConfig.slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DeviceConfig.slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DeviceConfig.slideLeft.setPower(power);
        DeviceConfig.slideRight.setPower(power);

    }

    public void incrementSlides(int ticks) {

        DeviceConfig.slideLeft.setTargetPosition(ticks);
        DeviceConfig.slideRight.setTargetPosition(ticks);

//        DeviceConfig.slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DeviceConfig.slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        DeviceConfig.slideLeft.setPower(1);
        DeviceConfig.slideRight.setPower(1);


    }



    public enum State {
        REST,
        RESTSLOW,
        BOTTOM_BASKET,
        TOP_BASKET,

        WALLRAISE,
        BOTTOMSPEC,
        TOPSPEC,
        TOPSPEC_UP
    }
}
