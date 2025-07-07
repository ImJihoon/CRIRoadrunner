package org.firstinspires.ftc.teamcode.RobotSystem;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LightBlinker;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;


public class DeviceConfig {


    public static DcMotor
            frontLeft,
            frontRight,
            backLeft,
            backRight,

            extendo,
            intakeRollers,
            slideLeft,
            slideRight;


    public static Servo
            intake,
            armLeft,
            armRight,
            claw,
            wrist,
            blocker;

    public static Servo blinker;

    public static CRServo hangLeft, hangRight;

    public static ColorRangeSensor colorSensor;

    public static IMU imu;


    public static void init(OpMode opMode) {

        // ------------------------DRIVE TRAIN-----------------------------------
        frontLeft = opMode.hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = opMode.hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = opMode.hardwareMap.get(DcMotor.class, "backLeft");
        backRight = opMode.hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reverse Motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        // --------------------------EXTENDO--------------------------------------
        extendo = opMode.hardwareMap.get(DcMotor.class, "extendo");
//        extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // -------------------------INTAKE--------------------------------------
        intake = opMode.hardwareMap.get(Servo.class, "intake");
        intakeRollers = opMode.hardwareMap.get(DcMotor.class, "intakeRollers");
//            intakeRollers.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --------------------------OUTTAKE--------------------------------------
        armLeft = opMode.hardwareMap.get(Servo.class, "armLeft");
        armRight = opMode.hardwareMap.get(Servo.class, "armRight");


        // --------------------------ARM--------------------------------------
        claw = opMode.hardwareMap.get(Servo.class, "claw");
        wrist = opMode.hardwareMap.get(Servo.class, "wrist");

        //----------------------------COLOR SENSOR-----------------------------------------

        colorSensor = opMode.hardwareMap.get(RevColorSensorV3.class, "colorSensor");


        // --------------------------SLIDES--------------------------------------
        slideLeft = opMode.hardwareMap.get(DcMotor.class, "slideLeft");
        slideRight = opMode.hardwareMap.get(DcMotor.class, "slideRight");
//
//            slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // --------------------------HANG--------------------------------------
        hangLeft = opMode.hardwareMap.get(CRServo.class, "hangLeft");
        hangRight = opMode.hardwareMap.get(CRServo.class, "hangRight");
//        hangLeft.setDirection(CRServo.Direction.REVERSE);
        hangLeft.setDirection(CRServo.Direction.REVERSE);

        // ---------------------------BLOCKER SERVO------------------------------
        blocker = opMode.hardwareMap.get(Servo.class, "blocker");


        //-------------------------IMU-----------------------------------------------
        imu = opMode.hardwareMap.get(IMU.class, "imu");

        //------------------------LIGHTS--------------------------------------------
//        blinker = opMode.hardwareMap.get(Servo.class, "blinker");

//         IMU Parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN
        ));


        imu.initialize(parameters);
    }
}
