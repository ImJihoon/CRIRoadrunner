package org.firstinspires.ftc.teamcode.RobotSystem.subsystems;


import static org.firstinspires.ftc.teamcode.RobotSystem.DeviceConfig.frontLeft;
import static org.firstinspires.ftc.teamcode.RobotSystem.DeviceConfig.imu;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.apache.commons.math3.ml.neuralnet.twod.util.TopographicErrorHistogram;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorKLNavxMicro;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotSystem.DeviceConfig;
import org.firstinspires.ftc.teamcode.RobotSystem.TomaOpMode;

@Config
public class Drivetrain extends TomaOpMode {

    private static double TARGET_HEADING = Math.toRadians(150);
    private static double kP = 3;

    public static boolean SLOW = false;
    public static boolean SUPER_SLOW = false;

    public Drivetrain(OpMode opMode) {
        super(opMode);
    }


    public void init() {

    }


    public void FieldCentricDrive() {

        double y = -opMode.gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = opMode.gamepad1.left_stick_x * 1; // Counteract imperfect strafing
//        double rx = SLOW ? opMode.gamepad1.right_stick_x / 3 : opMode.gamepad1.right_stick_x;  // reduce turning power BY NOT THAT MUCH

        double rx;

//        if (SUPER_SLOW) {
//            rx = opMode.gamepad1.right_stick_x / 5;
//        }
        if (SLOW) {
            rx = opMode.gamepad1.right_stick_x / 3.5;
        }

        else {
            rx = opMode.gamepad1.right_stick_x / 1.2;
        }


        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX *= 1.1; // TUNE

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

//        double power = SLOW ? 0.85 : 1;
        double power;

        if (SUPER_SLOW) power = 0.35;
        else if (SLOW) power = 0.8;
        else power = 1;

        DeviceConfig.frontLeft.setPower(frontLeftPower * power);
        DeviceConfig.frontRight.setPower(frontRightPower * power);
        DeviceConfig.backLeft.setPower(backLeftPower * power);
        DeviceConfig.backRight.setPower(backRightPower * power);

//        if (opMode.gamepad1.dpad_left) {
//            headingToBasket(getAngle());
//        }


    }

    public void RobotCentricDrive(){
        double y = -opMode.gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = opMode.gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = opMode.gamepad1.right_stick_x;  // reduce turning power BY NOT THAT MUCH

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        y = Math.tan(y);
        x = Math.tan(x);
        rx = Math.tan(rx);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        DeviceConfig.frontLeft.setPower(frontLeftPower);
        DeviceConfig.frontRight.setPower(frontRightPower);
        DeviceConfig.backLeft.setPower(backLeftPower);
        DeviceConfig.backRight.setPower(backRightPower);

    }

    private void headingToBasket(double currentHeading) {

        // remember this should be looped

        currentHeading = Math.toRadians(currentHeading);
        double err = TARGET_HEADING - currentHeading;

        // wrap angle to -pi to pi
        err = Math.atan2(Math.sin(err), Math.cos(err));

        double turnPower = err * kP;

        // limit to [-1, 1]
        turnPower = Math.max(-1, Math.min(1,  turnPower));

        DeviceConfig.frontLeft.setPower(turnPower);
        DeviceConfig.frontRight.setPower(-turnPower);
        DeviceConfig.backLeft.setPower(turnPower);
        DeviceConfig.backRight.setPower(-turnPower);

        if (Math.abs(err) < 0.05) {
            DeviceConfig.frontLeft.setPower(0);
            DeviceConfig.frontRight.setPower(0);
            DeviceConfig.backLeft.setPower(0);
            DeviceConfig.backRight.setPower(0);
            return;
        }



    }

    public double getAngle() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }


}


