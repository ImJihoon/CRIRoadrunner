package org.firstinspires.ftc.teamcode.Roadrunner.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Roadrunner.AutonMechDrive;

@TeleOp(name="MotorTrackTest")
@Disabled
public class MotorTrackOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AutonMechDrive drive = new AutonMechDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive())
            drive.setMotorPowers(
                    gamepad1.triangle ? 1 : 0,
                    gamepad1.circle ? 1 : 0,
                    gamepad1.cross ? 1 : 0,
                    gamepad1.square ? 1 : 0
            );

    }
}
