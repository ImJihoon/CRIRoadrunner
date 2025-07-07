package org.firstinspires.ftc.teamcode.Testing;



import android.bluetooth.BluetoothClass;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotSystem.DeviceConfig;
import org.firstinspires.ftc.teamcode.RobotSystem.RobotState;
import org.firstinspires.ftc.teamcode.RobotSystem.SubSystems;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Claw;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Slides;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.VirtualBar;

@Config
@TeleOp(name="Slides Tester")
public class SlidesTester extends LinearOpMode {

    SubSystems robotSubSystems;
    RobotState robotState;


    @Override
    public void runOpMode() throws InterruptedException {

        robotSubSystems = new SubSystems(this, true);
        robotState = new RobotState(this, robotSubSystems);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        DeviceConfig.init(this);

        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {

           robotState.handleSampleState(RobotState.SAMPLE_STATES.DEPOSIT);

            telemetry.addData("Slides Current Position", DeviceConfig.slideLeft.getCurrentPosition());
            telemetry.update();




        }
    }
}
