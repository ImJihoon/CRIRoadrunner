package org.firstinspires.ftc.teamcode.Testing;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotSystem.DeviceConfig;
import org.firstinspires.ftc.teamcode.RobotSystem.RobotState;
import org.firstinspires.ftc.teamcode.RobotSystem.SubSystems;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Claw;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Hang;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Slides;
@TeleOp(name="HANG TEST")
public class HangTest extends LinearOpMode {

    SubSystems robotSubSystems;

    @Override
    public void runOpMode() throws InterruptedException {

        robotSubSystems = new SubSystems(this, true);

        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {

          if (gamepad1.right_trigger > 0.5 ) {
              robotSubSystems.hang.setState(Hang.State.HANG);
          }

          else if (gamepad1.left_trigger > 0.5) robotSubSystems.hang.setState(Hang.State.REVERSE);

          else robotSubSystems.hang.setState(Hang.State.STOP);

          telemetry.addData("Hang State", robotSubSystems.hang.currentState);
          telemetry.update();


        }
    }
}
