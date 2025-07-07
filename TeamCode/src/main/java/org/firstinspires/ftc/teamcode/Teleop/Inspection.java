package org.firstinspires.ftc.teamcode.Teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotSystem.RobotState;
import org.firstinspires.ftc.teamcode.RobotSystem.SubSystems;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Intake;


@TeleOp(name = "Inspection")
public class Inspection extends LinearOpMode {

    RobotState robotState;
    SubSystems robotSubSystems;

    @Override
    public void runOpMode() throws InterruptedException {

        robotSubSystems = new SubSystems(this, true);

        robotSubSystems.extendo.setState(Extendo.State.SEMIEXTENDED);
        robotSubSystems.intake.setState(Intake.State.DOWN);


        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {


        }
    }
}
