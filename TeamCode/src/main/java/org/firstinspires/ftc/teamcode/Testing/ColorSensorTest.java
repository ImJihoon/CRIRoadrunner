package org.firstinspires.ftc.teamcode.Testing;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotSystem.DeviceConfig;
import org.firstinspires.ftc.teamcode.RobotSystem.Lynx;
import org.firstinspires.ftc.teamcode.RobotSystem.RobotState;
import org.firstinspires.ftc.teamcode.RobotSystem.SubSystems;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Claw;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Intake;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Slides;

@Config
@TeleOp(name="Color Sensor Tester")
public class ColorSensorTest extends LinearOpMode {

    SubSystems robotSubSystems;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robotSubSystems = new SubSystems(this, true);
        robotSubSystems.blocker.setState(Blocker.State.NOT_BLOCK);
        robotSubSystems.extendo.setState(Extendo.State.EXTENDED);
        robotSubSystems.intake.setState(Intake.State.DOWN);

        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {

            robotSubSystems.colorSensor.colorTelementry();
//

        }
    }
}
