package org.firstinspires.ftc.teamcode.Teleop;


import static org.firstinspires.ftc.teamcode.RobotSystem.DeviceConfig.imu;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotSystem.RobotState;
import org.firstinspires.ftc.teamcode.RobotSystem.SubSystems;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Claw;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Slides;
import org.firstinspires.ftc.teamcode.Teleop.util.Toggle;


@TeleOp
public class OneTeleop extends LinearOpMode {

    Toggle modeToggle = new Toggle();

    Toggle stateToggle = new Toggle();

    Toggle maxHeightToggle = new Toggle();

    public boolean isFieldCentric = false;

    SubSystems robotSubSystems;

    RobotState robotState;


    @Override
    public void runOpMode() throws InterruptedException {

//        imu.resetYaw();

        robotSubSystems = new SubSystems(this, false);
        robotState = new RobotState(this, robotSubSystems);
//        FailSafe.init(robotSubSystems);


        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {

//            robotSubSystems.colorSensor.testColors();




            robotSubSystems.drivetrain.FieldCentricDrive();

            if (gamepad1.y) {
                imu.resetYaw();
                gamepad1.rumble(500);
            }

            if (modeToggle.get(gamepad1.a)) {
                robotState.toggleMode();
            }

            if (maxHeightToggle.get(gamepad1.dpad_up)) Slides.MAX_HEIGHT = true;


            if (stateToggle.get(gamepad1.right_bumper)) {
                robotState.nextState();
            } else if (stateToggle.get(gamepad1.left_bumper)) {
                robotState.previousState();
            }


            if (gamepad1.dpad_down) {
                robotState.rumble();
                if (robotState.currentMode == RobotState.Mode.SAMPLE_MODE) {
                    robotState.LOW_BASKET = !robotState.LOW_BASKET;
                } else if (robotState.currentMode == RobotState.Mode.SPECIMEN_MODE) {
                    robotState.LOW_BAR = !robotState.LOW_BAR;
                }
            }


            if (robotState.currentMode == RobotState.Mode.SAMPLE_MODE) {      // INTAKE DOWN LOGIC
                if (robotState.currentSampleState == RobotState.SAMPLE_STATES.INTAKE)
                    if (gamepad1.right_trigger > 0.5 && gamepad1.left_trigger > 0.5) {
                        robotSubSystems.intake.setRollerPower(-1);
                        RobotState.SEMIMODE = false;
                    } else if (gamepad1.right_trigger > 0.5) {
                        robotSubSystems.intake.setRollerPower(1);
                        RobotState.SEMIMODE = false;
                    } else if (gamepad1.left_trigger > 0.5) {
                        robotSubSystems.intake.setRollerPower(-1);
                    } else {
                        robotSubSystems.intake.setRollerPower(0);
                        RobotState.SEMIMODE = true;
                    }
                else if (robotState.currentSampleState == RobotState.SAMPLE_STATES.TRANSFER && gamepad1.right_trigger < 0.5 && gamepad1.left_trigger < 0.5 && robotSubSystems.extendo.getSlidePos() <= Extendo.EXTENDED_POS) {
                    robotSubSystems.intake.setRollerPower(1);
                }
                else {
                    if (gamepad1.left_trigger > 0.5) {
                        robotSubSystems.intake.setRollerPower(-1);
                    } else if (gamepad1.right_trigger > 0.5) {
                        robotSubSystems.intake.setRollerPower(1);
                    } else {
                        robotSubSystems.intake.setRollerPower(0);
                    }
                }
            }



            robotState.handleRobotState();

//            robotState.updateTelemetry();

        }
    }
}
