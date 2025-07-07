package org.firstinspires.ftc.teamcode.Teleop;


import static org.firstinspires.ftc.teamcode.RobotSystem.DeviceConfig.imu;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotSystem.DeviceConfig;
import org.firstinspires.ftc.teamcode.RobotSystem.Lynx;
import org.firstinspires.ftc.teamcode.RobotSystem.RobotState;
import org.firstinspires.ftc.teamcode.RobotSystem.SubSystems;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Blinker;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Claw;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Color_Sensor;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Hang;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Slides;
import org.firstinspires.ftc.teamcode.Teleop.util.Toggle;


@TeleOp
public class MainTeleop extends LinearOpMode {

    Toggle modeToggle = new Toggle();
    Toggle hangToggle = new Toggle();

    Toggle stateToggle = new Toggle();
    Toggle basketToggle = new Toggle();
    Toggle maxToggle = new Toggle();

    Toggle blockerToggle = new Toggle();

    SubSystems robotSubSystems;
    RobotState robotState;

    Lynx lynxModule;


    int slidesManualTicks = Slides.TOPBASKET_POS;
    @Override
    public void runOpMode() throws InterruptedException {

        robotSubSystems = new SubSystems(this, false);
        robotState = new RobotState(this, robotSubSystems);
        robotSubSystems.extendo.setState(Extendo.State.SEMIEXTENDED);
        lynxModule = new Lynx(this);
        lynxModule.setManualMode();

        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {

            robotSubSystems.drivetrain.RobotCentricDrive();


            if (gamepad1.y) { // RESET IMU
                imu.resetYaw();
                gamepad1.rumble(500);
            }

            if (modeToggle.get(gamepad2.a)) { // TOGGLE ROBOT MODE
                robotState.toggleMode();
            }


            if (maxToggle.get(gamepad2.dpad_up)) {   //  SLIDE ADJUSTMENT TOGGLE

               Slides.MAX_HEIGHT = !Slides.MAX_HEIGHT;

            }

            if (stateToggle.get(gamepad2.right_bumper)) { // TOGGLE STATES
                robotState.nextState();
            } else if (stateToggle.get(gamepad2.left_bumper)) {
                robotState.previousState();
            }

            if (blockerToggle.get(gamepad2.square)) {   //  BLOCKER TOGGLE

                robotSubSystems.blocker.setState(robotSubSystems.blocker.currentState == Blocker.State.BLOCK ? Blocker.State.NOT_BLOCK : Blocker.State.BLOCK);

            }

            if (basketToggle.get(gamepad2.dpad_down)) { // TOGGLE BASKET MODE
                gamepad2.rumble(500);
                if (robotState.currentMode == RobotState.Mode.SAMPLE_MODE) {
                    robotState.LOW_BASKET = !robotState.LOW_BASKET;
                } else if (robotState.currentMode == RobotState.Mode.SPECIMEN_MODE) {
                    robotState.LOW_BAR = !robotState.LOW_BAR;
                }
            }

            while (gamepad2.options) { // RESET SLIDES

                robotSubSystems.slides.incrementSlides(DeviceConfig.slideLeft.getCurrentPosition() - 100);
//                telemetry.addData("Slide Pos", DeviceConfig.slideLeft.getCurrentPosition());
//                telemetry.update();
            }

            if (gamepad2.triangle) {
                DeviceConfig.slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                DeviceConfig.slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }


            if (robotState.currentMode == RobotState.Mode.SAMPLE_MODE) {      // INTAKE DOWN LOGIC
                if (robotState.currentSampleState == RobotState.SAMPLE_STATES.INTAKE)
                    if (gamepad2.right_trigger > 0.5 && gamepad2.left_trigger > 0.5) {
                        robotSubSystems.intake.setRollerPower(-1);
                        RobotState.SEMIMODE = false;
                    } else if (gamepad2.right_trigger > 0.5) {
                        robotSubSystems.intake.setRollerPower(1);
                        RobotState.SEMIMODE = false;
                    } else if (gamepad2.left_trigger > 0.5) {
                        robotSubSystems.intake.setRollerPower(-1);
                    } else {
                        robotSubSystems.intake.setRollerPower(0);
                        RobotState.SEMIMODE = true;
                    }
                else if (robotState.currentSampleState == RobotState.SAMPLE_STATES.TRANSFER && gamepad2.right_trigger < 0.5 && gamepad2.left_trigger < 0.5 && robotSubSystems.extendo.getSlidePos() <= Extendo.EXTENDED_POS) {
                    robotSubSystems.intake.setRollerPower(1);
                }
                else {
                    if (gamepad2.left_trigger >= 0.5) {
                        robotSubSystems.intake.setRollerPower(-1);
                    } else if (gamepad2.right_trigger >= 0.5) {
                        robotSubSystems.intake.setRollerPower(1);
                    } else {
                        robotSubSystems.intake.setRollerPower(0);
                    }
                }
            }



            if (hangToggle.get(gamepad1.square)) {   // HANG MODE TOGGLE
                Hang.HANG_MODE = !Hang.HANG_MODE;
                gamepad1.rumble(400);
            }

            if (Hang.HANG_MODE) {
                if (gamepad1.right_trigger > 0.5 ) {
                    robotSubSystems.hang.setState(Hang.State.HANG);
                }

                else if (gamepad1.left_trigger > 0.5) robotSubSystems.hang.setState(Hang.State.REVERSE);

                else robotSubSystems.hang.setState(Hang.State.STOP);
            }

//            if (robotSubSystems.colorSensor.isYellow() && robotState.currentSampleState == RobotState.SAMPLE_STATES.INTAKE  && autoRetracted == false) {
//                robotState.handleSampleState(RobotState.SAMPLE_STATES.TRANSFER);
//                autoRetracted = true;
//
//            }
//
//            if (!robotSubSystems.colorSensor.isYellow()) {
//                autoRetracted = false;
//            }

//            blinker();


            robotState.handleRobotState();
            robotState.updateTelemetry();

            lynxModule.clearCache();




        }
    }

//    public void blinker() {
//        Color_Sensor.State colorState = robotSubSystems.colorSensor.getState();
//
//        switch (colorState) {
//            case YELLOW:
//                robotSubSystems.blinker.setState(Blinker.State.YELLOW);
//                break;
//            case BLUE:
//                robotSubSystems.blinker.setState(Blinker.State.BLUE);
//                break;
//            case RED:
//                robotSubSystems.blinker.setState(Blinker.State.RED);
//                break;
//            case NOTHING:
//                robotSubSystems.blinker.setState(Blinker.State.OFF);
//                break;
//        }
//    }

}
