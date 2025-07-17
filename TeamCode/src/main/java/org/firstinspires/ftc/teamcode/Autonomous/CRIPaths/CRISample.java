package org.firstinspires.ftc.teamcode.Autonomous.CRIPaths;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.google.ar.core.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.Autonomous.Actions;
import org.firstinspires.ftc.teamcode.Autonomous.AutonSystems;
import org.firstinspires.ftc.teamcode.Autonomous.Timings;
import org.firstinspires.ftc.teamcode.Autonomous.TrajectoryUtil.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Autonomous.Vision.NewSampleSearch;
import org.firstinspires.ftc.teamcode.Roadrunner.AutonMechDrive;
import org.firstinspires.ftc.teamcode.RobotSystem.Lynx;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Color_Sensor;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Teleop.util.Toggle;

import java.util.LinkedList;
import java.util.Queue;

@Autonomous
public class CRISample extends OpMode {
    double changeX = 0;
    ElapsedTime cameraTimer = new ElapsedTime(), sampleTimer = new ElapsedTime();

    //States
    public enum State {
        PRELOAD, YELLOWSAMPLE1, YELLOWSAMPLE2, YELLOWSAMPLE3, SUB, VISION, SPIT
    }
    State currentState = State.PRELOAD;

    //Sample Intake Status
    boolean IS_COLOR_DETECT = false;

    //Hardware Control Vars
    AutonMechDrive drive;
    AutonSystems autonSystems;
    Actions actions;
    Lynx lynx;
    NewSampleSearch vision;

    //Trajectories
    TrajectorySequence preloadTraj, yellowSample1, yellowSample2, yellowSample3;

    //Coords
    Queue<List<Double>> coordQueue;
    Toggle dpadleft = new Toggle();
    Toggle dpadright = new Toggle();
    Toggle dpaddown = new Toggle();

    //Color Vars
    Color_Sensor.State alliance = Color_Sensor.State.BLUE;
    Color_Sensor.State opp = Color_Sensor.State.RED;

    @Override
    public void init() {

        //Create Coord list
        coordQueue = new LinkedList<>();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Declare hardware states
        drive = new AutonMechDrive(hardwareMap);
        autonSystems = new AutonSystems(this);
        autonSystems.robotSubSystems.blocker.setState(Blocker.State.NOT_BLOCK);
        autonSystems.robotSubSystems.extendo.setState(Extendo.State.RETRACTED);
        actions = new Actions(autonSystems);

        drive.setPoseEstimate(new Pose2d(-37.5, -63.5, Math.toRadians(0)));

        preloadTraj = drive.trajectorySequenceBuilder(new Pose2d(-37.5, -63.5, Math.toRadians(0)))
                .addTemporalMarker(actions.prepForDeposit())
                //Bucket + intake 1
                .UNSTABLE_addTemporalMarkerOffset(0.7, actions.flipForDeposit(true))
                .lineToLinearHeading(new Pose2d(-62,-59, Math.toRadians(66)))
                .waitSeconds(0.075)
                .addTemporalMarker(actions.fullExtendo())
                .UNSTABLE_addTemporalMarkerOffset(0, actions.openClawforDeposit())
                .build();

    }

    @Override
    public void init_loop() {

        if (dpadleft.get(gamepad1.dpad_left)) {
            changeX -= 5;
        }
        if (dpadright.get(gamepad1.dpad_right)) {
            changeX += 5;
        }


        if (dpaddown.get(gamepad1.dpad_down)) {
            List<Double> arr = List.of(changeX);
            coordQueue.add(arr);
        }

        if (gamepad1.dpad_up) {
            alliance = alliance == Color_Sensor.State.BLUE ? Color_Sensor.State.RED : Color_Sensor.State.RED;
            opp = opp == Color_Sensor.State.RED ? Color_Sensor.State.BLUE : Color_Sensor.State.RED;
        }


        telemetry.addData("Alliance", alliance);
        telemetry.addData("opp", opp);
        telemetry.addData("Change in X", changeX);

        telemetry.addData("Coord Queue", coordQueue);
        telemetry.update();
    }

    @Override
    public void start(){
        boolean isRed = (alliance == Color_Sensor.State.RED);
        vision = new NewSampleSearch(hardwareMap, isRed, !isRed, true);
        drive.followTrajectorySequenceAsync(preloadTraj);
        currentState = State.PRELOAD;
    }

    @Override
    public void loop() {
        drive.update();
        telemetry.addLine("   ");
        telemetry.addData("current state", currentState);
        telemetry.addData("isBusy", drive.isBusy());

        switch (currentState) {

            case PRELOAD:
                IS_COLOR_DETECT = false;

                if (!drive.isBusy()) {
                    actions.slidesRest();
//                        index++;
                    currentState = State.YELLOWSAMPLE1;
                    yellowSample1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            //intake
                            .UNSTABLE_addTemporalMarkerOffset(0,actions.intake())
                            .UNSTABLE_addTemporalMarkerOffset(0.25,actions.slidesRest())
                            .lineToLinearHeading(new Pose2d(-57,-47, Math.toRadians(72)))
                            .addTemporalMarker(() -> IS_COLOR_DETECT = true)
                            .forward(10)
                            .turn(Math.toRadians(10))
                            .turn(Math.toRadians(-20))
                            .waitSeconds(.5)
                            .back(20)
                            .build();
                    drive.followTrajectorySequenceAsync(yellowSample1);

                }
                break;

            case YELLOWSAMPLE1:

                if (IS_COLOR_DETECT && autonSystems.robotSubSystems.colorSensor.getState() != Color_Sensor.State.NOTHING) {

                    drive.breakFollowing();
                    IS_COLOR_DETECT = false;

                    TrajectorySequence toDeposit = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .addTemporalMarker(() -> autonSystems.setRollerPower(1))
                            .addTemporalMarker(actions.transfer())
                            .waitSeconds(Timings.Deposit.WAIT_FOR_TRANSFER)
                            .addTemporalMarker(() -> autonSystems.robotSubSystems.intake.setState(Intake.State.TRANSFER))
                            .UNSTABLE_addTemporalMarkerOffset(.4, actions.closeClaw())
                            .UNSTABLE_addTemporalMarkerOffset(.6, actions.prepForDeposit())
                            .UNSTABLE_addTemporalMarkerOffset(.8,() -> autonSystems.setRollerPower(-0.75))
                            .UNSTABLE_addTemporalMarkerOffset(1, actions.flipForDeposit(true))
                            .lineToLinearHeading(new Pose2d(-62, -58.5, Math.toRadians(81)))
//                            .UNSTABLE_addTemporalMarkerOffset(Timings.Deposit.CLAW_CLOSE_OFFSET_DELAY, actions.closeClaw())
//                            .addTemporalMarker(actions.prepForDeposit())
                            .waitSeconds(0.8)
                            .UNSTABLE_addTemporalMarkerOffset(0, actions.fullExtendo())
                            .addTemporalMarker(actions.openClawforDeposit())
                            .addTemporalMarker(()-> autonSystems.robotSubSystems.blocker.setState(Blocker.State.BLOCK))
                            .build();


                    drive.followTrajectorySequenceAsync(toDeposit);

                }

                // if timer is up then skip basket

                if (!drive.isBusy()) {
                    currentState = State.YELLOWSAMPLE2;
                    actions.slidesRest();
                    actions.fullExtendo();
                    yellowSample2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            //intake
                            .addTemporalMarker(actions.fullExtendo())
                            .addTemporalMarker(actions.intake())
                            .UNSTABLE_addTemporalMarkerOffset(0.25,actions.slidesRest())
                            .lineToLinearHeading(new Pose2d(-59.25,-50, Math.toRadians(86.6)))
                            .addTemporalMarker(() -> IS_COLOR_DETECT = true)
                            .forward(10)
                            .turn(Math.toRadians(10))
                            .turn(Math.toRadians(-20))
                            .waitSeconds(.5)
                            .back(10)
                            .build();
                    drive.followTrajectorySequenceAsync(yellowSample2);
                }

                break;

            case YELLOWSAMPLE2:
                if (IS_COLOR_DETECT && autonSystems.robotSubSystems.colorSensor.getState() != Color_Sensor.State.NOTHING) {

                    drive.breakFollowing();
                    IS_COLOR_DETECT = false;
                    TrajectorySequence toDeposit = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .addTemporalMarker(() -> autonSystems.setRollerPower(1))
                            .addTemporalMarker(actions.transfer())
                            .waitSeconds(Timings.Deposit.WAIT_FOR_TRANSFER)
                            .addTemporalMarker(() -> autonSystems.robotSubSystems.intake.setState(Intake.State.TRANSFER))
                            .UNSTABLE_addTemporalMarkerOffset(.4, actions.closeClaw())
                            .UNSTABLE_addTemporalMarkerOffset(.6, actions.prepForDeposit())
                            .UNSTABLE_addTemporalMarkerOffset(.6,() -> autonSystems.setRollerPower(-0.75))
                            .UNSTABLE_addTemporalMarkerOffset(1, actions.flipForDeposit(true))
                            .lineToLinearHeading(new Pose2d(-61, -57.75, Math.toRadians(73)))
                            .waitSeconds(0.8)
                            .addTemporalMarker(actions.openClawforDeposit())
                            .addTemporalMarker(()-> autonSystems.robotSubSystems.blocker.setState(Blocker.State.BLOCK))
                            .build();

                    drive.followTrajectorySequenceAsync(toDeposit);

                }

                // if timer is up then skip basket

                if (!drive.isBusy()) {
                    actions.slidesRest();
                    actions.semiExtendo();
                    currentState = State.YELLOWSAMPLE3;
                    yellowSample3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            //intake
                            .addTemporalMarker(actions.intake())
                            .UNSTABLE_addTemporalMarkerOffset(0.25,actions.slidesRest())
                            .lineToLinearHeading(new Pose2d(-55.75,-38.5, Math.toRadians(137.5)))
                            .addTemporalMarker(() -> IS_COLOR_DETECT = true)
                            .turn(Math.toRadians(10))
                            .turn(Math.toRadians(-20))
                            .waitSeconds(1)
                            .lineToLinearHeading(new Pose2d(-55.75,-40, Math.toRadians(90)))
                            .addTemporalMarker(actions.restExtendo())
                            .addTemporalMarker(actions.slidesRest())
                            .build();
                    actions.semiExtendo();
                    drive.followTrajectorySequenceAsync(yellowSample3);

                }

                break;

            case YELLOWSAMPLE3:

                if (IS_COLOR_DETECT && autonSystems.robotSubSystems.colorSensor.getState() != Color_Sensor.State.NOTHING) {
                    drive.breakFollowing();
                    actions.slidesRest();
                    IS_COLOR_DETECT = false;
                    TrajectorySequence toDeposit = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .addTemporalMarker(() -> autonSystems.setRollerPower(1))
                            .addTemporalMarker(actions.transfer())
                            .waitSeconds(Timings.Deposit.WAIT_FOR_TRANSFER)
                            .addTemporalMarker(() -> autonSystems.robotSubSystems.intake.setState(Intake.State.TRANSFER))
                            .addTemporalMarker(() -> autonSystems.setRollerPower(1))
                            .UNSTABLE_addTemporalMarkerOffset(.3, actions.closeClaw())
                            .UNSTABLE_addTemporalMarkerOffset(.6, actions.prepForDeposit())
                            .UNSTABLE_addTemporalMarkerOffset(.6,() -> autonSystems.setRollerPower(-0.75))
                            .UNSTABLE_addTemporalMarkerOffset(1, actions.flipForDeposit(true))
                            .lineToLinearHeading(new Pose2d(-58, -58, Math.toRadians(45)))
//                            .UNSTABLE_addTemporalMarkerOffset(Timings.Deposit.CLAW_CLOSE_OFFSET_DELAY, actions.closeClaw())
//                            .addTemporalMarker(actions.prepForDeposit())
                            .waitSeconds(0.6)
                            .addTemporalMarker(actions.openClawforDeposit())
                            .addTemporalMarker(actions.restExtendo())
                            .build();

                    drive.followTrajectorySequenceAsync(toDeposit);

                }

                if (!drive.isBusy()) {
                    actions.slidesRest();
                    currentState = State.SUB;
                    vision.enable();
                    drive.followTrajectorySequenceAsync(getToSubTraj(drive.getPoseEstimate()));
                }

                break;

            case SUB:
                if (!drive.isBusy()) {
                    cameraTimer.reset();
                    currentState = State.VISION;
                    drive.followTrajectorySequenceAsync(getIntakeTraj(drive.getPoseEstimate(),vision.getX(),vision.getY()));
                    vision.disable();
                }

                break;
            case VISION:
                if(IS_COLOR_DETECT) {
                    if (autonSystems.robotSubSystems.colorSensor.getState() == Color_Sensor.State.YELLOW || autonSystems.robotSubSystems.colorSensor.getState() == alliance){
                        drive.breakFollowing();
                        IS_COLOR_DETECT = false;
                        TrajectorySequence toDeposit = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addTemporalMarker(() -> autonSystems.robotSubSystems.intake.setState(Intake.State.UP))
                                .addTemporalMarker(() -> autonSystems.setRollerPower(1))
                                .addTemporalMarker(actions.transfer())
                                .addTemporalMarker(actions.restExtendo())
                                .addTemporalMarker(() -> autonSystems.setRollerPower(1))
                                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> autonSystems.robotSubSystems.intake.setState(Intake.State.TRANSFER))
                                .UNSTABLE_addTemporalMarkerOffset(0.7, actions.closeClaw())
                                .UNSTABLE_addTemporalMarkerOffset(1.1, actions.prepForDeposit())
                                .UNSTABLE_addTemporalMarkerOffset(1.1,() -> autonSystems.setRollerPower(-0.75))
                                .UNSTABLE_addTemporalMarkerOffset(1.1, actions.flipForDeposit(true))
                                .setReversed(true)
                                .splineTo(new Vector2d(-50, -50), Math.toRadians(225))
                                .lineToLinearHeading(new Pose2d(-59, -59,Math.toRadians(45)))
                                .setReversed(false)
                                .addTemporalMarker(actions.openClawforDeposit())
                                .addTemporalMarker(actions.restExtendo())
                                .build();

                        drive.followTrajectorySequenceAsync(toDeposit);
                    }
                    if (autonSystems.robotSubSystems.colorSensor.getState() == opp || !drive.isBusy()) {
                        drive.breakFollowing();
                        TrajectorySequence tryAgainTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addTemporalMarker(() -> autonSystems.setRollerPower(-0.75))
                                .addTemporalMarker(() -> autonSystems.robotSubSystems.intake.setState(Intake.State.UP))
                                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX(), -34, Math.toRadians(90)))
                                .build();

                        currentState = State.SUB;
                        drive.followTrajectorySequenceAsync(tryAgainTraj);
                    }
                }else if (!drive.isBusy()) {
                        actions.slidesRest();
                        currentState = State.SUB;
                        drive.followTrajectorySequenceAsync(getToSubTraj(drive.getPoseEstimate()));
                }

                break;

        }
    }

    @Override
    public void stop(){
        vision.close();
    }

    public TrajectorySequence getToSubTraj(Pose2d drivePos) {

        List<Double> coords;

        if (!coordQueue.isEmpty()) {
            coords = coordQueue.poll();
        }

        else {
            coords = List.of(0.0);
        }
        telemetry.addData("SUB Offset X", coords.get(0));

        telemetry.update();

        return drive.trajectorySequenceBuilder(drivePos)
                .addTemporalMarker(() -> autonSystems.setRollerPower(-0.75))
                .addTemporalMarker(() -> autonSystems.robotSubSystems.intake.setState(Intake.State.UP))
//                .splineTo(new Vector2d(coords.get(0),-35),Math.toRadians(90))
//                .addTemporalMarker(actions.semiExtendo())
//                .lineToLinearHeading(new Pose2d(coords.get(0),-25,Math.toRadians(90)))
//                .addTemporalMarker(actions.semiExtendo())
                .UNSTABLE_addTemporalMarkerOffset(0.25,actions.slidesRest())
                .lineToLinearHeading(new Pose2d(coords.get(0)+5,-29,Math.toRadians(90)))
                .addTemporalMarker(actions.fullExtendo())
                .lineToLinearHeading(new Pose2d(coords.get(0),-34,Math.toRadians(90)))
                .addTemporalMarker(()-> vision.beginProcessing(20))
                .build();
    }

    public TrajectorySequence getIntakeTraj(Pose2d drivePos, double offsetX, double offsetY) {
        return drive.trajectorySequenceBuilder(drivePos)
                .addTemporalMarker(() -> autonSystems.setRollerPower(1))
                .lineToConstantHeading(new Vector2d(drivePos.getX()+offsetX+1, drivePos.getY()+offsetY+8))
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> autonSystems.robotSubSystems.intake.setState(Intake.State.DOWN))
                .addTemporalMarker(actions.intake())
                .addTemporalMarker(() -> IS_COLOR_DETECT = true)
                .addTemporalMarker(() -> autonSystems.robotSubSystems.intake.setState(Intake.State.DOWN))
                .waitSeconds(0.1)
                .forward(7)
                .waitSeconds(0.5)
                .build();
    }
}
