package org.firstinspires.ftc.teamcode.Autonomous.ArchivedPaths;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.Autonomous.AutonSystems;
import org.firstinspires.ftc.teamcode.Autonomous.Field;
import org.firstinspires.ftc.teamcode.Autonomous.Actions;
import org.firstinspires.ftc.teamcode.Autonomous.Timings;
import org.firstinspires.ftc.teamcode.Autonomous.TrajectoryUtil.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Roadrunner.AutonMechDrive;
import org.firstinspires.ftc.teamcode.RobotSystem.Lynx;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Color_Sensor;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Teleop.util.Toggle;

import java.util.LinkedList;
import java.util.Queue;

@Disabled
@Autonomous(name = "Amazing Auton")
public class WorldsAuton_StateMachine extends OpMode implements Field {

    private TelemetryPacket telePacket;

    // --------------------------------STATES----------------------------
    public enum State {
        PRELOAD, YELLOWSAMPLE1, YELLOWSAMPLE2, YELLOWSAMPLE3, SUB1, SUB2, SUB3
    }

    List<State> yellowSampleStates = List.of(State.YELLOWSAMPLE1, State.YELLOWSAMPLE2, State.YELLOWSAMPLE3);
    int index = 0;

    State currentState = State.PRELOAD;

    boolean IS_COLOR_DETECT = false;

    ElapsedTime autonTimer, sampleCollectTimer;

    // -----------------------Variables----------------------------------

    AutonMechDrive drive;
    AutonSystems autonSystems;
    Actions actions;
    Lynx lynx;

    //-----------------------------------Trajectories-----------------------------------------------
    TrajectorySequence preloadTraj, toYellowSample1, toYellowSample2, toYellowSample3, toSub;

    // ---------------------------Coords----------------------
    double changeX = 0;
    double changeY = 650;
    double changeHeading = 0;

    double incrX = 2; // inches
    double incrY = 50; // ticks
    double incrHeading = 5; // degrees

    Queue<List<Double>> coordQueue;

    Toggle dpadleft = new Toggle();
    Toggle dpadright = new Toggle();
    Toggle dpadup = new Toggle();
    Toggle dpaddown = new Toggle();
    Toggle y = new Toggle();

    Toggle rightBumber = new Toggle();
    Toggle leftBumber = new Toggle();



    //------------------------------------COLOR-----------------------------------------------
    Color_Sensor.State alliance = Color_Sensor.State.BLUE;
    Color_Sensor.State opp = Color_Sensor.State.RED;


    @Override
    public void init() {
        autonTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        sampleCollectTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        coordQueue = new LinkedList<>();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TelemetryPacket telePacket = new TelemetryPacket();
        telePacket.fieldOverlay().setRotation(Math.toRadians(90));

        FtcDashboard.getInstance().sendTelemetryPacket(telePacket);

        drive = new AutonMechDrive(hardwareMap);
        autonSystems = new AutonSystems(this);
        autonSystems.robotSubSystems.blocker.setState(Blocker.State.NOT_BLOCK);
        autonSystems.robotSubSystems.extendo.setState(Extendo.State.RETRACTED);

        actions = new Actions(autonSystems);
        lynx = new Lynx(this);


        drive.setPoseEstimate(POS_2);

        preloadTraj = drive.trajectorySequenceBuilder(POS_2)
                .addTemporalMarker(actions.prepForDeposit())
                .lineToLinearHeading(BASKET)
                .UNSTABLE_addTemporalMarkerOffset(-.55, actions.flipForDeposit(true))
                .waitSeconds(.15)
                .addTemporalMarker(actions.openClawforDeposit())
                .waitSeconds(.45)
                .addTemporalMarker(actions.slidesRest())
                .build();

        toYellowSample1 = drive.trajectorySequenceBuilder(BASKET)
                .lineToLinearHeading(YELLOWSAMPLE1)
                .UNSTABLE_addTemporalMarkerOffset(-.5, actions.fullExtendo())
                .UNSTABLE_addTemporalMarkerOffset(-.5, actions.intake())
                .addTemporalMarker(() -> IS_COLOR_DETECT = true)

//                .waitSeconds(1)
                .turn(Math.toRadians(10))
                .turn(Math.toRadians(-10))
                .forward(5)

                .turn(Math.toRadians(10))
                .turn(Math.toRadians(-20))

//                .waitSeconds(1)

                .build();

        toYellowSample2 = drive.trajectorySequenceBuilder(BASKET)
                .lineToLinearHeading(YELLOWSAMPLE2)
                .UNSTABLE_addTemporalMarkerOffset(-.5, actions.fullExtendo())
                .UNSTABLE_addTemporalMarkerOffset(-.5, actions.intake())
                .addTemporalMarker(() -> IS_COLOR_DETECT = true)

//                .waitSeconds(1)
                .turn(Math.toRadians(10))
                .turn(Math.toRadians(-20))

                .turn(Math.toRadians(10))
                .turn(Math.toRadians(-20))

                .waitSeconds(1)

                .build();

        toYellowSample3 = drive.trajectorySequenceBuilder(BASKET)
                .lineToLinearHeading(YELLOWSAMPLE3)
                .UNSTABLE_addTemporalMarkerOffset(-.5, actions.fullExtendo())
                .UNSTABLE_addTemporalMarkerOffset(-.5, actions.intake())
                .addTemporalMarker(() -> IS_COLOR_DETECT = true)

                .waitSeconds(1)
                .turn(Math.toRadians(10))
                .turn(Math.toRadians(-20))

                .turn(Math.toRadians(10))
                .turn(Math.toRadians(-20))

                .waitSeconds(1)

                .build();


        drive.followTrajectorySequenceAsync(preloadTraj);

    }

    @Override
    public void init_loop() {

        if (dpadleft.get(gamepad1.dpad_left)) {
            changeX -= incrX;
        }
        if (dpadright.get(gamepad1.dpad_right)) {
            changeX += incrX;
        }
        if (dpaddown.get(gamepad1.dpad_down)) {
            changeY -= incrY;
        }
        if (dpadup.get(gamepad1.dpad_up)) {
            changeY += incrY;
        }

        if (leftBumber.get(gamepad1.left_bumper)) {
            changeHeading -= incrHeading;
        }
        if (rightBumber.get(gamepad1.right_bumper)) {
            changeHeading += incrHeading;
        }

        if (y.get(gamepad1.triangle)) {
            List<Double> arr = List.of(changeX, changeY, changeHeading);
            coordQueue.add(arr);
        }

        if (gamepad1.square) {
            alliance = alliance == Color_Sensor.State.BLUE ? Color_Sensor.State.RED : Color_Sensor.State.RED;
            opp = opp == Color_Sensor.State.RED ? Color_Sensor.State.BLUE : Color_Sensor.State.RED;
        }


        telemetry.addData("Alliance", alliance);
        telemetry.addData("opp", opp);
        telemetry.addData("Change in X", changeX);
        telemetry.addData("Change in Y", changeY);
        telemetry.addData("Change in Heading", changeHeading);

        telemetry.addData("Coord Queue", coordQueue);
        telemetry.update();
    }


    @Override
    public void loop() {

        drive.update();
        telemetry.addLine("   ");
        telemetry.addData("current state", currentState);
        telemetry.addData("isBusy", drive.isBusy());
        telemetry.addData("Auton Timer", autonTimer.seconds());


        switch (currentState) {

            case PRELOAD:
                IS_COLOR_DETECT = false;

                if (!drive.isBusy()) {
                    sampleCollectTimer.reset();
//                        index++;
                    currentState = State.YELLOWSAMPLE1;
                    drive.followTrajectorySequenceAsync(toYellowSample1);

                }
                break;

            case YELLOWSAMPLE1:

                if (IS_COLOR_DETECT && autonSystems.robotSubSystems.colorSensor.getState() != Color_Sensor.State.NOTHING) {

                    drive.breakFollowing();
                    IS_COLOR_DETECT = false;

                    TrajectorySequence toDeposit = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .addTemporalMarker(actions.transfer())
                            .waitSeconds(Timings.Deposit.WAIT_FOR_TRANSFER)
                            .addTemporalMarker(()->autonSystems.robotSubSystems.intake.setState(Intake.State.TRANSFER))
                            .addTemporalMarker(()->autonSystems.setRollerPower(1))

                            .lineToLinearHeading(BASKET)
                            .UNSTABLE_addTemporalMarkerOffset(Timings.Deposit.CLAW_CLOSE_OFFSET_DELAY, actions.closeClaw())
                            .UNSTABLE_addTemporalMarkerOffset(Timings.Deposit.SLIDES_OFFSET_DELAY, actions.slidesToTopBasket())
                            .UNSTABLE_addTemporalMarkerOffset(Timings.Deposit.PIVOT_OFFSET_DELAY, actions.flipForDeposit(true))
                            .waitSeconds(Timings.Deposit.WAIT_FOR_FLIP)
                            .addTemporalMarker(actions.openClawforDeposit())
                            .waitSeconds(Timings.Deposit.CLAW_WAIT)
                            .addTemporalMarker(actions.slidesRest())
                            .build();


                    drive.followTrajectorySequenceAsync(toDeposit);

                }

                // if timer is up then to basket

                if (!drive.isBusy()) {
                    index++;
                    currentState = getNextYellowSample();
                    sampleCollectTimer.reset();
                    drive.followTrajectorySequenceAsync(toYellowSample2);
                }

                break;

            case YELLOWSAMPLE2:

                if (IS_COLOR_DETECT && autonSystems.robotSubSystems.colorSensor.getState() != Color_Sensor.State.NOTHING) {

                    drive.breakFollowing();
                    IS_COLOR_DETECT = false;

                    TrajectorySequence toDeposit = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .addTemporalMarker(actions.transfer())
                            .waitSeconds(Timings.Deposit.WAIT_FOR_TRANSFER)
                            .addTemporalMarker(()->autonSystems.robotSubSystems.intake.setState(Intake.State.TRANSFER))
                            .addTemporalMarker(()->autonSystems.setRollerPower(1))


                            .lineToLinearHeading(BASKET)
                            .UNSTABLE_addTemporalMarkerOffset(Timings.Deposit.CLAW_CLOSE_OFFSET_DELAY+0.2, actions.closeClaw())
                            .UNSTABLE_addTemporalMarkerOffset(Timings.Deposit.SLIDES_OFFSET_DELAY+0.2, actions.slidesToTopBasket())
                            .UNSTABLE_addTemporalMarkerOffset(Timings.Deposit.PIVOT_OFFSET_DELAY+0.2, actions.flipForDeposit(true))
//                                .waitSeconds(Timings.Deposit.WAIT_FOR_SLIDES)
                            .waitSeconds(Timings.Deposit.WAIT_FOR_FLIP)
                            .addTemporalMarker(actions.openClawforDeposit())
                            .waitSeconds(Timings.Deposit.CLAW_WAIT)
                            .addTemporalMarker(actions.slidesRest())
                            .build();

                    drive.followTrajectorySequenceAsync(toDeposit);

                }

                // if timer is up then to basket

                if (!drive.isBusy()) {
                    index++;
                    currentState = getNextYellowSample();
                    sampleCollectTimer.reset();
                    drive.followTrajectorySequenceAsync(toYellowSample3);

                }

                break;

            case YELLOWSAMPLE3:

                if (IS_COLOR_DETECT && autonSystems.robotSubSystems.colorSensor.getState() != Color_Sensor.State.NOTHING) {

                    drive.breakFollowing();
                    IS_COLOR_DETECT = false;

                    TrajectorySequence toDeposit = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .addTemporalMarker(actions.transfer())
                            .waitSeconds(Timings.Deposit.WAIT_FOR_TRANSFER)
                            .addTemporalMarker(()->autonSystems.robotSubSystems.intake.setState(Intake.State.TRANSFER))
                            .addTemporalMarker(()->autonSystems.setRollerPower(1))


                            .lineToLinearHeading(BASKET)
                            .UNSTABLE_addTemporalMarkerOffset(Timings.Deposit.CLAW_CLOSE_OFFSET_DELAY, actions.closeClaw())
                            .UNSTABLE_addTemporalMarkerOffset(Timings.Deposit.SLIDES_OFFSET_DELAY, actions.slidesToTopBasket())
                            .UNSTABLE_addTemporalMarkerOffset(Timings.Deposit.PIVOT_OFFSET_DELAY, actions.flipForDeposit(true))
//                                .waitSeconds(Timings.Deposit.WAIT_FOR_SLIDES)
                            .waitSeconds(Timings.Deposit.WAIT_FOR_FLIP)
                            .addTemporalMarker(actions.openClawforDeposit())
                            .waitSeconds(Timings.Deposit.CLAW_WAIT)
                            .addTemporalMarker(actions.slidesRest())
                            .build();

                    drive.followTrajectorySequenceAsync(toDeposit);

                }

                // if timer is up then to basket

                if (!drive.isBusy()) {
                    sampleCollectTimer.reset();
                    currentState = State.SUB1;
                    drive.followTrajectorySequenceAsync(toSub);
                }

                break;


            case SUB1:

                if (IS_COLOR_DETECT && (autonSystems.robotSubSystems.colorSensor.getState() == Color_Sensor.State.YELLOW || autonSystems.robotSubSystems.colorSensor.getState() == alliance)) {

                    drive.breakFollowing();
                    IS_COLOR_DETECT = false;

                    TrajectorySequence toDeposit = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                            .addTemporalMarker(()->autonSystems.setRollerPower(-1))
//                            .addTemporalMarker(actions.fullExtendo())
//                            .waitSeconds(.10)
                            .addTemporalMarker(()->autonSystems.setRollerPower(1))
                            .addTemporalMarker(actions.transfer())
                            .waitSeconds(.67)
                            .addTemporalMarker(()->autonSystems.robotSubSystems.intake.setState(Intake.State.TRANSFER))

                            .setReversed(true)
                            .splineToLinearHeading(BASKET, Math.toRadians(295))
                            .addTemporalMarker(()->autonSystems.setRollerPower(1))

                            .UNSTABLE_addTemporalMarkerOffset(-1.75+0.15, actions.closeClaw())
                            .UNSTABLE_addTemporalMarkerOffset(-1.25+0.15, actions.slidesToTopBasket())
                            .UNSTABLE_addTemporalMarkerOffset(-.75+0.15, actions.flipForDeposit(true))
                            .addTemporalMarker(()->autonSystems.setRollerPower(-1))
                            .addTemporalMarker(()->autonSystems.robotSubSystems.extendo.setState(Extendo.State.RETRACTED))
                            .waitSeconds(.15)
                            .addTemporalMarker(actions.openSpecClaw())
                            .waitSeconds(.1)
                            .addTemporalMarker(actions.slidesRest())
                            .build();

                    drive.followTrajectorySequenceAsync(toDeposit);

                }

                if (IS_COLOR_DETECT && autonSystems.robotSubSystems.colorSensor.getState() == opp) {
//                    autonSystems.robotSubSystems.blocker.setState(Blocker.State.NOT_BLOCK);
//                    autonSystems.robotSubSystems.intake.setState(Intake.State.SEMI);
                    autonSystems.setRollerPower(-1);
                }

                if (IS_COLOR_DETECT && autonSystems.robotSubSystems.colorSensor.getState() == Color_Sensor.State.NOTHING) {
                    autonSystems.robotSubSystems.blocker.setState(Blocker.State.BLOCK);
                }

                // if timer is up then to basket

                if (!drive.isBusy()) {
                    sampleCollectTimer.reset();
                    currentState = State.SUB1;
                    toSub = getToSubTraj();
                    drive.followTrajectorySequenceAsync(toSub);                    }

                break;

        }

        telemetry.update();
        lynx.clearCache();
    }


    public State getNextYellowSample() {
        return yellowSampleStates.get(index);
    }

    public TrajectorySequence getToSubTraj() {

        List<Double> coords;

        if (!coordQueue.isEmpty()) {
            coords = coordQueue.poll();
        }

        else {
            coords = List.of(0.0, 0.0, 0.0);
        }
        telemetry.addData("SUB Offset X", coords.get(0));
        telemetry.addData("SUB Offset Y", coords.get(1));
        telemetry.addData("SUB Offset Heading", coords.get(2));

        telemetry.update();

        return drive.trajectorySequenceBuilder(BASKET)
                .setConstraints(
                        new TrajectoryVelocityConstraint() {
                            @Override
                            public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                                return 80;
                            }
                        },
                        new TrajectoryAccelerationConstraint() {
                            @Override
                            public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                                return 80;
                            }
                        }
                )

                .lineToSplineHeading(new Pose2d(0, -20, Math.toRadians(90))) // fix path maybe

                .lineToLinearHeading(new Pose2d(SUBBOTTOM.getX() + coords.get(0), SUBBOTTOM.getY(), SUBBOTTOM.getHeading() + Math.toRadians(coords.get(2))))
                .UNSTABLE_addTemporalMarkerOffset(-1, actions.semiExtendo())
                .addTemporalMarker(() -> IS_COLOR_DETECT = true)

                .addTemporalMarker(actions.subExtendo(coords.get(1)))
                .addTemporalMarker(actions.intake())

                .addTemporalMarker(actions.maxExtendo())
                .addTemporalMarker(actions.intake())

                .waitSeconds(.35)
                .addTemporalMarker(actions.semiExtendo())
                .addTemporalMarker(actions.intake())

                .waitSeconds(.35)
                .addTemporalMarker(actions.maxExtendo())
                .addTemporalMarker(actions.intake())
                .turn(Math.toRadians(-20))

                .waitSeconds(.35)
                .addTemporalMarker(actions.maxExtendo())
                .addTemporalMarker(actions.intake())

                .waitSeconds(.35)
                .addTemporalMarker(actions.semiExtendo())
                .addTemporalMarker(actions.intake())

                .turn(Math.toRadians(20))


                .waitSeconds(.35)
                .addTemporalMarker(actions.semiExtendo())
                .addTemporalMarker(actions.intake())


                .addTemporalMarker(()->autonSystems.setRollerPower(-1))
                .waitSeconds(.1)

                .addTemporalMarker(actions.maxExtendo())
                .addTemporalMarker(actions.intake())

                .waitSeconds(.35)
                .addTemporalMarker(actions.semiExtendo())
                .addTemporalMarker(actions.intake())

                .turn(Math.toRadians(-20))

                .addTemporalMarker(actions.maxExtendo())
                .addTemporalMarker(actions.intake())

                .waitSeconds(.35)
                .addTemporalMarker(actions.maxExtendo())
                .addTemporalMarker(actions.intake())


                .addTemporalMarker(actions.maxExtendo())
                .addTemporalMarker(actions.intake())

                .turn(Math.toRadians(-0))


                .waitSeconds(.35)
                .addTemporalMarker(actions.semiExtendo())
                .addTemporalMarker(actions.intake())

                .waitSeconds(.35)
                .addTemporalMarker(actions.maxExtendo())
                .addTemporalMarker(actions.intake())
//                .strafeRight(-1)

                .waitSeconds(.35)
                .addTemporalMarker(actions.maxExtendo())
                .addTemporalMarker(actions.intake())

                .turn(Math.toRadians(20))


                .addTemporalMarker(actions.maxExtendo())
                .addTemporalMarker(actions.intake())

                .waitSeconds(.35)
                .addTemporalMarker(actions.semiExtendo())
                .addTemporalMarker(actions.intake())

                .strafeLeft(3)


                .waitSeconds(.35)
                .addTemporalMarker(actions.maxExtendo())
                .addTemporalMarker(actions.intake())
//                .strafeRight(-1)

                .waitSeconds(.35)
                .addTemporalMarker(actions.maxExtendo())
                .addTemporalMarker(actions.intake())



                .build();
    }





}
