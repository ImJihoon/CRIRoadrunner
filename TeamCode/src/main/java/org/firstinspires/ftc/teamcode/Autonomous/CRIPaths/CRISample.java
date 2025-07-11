package org.firstinspires.ftc.teamcode.Autonomous.CRIPaths;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.Autonomous.Actions;
import org.firstinspires.ftc.teamcode.Autonomous.AutonSystems;
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

@Autonomous
public class CRISample extends OpMode {
    //Field positions

    //States
    public enum State {
        PRELOAD, YELLOWSAMPLE1, YELLOWSAMPLE2, YELLOWSAMPLE3, SUB1, SUB2, SUB3
    }
    State currentState = State.PRELOAD;

    //Sample Intake Status
    boolean IS_COLOR_DETECT = false;

    //Timers
    ElapsedTime autonTimer, sampleCollectTimer;

    //Hardware Control Vars
    AutonMechDrive drive;
    AutonSystems autonSystems;
    Actions actions;
    Lynx lynx;

    //Trajectories
    TrajectorySequence preloadTraj, yellowSample1, yellowSample2, yellowSample3;

    //Coords
    Queue<List<Double>> coordQueue;
    Toggle dpadleft = new Toggle();
    Toggle dpadright = new Toggle();
    Toggle dpadup = new Toggle();
    Toggle dpaddown = new Toggle();
    Toggle y = new Toggle();
    Toggle rightBumber = new Toggle();
    Toggle leftBumber = new Toggle();

    //Color Vars
    Color_Sensor.State alliance = Color_Sensor.State.BLUE;
    Color_Sensor.State opp = Color_Sensor.State.RED;

    @Override
    public void init() {
        //Declare the timers
        autonTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        sampleCollectTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

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
                .lineToLinearHeading(new Pose2d(-61,-58, Math.toRadians(66)))
                .addTemporalMarker(actions.fullExtendo())
                .UNSTABLE_addTemporalMarkerOffset(-.55, actions.flipForDeposit(true))
                .waitSeconds(.05)
                .addTemporalMarker(actions.openClawforDeposit())
                .waitSeconds(.25)
                .addTemporalMarker(actions.slidesRest())
                .build();

    }

    @Override
    public void start(){
        drive.followTrajectorySequenceAsync(preloadTraj);
        currentState = State.PRELOAD;
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
                    yellowSample1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            //intake
                            .addTemporalMarker(actions.intake())
                            .lineToLinearHeading(new Pose2d(-57,-48, Math.toRadians(72)))
                            .addTemporalMarker(() -> IS_COLOR_DETECT = true)
                            .forward(10)
                            .turn(Math.toRadians(10))
                            .turn(Math.toRadians(-20))
                            .waitSeconds(.5)
                            .build();
                    drive.followTrajectorySequenceAsync(yellowSample1);

                }
                break;

            case YELLOWSAMPLE1:

                if (IS_COLOR_DETECT && autonSystems.robotSubSystems.colorSensor.getState() != Color_Sensor.State.NOTHING) {

                    drive.breakFollowing();
                    IS_COLOR_DETECT = false;

                    TrajectorySequence toDeposit = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .addTemporalMarker(actions.transfer())
                            .waitSeconds(Timings.Deposit.WAIT_FOR_TRANSFER)
                            .addTemporalMarker(() -> autonSystems.robotSubSystems.intake.setState(Intake.State.TRANSFER))
                            .addTemporalMarker(() -> autonSystems.setRollerPower(1))
                            .UNSTABLE_addTemporalMarkerOffset(.4, actions.closeClaw())
                            .lineToLinearHeading(new Pose2d(-61, -56, Math.toRadians(83)))
//                            .UNSTABLE_addTemporalMarkerOffset(Timings.Deposit.CLAW_CLOSE_OFFSET_DELAY, actions.closeClaw())
                            .addTemporalMarker(actions.prepForDeposit())
                            .waitSeconds(1)
                            .UNSTABLE_addTemporalMarkerOffset(-.55, actions.flipForDeposit(true))
                            .waitSeconds(.05)
                            .addTemporalMarker(actions.openClawforDeposit())
                            .addTemporalMarker(actions.fullExtendo())
                            .waitSeconds(.25)
                            .addTemporalMarker(actions.slidesRest())
                            .build();


                    drive.followTrajectorySequenceAsync(toDeposit);

                }

                // if timer is up then skip basket

                if (!drive.isBusy()) {
                    currentState = State.YELLOWSAMPLE2;
                    sampleCollectTimer.reset();
                    yellowSample2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            //intake
                            .addTemporalMarker(actions.intake())
                            .lineToLinearHeading(new Pose2d(-59.25,-51, Math.toRadians(86.6)))
                            .addTemporalMarker(() -> IS_COLOR_DETECT = true)
                            .forward(10)
                            .turn(Math.toRadians(10))
                            .turn(Math.toRadians(-20))
                            .waitSeconds(.5)
                            .build();
                    drive.followTrajectorySequenceAsync(yellowSample2);
                }

                break;

            case YELLOWSAMPLE2:

                if (IS_COLOR_DETECT && autonSystems.robotSubSystems.colorSensor.getState() != Color_Sensor.State.NOTHING) {

                    drive.breakFollowing();
                    IS_COLOR_DETECT = false;
                    TrajectorySequence toDeposit = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .addTemporalMarker(actions.transfer())
                            .waitSeconds(Timings.Deposit.WAIT_FOR_TRANSFER)
                            .addTemporalMarker(() -> autonSystems.robotSubSystems.intake.setState(Intake.State.TRANSFER))
                            .addTemporalMarker(() -> autonSystems.setRollerPower(1))
                            .UNSTABLE_addTemporalMarkerOffset(.2, actions.closeClaw())
                            .lineToLinearHeading(new Pose2d(-60, -56, Math.toRadians(80)))
//                            .UNSTABLE_addTemporalMarkerOffset(Timings.Deposit.CLAW_CLOSE_OFFSET_DELAY, actions.closeClaw())
                            .addTemporalMarker(actions.prepForDeposit())
                            .waitSeconds(1)
                            .UNSTABLE_addTemporalMarkerOffset(-.55, actions.flipForDeposit(true))
                            .waitSeconds(.05)
                            .addTemporalMarker(actions.openClawforDeposit())
                            .waitSeconds(.25)
                            .addTemporalMarker(actions.slidesRest())
                            .build();

                    drive.followTrajectorySequenceAsync(toDeposit);

                }

                // if timer is up then skip basket

                if (!drive.isBusy()) {
                    currentState = State.YELLOWSAMPLE3;
                    sampleCollectTimer.reset();
                    yellowSample3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            //intake
                            .addTemporalMarker(actions.intake())
                            .lineToLinearHeading(new Pose2d(-56,-38, Math.toRadians(137.5)))
                            .addTemporalMarker(() -> IS_COLOR_DETECT = true)
                            .forward(5)
                            .turn(Math.toRadians(10))
                            .turn(Math.toRadians(-20))
                            .build();
                    actions.semiExtendo();
                    drive.followTrajectorySequenceAsync(yellowSample3);

                }

                break;

            case YELLOWSAMPLE3:

                if (IS_COLOR_DETECT && autonSystems.robotSubSystems.colorSensor.getState() != Color_Sensor.State.NOTHING) {

                    drive.breakFollowing();
                    IS_COLOR_DETECT = false;
                    TrajectorySequence toDeposit = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .addTemporalMarker(actions.transfer())
                            .waitSeconds(Timings.Deposit.WAIT_FOR_TRANSFER)
                            .addTemporalMarker(() -> autonSystems.robotSubSystems.intake.setState(Intake.State.TRANSFER))
                            .addTemporalMarker(() -> autonSystems.setRollerPower(1))
                            .UNSTABLE_addTemporalMarkerOffset(.4, actions.closeClaw())
                            .lineToLinearHeading(new Pose2d(-56, -57.6, Math.toRadians(55.7)))
//                            .UNSTABLE_addTemporalMarkerOffset(Timings.Deposit.CLAW_CLOSE_OFFSET_DELAY, actions.closeClaw())
                            .addTemporalMarker(actions.prepForDeposit())
                            .waitSeconds(1)
                            .UNSTABLE_addTemporalMarkerOffset(-.55, actions.flipForDeposit(true))
                            .waitSeconds(.05)
                            .addTemporalMarker(actions.openClawforDeposit())
                            .addTemporalMarker(actions.fullExtendo())
                            .waitSeconds(.25)
                            .addTemporalMarker(actions.slidesRest())
                            .build();

                    drive.followTrajectorySequenceAsync(toDeposit);

                }

                // if timer is up then skip basket

                if (!drive.isBusy()) {
                    sampleCollectTimer.reset();
//                    currentState = WorldsAuton_StateMachine.State.SUB1;
//                    drive.followTrajectorySequenceAsync(toSub);
                }

                break;
        }
    }
}
