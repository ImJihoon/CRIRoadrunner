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
public class RedSpec extends OpMode {
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
    Toggle dpadup = new Toggle();

    //Color Vars
    Color_Sensor.State alliance = Color_Sensor.State.RED;
    Color_Sensor.State opp = Color_Sensor.State.BLUE;
    boolean onlyYellow = false;

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

        drive.setPoseEstimate(new Pose2d(10.5, -63.5, Math.toRadians(0)));

        preloadTraj = drive.trajectorySequenceBuilder(new Pose2d(10.5, -63.5, Math.toRadians(0)))
                .addTemporalMarker(actions.fullExtendo())
                .addTemporalMarker(actions.intake())
                .lineToLinearHeading(new Pose2d(36,-59, Math.toRadians(0)))
                .waitSeconds(0.15)
                .addTemporalMarker(actions.transfer())
                .waitSeconds(Timings.Deposit.WAIT_FOR_TRANSFER)
                .addTemporalMarker(() -> autonSystems.robotSubSystems.intake.setState(Intake.State.TRANSFER))
                .addTemporalMarker(() -> autonSystems.setRollerPower(1))
                .UNSTABLE_addTemporalMarkerOffset(.1, ()-> autonSystems.robotSubSystems.blocker.setState(Blocker.State.NOT_BLOCK))
                .UNSTABLE_addTemporalMarkerOffset(.4, actions.closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(.8, actions.prepForDeposit())
                .UNSTABLE_addTemporalMarkerOffset(1, actions.slidesRest())
                .UNSTABLE_addTemporalMarkerOffset(1.2, actions.flipForDeposit(true))
                .UNSTABLE_addTemporalMarkerOffset(1.5,() -> autonSystems.setRollerPower(-0.75))
                .back(3)
                .waitSeconds(2)
                .addTemporalMarker(actions.openClawforDeposit())

                .addTemporalMarker(actions.fullExtendo())
                .addTemporalMarker(actions.intake())
                .lineToLinearHeading(new Pose2d(36,-50, Math.toRadians(90)))
                .waitSeconds(0.15)
                .addTemporalMarker(actions.transfer())
                .waitSeconds(Timings.Deposit.WAIT_FOR_TRANSFER)
                .addTemporalMarker(() -> autonSystems.robotSubSystems.intake.setState(Intake.State.TRANSFER))
                .addTemporalMarker(() -> autonSystems.setRollerPower(1))
                .UNSTABLE_addTemporalMarkerOffset(.1, ()-> autonSystems.robotSubSystems.blocker.setState(Blocker.State.NOT_BLOCK))
                .UNSTABLE_addTemporalMarkerOffset(.4, actions.closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(.8, actions.prepForDeposit())
                .UNSTABLE_addTemporalMarkerOffset(1, actions.slidesRest())
                .UNSTABLE_addTemporalMarkerOffset(1.2, actions.flipForDeposit(true))
                .UNSTABLE_addTemporalMarkerOffset(1.5,() -> autonSystems.setRollerPower(-0.75))
                .back(5)
                .waitSeconds(2)
                .addTemporalMarker(actions.openClawforDeposit())

                .addTemporalMarker(actions.fullExtendo())
                .addTemporalMarker(actions.intake())
                .forward(14)
                .waitSeconds(0.15)
                .addTemporalMarker(actions.transfer())
                .waitSeconds(Timings.Deposit.WAIT_FOR_TRANSFER)
                .addTemporalMarker(() -> autonSystems.robotSubSystems.intake.setState(Intake.State.TRANSFER))
                .addTemporalMarker(() -> autonSystems.setRollerPower(1))
                .UNSTABLE_addTemporalMarkerOffset(.1, ()-> autonSystems.robotSubSystems.blocker.setState(Blocker.State.NOT_BLOCK))
                .UNSTABLE_addTemporalMarkerOffset(.4, actions.closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(.8, actions.prepForDeposit())
                .UNSTABLE_addTemporalMarkerOffset(1, actions.slidesRest())
                .UNSTABLE_addTemporalMarkerOffset(1.2, actions.flipForDeposit(true))
                .UNSTABLE_addTemporalMarkerOffset(1.5,() -> autonSystems.setRollerPower(-0.75))
                .back(25)
                .waitSeconds(2)
                .addTemporalMarker(actions.openClawforDeposit())

                .addTemporalMarker(actions.fullExtendo())
                .addTemporalMarker(actions.intake())
                .forward(34)
                .waitSeconds(0.15)
                .addTemporalMarker(actions.transfer())
                .waitSeconds(Timings.Deposit.WAIT_FOR_TRANSFER)
                .addTemporalMarker(() -> autonSystems.robotSubSystems.intake.setState(Intake.State.TRANSFER))
                .addTemporalMarker(() -> autonSystems.setRollerPower(1))
                .UNSTABLE_addTemporalMarkerOffset(.1, ()-> autonSystems.robotSubSystems.blocker.setState(Blocker.State.NOT_BLOCK))
                .UNSTABLE_addTemporalMarkerOffset(.4, actions.closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(.8, actions.prepForDeposit())
                .UNSTABLE_addTemporalMarkerOffset(1, actions.slidesRest())
                .UNSTABLE_addTemporalMarkerOffset(1.2, actions.flipForDeposit(true))
                .UNSTABLE_addTemporalMarkerOffset(1.5,() -> autonSystems.setRollerPower(-0.75))
                .lineToLinearHeading(new Pose2d(52, -55, Math.toRadians(90)))
                .waitSeconds(2)
                .addTemporalMarker(actions.openClawforDeposit())

                .addTemporalMarker(actions.fullExtendo())
                .addTemporalMarker(actions.intake())
                .forward(5)
                .waitSeconds(0.15)
                .addTemporalMarker(actions.transfer())
                .waitSeconds(Timings.Deposit.WAIT_FOR_TRANSFER)
                .addTemporalMarker(() -> autonSystems.robotSubSystems.intake.setState(Intake.State.TRANSFER))
                .addTemporalMarker(() -> autonSystems.setRollerPower(1))
                .UNSTABLE_addTemporalMarkerOffset(.1, ()-> autonSystems.robotSubSystems.blocker.setState(Blocker.State.NOT_BLOCK))
                .UNSTABLE_addTemporalMarkerOffset(.4, actions.closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(.8, actions.prepForDeposit())
                .UNSTABLE_addTemporalMarkerOffset(1, actions.slidesRest())
                .UNSTABLE_addTemporalMarkerOffset(1.2, actions.flipForDeposit(true))
                .UNSTABLE_addTemporalMarkerOffset(1.5,() -> autonSystems.setRollerPower(-0.75))
                .back(5)
                .waitSeconds(2)
                .addTemporalMarker(actions.openClawforDeposit())

                .addTemporalMarker(actions.fullExtendo())
                .addTemporalMarker(actions.intake())
                .forward(9)
                .waitSeconds(0.15)
                .addTemporalMarker(actions.transfer())
                .waitSeconds(Timings.Deposit.WAIT_FOR_TRANSFER)
                .addTemporalMarker(() -> autonSystems.robotSubSystems.intake.setState(Intake.State.TRANSFER))
                .addTemporalMarker(() -> autonSystems.setRollerPower(1))
                .UNSTABLE_addTemporalMarkerOffset(.1, ()-> autonSystems.robotSubSystems.blocker.setState(Blocker.State.NOT_BLOCK))
                .UNSTABLE_addTemporalMarkerOffset(.4, actions.closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(.8, actions.prepForDeposit())
                .UNSTABLE_addTemporalMarkerOffset(1, actions.slidesRest())
                .UNSTABLE_addTemporalMarkerOffset(1.2, actions.flipForDeposit(true))
                .UNSTABLE_addTemporalMarkerOffset(1.5,() -> autonSystems.setRollerPower(-0.75))
                .back(14)
                .waitSeconds(2)
                .addTemporalMarker(actions.openClawforDeposit())

                .addTemporalMarker(actions.fullExtendo())
                .addTemporalMarker(actions.intake())
                .forward(23)
                .waitSeconds(0.15)
                .addTemporalMarker(actions.transfer())
                .waitSeconds(Timings.Deposit.WAIT_FOR_TRANSFER)
                .addTemporalMarker(() -> autonSystems.robotSubSystems.intake.setState(Intake.State.TRANSFER))
                .addTemporalMarker(() -> autonSystems.setRollerPower(1))
                .UNSTABLE_addTemporalMarkerOffset(.1, ()-> autonSystems.robotSubSystems.blocker.setState(Blocker.State.NOT_BLOCK))
                .UNSTABLE_addTemporalMarkerOffset(.4, actions.closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(.8, actions.prepForDeposit())
                .UNSTABLE_addTemporalMarkerOffset(1, actions.slidesRest())
                .UNSTABLE_addTemporalMarkerOffset(1.2, actions.flipForDeposit(true))
                .UNSTABLE_addTemporalMarkerOffset(1.5,() -> autonSystems.setRollerPower(-0.75))
                .back(28)
                .waitSeconds(2)
                .addTemporalMarker(actions.openClawforDeposit())

                .build();

    }

    @Override
    public void start(){
        drive.followTrajectorySequenceAsync(preloadTraj);
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop(){
        vision.close();
    }
}
