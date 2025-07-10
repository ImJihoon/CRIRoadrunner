package org.firstinspires.ftc.teamcode.Autonomous.ArchivedPaths;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autonomous.Actions;
import org.firstinspires.ftc.teamcode.Autonomous.AutonSystems;
import org.firstinspires.ftc.teamcode.Autonomous.Field;
import org.firstinspires.ftc.teamcode.Autonomous.TrajectoryUtil.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Roadrunner.AutonMechDrive;
import org.firstinspires.ftc.teamcode.RobotSystem.DeviceConfig;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Claw;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Slides;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.VirtualBar;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Wrist;

@Config
@Disabled
@Autonomous(name="5 Spec Auton", group = "Autonomous")
public class Auton5Spec extends LinearOpMode implements Field {

    private final Pose2d startPose = POS_1;

    @Override
    public void runOpMode() throws InterruptedException {


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TelemetryPacket telPacket = new TelemetryPacket();
        telPacket.fieldOverlay().setRotation(Math.toRadians(90));

        FtcDashboard.getInstance().sendTelemetryPacket(telPacket);

        AutonMechDrive drive = new AutonMechDrive(hardwareMap);
        AutonSystems autonSystems = new AutonSystems(this);
        Actions actions = new Actions(autonSystems);


        DeviceConfig.slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DeviceConfig.slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DeviceConfig.extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //-------------------------------------------------------------------------------

        int fwdDistance = 40;
        int bwdDistance = 40;

        autonSystems.robotSubSystems.claw.setState(Claw.State.CLOSED);
        autonSystems.robotSubSystems.virtualBar.setState(VirtualBar.State.MID);
        autonSystems.robotSubSystems.wrist.setState(Wrist.State.GET_SPEC);
        autonSystems.robotSubSystems.extendo.setState(Extendo.State.RETRACTED);




        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(
                        new TrajectoryVelocityConstraint() {
                            @Override
                            public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                                return 85;
                            }
                        },
                        new TrajectoryAccelerationConstraint() {
                            @Override
                            public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                                return 85;
                            }
                        }
                )

                .addTemporalMarker(()->autonSystems.robotSubSystems.extendo.setState(Extendo.State.RETRACTED))

                .lineToLinearHeading(PRELOAD_SUBRIGHT)
                .UNSTABLE_addTemporalMarkerOffset(-1, actions.prepToScoreSpec())
                .addTemporalMarker(actions.scoreSpec())
                .waitSeconds(.45)
                .addTemporalMarker(actions.openSpecClaw())
                .addTemporalMarker(actions.specRest())

                .lineTo(new Vector2d(55, 15))
                .splineToLinearHeading(new Pose2d(15, 39, Math.toRadians(180)), Math.toRadians(180))

                .strafeRight(7)
                .back(42)

                .waitSeconds(.01)
                .splineToLinearHeading(new Pose2d(10, 50, Math.toRadians(180)), Math.toRadians(90))


                .strafeRight(7)
                .back(42)
                .waitSeconds(.01)
                .splineToLinearHeading(new Pose2d(10, 55, Math.toRadians(90)), Math.toRadians(90))


                .strafeRight(42)

                .resetConstraints()

                .lineToLinearHeading(SPEC_PICKUP)
                .waitSeconds(0.25)
                .addTemporalMarker(actions.closeSpecClaw())

                .lineToLinearHeading(SUBRIGHT)
                .UNSTABLE_addTemporalMarkerOffset(-1, actions.prepToScoreSpec())
                .addTemporalMarker(actions.scoreSpec())
                .waitSeconds(.65)

                .addTemporalMarker(()->{
                    autonSystems.robotSubSystems.slides.setState(Slides.State.TOPSPEC);
                    autonSystems.robotSubSystems.wrist.setState(Wrist.State.DEPOSIT);
                })

                .UNSTABLE_addTemporalMarkerOffset(.5 ,actions.openSpecClaw())
                .UNSTABLE_addTemporalMarkerOffset(.7, actions.specRest())
                .lineToLinearHeading(SPEC_PICKUP)
                .waitSeconds(0.25)
                .addTemporalMarker(actions.closeSpecClaw())

                .lineToLinearHeading(SUBRIGHT)
                .UNSTABLE_addTemporalMarkerOffset(-1, actions.prepToScoreSpec())
                .addTemporalMarker(actions.scoreSpec())
                .waitSeconds(.75)

                .addTemporalMarker(()->{
                    autonSystems.robotSubSystems.slides.setState(Slides.State.TOPSPEC);
                    autonSystems.robotSubSystems.wrist.setState(Wrist.State.DEPOSIT);
                })

                .UNSTABLE_addTemporalMarkerOffset(.5 ,actions.openSpecClaw())
                .UNSTABLE_addTemporalMarkerOffset(.7, actions.specRest())

                .lineToLinearHeading(SPEC_PICKUP)
                .waitSeconds(0.25)
                .addTemporalMarker(actions.closeSpecClaw())

                .lineToLinearHeading(SUBRIGHT)
                .UNSTABLE_addTemporalMarkerOffset(-1, actions.prepToScoreSpec())
                .addTemporalMarker(actions.scoreSpec())
                .waitSeconds(.75)

                .addTemporalMarker(()->{
                    autonSystems.robotSubSystems.slides.setState(Slides.State.TOPSPEC);
                    autonSystems.robotSubSystems.wrist.setState(Wrist.State.DEPOSIT);
                })

                .UNSTABLE_addTemporalMarkerOffset(.5 ,actions.openSpecClaw())
                .UNSTABLE_addTemporalMarkerOffset(.7, actions.specRest())

                .lineToLinearHeading(SPEC_PICKUP)
                .waitSeconds(0.25)
                .addTemporalMarker(actions.closeSpecClaw())

                .lineToLinearHeading(SUBRIGHT)
                .UNSTABLE_addTemporalMarkerOffset(-1, actions.prepToScoreSpec())
                .addTemporalMarker(actions.scoreSpec())
                .waitSeconds(.75)

                .addTemporalMarker(()->{
                    autonSystems.robotSubSystems.slides.setState(Slides.State.TOPSPEC);
                    autonSystems.robotSubSystems.wrist.setState(Wrist.State.DEPOSIT);
                })

                .UNSTABLE_addTemporalMarkerOffset(.5 ,actions.openSpecClaw())
                .UNSTABLE_addTemporalMarkerOffset(.7, actions.specRest())

                .waitSeconds(1)




                .build();


        waitForStart();
        if (isStopRequested()) {return;}

        while (opModeIsActive()) {

            drive.followTrajectorySequence(trajectory);

            break;
        }

    }






}
