//package org.firstinspires.ftc.teamcode.Autonomous.Paths;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.Autonomous.AutonSystems;
//import org.firstinspires.ftc.teamcode.Autonomous.Field;
//import org.firstinspires.ftc.teamcode.Autonomous.TrajectoryUtil.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.Roadrunner.AutonMechDrive;
//import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.VirtualBar;
//import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Claw;
//import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Slides;
//
//
//@Autonomous(name="TEST", group = "Autonomous")
//public class test extends LinearOpMode implements Field {
//
//    private final Pose2d startPose = POS_2;
//
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        AutonMechDrive drive = new AutonMechDrive(hardwareMap);
//        AutonSystems autonSystems = new AutonSystems(this);
//        autonSystems.robotSubSystems.claw.setState(Claw.State.OPEN);
//
//
//
//        // -----------------------------------------------------
//
//        MarkerCallback deposit = autonSystems::depositSeq;
//        MarkerCallback flip = () -> {
//            autonSystems.dropSeq();
//            autonSystems.robotSubSystems.intake.setState(Intake.State.DOWN);
//        };
//
//        MarkerCallback restSlides = ()->{
//            autonSystems.restSlidesWithIntakeDown();
//            autonSystems.setRollerPower(.5);
//        };
//
//
//        MarkerCallback intake = ()->{
//            autonSystems.robotSubSystems.intake.setState(Intake.State.DOWN);
//            autonSystems.setRollerPower(1);
//        };
//        MarkerCallback transfer = () -> { autonSystems.transferSeq(); };
//
//
//        MarkerCallback rollers_stop = ()->autonSystems.setRollerPower(0);
//        MarkerCallback rollers_on = ()->autonSystems.setRollerPower(1);
//        MarkerCallback rollers_off = ()->autonSystems.setRollerPower(-1);
//
//        MarkerCallback prepForSub = () -> {
//            autonSystems.robotSubSystems.virtualBar.setState(VirtualBar.State.REST);
//            autonSystems.robotSubSystems.slides.setState(Slides.State.REST);
//            autonSystems.robotSubSystems.intake.setState(Intake.State.SEMI);
//        };
//
//
//        drive.setPoseEstimate(startPose);
//
//        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(startPose)
//
//
//                .forward(10)
//                .waitSeconds(.25)
//                .addTemporalMarker(()->{
//
//                    autonSystems.robotSubSystems.intake.setState(Intake.State.DOWN);
//                    autonSystems.setRollerPower(1);
////                    while (true) {
//////                        if (autonSystems.robotSubSystems.colorSensor.isRed()) {break;}
////                    }
//                })
//                .addTemporalMarker(rollers_stop)
//                .lineToLinearHeading(BASKET)
//
//                .waitSeconds(5)
//
//                .build();
//
//
//
//
//
//        telemetry.addData("STATUS", "INITIALIZED");
//        telemetry.update();
//
//
//        waitForStart();
//        if (isStopRequested()) {return;}
//
//        while (opModeIsActive()) {
//
//            drive.followTrajectorySequence(trajectory);
//
//            break;
//        }
//
//    }
//
//
//}
