//package org.firstinspires.ftc.teamcode.Autonomous.Paths;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.sun.tools.javac.util.List;
//
//import org.firstinspires.ftc.teamcode.Autonomous.AutonSystems;
//import org.firstinspires.ftc.teamcode.Autonomous.Field;
//import org.firstinspires.ftc.teamcode.Autonomous.TrajectoryUtil.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.Roadrunner.AutonMechDrive;
//import org.firstinspires.ftc.teamcode.RobotSystem.Lynx;
//import org.firstinspires.ftc.teamcode.RobotSystem.RobotState;
//import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Blocker;
//import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Color_Sensor;
//import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Extendo;
//import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.VirtualBar;
//import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Claw;
//import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Slides;
//import org.jetbrains.annotations.NotNull;
//
//@Disabled
//@Autonomous(name="5 Sample Auton", group = "Autonomous")
//public class Auton0_5 extends OpMode implements Field {
//
//    private final Pose2d startPose = POS_2;
//
//    AutonMechDrive drive;
//    AutonSystems autonSystems;
//
//    boolean colorDetected;
//
//    Lynx lynx;
//
//
//
//    double SLIDES_DELAY = -.55;
//    double INTAKE_DELAY = .45;
//    double FLIP_WAIT = 0.77;
//    double CLOSE_DELAY = 0.45;
//    double DROP_WAIT = .15;
//    double TRANSFER_WAIT = .8;
//
//    MarkerCallback deposit;
//    MarkerCallback flip;
//    MarkerCallback drop;
//    MarkerCallback close;
//    MarkerCallback restSlidesWithIntakeDown;
//    MarkerCallback intake;
//    MarkerCallback transfer;
//    MarkerCallback rollers_stop;
//    MarkerCallback rollers_on;
//    MarkerCallback rollers_opp;
//    MarkerCallback prepForSub;
//
//    List<TrajectorySequence> trajectories;
//    int trajectoriesCounter = -1;
//
//    boolean COLOR_DETECTION = false;
//    boolean SUB = false;
//
//    static boolean BLUE = false;
//
//    Color_Sensor.State alliance;
//    Color_Sensor.State opp;
//
//
//    @Override
//    public void init() {
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//
//        drive = new AutonMechDrive(hardwareMap);
//        autonSystems = new AutonSystems(this);
//
//        telemetry.addData("Color State", autonSystems.robotSubSystems.colorSensor.getState());
//        telemetry.update();
//
//        lynx = new Lynx(this);
//
//        autonSystems.robotSubSystems.claw.setState(Claw.State.CLOSED);
//        autonSystems.robotSubSystems.blocker.setState(Blocker.State.NOT_BLOCK);
//
//        deposit = () -> {autonSystems.depositSeq();};
//        flip = () -> {
//            autonSystems.flipSeq();
//            autonSystems.robotSubSystems.extendo.setState(Extendo.State.EXTENDED);
//
//        };
//        drop = () -> {autonSystems.dropSeq();};
//        close = () -> {
//            autonSystems.robotSubSystems.blocker.setState(Blocker.State.NOT_BLOCK);
//            autonSystems.robotSubSystems.claw.setState(Claw.State.CLOSED);
//        };
//
//        restSlidesWithIntakeDown = ()->{autonSystems.restSlidesWithIntakeDown();};
//
//        intake = () -> {
//            autonSystems.intakeSeq();
//            autonSystems.setRollerPower(1);
//        };
//
//        transfer = () -> { autonSystems.transferSeq(); };
//
//        rollers_stop = ()->autonSystems.setRollerPower(0);
//        rollers_on = ()->autonSystems.setRollerPower(1);
//        rollers_opp = ()->autonSystems.setRollerPower(-1);
//
//        prepForSub = () -> {
//            autonSystems.robotSubSystems.intake.setRollerPower(0);
//            autonSystems.robotSubSystems.virtualBar.setState(VirtualBar.State.REST);
//            autonSystems.robotSubSystems.claw.setState(Claw.State.OPEN);
//            autonSystems.robotSubSystems.slides.setState(Slides.State.REST);
//            autonSystems.robotSubSystems.intake.setState(Intake.State.SEMI);
//            autonSystems.robotSubSystems.extendo.setState(Extendo.State.RETRACTED);
//        };
//
//        drive.setPoseEstimate(startPose);
//
//        TrajectorySequence toYellowSample1 = drive.trajectorySequenceBuilder(BASKET)
//
//                .lineToLinearHeading(YELLOWSAMPLE1)
//                .addTemporalMarker(intake)
//                .waitSeconds(.25)
//                .turn(Math.toRadians(10))
//                .turn(Math.toRadians(-10))
//                .turn(Math.toRadians(10))
//                .turn(Math.toRadians(-10))
//
//
//                .build();
//
//        TrajectorySequence preloadTraj = drive.trajectorySequenceBuilder(startPose)
//                .addTemporalMarker(deposit)
//                .lineToLinearHeading(BASKET)
//                .UNSTABLE_addTemporalMarkerOffset(SLIDES_DELAY-.25, flip)
////                .addTemporalMarker(flip)
////                .waitSeconds(.15)
//                .addTemporalMarker(drop)
//                .waitSeconds(.25)
//
//                .addTemporalMarker(restSlidesWithIntakeDown)
//                .addTemporalMarker(rollers_on)
//
//                .addTemporalMarker(()->COLOR_DETECTION = true)
//
//                .addTemporalMarker(()->drive.followTrajectorySequenceAsync(toYellowSample1))
//
//                .build();
//
//
//        TrajectorySequence toYellowSample2 = drive.trajectorySequenceBuilder(BASKET)
//                .addTemporalMarker(()->autonSystems.robotSubSystems.blocker.setState(Blocker.State.BLOCK))
//                .addTemporalMarker(()->COLOR_DETECTION = true)
//
//                .lineToLinearHeading(YELLOWSAMPLE2)
//                .addTemporalMarker(intake)
//                .turn(Math.toRadians(5))
//                .turn(Math.toRadians(-15))
//
//                .build();
//        TrajectorySequence toYellowSample3 = drive.trajectorySequenceBuilder(BASKET)
//                .addTemporalMarker(()->autonSystems.robotSubSystems.blocker.setState(Blocker.State.BLOCK))
//                .addTemporalMarker(()->COLOR_DETECTION = true)
//
//                .lineToLinearHeading(YELLOWSAMPLE3)
//                .addTemporalMarker(intake)
//                .turn(Math.toRadians(5))
//                .turn(Math.toRadians(-10))
//                .build();
//
//
//        TrajectorySequence searchTraj = drive.trajectorySequenceBuilder(SUB_LEFT)
//                .addTemporalMarker(()->autonSystems.robotSubSystems.extendo.setState(Extendo.State.EXTENDED))
//
//                .waitSeconds(.25)
//
//                .addTemporalMarker(()->{
//                    COLOR_DETECTION = true;
//                    SUB = true;
//                    autonSystems.robotSubSystems.intake.setRollerPower(1);
//                    autonSystems.robotSubSystems.blocker.setState(Blocker.State.BLOCK);
//                    autonSystems.robotSubSystems.intake.setState(Intake.State.DOWN);
//                })
//
//
//
//                .turn(Math.toRadians(-25))
//                .waitSeconds(.25)
//                .strafeRight(1)
//                .turn(Math.toRadians(25))
//                .waitSeconds(.25)
//                .forward(5)
//                .turn(Math.toRadians(-25))
//                .waitSeconds(.25)
//                .strafeRight(1)
//                .turn(Math.toRadians(25))
//
////                .waitSeconds(.25)
//
//                .addTemporalMarker(()->autonSystems.robotSubSystems.intake.setState(Intake.State.SEMI))
//
//                .forward(10)
//
//                .addTemporalMarker(()->autonSystems.robotSubSystems.intake.setState(Intake.State.DOWN))
//
//                .turn(Math.toRadians(-25))
//                .waitSeconds(.25)
//                .turn(Math.toRadians(25))
//                .waitSeconds(.25)
////                .forward(5)
//                .turn(Math.toRadians(-25))
//                .waitSeconds(.25)
//                .turn(Math.toRadians(25))
//
//
//                .addTemporalMarker(()->drive.breakFollowing())
//
//                .build();
//
//        TrajectorySequence toSubTraj = drive.trajectorySequenceBuilder(BASKET)
//                .addTemporalMarker(()->COLOR_DETECTION = false)
//                .setConstraints(new TrajectoryVelocityConstraint() {
//                    @Override
//                    public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
//                        return 80;
//                    }
//                }, new TrajectoryAccelerationConstraint() {
//                    @Override
//                    public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
//                        return 90;
//                    }
//                })
//
////                .splineToLinearHeading(SUB_LEFT, Math.toRadians(125))
//                .lineToLinearHeading(SUB_LEFT)
//
//                .UNSTABLE_addTemporalMarkerOffset(-2.25, prepForSub)
//
//                .addTemporalMarker(()->drive.followTrajectorySequenceAsync(searchTraj))
//                .build();
//
//        TrajectorySequence getOut = drive.trajectorySequenceBuilder(BASKET)
//
//                .lineToLinearHeading(new Pose2d(30, -50, Math.toRadians(90)))
//                .addTemporalMarker(()->autonSystems.robotState.handleSampleState(RobotState.SAMPLE_STATES.REST))
//
//                .build();
//
//
//        drive.followTrajectorySequenceAsync(preloadTraj);
//
//        trajectories = List.of(toYellowSample2, toYellowSample3, toSubTraj, toSubTraj, getOut);
//    }
//
//    @Override
//    public void init_loop() {
//
//        if (gamepad1.left_bumper) {
//            alliance = Color_Sensor.State.BLUE;
//            opp = Color_Sensor.State.RED;
//        }
//
//        if (gamepad1.right_bumper) {
//            alliance = Color_Sensor.State.RED;
//            opp = Color_Sensor.State.BLUE;
//        }
//
//        telemetry.addData("Alliance ", alliance);
//        telemetry.addData("Opp ", opp);
//
//        telemetry.update();
//    }
//
//
//    public void loop() {
//
//        drive.update();
//
//        if (COLOR_DETECTION && (autonSystems.robotSubSystems.colorSensor.getState() == alliance || autonSystems.robotSubSystems.colorSensor.getState() == Color_Sensor.State.YELLOW)) {
//
//            drive.breakFollowing();
//            COLOR_DETECTION = false;
//
//            TrajectorySequence toDepositTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .addTemporalMarker(transfer)
//                    .addTemporalMarker(rollers_on)
//                    .waitSeconds(TRANSFER_WAIT)
//                    .addTemporalMarker(rollers_stop)
//
//                    .lineToLinearHeading(BASKET)
//                    .UNSTABLE_addTemporalMarkerOffset(SLIDES_DELAY, close)
//                    .addTemporalMarker(deposit)
//                    .addTemporalMarker(flip)
//                    .addTemporalMarker(rollers_opp)
//                    .waitSeconds(FLIP_WAIT)
//                    .addTemporalMarker(drop)
//                    .waitSeconds(DROP_WAIT)
//
//
//                    .addTemporalMarker(restSlidesWithIntakeDown)
//                    .addTemporalMarker(rollers_on)
//
//                    .addTemporalMarker(()->trajectoriesCounter+=1)
//                    .addTemporalMarker(()->drive.followTrajectorySequenceAsync(trajectories.get(trajectoriesCounter)))
//
//                    .build();
//
//
//            TrajectorySequence toDepositSubTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .addTemporalMarker(transfer)
//                    .addTemporalMarker(rollers_on)
//                    .waitSeconds(TRANSFER_WAIT)
//                    .addTemporalMarker(rollers_stop)
////                    .addTemporalMarker(close)
//
////
//                    .lineToLinearHeading(new Pose2d(65, -62, Math.toRadians(137)))
//
////                    .UNSTABLE_addTemporalMarkerOffset(-1.79, rollers_stop)
//                    .UNSTABLE_addTemporalMarkerOffset(-1.5, close)
//
//                    .UNSTABLE_addTemporalMarkerOffset(-1.25, ()->autonSystems.robotSubSystems.blocker.setState(Blocker.State.NOT_BLOCK))
//                    .UNSTABLE_addTemporalMarkerOffset(-1.25, deposit)
//
//                    .UNSTABLE_addTemporalMarkerOffset(-.75, flip)
//                    .addTemporalMarker(rollers_opp)
////                    .waitSeconds(FLIP_WAIT)
//                    .addTemporalMarker(drop)
//                    .waitSeconds(.25)
//
//
//                    .addTemporalMarker(restSlidesWithIntakeDown)
//                    .addTemporalMarker(rollers_on)
//
//                    .addTemporalMarker(()->trajectoriesCounter+=1)
//                    .addTemporalMarker(()->drive.followTrajectorySequenceAsync(trajectories.get(trajectoriesCounter)))
//
//                    .build();
//
//            if (SUB) drive.followTrajectorySequenceAsync(toDepositSubTraj);
//            else drive.followTrajectorySequenceAsync(toDepositTraj);
//
//        }
//
//        if (SUB && autonSystems.robotSubSystems.colorSensor.getState() == opp) {
//            autonSystems.robotSubSystems.blocker.setState(Blocker.State.NOT_BLOCK);
//            autonSystems.robotSubSystems.intake.setRollerPower(1);
//        }
//
//        if (SUB && autonSystems.robotSubSystems.colorSensor.getState() == Color_Sensor.State.NOTHING) {
//            autonSystems.robotSubSystems.blocker.setState(Blocker.State.BLOCK);
//
//        }
//
//        telemetry.addData("Trajectories Counter ", trajectoriesCounter);
//        telemetry.addData("COLOR DETECTION ", COLOR_DETECTION);
//
//        telemetry.addData("Color State", autonSystems.robotSubSystems.colorSensor.getState());
//        telemetry.update();
//
//        lynx.clearCache();
//
//    }
//
//}
//
