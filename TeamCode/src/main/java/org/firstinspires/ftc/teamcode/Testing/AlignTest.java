package org.firstinspires.ftc.teamcode.Testing;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.TrajectoryUtil.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Roadrunner.AutonMechDrive;
import org.firstinspires.ftc.teamcode.RobotSystem.RobotState;
import org.firstinspires.ftc.teamcode.RobotSystem.SubSystems;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.RotatedRect;

import java.io.CharArrayWriter;
import java.sql.Blob;

@Config
@TeleOp(name="Align Test")
public class AlignTest extends LinearOpMode {

    SubSystems robotSubSystems;
    AutonMechDrive drive;
    private static final double IMG_CENTER_X = 160; // For 320px wide resolution
    private static final double IMG_CENTER_Y = 120;
    private static final double PIXELS_PER_INCH = 30; // Camera-specific scaling
    private static final double MIN_ERROR = 5; // pixels
    private static final double MAX_STRAFE = 24; // inches


    public static boolean ALIGN = false;

    @Override
    public void runOpMode() throws InterruptedException {

        robotSubSystems = new SubSystems(this, true);
        drive = new AutonMechDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d());

        robotSubSystems.camera.init();


        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {

            robotSubSystems.camera.testCamera();

            double error = getAlign();

            TrajectorySequence strafe = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeRight(error)
                    .build();

            drive.followTrajectorySequence(strafe);

//            drive.update();

        }
    }

    public double getAlign() {
        ColorBlobLocatorProcessor.Blob blob = robotSubSystems.camera.getBestBlob();

//        if (blob == null) return;

        RotatedRect boxFit = blob.getBoxFit();

        // 2. Calculate errors (assuming 320x240 resolution)
        double imgCenterX = 160;
        double errorX = (boxFit.center.x - IMG_CENTER_X) / PIXELS_PER_INCH; // Convert to inches
        double errorY = (boxFit.center.y - IMG_CENTER_Y) / PIXELS_PER_INCH;

        telemetry.addData("Box Center", "X: %d, Y: %d",
                (int)boxFit.center.x, (int)boxFit.center.y);
        telemetry.addData("Box Size", "%.1f x %.1f",
                boxFit.size.width, boxFit.size.height);

        telemetry.addData("Box Angle", "%.1f°", boxFit.angle);


        telemetry.addData("Error X", errorX);
        telemetry.addData("Error Y", errorY);
        telemetry.update();

        // 3. Only move if error is significant
        if (Math.abs(errorX) > MIN_ERROR/PIXELS_PER_INCH) {
            // Limit maximum strafe distance
            errorX = Math.max(-MAX_STRAFE, Math.min(MAX_STRAFE, errorX));
        }
        else errorX = 0;

        return errorX;


    }

}
