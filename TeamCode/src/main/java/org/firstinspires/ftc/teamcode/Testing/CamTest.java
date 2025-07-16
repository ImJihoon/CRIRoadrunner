package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Vision.NewSampleSearch;

public class CamTest extends LinearOpMode {
    NewSampleSearch search;
    boolean yellow, red, blue;
    boolean a, b, x;
    @Override
    public void runOpMode() throws InterruptedException {
        a = b = x = false;
        search = new NewSampleSearch(hardwareMap, false, false, false);
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addLine(String.format("enabled (r,y,b): %s, %s, %s", red, yellow, blue));
            telemetry.addLine(String.format("delta x, delta y: %s, %s", search.getX(), search.getY()));
            telemetry.update();
            if(gamepad1.a && Boolean.FALSE.equals(a)){
                red = !red;
                search.setColor(red, blue, yellow);
                a = true;
            }else{
                a = false;
            }
            if(gamepad1.b && Boolean.FALSE.equals(b)){
                blue = !blue;
                search.setColor(red, blue, yellow);
                b = true;
            }else{
                b = false;
            }
            if(gamepad1.x && Boolean.FALSE.equals(x)){
                yellow = !yellow;
                search.setColor(red, blue, yellow);
                x = true;
            }else{
                x = false;
            }

        }
    }

}
