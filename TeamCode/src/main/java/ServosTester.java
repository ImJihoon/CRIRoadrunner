import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotSystem.SubSystems;

@Config
@TeleOp
public class ServosTester extends OpMode {

    public static Servo
            intake,
            armLeft,
            armRight,
            claw,
            wrist,
            blocker;

    public static double intakePos = 0, armLeftPos = 0, armRightPos = 0, wristPos = 0, clawPos = 0,blockerPos = 0 ;
    @Override
    public void init() {
        intake = hardwareMap.get(Servo.class, "intake");
        // --------------------------OUTTAKE--------------------------------------
        armLeft = hardwareMap.get(Servo.class, "armLeft");
        armRight = hardwareMap.get(Servo.class, "armRight");


        // --------------------------ARM--------------------------------------
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        blocker = hardwareMap.get(Servo.class, "blocker");
    }

    @Override
    public void loop() {
        intake.setPosition(intakePos);
        armLeft.setPosition(armLeftPos);
        armRight.setPosition(armRightPos);
        wrist.setPosition(wristPos);
        claw.setPosition(clawPos);
        blocker.setPosition(blockerPos);
    }
}
