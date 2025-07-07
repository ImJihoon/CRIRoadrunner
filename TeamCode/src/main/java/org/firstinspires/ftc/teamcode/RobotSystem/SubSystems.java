package org.firstinspires.ftc.teamcode.RobotSystem;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Blinker;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Camera;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.VirtualBar;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Claw;
//import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Color_Sensor;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Color_Sensor;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Hang;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Intake;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Slides;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Wrist;


public class SubSystems {

    public Drivetrain drivetrain;
    public Extendo extendo;
    public Intake intake;
    public VirtualBar virtualBar;
    public Slides slides;
    public Claw claw;
    public Hang hang;
    public Blocker blocker;
    public Camera camera;
    public Wrist wrist;

    public Color_Sensor colorSensor;
    public Blinker blinker;


    public SubSystems(OpMode opmode, boolean isReset) {


        DeviceConfig.init(opmode);

        if (isReset) {
            DeviceConfig.slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            DeviceConfig.slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            DeviceConfig.extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }



        // ALREADY DONE IN AUTO MODE!!!!! ToDO: FIX THIS

        // ----------Create instances of Subsystems------------------
        drivetrain = new Drivetrain(opmode);
        extendo = new Extendo(opmode);
        intake = new Intake(opmode);
        virtualBar = new VirtualBar(opmode);
        slides = new Slides(opmode);
        claw = new Claw(opmode);

        colorSensor = new Color_Sensor(opmode);

        hang = new Hang(opmode);

        blocker = new Blocker(opmode);

        camera = new Camera(opmode);

        wrist = new Wrist(opmode);

        blinker = new Blinker(opmode);


        // -------------------INIT--------------------------------
        drivetrain.init();
        extendo.init();
        intake.init();
        virtualBar.init();
        slides.init();
        claw.init();
        blocker.init();
        wrist.init();
        camera.init();

    }


}
