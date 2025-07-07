package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotSystem.DeviceConfig;
import org.firstinspires.ftc.teamcode.RobotSystem.RobotState;
import org.firstinspires.ftc.teamcode.RobotSystem.SubSystems;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Claw;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Intake;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Slides;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.VirtualBar;

public class AutonSystems {

    public SubSystems robotSubSystems;
    public RobotState robotState;
    private OpMode opMode;


    public AutonSystems(OpMode opMode) {

        this.opMode = opMode;
        robotSubSystems = new SubSystems(opMode, true);
        robotState = new RobotState(opMode, robotSubSystems);

        robotSubSystems.claw.setState(Claw.State.CLOSED);

    }


    public void setRollerPower(double power) { robotSubSystems.intake.setRollerPower(power); }


}
