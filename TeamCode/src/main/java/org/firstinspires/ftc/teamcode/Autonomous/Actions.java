package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotSystem.DeviceConfig;
import org.firstinspires.ftc.teamcode.RobotSystem.RobotState;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Claw;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Intake;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Slides;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.VirtualBar;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Wrist;

public class Actions {

    AutonSystems autonSystems;

    public Actions(AutonSystems autonSystems) {
        this.autonSystems = autonSystems;
    }

    public MarkerCallback rest() {
        return () -> {
            autonSystems.robotState.handleSampleState(RobotState.SAMPLE_STATES.REST);
            autonSystems.setRollerPower(0);
        };
    }

    public MarkerCallback slidesRest() {
        return () -> {
            autonSystems.robotSubSystems.virtualBar.setState(VirtualBar.State.REST);
            autonSystems.robotSubSystems.slides.setState(Slides.State.REST);
            autonSystems.robotSubSystems.wrist.setState(Wrist.State.REST);
        };
    }

    public MarkerCallback fullExtendo() {
        return () -> {
            autonSystems.robotSubSystems.blocker.setState(Blocker.State.BLOCK);
            autonSystems.robotSubSystems.extendo.setState(Extendo.State.EXTENDED);
//            autonSystems.robotSubSystems.intake.setState(Intake.State.UP);
        };
    }

    public MarkerCallback maxExtendo() {
        return () -> {
            autonSystems.robotSubSystems.blocker.setState(Blocker.State.BLOCK);
            autonSystems.robotSubSystems.extendo.setState(Extendo.State.MAXEXTENDED);
            autonSystems.robotSubSystems.intake.setState(Intake.State.UP);
        };
    }

    public MarkerCallback semiExtendo() {
        return () -> {
            autonSystems.robotSubSystems.blocker.setState(Blocker.State.BLOCK);
            autonSystems.robotSubSystems.extendo.setState(Extendo.State.SEMIEXTENDED);
            autonSystems.robotSubSystems.intake.setState(Intake.State.TRANSFER);
        };
    }
    public MarkerCallback subExtendo(final double y) {
        return () -> {
            autonSystems.robotSubSystems.blocker.setState(Blocker.State.BLOCK);
            if (y >= Extendo.MAXEXTENDED_POS) {
                autonSystems.robotSubSystems.extendo.setState(Extendo.State.MAXEXTENDED);
            }
            else {
                DeviceConfig.extendo.setTargetPosition((int) y);
                DeviceConfig.extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                DeviceConfig.extendo.setPower(1);
            }
            autonSystems.robotSubSystems.intake.setState(Intake.State.UP);
        };
    }
    public MarkerCallback restExtendo() {
        return () -> {
            autonSystems.robotSubSystems.blocker.setState(Blocker.State.BLOCK);
            autonSystems.robotSubSystems.extendo.setState(Extendo.State.RETRACTED);
            autonSystems.robotSubSystems.intake.setState(Intake.State.UP);
        };
    }

    public MarkerCallback intake() {
        return ()-> {
            autonSystems.robotSubSystems.intake.setState(Intake.State.DOWN);
            autonSystems.setRollerPower(1);
        };
    }

    public MarkerCallback transfer() {
        return () -> {
            autonSystems.robotSubSystems.intake.setState(Intake.State.UP);
            autonSystems.robotSubSystems.slides.setState(Slides.State.REST);
            autonSystems.robotSubSystems.virtualBar.setState(VirtualBar.State.REST);
            autonSystems.robotSubSystems.wrist.setState(Wrist.State.REST);
            autonSystems.robotSubSystems.extendo.setState(Extendo.State.RETRACTED);
//            autonSystems.robotSubSystems.blocker.setState(Blocker.State.NOT_BLOCK); //TEST AUTO TRANSFER
            autonSystems.setRollerPower(.75);
        };
    }

    public MarkerCallback closeClaw() {
        return ()-> {
            autonSystems.robotSubSystems.blocker.setState(Blocker.State.NOT_BLOCK);
            autonSystems.robotSubSystems.claw.setState(Claw.State.CLOSED);
        };
    }

    public MarkerCallback slidesToTopBasket() {
        return ()-> {
            autonSystems.robotSubSystems.slides.setState(Slides.State.TOP_BASKET);
        };
    }

    public MarkerCallback prepForDeposit() {
        return ()-> {
            autonSystems.robotSubSystems.blocker.setState(Blocker.State.NOT_BLOCK);
            autonSystems.robotSubSystems.claw.setState(Claw.State.CLOSED);
            autonSystems.robotSubSystems.slides.setState(Slides.State.TOP_BASKET);
        };
    }

    public MarkerCallback flipForDeposit(boolean extend) {
        return ()-> {
            autonSystems.robotSubSystems.virtualBar.setState(VirtualBar.State.DEPOSIT);
            autonSystems.robotSubSystems.wrist.setState(Wrist.State.DEPOSIT);

            if (extend) {
                autonSystems.robotSubSystems.blocker.setState(Blocker.State.BLOCK);
                autonSystems.robotSubSystems.extendo.setState(Extendo.State.SEMIEXTENDED);
//                autonSystems.robotSubSystems.intake.setState(Intake.State.DOWN);
                autonSystems.setRollerPower(1);
            }
        };
    }

    public MarkerCallback openClawforDeposit() { return ()-> autonSystems.robotSubSystems.claw.setState(Claw.State.OPEN);}



    // -------------------------------SPEC------------------------------------

    public MarkerCallback prepToScoreSpecPreload() {
        return ()-> {
            autonSystems.robotSubSystems.claw.setState(Claw.State.CLOSED);
            autonSystems.robotSubSystems.virtualBar.setState(VirtualBar.State.MID);
            autonSystems.robotSubSystems.slides.setState(Slides.State.TOPSPEC_UP);
            autonSystems.robotSubSystems.wrist.setState(Wrist.State.SPEC);
        };
    }

    public MarkerCallback scoreSpecPreload() {
        return ()-> {
            autonSystems.robotSubSystems.claw.setState(Claw.State.CLOSED);
            autonSystems.robotSubSystems.virtualBar.setState(VirtualBar.State.PIVOT);
            autonSystems.robotSubSystems.slides.setState(Slides.State.TOPSPEC);
            autonSystems.robotSubSystems.wrist.setState(Wrist.State.SPEC);
        };
    }



    public MarkerCallback prepToScoreSpec() {
        return ()-> {
            autonSystems.robotSubSystems.claw.setState(Claw.State.CLOSED);
            autonSystems.robotSubSystems.virtualBar.setState(VirtualBar.State.PIVOT);
            autonSystems.robotSubSystems.slides.setState(Slides.State.TOPSPEC);
            autonSystems.robotSubSystems.wrist.setState(Wrist.State.SPEC);
        };
    }

    public MarkerCallback scoreSpec() {
        return ()-> {
            autonSystems.robotSubSystems.claw.setState(Claw.State.CLOSED);
            autonSystems.robotSubSystems.wrist.setState(Wrist.State.SPEC);
            autonSystems.robotSubSystems.slides.setState(Slides.State.TOPSPEC_UP);
            autonSystems.robotSubSystems.virtualBar.setState(VirtualBar.State.MID);
        };
    }

    public MarkerCallback openSpecClaw() {
        return ()-> {
            autonSystems.robotSubSystems.claw.setState(Claw.State.OPEN);
        };
    }

    public MarkerCallback closeSpecClaw() {
        return ()-> {
            autonSystems.robotSubSystems.claw.setState(Claw.State.CLOSED);
        };
    }

    public MarkerCallback specRest() {
        return ()-> {
            autonSystems.robotState.handleSpecimenState(RobotState.SPECIMEN_STATES.START);
        };
    }







}
