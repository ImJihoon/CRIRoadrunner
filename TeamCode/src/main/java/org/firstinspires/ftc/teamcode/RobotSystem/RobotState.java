package org.firstinspires.ftc.teamcode.RobotSystem;


import com.acmerobotics.dashboard.config.Config;
//import com.google.ar.core.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.VirtualBar;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Claw;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Intake;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Slides;
import org.firstinspires.ftc.teamcode.RobotSystem.subsystems.Wrist;

@Config
public class RobotState extends TomaOpMode {

    public Mode currentMode = Mode.SAMPLE_MODE;

    public SAMPLE_STATES currentSampleState = SAMPLE_STATES.REST;
    public SPECIMEN_STATES currentSpecimenState = SPECIMEN_STATES.START;

    public SubSystems robotSubSystems;

    public boolean LOW_BASKET = false;
    public boolean LOW_BAR = false;

    public static boolean SEMIMODE = true;
    public boolean block = true;

    public enum Mode {
        SAMPLE_MODE,
        SPECIMEN_MODE
    }

    public RobotState(OpMode opMode, SubSystems robotSubSystems){
        super(opMode);
        this.robotSubSystems = robotSubSystems;
    }

    public void toggleMode() {
        rumble();
        currentMode = (currentMode == Mode.SAMPLE_MODE) ? Mode.SPECIMEN_MODE : Mode.SAMPLE_MODE;
        resetStates();
    }



    private void resetStates() {
        if (currentMode == Mode.SAMPLE_MODE) {
            handleSampleState(SAMPLE_STATES.REST);
        } else {
            handleSpecimenState(SPECIMEN_STATES.START);
        }
    }

    public void nextState() {
        rumble();
        if (currentMode == Mode.SAMPLE_MODE) {
            currentSampleState = getNextSampleState();
        } else {
            currentSpecimenState = getNextSpecimenState();
        }
    }

    public void previousState() {
        rumble();
        if (currentMode == Mode.SAMPLE_MODE) {
            currentSampleState = getPreviousSampleState();
        } else {
            currentSpecimenState = getPreviousSpecimenState();
        }
    }

    public void handleRobotState() {
        if (currentMode == Mode.SAMPLE_MODE) {
            handleSampleState(currentSampleState);
        } else {
            handleSpecimenState(currentSpecimenState);
        }
    }



    //---------------------------------SAMPLE LOGIC--------------------------------------

    public enum SAMPLE_STATES {
        REST, INTAKE, TRANSFER, DEPOSIT, DROP
    }

    public void handleSampleState(SAMPLE_STATES state) {
        if (currentSampleState != state) {
            currentSampleState = state;
        }

        switch (state) {
            case REST:
                sampleRest();
                break;

            case INTAKE:
                intakeSequence();
                break;

            case TRANSFER:
                transferSequence();
                break;

//            case GRAB:
//                grabSequence();
//                break;

            case DEPOSIT:
                depositSequence();
                break;

            case DROP:
                dropSequence();
                break;
        }
    }

    private void sampleRest() {
        robotSubSystems.blocker.setState(Blocker.State.BLOCK);
        robotSubSystems.claw.setState(Claw.State.OPEN);
        robotSubSystems.intake.setState(Intake.State.UP);
        robotSubSystems.virtualBar.setState(VirtualBar.State.REST);
        robotSubSystems.wrist.setState((Wrist.State.REST));
        robotSubSystems.slides.setState(LOW_BASKET ? Slides.State.RESTSLOW : Slides.State.REST);
        robotSubSystems.extendo.setState(Extendo.State.RETRACTED);
        Drivetrain.SLOW = false;

    }

    private void intakeSequence() {
            robotSubSystems.blocker.setState(Blocker.State.BLOCK);
            robotSubSystems.claw.setState(Claw.State.OPEN);
            robotSubSystems.extendo.setState(SEMIMODE ? Extendo.State.LESSEXTEND : Extendo.State.MAXEXTENDED);
        if (robotSubSystems.extendo.getSlidePos() < Extendo.MIN_INTAKE_POS) {
            robotSubSystems.intake.setState(Intake.State.UP);
        }
        else if (robotSubSystems.extendo.getSlidePos() >= Extendo.MIN_INTAKE_POS) {
            robotSubSystems.intake.setState(SEMIMODE ? Intake.State.SEMI : Intake.State.DOWN);
        }
//            Drivetrain.SLOW = true;
    }

    private void transferSequence() {
//        if (block) robotSubSystems.blocker.setState(Blocker.State.BLOCK);

        robotSubSystems.slides.setState(Slides.State.REST);
        robotSubSystems.extendo.setState(Extendo.State.RETRACTED);
        robotSubSystems.virtualBar.setState(VirtualBar.State.REST);
        robotSubSystems.wrist.setState((Wrist.State.REST));

        if (robotSubSystems.extendo.getSlidePos() >= Extendo.MIN_INTAKE_POS) {
            robotSubSystems.intake.setState(Intake.State.UP);
        }

        if (robotSubSystems.extendo.getSlidePos() <= 100) {
            robotSubSystems.blocker.setState(Blocker.State.NOT_BLOCK); //TEST AUTO TRANSFER
        }
        if (robotSubSystems.extendo.getSlidePos() <= 15) {
//            block = false;
            robotSubSystems.intake.setState(Intake.State.TRANSFER);
            robotSubSystems.claw.setState(Claw.State.CLOSED); //TEST AUTO TRANSFER
        }

//        Drivetrain.SLOW = false;
    }

//    private void grabSequence() {
//        robotSubSystems.intake.setState(Intake.State.TRANSFER);
//        robotSubSystems.blocker.setState(Blocker.State.NOT_BLOCK);
//        robotSubSystems.claw.setState(Claw.State.CLOSED);
//        robotSubSystems.virtualBar.setState(VirtualBar.State.REST);
//        robotSubSystems.wrist.setState(Wrist.State.REST);
//
//        Drivetrain.SLOW = false;
//
//    }


    private void depositSequence() {
        robotSubSystems.blocker.setState(Blocker.State.NOT_BLOCK);
        robotSubSystems.claw.setState(Claw.State.CLOSED);
        robotSubSystems.intake.setState(Intake.State.UP);
        robotSubSystems.slides.setState(LOW_BASKET ? Slides.State.BOTTOM_BASKET : Slides.State.TOP_BASKET);

        if (!LOW_BASKET) {
            if (robotSubSystems.slides.getSlidePos() >= Slides.TOPBASKET_POS / 2.0) {
                robotSubSystems.virtualBar.setState(VirtualBar.State.DEPOSIT);
                robotSubSystems.wrist.setState(Wrist.State.DEPOSIT);
            }
        }
        else {
                if (robotSubSystems.slides.getSlidePos() >= Slides.BOTTOMBASKET_POS / 2.0) {
                    robotSubSystems.virtualBar.setState(VirtualBar.State.DEPOSIT);
                    robotSubSystems.wrist.setState(Wrist.State.DEPOSIT);
                }
        }


//        Drivetrain.SLOW = true;
    }

    private void dropSequence() {
        robotSubSystems.slides.setState(LOW_BASKET ? Slides.State.BOTTOM_BASKET : Slides.State.TOP_BASKET);
        robotSubSystems.virtualBar.setState(VirtualBar.State.DEPOSIT);
        robotSubSystems.wrist.setState(Wrist.State.DEPOSIT);

        robotSubSystems.claw.setState(Claw.State.OPEN);

//        Drivetrain.SLOW = true;

    }

    //---------------------------------SAMPLE LOGIC--------------------------------------




    //--------------------------------SPECIMEN LOGIC-------------------------------------
    public enum SPECIMEN_STATES {
        START, GRAB, PIVOT, SCORE
    }

    public void handleSpecimenState(SPECIMEN_STATES state) {
        if (currentSpecimenState != state) {
            currentSpecimenState = state;
        }

        switch (state) {
            case START:
                startSequence();
                break;

            case GRAB:
                grabSpecSequence();
                break;

            case PIVOT:
                pivotSequence();
                break;

            case SCORE:
                scoreSequence();


        }
    }

    private void startSequence() {

        robotSubSystems.intake.setState(Intake.State.UP);
        robotSubSystems.slides.setState(Slides.State.REST);
        robotSubSystems.virtualBar.setState(VirtualBar.State.SPEC_PICKUP);
        robotSubSystems.claw.setState(Claw.State.OPEN);
        robotSubSystems.wrist.setState(Wrist.State.GET_SPEC);


    }

    private void grabSpecSequence() {
        robotSubSystems.wrist.setState(Wrist.State.GET_SPEC);
        robotSubSystems.claw.setState(Claw.State.CLOSED);
    }

    private void pivotSequence() {
        robotSubSystems.claw.setState(Claw.State.CLOSED);
        robotSubSystems.virtualBar.setState(VirtualBar.State.PIVOT);
        robotSubSystems.slides.setState(LOW_BAR ? Slides.State.BOTTOMSPEC : Slides.State.TOPSPEC);
        robotSubSystems.wrist.setState(Wrist.State.SPEC);
//        robotSubSystems.intake.setState(Intake.State.SEMI);

    }

    private void scoreSequence() {
//        robotSubSystems.intake.setState(Intake.State.SEMI);
        robotSubSystems.wrist.setState(Wrist.State.SPEC);
//        robotSubSystems.wrist.setState(Wrist.State.DOWN);

        robotSubSystems.slides.setState(Slides.State.TOPSPEC_UP);
        robotSubSystems.virtualBar.setState(VirtualBar.State.MID);

        if (robotSubSystems.slides.getSlidePos() >= Slides.TOPSPEC_UP_POS-50) {
            robotSubSystems.claw.setState(Claw.State.OPEN);
        }
    }




    //--------------------------------///SPECIMEN LOGIC-------------------------------------


    // -----------------------------------HELPER METHODS-------------------------------------
    public void rumble() {
        opMode.gamepad2.rumble(300);
    }

    private SAMPLE_STATES getNextSampleState() {
        int nextOrdinal = (currentSampleState.ordinal() + 1) % SAMPLE_STATES.values().length;
        return SAMPLE_STATES.values()[nextOrdinal];
    }

    private SAMPLE_STATES getPreviousSampleState() {
        int prevOrdinal = (currentSampleState.ordinal() - 1 + SAMPLE_STATES.values().length) % SAMPLE_STATES.values().length;
        return SAMPLE_STATES.values()[prevOrdinal];
    }

    private SPECIMEN_STATES getNextSpecimenState() {
        int nextOrdinal = (currentSpecimenState.ordinal() + 1) % SPECIMEN_STATES.values().length;
        return SPECIMEN_STATES.values()[nextOrdinal];
    }

    private SPECIMEN_STATES getPreviousSpecimenState() {
        int prevOrdinal = (currentSpecimenState.ordinal() - 1 + SPECIMEN_STATES.values().length) % SPECIMEN_STATES.values().length;
        return SPECIMEN_STATES.values()[prevOrdinal];
    }

    public void updateTelemetry() {

        // General info
        opMode.telemetry.addData("Current Mode", currentMode);
        if (currentMode == Mode.SAMPLE_MODE) {
            opMode.telemetry.addData("Current Sample State", currentSampleState);
            opMode.telemetry.addData("Low Basket", LOW_BASKET ? "Enabled" : "Disabled");
        } else {
            opMode.telemetry.addData("Current Specimen State", currentSpecimenState);
            opMode.telemetry.addData("Low Bar", LOW_BAR);
        }

        // Subsystem info
        opMode.telemetry.addLine("Subsystem States:");
        opMode.telemetry.addData("Bucket State", robotSubSystems.virtualBar.currentState);
        opMode.telemetry.addData("Slides State", robotSubSystems.slides.currentState);
        opMode.telemetry.addData("Intake State", robotSubSystems.intake.currentState);
        opMode.telemetry.addData("Extendo State", robotSubSystems.extendo.currentState);
        opMode.telemetry.addData("Claw State", robotSubSystems.claw.currentState);
        opMode.telemetry.addData("Blocker State", robotSubSystems.blocker.currentState);

        opMode.telemetry.addData("DriveTrain Mode", Drivetrain.SLOW);

        opMode.telemetry.addData("Motor Extendo", DeviceConfig.extendo.getCurrentPosition());

        opMode.telemetry.update();
    }


}
