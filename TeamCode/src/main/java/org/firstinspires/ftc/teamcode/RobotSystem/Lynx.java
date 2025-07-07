package org.firstinspires.ftc.teamcode.RobotSystem;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.util.List;

public class Lynx extends TomaOpMode {

    private final List<LynxModule> controls;
    public Lynx(OpMode opmode) {
        super(opmode);

        controls = this.opMode.hardwareMap.getAll(LynxModule.class);
    }

    public void setManualMode() {
        controls.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
    }

    public void clearCache() {
        controls.forEach(LynxModule::clearBulkCache);
    }

    public double[] getVoltage() {
        double[] voltages = new double[2];
        int counter = 0;
        for (LynxModule hub : controls) {
            voltages[counter] = hub.getInputVoltage(VoltageUnit.VOLTS);
            counter++;
        }

        return voltages;
    }
}
