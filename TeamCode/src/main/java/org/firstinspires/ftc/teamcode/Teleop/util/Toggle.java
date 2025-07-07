package org.firstinspires.ftc.teamcode.Teleop.util;


import com.qualcomm.robotcore.util.ElapsedTime;


public class Toggle {
    private static final double THRESHOLD_MS = 400;
    private final ElapsedTime timer = new ElapsedTime();


    public boolean get(boolean inputValue) {
        // Register the press immediately on the first input and ignore until after threshold
        if (inputValue && timer.milliseconds() > THRESHOLD_MS) {
            timer.reset();
            return true;
        }
        return false;
    }
}
