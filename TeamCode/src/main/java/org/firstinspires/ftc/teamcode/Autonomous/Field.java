package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public interface Field {


    // TIMINGS

    // Coordinates
    Pose2d POS_1 = new Pose2d(63.5, 8, Math.toRadians(180)); // SPEC AUTON
    Pose2d POS_2 = new Pose2d(65.5, -41.5, Math.toRadians(90)); // near near basket

    Pose2d SUB_LEFT = new Pose2d(5, -24, Math.toRadians(90));
    Pose2d SUBBOTTOM = new Pose2d(4, -16.5, Math.toRadians(90));


    Pose2d PRELOAD_SUBRIGHT = new Pose2d(24, -1, Math.toRadians(180));
    Pose2d SUBRIGHT = new Pose2d(25, -3, Math.toRadians(180));

    Pose2d SUBRIGHT2 = new Pose2d(25.5, -1, Math.toRadians(180));
    Pose2d SUBRIGHT3 = new Pose2d(25.5, -3, Math.toRadians(180));
    Pose2d SUBRIGHT4 = new Pose2d(25.5, 1, Math.toRadians(180));
    Pose2d SUBRIGHT5 = new Pose2d(25.5, -1, Math.toRadians(180));

    Pose2d BASKET = new Pose2d(63.5, -63.5, Math.toRadians(135));

    Pose2d YELLOWSAMPLE1 = new Pose2d(50, -53.5, Math.toRadians(175));
    Pose2d YELLOWSAMPLE2 = new Pose2d(50, -60, Math.toRadians(185));
    Pose2d YELLOWSAMPLE3 = new Pose2d(38, -48, Math.toRadians(250));

    Pose2d OBSERV1 = new Pose2d(56, 32, Math.toRadians(35));
    Pose2d OBSERV2 = new Pose2d(56, 30, Math.toRadians(15));

    Pose2d COLORSAMPLE1 = new Pose2d(10, 50, Math.toRadians(180));
    Pose2d COLORSAMPLE2 = new Pose2d(50, 42, Math.toRadians(140)); //145 angle
    Pose2d COLORSAMPLE3 = new Pose2d(48, 48, Math.toRadians(135));

    Pose2d STOP = new Pose2d(55, 42, Math.toRadians(180));

    Pose2d SPEC_PICKUP = new Pose2d(66, 35, Math.toRadians(180));


}
