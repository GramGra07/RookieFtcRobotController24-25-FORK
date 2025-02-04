package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.AutoHardware;


@Autonomous
public class AUTOTESTING extends LinearOpMode {
    AutoHardware robot = null;
    ///
    ///
    ///
    ///
    ///
    ///
    ///THIS FILE IS ONLY FOR TESTING AUTONOMOUS OUT NO AUTO RUNS ARE TO BE STORED HERE!!!!!!!!!
    ///
    ///
    ///
    ///
    ///
    @Override//
    public void runOpMode() throws InterruptedException {
        robot = new AutoHardware(this, hardwareMap, new Pose2d(-36,-63,Math.toRadians(90.0)));
        waitForStart();
        if (opModeIsActive()){
            robot.preloadsample();
            robot.sample();
        }
    }
}

