package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.AutoHardware;


@Autonomous
public class RR_AUTO_L extends LinearOpMode {
    AutoHardware robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new AutoHardware(this, hardwareMap, new Pose2d(-14,-63,Math.toRadians(90.0)));
        waitForStart();
        if (opModeIsActive()){
            robot.placePreloadSpeci();
           // robot.placeYellowSample1();
            //robot.placeYellowSample2();
            //robot.parkclose();
        }
    }
}

