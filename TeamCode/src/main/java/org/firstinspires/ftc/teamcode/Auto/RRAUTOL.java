package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.AutoHardware;


@Autonomous
public class RRAUTOL extends LinearOpMode {
    AutoHardware robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new AutoHardware(this, hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();
        if (opModeIsActive()){
            robot.placePreloadSpeciL();
            robot.grabSpikeSample();
            robot.grabSpikeSample2();
            robot.grabSpikeSample3();
            robot.parkclose();
        }
    }
}

