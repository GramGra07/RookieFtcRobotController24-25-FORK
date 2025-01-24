package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.AutoHardware;

@Autonomous
public class parkautoleftside extends LinearOpMode {
    enum autostates {
        moveright, finish, armup, reset, clawf, resetarm, openclaw, stop, moveright2, moveright3, grab, armup2, transfer

    }

    AutoHardware robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new AutoHardware(this, hardwareMap, new Pose2d(0, 0, 0));


        if (opModeIsActive()) {
            robot.sideWaysEncoderDrive(1, 56);
        }
    }
}
