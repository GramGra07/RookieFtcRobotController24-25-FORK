package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.AutoHardware;
import org.firstinspires.ftc.teamcode.util.PIDVals;
import org.gentrifiedApps.statemachineftc.StateMachine;

@Autonomous
public class parkautorightside extends LinearOpMode {
    enum autostates {
        moveright,finish, armup,reset,clawf,resetarm, openclaw, stop,moveright2, moveright3,grab,armup2,transfer

    }

    AutoHardware robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new AutoHardware(this, hardwareMap);


        if (opModeIsActive()) {
            robot.sideWaysEncoderDrive(1,56);
        }
    }
}
