package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.HardwareConfig;

@TeleOp
public class mecanum extends LinearOpMode {
    HardwareConfig robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareConfig(this, hardwareMap, false);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            robot.dobulk();
        }
    }
}
