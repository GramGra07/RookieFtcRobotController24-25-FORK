package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Arrays;
import java.util.List;

@TeleOp
public class SETUP_FOR_MATCH_RUN_BEFORE_EVERY_MATCH extends LinearOpMode {
    private List<DcMotorEx> motorEx;

    @Override
    public void runOpMode() {
        motorEx = Arrays.asList(
                hardwareMap.get(DcMotorEx.class, "frontRightMotor"),
                hardwareMap.get(DcMotorEx.class, "hangmotor1"),
                hardwareMap.get(DcMotorEx.class, "hangmotor")
        );

        waitForStart();

        if (opModeIsActive()) {
            for (DcMotorEx motor : motorEx) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }
}