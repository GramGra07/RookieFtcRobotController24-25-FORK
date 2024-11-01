package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.setpose;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class mecanum_test extends LinearOpMode {
    boolean touchpadpressed = false;
    boolean slowmode = false;
    boolean touchpadwpressed = false;
    boolean clawclose = false;
    boolean clawopen = false;
    boolean hangup = false;
    boolean hangdown = false;
    boolean hangdown_new = false;
    boolean hangup_new = false;
    boolean unloaded = false;
    boolean loaded = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        Servo clawServo = hardwareMap.servo.get("clawServo");
        Servo primeServo = hardwareMap.servo.get("primeServo");
        primeServo.setDirection(Servo.Direction.REVERSE);
        clawServo.setDirection(Servo.Direction.FORWARD);
        DcMotor armMotor1 = hardwareMap.dcMotor.get("armMotor1");
        DcMotor hangmotor = hardwareMap.dcMotor.get("hangmotor");
        DcMotor hangmotor1 = hardwareMap.dcMotor.get("hangmotor1");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        hangmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        hangmotor1.setDirection(DcMotorSimple.Direction.FORWARD);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            clawclose = gamepad1.right_bumper;
            clawopen = gamepad1.left_bumper;
            loaded = gamepad1.b;//Set these differently
            unloaded = gamepad1.x;//Set these differently
            hangup = gamepad1.dpad_up;
            hangdown = gamepad1.dpad_down;
            hangup_new = gamepad1.dpad_up;
            hangdown_new = gamepad1.dpad_down;
            touchpadpressed = gamepad1.touchpad;
            if (touchpadpressed && ! touchpadwpressed) {
                slowmode = ! slowmode;
            }
            double slowmodemultiplier = 0.5;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double multiplier = 1; if (slowmode){multiplier = slowmodemultiplier;};
            double frontLeftPower = ((y - x + rx) / denominator) * multiplier;
            double backLeftPower = ((y + x + rx) / denominator) * multiplier;
            double frontRightPower = ((y - x - rx) / denominator) * multiplier;
            double backRightPower = ((y + x - rx) / denominator) * multiplier;

           double armpower = 0;
           if (gamepad1.right_trigger > 0) {
               armpower = gamepad1.right_trigger;
           } else if (gamepad1.left_trigger > 0) {
               armpower = -gamepad1.left_trigger;
           }
            armpower = Range.clip(armpower, -0.5,0.5);

           double hangpower = 0;
           if (gamepad1.dpad_up) {
               hangpower = 1;
           } else if (gamepad1.dpad_down) {
               hangpower = -1;
           }
            double hangpower1 = 0;
            if (gamepad1.dpad_up) {
                hangpower1 = 1;
            } else if (gamepad1.dpad_down) {
                hangpower1 = -1;
            }
//           double clawpower = 0.5;
           if (gamepad1.right_bumper) {
               setpose(clawServo, 90);   // keep 90 always
           }   else if (gamepad1.left_bumper) {
               setpose(clawServo,  70);  //  keep at 70 increase  to open less
            }
           if (gamepad1.b) {
               setpose(primeServo,  120);// press b first ALWAYS.
           } else if (gamepad1.x) {
               setpose(primeServo, 0);//change degrees in small increments.
           }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            armMotor1.setPower(armpower);
            hangmotor.setPower(hangpower);
            hangmotor1.setPower(hangpower1);
            touchpadwpressed = touchpadpressed;
            telemetry.addData("slowmode",slowmode);
            unloaded = loaded;
            hangup_new = hangup;
            clawopen = clawclose;
            telemetry.addData("claw",gamepad1.right_bumper);
            telemetry.addData("b",gamepad1.b);
            telemetry.addData("up", gamepad1.dpad_up);
            telemetry.update();
        }
    }
}