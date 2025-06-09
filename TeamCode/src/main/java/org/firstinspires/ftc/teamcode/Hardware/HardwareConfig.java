package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSub;
import org.firstinspires.ftc.teamcode.subsystems.ClawSub;
import org.gentrifiedApps.gentrifiedAppsUtil.looptime.LoopTimeController;

public class HardwareConfig {
    boolean slowmode = false;
    Telemetry telemetry = null;
    LinearOpMode opMode = null;
    public ClawSub clawsub = null;
    public ArmSub armSub = null;
    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;
    Limelight3A limelight = null;
    DcMotor armMotor1 = null;
    ElapsedTime elapsedTime = null;
    LoopTimeController loopTimeController = null;

    public HardwareConfig(LinearOpMode om, HardwareMap hwmap, Boolean auto) {
        initrobot(hwmap, om, auto);
    }

    void initrobot(HardwareMap hwmap, LinearOpMode om, Boolean auto) {
        opMode = om;//
        telemetry = om.telemetry;
        clawsub = new ClawSub(hwmap);
        armSub = new ArmSub(hwmap, auto);
        frontLeftMotor = hwmap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hwmap.dcMotor.get("backLeftMotor");
        frontRightMotor = hwmap.dcMotor.get("frontRightMotor");
        backRightMotor = hwmap.dcMotor.get("backRightMotor");
        armMotor1 = hwmap.dcMotor.get("armMotor1");
        // limelight = hwmap.get(Limelight3A.class, "limelight");
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//
            armMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        elapsedTime = new ElapsedTime();
        loopTimeController = new LoopTimeController();
    }

    public void buildtelemetry() {
        telemetry.addData("slowmode", slowmode);
        telemetry.addData("claw", opMode.gamepad1.right_bumper);
        telemetry.addData("b", opMode.gamepad1.b);
        telemetry.addData("up", opMode.gamepad1.dpad_up);
        telemetry.addData("armout", armMotor1.getCurrentPosition());
        telemetry.addData("rightstick", opMode.gamepad2.right_stick_y);
        armSub.telemetry(telemetry);
        loopTimeController.telemetry(telemetry);
        telemetry.update();
    }

    boolean touchpadwpressed = false;

    public void dobulk() {

        double y = opMode.gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = -opMode.gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = -opMode.gamepad1.right_stick_x;
        boolean touchpadpressed = opMode.gamepad1.touchpad;
        if (touchpadpressed && !touchpadwpressed) {
            slowmode = !slowmode;
        }
        touchpadwpressed = touchpadpressed;
        double slowmodemultiplier = 0.5;


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double multiplier = 1;
        if (slowmode) {
            multiplier = slowmodemultiplier;
        }
        double frontLeftPower = ((y + x + rx) / denominator) * multiplier;
        double backLeftPower = ((y - x + rx) / denominator) * multiplier;
        double frontRightPower = ((y - x - rx) / denominator) * multiplier;
        double backRightPower = ((y + x - rx) / denominator) * multiplier;

        double armpower = 0;
        if (opMode.gamepad1.right_trigger > 0) {
            if (armMotor1.getCurrentPosition() < 1500)//
                armpower = opMode.gamepad1.right_trigger;
        } else if (opMode.gamepad1.left_trigger > 0) {//
                armpower = -opMode.gamepad1.left_trigger;
        }
        armpower = Range.clip(armpower, -0.5, 0.5);

//           double clawpower = 0.5;
        if (opMode.gamepad1.right_bumper) {
            clawsub.setClawCLOSE();   // keep 90 always
        } else if (opMode.gamepad1.left_bumper) {
            clawsub.setClawOPEN();  //  keep at 60 increase  to open less
        }
        if (opMode.gamepad1.y) {
            clawsub.setPrimeTOP();// press b first ALWAYS.
        } else if (opMode.gamepad1.a) {
            clawsub.setPrimeBOTTOM();//change degrees in small increments.
        } else if (opMode.gamepad1.b) {
            clawsub.setPrimeMIDDLE();// keep for halfway
        } //drive forward down and open simultaniously
        if (opMode.gamepad1.x) {
            clawsub.setPrimeLOW();

        }
        if (opMode.gamepad2.a) {
            clawsub.setHangBOTTOM();

        }
        if (opMode.gamepad2.b) {
            clawsub.setHangMIDDLE();
        }
        if (opMode.gamepad2.y) {
            clawsub.setHangTOP();
        }
        if (opMode.gamepad2.right_bumper) {
            clawsub.setUClawOPEN();
        } else if (opMode.gamepad2.left_bumper) {
            clawsub.setUClawCLOSE();

        }

        if (opMode.gamepad2.dpad_up) {
            armSub.setUptarget(2200);
        } else if (opMode.gamepad2.dpad_down) {
            armSub.setUptarget(100);//k
        }
        if (opMode.gamepad2.dpad_right) {
            armSub.setUptarget(900);
            clawsub.setHangBOTTOM();
        }
        if (opMode.gamepad2.dpad_left) {
            armSub.setUptarget(280);
            clawsub.setHangMIDDLE();
        }
        if (opMode.gamepad2.right_trigger > 0) {
            clawsub.setSwitchLoad();
        }
        if (opMode.gamepad2.left_trigger > 0) {
            clawsub.setSwitchPrime();
        }
        if (opMode.gamepad2.x) {
            clawsub.setFREAKY();
        }


        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        armMotor1.setPower(armpower);
        clawsub.update();
        loopTimeController.update();
        armSub.update();
        buildtelemetry();

    }
}
