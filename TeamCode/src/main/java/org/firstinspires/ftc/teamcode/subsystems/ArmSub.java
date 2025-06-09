package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.DualEncoder;
import org.firstinspires.ftc.teamcode.util.PIDVals;

import java.util.List;

public class ArmSub {
    DcMotor hangmotor = null;
    DcMotor hangmotor1 = null;
    public static DualEncoder dualEncoder = null;
    double power = 0;
    double power1 = 0;
    double uptarget = 0;
    PIDFController uppidf = new PIDFController(0, 0, 0, 0);
    PIDFController uppidf1 = new PIDFController(0, 0, 0, 0);

    boolean auto;

    public ArmSub(HardwareMap hwMap, Boolean auto) {
        this.auto = auto;
        hangmotor = hwMap.dcMotor.get("hangmotor");
        hangmotor1 = hwMap.dcMotor.get("hangmotor1");
        dualEncoder = new DualEncoder(hangmotor, hangmotor1);
    }

    public void update() {
        updatePID();
        setPowerup(uptarget);
        power();
    }

    // this is where you put your update functions to switch between states
    public void telemetry(Telemetry telemetry) {
        dualEncoder.telemetry(telemetry);
    }

    private void updatePID() {
        uppidf.setPIDF(
                PIDVals.upco.p,
                PIDVals.upco.i,
                PIDVals.upco.d,
                PIDVals.upco.f
        );
        uppidf1.setPIDF(
                PIDVals.upco1.p,
                PIDVals.upco1.i,
                PIDVals.upco1.d,
                PIDVals.upco1.f
        );
    }

    private double calculatePID(PIDFController controller, double currentVal, double target) {
        return Range.clip(
                controller.calculate(
                        currentVal,
                        target
                ), -1, 1);

    }

    void power() {
        hangmotor.setPower(power);
        hangmotor1.setPower(power1);
    }

    public void setUptarget(double target) {
        uptarget = target;
    }

    private void setPowerup(double target) {
        power = calculatePID(uppidf, hangmotor.getCurrentPosition(), target);
        power1 = calculatePID(uppidf1, -hangmotor1.getCurrentPosition(), target);
    }

    public boolean isUpAtTarget(double tolerance) {
        double most = dualEncoder.getmost();
        return (uptarget - tolerance < most) && (most < uptarget + tolerance);
    }


    // custom rr implementation
    public Action armAction(List<Runnable> funcs) {
        return new ArmSub.ArmAction(funcs);
    }
    class ArmAction implements Action {
        List<Runnable> funcs;

        public ArmAction(List<Runnable> funcs) {
            this.funcs = funcs;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            for (Runnable func : funcs) {
                func.run();
            }
            return false;
        }
    }
}