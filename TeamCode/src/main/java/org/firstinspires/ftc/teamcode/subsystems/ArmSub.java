package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDVals;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.generics.pid.PIDFCoefficients;
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.motor.DualEncoder;
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.motor.PIDMotor;
import org.gentrifiedApps.gentrifiedAppsUtil.motion.controllers.PIDFController;

import java.util.List;

public class ArmSub {
    PIDMotor hangMotor = null;
    PIDMotor hangMotor1 = null;
    public static DualEncoder dualEncoder = null;
    PIDFCoefficients uppidf = new PIDFCoefficients(0, 0, 0, 0);
    PIDFCoefficients uppidf1 = new PIDFCoefficients(0, 0, 0, 0);

    boolean auto;

    public ArmSub(HardwareMap hwMap, Boolean auto) {
        this.auto = auto;
        hangMotor = new PIDMotor(hwMap,"hangMotor",uppidf);
        hangMotor1 = new PIDMotor(hwMap,"hangMotor1",uppidf1).currentReversed();
        dualEncoder = new DualEncoder(hangMotor, hangMotor1);
    }

    public void update() {
        updatePID();
        power();
    }

    // this is where you put your update functions to switch between states
    public void telemetry(Telemetry telemetry) {
//        dualEncoder.telemetry(telemetry);
    }

    private void updatePID() {
        hangMotor.setPIDF(
                PIDVals.upco.p,
                PIDVals.upco.i,
                PIDVals.upco.d,
                PIDVals.upco.f);
        hangMotor1.setPIDF(
                PIDVals.upco1.p,
                PIDVals.upco1.i,
                PIDVals.upco1.d,
                PIDVals.upco1.f);
    }
    void power() {
        hangMotor.setPIDPower();
        hangMotor1.setPIDPower();
    }

    public void setUptarget(double target) {
        hangMotor.setTarget(target);
        hangMotor1.setTarget(target);
    }

    public boolean isUpAtTarget(double tolerance) {
        double most = dualEncoder.getMost();
        double uptarget = hangMotor.getTarget();
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