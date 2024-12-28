package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.DualEncoder;
import org.firstinspires.ftc.teamcode.util.PIDVals;

public class ArmSub {
    DcMotor hangmotor = null;
    DcMotor hangmotor1 = null;
    DualEncoder dualEncoder = null;
    double power = 0;
    double uptarget = 0;
    PIDFController uppidf = new PIDFController(0,0,0,0);

    //this is where you put all enums and variables
    public ArmSub(HardwareMap hwMap) {
        hangmotor = hwMap.dcMotor.get("hangmotor");
        hangmotor1 = hwMap.dcMotor.get("hangmotor1");

        dualEncoder = new DualEncoder(hangmotor,hangmotor1);

//        hangmotor1.setDirection(DcMotorSimple.Direction.REVERSE);
//        hangmotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
    }
    private double calculatePID(PIDFController controller, double currentVal, double target){
        return Range.clip(
                controller.calculate(
                        currentVal,
                        target
                ),-1,1);

    }
    void power() {
        hangmotor.setPower(power);
        hangmotor1.setPower(power);
    }
    public void setUptarget(double target) {
        uptarget = target;
    }
    private void setPowerup (double target) {
        power = calculatePID(uppidf, dualEncoder.getmost(), target);
    }
    public boolean isUpAtTarget(double tolerance) {
        double most = dualEncoder.getmost();
        return (uptarget - tolerance < most) && (most < uptarget + tolerance);
    }

}