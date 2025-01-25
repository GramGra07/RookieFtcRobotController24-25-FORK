package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.util.setpose;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.ServoUtil;

import java.util.List;

public class ClawSub {
    private final Servo upclaw;
    private final Servo hangservo;
    private final Servo hangservo1;
    private final Servo primeservo;
    private final Servo clawservo;
    private final Servo switchservo;
    private final Servo switchservo1;

    public enum ClawState {OPEN, CLOSE, IDLE}

    private ClawState ClawStateVar = ClawState.IDLE;

    public void setClawOPEN() {
        ClawStateVar = ClawState.OPEN;
    }

    public void setClawCLOSE() {
        ClawStateVar = ClawState.CLOSE;
    }

    public void setClawIDLE() {
        ClawStateVar = ClawState.IDLE;
    }

    public enum PrimeState {TOP, MIDDLE, LOW, BOTTOM, IDLE}

    private PrimeState PrimeStateVar = PrimeState.IDLE;

    public void setPrimeTOP() {
        PrimeStateVar = PrimeState.TOP;
    }

    public void setPrimeMIDDLE() {
        PrimeStateVar = PrimeState.MIDDLE;
    }

    public void setPrimeLOW() {
        PrimeStateVar = PrimeState.LOW;
    }

    public void setPrimeBOTTOM() {
        PrimeStateVar = PrimeState.BOTTOM;
    }

    public void setPrimeIDLE() {
        PrimeStateVar = PrimeState.IDLE;
    }

    public enum UpclawState {OPEN, CLOSE, IDLE}

    private UpclawState UpclawStateVar = UpclawState.IDLE;

    public void setUClawOPEN() {
        UpclawStateVar = UpclawState.OPEN;
    }

    public void setUClawCLOSE() {
        UpclawStateVar = UpclawState.CLOSE;
    }

    public void setUClawIDLE() {
        UpclawStateVar = UpclawState.IDLE;
    }

    public enum HangState {TOP, MIDDLE, FREAK, BOTTOM, IDLE}

    private HangState HangStateVar = HangState.IDLE;

    public void setHangTOP() {
        HangStateVar = HangState.TOP;
    }

    public void setHangMIDDLE() {
        HangStateVar = HangState.MIDDLE;
    }

    public void setHangBOTTOM() {
        HangStateVar = HangState.BOTTOM;
    }

    public void setFREAKY() {
        HangStateVar = HangState.FREAK;
    }

    public void setHangIDLE() {
        HangStateVar = HangState.IDLE;
    }

    public enum SwitchState {PRIME, LOAD, IDLE}

    private SwitchState SwitchStateVar = SwitchState.IDLE;

    public void setSwitchPrime() {
        SwitchStateVar = SwitchState.PRIME;
    }

    public void setSwitchLoad() {
        SwitchStateVar = SwitchState.LOAD;
    }

    public void setSwitchIdle() {
        SwitchStateVar = SwitchState.IDLE;
    }

    //this is where you put all enums and variables
    public ClawSub(HardwareMap hwMap) {
        upclaw = hwMap.get(Servo.class, "upclawservo");
        hangservo = hwMap.get(Servo.class, "hangservo");
        hangservo1 = hwMap.get(Servo.class, "hangservo1");
        primeservo = hwMap.get(Servo.class, "primeServo");
        clawservo = hwMap.get(Servo.class, "clawServo");
        primeservo.setDirection(Servo.Direction.REVERSE);
        clawservo.setDirection(Servo.Direction.FORWARD);
        hangservo1.setDirection(Servo.Direction.FORWARD);
        hangservo.setDirection(Servo.Direction.REVERSE);
        switchservo = hwMap.get(Servo.class, "switchServo");
        switchservo.setDirection(Servo.Direction.REVERSE);
        switchservo1 = hwMap.get(Servo.class, "switchServo1");
        switchservo1.setDirection(Servo.Direction.FORWARD);

    }

    public void update() {
        // this is where you put your state machines and all power functions (call this in our main code)


        switch (ClawStateVar) {
            case OPEN:
                setpose(clawservo, ServoUtil.clawopen);//actually close increase to close more

                break;
            case CLOSE:
                setpose(clawservo, ServoUtil.clawclose);//actually open decrease to open more
                break;
            case IDLE:

                break;
        }
        switch (SwitchStateVar) {
            case PRIME:
                setpose(switchservo, ServoUtil.switchprime);
                setpose(switchservo1, ServoUtil.switchprime1);
                break;
            case LOAD:
                setpose(switchservo, ServoUtil.switchload);
                setpose(switchservo1, ServoUtil.switchload1);
                break;

        }
        switch (PrimeStateVar) {
            case TOP:
                setpose(primeservo, ServoUtil.primetop);
                break;
            case MIDDLE:
                setpose(primeservo, ServoUtil.primemiddle);
                break;
            case BOTTOM:
                setpose(primeservo, ServoUtil.primebottom);
                break;
            case IDLE:

                break;
            case LOW:
                setpose(primeservo, ServoUtil.primelow);
                break;
        }
        switch (UpclawStateVar) {
            case OPEN:
                setpose(upclaw, ServoUtil.upclawopen);
                break;
            case CLOSE:
                setpose(upclaw, ServoUtil.upclawclose);
                break;
            case IDLE:

                break;
        }
        switch (HangStateVar) {
            case TOP:
                setpose(hangservo, ServoUtil.hangtop);//keep
                setpose(hangservo1, ServoUtil.hangtop);
                break;
            case MIDDLE:
                setpose(hangservo, ServoUtil.hangmiddle);
                setpose(hangservo1, ServoUtil.hangmiddle);//keep
                break;
            case FREAK:
                setpose(hangservo, ServoUtil.hangfreak);
                setpose(hangservo1, ServoUtil.hangfreak);
                break;
            case BOTTOM:
                setpose(hangservo, ServoUtil.hangbottom);
                setpose(hangservo1, ServoUtil.hangbottom);
                break;
            case IDLE:

                break;
        }


    }

    // this is where you put your update functions to switch between states
    public void telemetry(Telemetry telemetry) {
        // add telemetry data here

    }

    //    Actions.runBlocking(
//    clawAction(
//            List.of(
//            () -> clawSub.openClaw();
//                        )
//                                )
//                                );
    public Action clawAction(ClawSub clawSub,List<Runnable> funcs) {
        return new ClawAction(clawSub,funcs);
    }

    class ClawAction implements Action {
        List<Runnable> funcs;
        private ClawSub clawSub;

        public ClawAction(ClawSub clawSub,List<Runnable> funcs) {
            this.funcs = funcs;
            this.clawSub = clawSub;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            for (Runnable func : funcs) {
                func.run();
            }
            clawSub.update();// removes the need for the update to be run after simply updating a claw

            return false;
        }
    }
}