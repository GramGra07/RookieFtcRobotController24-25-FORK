package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.setpose;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class ClawSub {
    private Servo upclaw;
    private Servo hangservo;
    private Servo hangservo1;
    private Servo primeservo;
    private Servo clawservo;
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
    public enum PrimeState {TOP, MIDDLE, BOTTOM, IDLE}

    private PrimeState PrimeStateVar = PrimeState.IDLE;

    public void setPrimeTOP() {
        PrimeStateVar = PrimeState.TOP;
    }

    public void setPrimeMIDDLE() {
        PrimeStateVar = PrimeState.MIDDLE;
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
    public enum HangState {TOP, MIDDLE, BOTTOM, IDLE}

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

    public void setHangIDLE() {
        HangStateVar = HangState.IDLE;
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
        hangservo1.setDirection(Servo.Direction.REVERSE);
        hangservo.setDirection(Servo.Direction.FORWARD);

    }

    public void update() {
        // this is where you put your state machines and all power functions (call this in our main code)


        switch (ClawStateVar) {
            case OPEN:
                setpose(clawservo, 125);

                break;
            case CLOSE:
                setpose(clawservo,  100);
                break;
            case IDLE:

                break;
        }
        switch (PrimeStateVar) {
            case TOP:
                setpose(primeservo,  120);
                break;
            case MIDDLE:
                setpose(primeservo, 57.5);
                break;
            case BOTTOM:
                setpose(primeservo, 0);
                break;
            case IDLE:

                break;
        }
        switch (UpclawStateVar) {
            case OPEN:
                setpose(upclaw, 140);
                break;
            case CLOSE:
                setpose(upclaw,70);
                break;
            case IDLE:

                break;
        }
        switch (HangStateVar) {
            case TOP:
                setpose(hangservo,11);//keep
                setpose(hangservo1,11);
                break;
            case MIDDLE:
                setpose(hangservo,76);
                setpose(hangservo1,76);//keep
                break;
            case BOTTOM:
                setpose(hangservo,133);
                setpose(hangservo1,133);
                break;
            case IDLE:

                break;
        }


    }

    // this is where you put your update functions to switch between states
    public void telemetry(Telemetry telemetry) {
        // add telemetry data here
    }
}