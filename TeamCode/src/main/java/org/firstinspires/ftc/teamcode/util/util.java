package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Servo;

public class util {
    public static void setpose(Servo servo, double degrees){
        final double degreemult = 0.00555555554;
        servo.setPosition(degreemult * degrees);
    }
}
