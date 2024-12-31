package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class PIDVals {
    public static PIDFCoefficients upco = new PIDFCoefficients(0.004,0,0.0007,0.0001);

}
