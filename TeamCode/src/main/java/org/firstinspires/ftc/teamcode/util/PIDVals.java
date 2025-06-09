package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class PIDVals {
    public static PIDFCoefficients upco = new PIDFCoefficients(0.005, 0, 0.000, 0.000);
    public static PIDFCoefficients upco1 = new PIDFCoefficients(0.005, 0.0, 0.0000, 0.000);
    // 0.005
}
