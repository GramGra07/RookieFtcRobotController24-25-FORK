package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.gentrifiedApps.velocityvision.enums.Color;

@Autonomous
public class llauto extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(1);

        /*
         * Starts polling for data.
         */
        limelight.start();


        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    double[] outputs = result.getPythonOutput();
                    double angle = outputs[0];
                    double cx = outputs[1];
                    double cy = outputs[2];
                    double c1 = outputs[3];
                    double c2 = outputs[4];
                    Color color;
                    if (c1 == 1 && c2 == 1) {
                        color = Color.YELLOW;
                    } else if (c1 == 1 && c2 == 0) {
                        color = Color.BLUE;
                    } else if (c1 == 0 && c2 == 1) {
                        color = Color.RED;
                    } else {
                        color = Color.NONE;
                    }
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("outputs", outputs.toString());


                }
            }
        }  }}