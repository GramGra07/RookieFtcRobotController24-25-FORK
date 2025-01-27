package com.example.meepmeepmod;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startpose = new Pose2d(-14,-63,Math.toRadians(90.0));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(18,18)
                .setStartPose(startpose)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(startpose)
                //move to sub
//                .splineToConstantHeading(new Vector2d(0,-34),Math.toRadians(90.0))
//                .setTangent(Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(56, -39), Math.toRadians(90.0))
//                        .turnTo(Math.toRadians(270.0))
//
//                        .turnTo(Math.toRadians(90.0))
//                .setTangent(Math.toRadians(270.0))
//                .splineToConstantHeading(new Vector2d(0,-34),Math.toRadians(90.0))
//                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225.0)), Math.toRadians(225.0))
                               // .strafeTo(new Vector2d(-56,-56))
                //
                .strafeTo(new Vector2d(-56.5,-56.5))
                .strafeTo(new Vector2d(-48,-39))
                .strafeTo(new Vector2d(-56.5,-56.5))
                .strafeTo(new Vector2d(-58,-39))
                .strafeTo(new Vector2d(-56.5,-56.5))
//                .turnTo(Math.toRadians(225.0))
//                //
//                .strafeTo(new Vector2d(-48,-39))
//                                .turnTo(Math.toRadians(90.0))
//                                .lineToY(-37)



                // .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225.0)), Math.toRadians(225.0))


//                .splineTo(new Vector2d(0,-34),Math.toRadians(90.0))
//                //Insert the code for top speci hang
//
//                //start next movement to the first sample
//                //.setTangent(Math.toRadians(270))
//                //.splineToConstantHeading(new Vector2d(-48,-39),Math.toRadians(90.0))
//                //insert code for the pickup, transfer, and deposit
//
//                //start next movement to the basket
//                //.splineToLinearHeading(new Pose2d(-56,-56,Math.toRadians(225.0)),Math.toRadians(225.0))
//                //insert code for pickup, transfer, and deposit
//
//                //start next movement to the second sample
//                //.turnTo(Math.toRadians(90.0))
//                //.strafeToConstantHeading(new Vector2d(-57,-39))
//                //set the robot ready for the next steps(claw up and open)
//
////                .setTangent(Math.toRadians(90))
////                .splineTo(new Vector2d(-57,-39),Math.toRadians(-45.0))
//
//
//                //start next movement to the basket
//                //.splineToLinearHeading(new Pose2d(-56,-56,Math.toRadians(225.0)),Math.toRadians(225.0))
//                //
//                //start to third sample
//
//
//
//
//                //move to the parking zone
//
//                //.splineTo(new Vector2d(-56,-56),Math.toRadians(225.0))
//                //.splineToLinearHeading(new Pose2d(-56,-56,Math.toRadians(225.0)))
//                //.turnTo(Math.toRadians(90))
//
//                .strafeTo(new Vector2d(56,-56))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}