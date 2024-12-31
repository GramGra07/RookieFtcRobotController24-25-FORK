package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.AutoHardware;
import org.gentrifiedApps.statemachineftc.StateMachine;

@Autonomous
public class BestAuto extends LinearOpMode {
    enum autostates {
        moveright, armup, openclaw, stop,moveright2, moveright3,grab,

    }

    AutoHardware robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new AutoHardware(this, hardwareMap);
        StateMachine<autostates> machine = new StateMachine.Builder<autostates>()
                .state(autostates.moveright)
                .onEnter(autostates.moveright, () -> {
//                    robot.turnToHeading(robot.TURN_SPEED, 0);
//                    robot.sideWaysEncoderDrive(1, 9);
                    robot.clawsub.setUClawOPEN();
                    robot.clawsub.update();
                })
                .whileState(autostates.moveright, () -> {
                    return true;
                }, () -> {
                    robot.sideWaysEncoderDrive(1, 9);
                }).onExit(autostates.moveright,()->{
                    robot.clawsub.setHangBOTTOM();
                    robot.clawsub.update();
                })
                .transition(autostates.moveright, () -> {
                    return true;
                }, 1)
                .state(autostates.armup)
                .onEnter(autostates.armup, () -> {
                    robot.armSub.setUptarget(250);
                }).whileState(autostates.armup, () -> {
                    return robot.armSub.isUpAtTarget(50);
                }, () -> {
                    robot.armSub.update();
                    robot.armSub.telemetry(telemetry);
                    telemetry.update();
                }).onExit(autostates.armup, () -> {
                    robot.clawsub.setHangMIDDLE();
                    robot.clawsub.update();
                }).transition(autostates.armup, () -> {
                    return true;
                }, 2)
                .state(autostates.openclaw)
                .onEnter(autostates.openclaw, () -> {
                    robot.clawsub.setUClawCLOSE();
                    robot.clawsub.setHangTOP();
                    robot.clawsub.update();
                })
                .transition(autostates.openclaw, () -> {
                    return true;
                }, 2)//
                .state(autostates.moveright3)
                //Start of baskets
                .onEnter(autostates.moveright3, () -> {
//                    robot.turnToHeading(robot.TURN_SPEED, 0);
//                    robot.sideWaysEncoderDrive(1, 9);
                    robot.clawsub.setPrimeMIDDLE();
                    robot.clawsub.setClawOPEN();
                    robot.clawsub.update();
                })
                //opens bottom claw and puts middle pose to stay out of way
                .whileState(autostates.moveright3, () -> {
                    return true;
                }, () -> {
                    robot.sideWaysEncoderDrive(1, 20);//change inch val
                }).onExit(autostates.moveright3,()->{
                    robot.clawsub.setPrimeBOTTOM();
                    robot.clawsub.update();
                }).transition(autostates.moveright3, () -> {
                    return true;
                }, 1)
                .state(autostates.moveright2)
                //moves right and grabs sample
                .onEnter(autostates.moveright2, () -> {
//                    robot.turnToHeading(robot.TURN_SPEED, 0);
//                    robot.sideWaysEncoderDrive(1, 9);
                    robot.clawsub.setClawCLOSE();
                    robot.clawsub.update();
                })
                //gives the top claw the sample and primes it for transfer
                .whileState(autostates.moveright2, () -> {
                    return true;
                }, () -> {
                    }).transition(autostates.moveright2,() -> {
                    return true;
                },1)
                .state(autostates.grab)
                .onEnter(autostates.grab,()->{
                    robot.clawsub.setPrimeTOP();
                    robot.sideWaysEncoderDrive(1, -20);//change inch val to go to basket

                    robot.clawsub.update();


                    robot.clawsub.update();
                })
                .whileState(autostates.grab, () -> {
                    robot.clawsub.setUClawOPEN();
                    robot.clawsub.setClawCLOSE();
                    robot.clawsub.setHangBOTTOM();
                    robot.turnToHeading(robot.TURN_SPEED,-120);
                    robot.armSub.setUptarget(2100);
                    robot.driveStraight(robot.DRIVE_SPEED, 5,-120);//change distance val
                    robot.clawsub.setUClawCLOSE();
                    return true;
                }, () -> {

                }).transition(autostates.grab, () -> {
                    return true;
                },0)
                //drives over to baskets and drops sample in baskets
                .stopRunning(autostates.stop).build();
        waitForStart();
        machine.start();
        while (machine.mainLoop(this)) {
            machine.update();
        }
    }
}
