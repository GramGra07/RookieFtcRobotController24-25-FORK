package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.AutoHardware;
import org.firstinspires.ftc.teamcode.util.PIDVals;
import org.gentrifiedApps.statemachineftc.StateMachine;

@Autonomous
public class BestAuto extends LinearOpMode {
    enum autostates {
        moveright,finish, armup,reset,clawf,resetarm, openclaw, stop,moveright2, moveright3,grab,armup2,transfer

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
                    robot.clawsub.setPrimeTOP();
                    robot.clawsub.update();
                })
                .whileState(autostates.moveright, () -> {
                    return true;
                }, () -> {
                    robot.sideWaysEncoderDrive(1, 10);
                    robot.driveStraight(robot.DRIVE_SPEED, 16,0);//change distance val
                }).onExit(autostates.moveright,()->{
                    robot.clawsub.setHangMIDDLE();
                    robot.clawsub.update();
                })
                .transition(autostates.moveright, () -> {
                    return true;
                }, 1)
                .state(autostates.armup)
                .onEnter(autostates.armup, () -> {
                    robot.armSub.setUptarget(300);
                }).whileState(autostates.armup, () -> {
                    return robot.armSub.isUpAtTarget(50);
                }, () -> {
                    robot.armSub.update();
                    robot.armSub.telemetry(telemetry);
                    telemetry.update();
                }).onExit(autostates.armup, () -> {
                    robot.clawsub.setHangBOTTOM();
                    robot.clawsub.update();
                }).transition(autostates.armup, () -> {
                    return true;
                }, 2)
                .state(autostates.openclaw)
                .onEnter(autostates.openclaw, () -> {
                    robot.driveStraight(AutoHardware.DRIVE_SPEED,-7,0);
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
                    robot.clawsub.setPrimeMIDDLE();
                    robot.clawsub.setClawOPEN();
                    robot.clawsub.update();
                })
                //opens bottom claw and puts middle pose to stay out of way
                .whileState(autostates.moveright3, () -> {
                    return true;
                }, () -> {

                }).onExit(autostates.moveright3,()->{
                    robot.clawsub.setPrimeBOTTOM();
                    robot.sideWaysEncoderDrive(1, -44);//change inch val
                    robot.driveStraight(robot.DRIVE_SPEED,6,10);
                    robot.clawsub.update();
                    //robot.driveStraight(robot.DRIVE_SPEED, 2,0);//change distance val
                }).transition(autostates.moveright3, () -> {
                    return true;
                }, 2)
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
                    //robot.sideWaysEncoderDrive(1, -20);//change inch val to go to basket
                    robot.clawsub.update();
                })
                .whileState(autostates.grab, () -> {
                    robot.turnToHeading(robot.TURN_SPEED,-120);
                    robot.clawsub.update();
                    return true;
                }, () -> {

                }).transition(autostates.grab, () -> {
                    return true;
                },0)
                .state(autostates.reset)
                .onEnter(autostates.reset,()->{
                    robot.armSub.setUptarget(75);
                }).whileState(autostates.reset, () -> {
                    return robot.armSub.isUpAtTarget(50);
                }, () -> {
                    robot.armSub.update();
                    robot.armSub.telemetry(telemetry);
                    telemetry.update();
                }).transition(autostates.reset,()->{
                    return true;
                },0.5)
                .state(autostates.transfer)
                .onEnter(autostates.transfer, () -> {
                    robot.clawsub.setUClawOPEN();
                    robot.clawsub.update();
                }).whileState(autostates.transfer,  () -> {

                    return true;
                }, () -> {

                }).transition(autostates.transfer, () -> {
                    return true;
                },1)
                .state(autostates.clawf)
                .onEnter(autostates.clawf,()->{
                    robot.clawsub.setClawOPEN();
                    robot.clawsub.setHangBOTTOM();
                    robot.clawsub.update();
                }).transition(autostates.clawf,()->{
                    return true;
                },0.5)
                .state(autostates.armup2)
                .onEnter(autostates.armup2, () -> {

                    robot.armSub.setUptarget(2100);
                }).whileState(autostates.armup2, () -> {
                    return robot.armSub.isUpAtTarget(50);
                }, () -> {
                    robot.armSub.update();
                    robot.armSub.telemetry(telemetry);
                    telemetry.update();
                }).onExit(autostates.armup2, () -> {
                    robot.clawsub.setHangBOTTOM();
                    robot.driveStraight(robot.DRIVE_SPEED, 5,-120);//change distance val
                }).transition(autostates.armup2, () -> {
                    return true;
                }, 1)
                .state(autostates.finish)
                .onEnter(autostates.finish, () -> {
                    robot.clawsub.setUClawCLOSE();
                    robot.clawsub.update();
                }).transition(autostates.finish,() -> {
                    return true;
                },1)
                .state(autostates.resetarm)
                .onEnter(autostates.resetarm, () -> {
                    robot.armSub.setUptarget(100);
                }).whileState(autostates.resetarm, () -> {
                    return robot.armSub.isUpAtTarget(50);
                }, () -> {

                }).onExit(autostates.resetarm, () -> {
                    robot.armSub.update();
                    robot.armSub.telemetry(telemetry);
                    telemetry.update();
                }).transition(autostates.resetarm, () -> {
                    return true;
                },1)


                //drives over to baskets and drops sample in baskets
                .stopRunning(autostates.stop).build();
        waitForStart();
        machine.start();
        while (machine.mainLoop(this)) {
            machine.update();
            robot.armSub.update();
        }
    }
}
