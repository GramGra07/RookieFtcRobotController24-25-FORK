package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.AutoHardware;
import org.gentrifiedApps.statemachineftc.StateMachine;

@Autonomous
public class BestAuto extends LinearOpMode {
    enum autostates {
        moveright, armup, openclaw, stop

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
                    return robot.armSub.isUpAtTarget(100);
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
                    robot.clawsub.update();
                })
                .transition(autostates.openclaw, () -> {
                    return true;
                }, 2)
                .stopRunning(autostates.stop).build();
        waitForStart();
        machine.start();
        while (machine.mainLoop(this)) {
            machine.update();
        }
    }
}
