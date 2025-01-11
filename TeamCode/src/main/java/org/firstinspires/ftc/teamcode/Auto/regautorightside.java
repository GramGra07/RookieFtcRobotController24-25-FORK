package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.AutoHardware;
import org.firstinspires.ftc.teamcode.util.PIDVals;
import org.gentrifiedApps.statemachineftc.StateMachine;

@Autonomous
public class regautorightside extends LinearOpMode {
    enum autostates {
        moveright,finish, armup,reset,park, clawf,resetarm, openclaw, stop,moveright2, moveright3,grab,armup2,transfer

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
                    robot.sideWaysEncoderDrive(1, -10);
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
                    robot.armSub.setUptarget(305);
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
                .state(autostates.park)
                .onEnter(autostates.park, () -> {
                    robot.driveStraight(AutoHardware.DRIVE_SPEED,-4,0);
                    robot.sideWaysEncoderDrive(1,32);
                })
                .stopRunning(autostates.stop).build();
        waitForStart();
        machine.start();
        while (machine.mainLoop(this)) {
            machine.update();
            robot.armSub.update();
        }
    }
}
