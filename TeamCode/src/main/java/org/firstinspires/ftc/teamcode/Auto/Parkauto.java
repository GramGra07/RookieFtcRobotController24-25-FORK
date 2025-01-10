package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.AutoHardware;
import org.firstinspires.ftc.teamcode.util.PIDVals;
import org.gentrifiedApps.statemachineftc.StateMachine;

@Autonomous
public class Parkauto extends LinearOpMode {
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
                    robot.sideWaysEncoderDrive(1,56);
                })
                .onExit(autostates.moveright, () -> {

                }).transition(autostates.moveright,()->{
                    return true;
                },1)
                .stopRunning(autostates.stop).build();
        waitForStart();
        machine.start();
        while (machine.mainLoop(this)) {
            machine.update();
            robot.armSub.update();
        }
    }
}
