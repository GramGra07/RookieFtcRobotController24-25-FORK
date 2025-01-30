package org.firstinspires.ftc.teamcode.Hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.followers.roadrunner.MecanumDrive;

import java.util.List;

public class AutoHardware extends HardwareConfig {
    Pose2d startPose = null;
    private IMU imu = null;      // Control/Expansion Hub IMU
    MecanumDrive drive = null;

    private double headingError = 0;
    public static final double COUNTS_PER_INCH_Side = -100 * 0.50;


    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftfrontSpeed = 0;
    private double rightfrontSpeed = 0;
    private double rightbackSpeed = 0;
    private double leftbackSpeed = 0;
    private int leftfrontTarget = 0;
    private int rightfrontTarget = 0;
    private int backrightTarget = 0;
    private int backleftTarget = 0;
    private double armMotorTarget = 0;
    public static boolean drivefinished = true;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 28.0;   // eg: Rev ultraplanetary 20:1 28 counts
    static final double DRIVE_GEAR_REDUCTION = 20.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double COUNT_PER_INCH_ARM = 384.5;


    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    public static final double DRIVE_SPEED = 1.0;      // Max driving speed for better distance accuracy.
    public static final double TURN_SPEED = 0.2;     // Max turn speed to limit turn rate.
    static final double HEADING_THRESHOLD = 1.0;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable.
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable.
    public static final double ARM_EXTEND = 1.0;
    public static Pose2d lastPose;


    public AutoHardware(LinearOpMode om, HardwareMap hwmap, Pose2d startPose) {
        super(om, hwmap, true);
        this.startPose = startPose;
        lastPose = startPose;

        drive = new MecanumDrive(hwmap, startPose);
    }


    public void parkclose() {
        drivefinished = true;
        Actions.runBlocking(
                //!this needs help
                new SequentialAction(
                        new ParallelAction(
                                new SequentialAction(

                                        // no update action here
                                        new SequentialAction(
                                                drive.actionBuilder(startPose)
                                                        .turnTo(Math.toRadians(90))
                                                        .strafeTo(new Vector2d(56, -56))
                                                        .build(),
                                                endAction()
                                        ),
                                        //ready everything for op
                                        armSub.armAction(List.of(() -> armSub.setUptarget(100))),
                                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE())),
                                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangTOP())),
                                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setPrimeTOP())),
                                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setClawOPEN()))
                                )
                        ))
        );
    }


    public void placeSampleCycle() {
        drivefinished = true;
        Actions.runBlocking(
                //! i think you mean to put this in a sequential, but it isn't structured like that
                //!you need more actions in this method

                new SequentialAction(  //unnecessary
                        //?
                        new SequentialAction(
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawOPEN())),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangBOTTOM())),
                                //drive to the basket
                                drive.actionBuilder(lastPose)
                                        //.splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225.0)), Math.toRadians(225.0))
                                        .strafeTo(new Vector2d(-50, -50))
                                        .turnTo(Math.toRadians(225.0))//
                                        .build(),
                                endAction()

                        ),
                        new InstantAction(() -> drivefinished = false),
                        new ParallelAction(
                                armSub.armAction(List.of(() -> armSub.setUptarget(500))),
                                Update()
                        ),
                        new InstantAction(() -> drivefinished = true),
                        new SequentialAction(


                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setFREAKY())),
                                new SleepAction(1),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE()))
                        ),
                        new SleepAction(1),
                        //reset the claw in order to grab next sample
                        new SequentialAction(
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangTOP())),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE())),
                                armSub.armAction(List.of(() -> armSub.setUptarget(100)))
                        ), new SleepAction(1)


                )
        );
    }

    public void placepreloadsample() {
        drivefinished = true;
        Actions.runBlocking(
                new SequentialAction(
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawOPEN())),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangBOTTOM())),

                        new InstantAction(() -> drivefinished = false),
                        new ParallelAction(
                                armSub.armAction(List.of(() -> armSub.setUptarget(500))),
                                Update()
                        ),
                        new InstantAction(() -> drivefinished = true),
                        //set arm at correct height
                        new ParallelAction(
                                armSub.armAction(List.of(() -> armSub.setUptarget(2000))),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setFREAKY())),
                                new SequentialAction(
                                        drive.actionBuilder(lastPose)
                                                //.splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225.0)), Math.toRadians(225.0))
                                                .strafeTo(new Vector2d(-58., -58.))
                                                .turnTo(Math.toRadians(225.0))//
                                                .build(),
                                        endAction()
                                ),
                                Update()
                        ),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE())),
                        //new SleepAction(1),
                        //reset the claw in order to grab next sample
                        new SequentialAction(
                                new InstantAction(() -> drivefinished = false),
                                new ParallelAction(
                                        armSub.armAction(List.of(() -> armSub.setUptarget(150))),
                                        armSub.armAction(List.of(() -> armSub.isUpAtTarget(50))),
                                        Update()
                                ),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangTOP())),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE()))

                        )
                        //increase arm height when in sub
                        //set claw to top position
                )
        );
    }


    public void spacer() {
        drivefinished = true;
        Actions.runBlocking(
                new SequentialAction(
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setClawOPEN()))
                )
        );
    }

    public void placepreloadsamplegood() {
        drivefinished = true;
        Actions.runBlocking(
                new SequentialAction(
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawOPEN())),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangBOTTOM())),

                        new InstantAction(() -> drivefinished = false),
                        new ParallelAction(
                                armSub.armAction(List.of(() -> armSub.setUptarget(500))),
                                Update()
                        ),
                        new InstantAction(() -> drivefinished = true),
                        //set arm at correct height
                        new ParallelAction(
                                armSub.armAction(List.of(() -> armSub.setUptarget(2100))),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setFREAKY())),
                                new SequentialAction(
                                        drive.actionBuilder(lastPose)
                                                //.splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225.0)), Math.toRadians(225.0))
                                                .strafeTo(new Vector2d(-56.25, -56.25))
                                                .turnTo(Math.toRadians(226.0))//
                                                .build(),
                                        endAction()
                                ),
                                Update()
                        ),
                        new SleepAction(0.5),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE())),
                        //new SleepAction(1),
                        //reset the claw in order to grab next sample
                        new SequentialAction(
                                new InstantAction(() -> drivefinished = false),
                                new ParallelAction(
                                        armSub.armAction(List.of(() -> armSub.setUptarget(150))),
                                        armSub.armAction(List.of(() -> armSub.isUpAtTarget(50))),
                                        Update()
                                ),
                                new InstantAction(() -> drivefinished = true),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangTOP())),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE()))

                        )
                        //increase arm height when in sub
                        //set claw to top position
                )
        );
    }


    public void placePreloadSpeci() {
        drivefinished = true;
        Actions.runBlocking(
                new SequentialAction(
                        //set claw middle to insert
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangMIDDLE())),
                        //armSub.armAction(List.of(() -> armSub.setUptarget(300))),
                        // clawsub.clawAction(clawsub,List.of(() -> clawsub.setHangBOTTOM())),

                        new InstantAction(() -> drivefinished = false),
                        new ParallelAction(
                                armSub.armAction(List.of(() -> armSub.setUptarget(300))),
                                Update()
                        ),
                        new InstantAction(() -> drivefinished = true),
                        //set arm at correct height
                        new ParallelAction(
                                new SequentialAction(

                                        drive.actionBuilder(lastPose)
                                                .strafeToLinearHeading(new Vector2d(0, -34), Math.toRadians(90.0))

                                                .build(),
                                        endAction()
                                ),
                                Update()
                        ),
                        //increase arm height when in sub
                        //set claw to top position

                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangBOTTOM())),
                        new SleepAction(1.0)
                )
        );
    }


//    public void placeSpeciCycle() {
//        drivefinished = true;
//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                new InstantAction(() -> drivefinished = true), //unnecessary
//                                //Drive to the observation zone facing human player
//                                drive.actionBuilder(lastPose)
//                                        .setTangent(Math.toRadians(270))
//                                        .splineToConstantHeading(new Vector2d(56, -39), Math.toRadians(90.0))
//                                        .turnTo(Math.toRadians(270.0))
//                                        .build(),
//                                endAction(),
//                                //prep for grab
//                                clawsub.clawAction(clawsub,List.of(() -> clawsub.setPrimeBOTTOM())),
//                                clawsub.clawAction(clawsub,List.of(() -> clawsub.setClawOPEN())),
//                                armSub.armAction(List.of(() -> armSub.setUptarget(100))),
//                                clawsub.clawAction(clawsub,List.of(() -> clawsub.setHangTOP())),
//                                clawsub.clawAction(clawsub,List.of(() -> clawsub.setUClawCLOSE()))
//                        ), Update(),
//                        new SequentialAction(// why is this here if its not running
//                                new InstantAction(() -> drivefinished = true),// good job with this
//                                //drive forward so that the claw is over the sample
//                                drive.actionBuilder(lastPose)
//                                        // .lineToY(-52,0,0)
//                                        .build(),
//                                endAction()
//
//                        ), Update(),// shouldnt this be incvluded in a parallel
//                        new SequentialAction(
//                                //grab the sample with the claw in the zone
//                                new ParallelAction(
//                                        clawsub.clawAction(clawsub,List.of(() -> clawsub.setClawCLOSE())),
//                                        clawsub.clawAction(clawsub,List.of(() -> clawsub.setPrimeTOP())),
//                                        clawsub.clawAction(clawsub,List.of(() -> clawsub.setUClawOPEN())),
//                                        clawsub.clawAction(clawsub,List.of(() -> clawsub.setClawOPEN())),
//                                        clawsub.clawAction(clawsub,List.of(() -> clawsub.setHangBOTTOM())),
//                                        clawsub.clawAction(clawsub,List.of(() -> clawsub.setPrimeBOTTOM()))
//
//                                ),
//                                new InstantAction(() -> drivefinished = true),//good
//                                //drive to submersable zone and get arm in position for score
//                                drive.actionBuilder(lastPose)
//                                        .turnTo(Math.toRadians(270.0))
//                                        .splineToConstantHeading(new Vector2d(0, -34), Math.toRadians(90.0))
//                                        .build(),
//                                endAction(),// shouldn this be in a sequential in a parallel with an update?
//                                //set arm up and ready to score
//                                new ParallelAction(//great
//                                        armSub.armAction(List.of(() -> armSub.setUptarget(300))),
//                                        Update()
//                                ),
//                                new InstantAction(() -> drivefinished = true),//no
//                                clawsub.clawAction(clawsub,List.of(() -> clawsub.setHangBOTTOM())),
//                                Update(),
//                                new SleepAction(1.0)
//                        ))
//        );
//
//    }

    public void sample1bd() {
        drivefinished = true;
        Actions.runBlocking(
                new SequentialAction(
                        new SequentialAction(
                                drive.actionBuilder(lastPose)
                                        .turnTo(Math.toRadians(90.0))
                                        .strafeTo(new Vector2d(-50, -34.5))
                                        .build(),

                                endAction()
                        ),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setPrimeBOTTOM())),
                        new SleepAction(1),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setClawCLOSE())),
                        new SleepAction(1),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setPrimeTOP())),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangBOTTOM())),
                        new SleepAction(1),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawOPEN())),
                        new SleepAction(1),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setClawOPEN())),
                        new InstantAction(() -> drivefinished = true),
                        new ParallelAction(
                                armSub.armAction(List.of(() -> armSub.setUptarget(2000))),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setFREAKY())),

                                new SequentialAction(
                                        drive.actionBuilder(lastPose)
                                                //.splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225.0)), Math.toRadians(225.0))
                                                //.strafeTo(new Vector2d(-56.5, -56.5))
                                                .turnTo(Math.toRadians(225))//
                                                .splineToConstantHeading(new Vector2d(-58.25,-58.25),Math.toRadians(225))

                                                //.strafeTo(new Vector2d(56,-56))

                                                .build(),
                                        endAction()
                                ),
                                Update()
                        ),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE())),
                        //new SleepAction(1),
                        //reset the claw in order to grab next sample
                        new SequentialAction(
                                new InstantAction(() -> drivefinished = false),
                                new ParallelAction(
                                        armSub.armAction(List.of(() -> armSub.setUptarget(150))),
                                        armSub.armAction(List.of(() -> armSub.isUpAtTarget(50))),
                                        Update()
                                ),
                                new InstantAction(() -> drivefinished = true),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangTOP())),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE()))

                        ), new SleepAction(1)
                )
        );
    }


    public void sample1() {
        drivefinished = true;
        Actions.runBlocking(
                new SequentialAction(

                        new SequentialAction(
                                drive.actionBuilder(lastPose)
                                        .turnTo(Math.toRadians(90.0))
                                        .strafeTo(new Vector2d(-50, -33.5))//
                                        //.lineToY(-30)
                                        .build(),

                                endAction()
                        ),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setPrimeBOTTOM())),
                        new SleepAction(1),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setClawCLOSE())),
                        new SleepAction(1),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setPrimeTOP())),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangBOTTOM())),
                        new SleepAction(1),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawOPEN())),
                        new SleepAction(1),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setClawOPEN())),
                        new InstantAction(() -> drivefinished = true),
                        new ParallelAction(
                                armSub.armAction(List.of(() -> armSub.setUptarget(2000))),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setFREAKY())),
                                new SequentialAction(
                                        drive.actionBuilder(lastPose)
                                                //.splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225.0)), Math.toRadians(225.0))

                                                .strafeTo(new Vector2d(-58.25, -58.25))
                                                .turnTo(Math.toRadians(225.0))//
                                                .build(),
                                        endAction()
                                ),
                                Update()

//                                new SequentialAction(
//                                        drive.actionBuilder(lastPose)
//                                                //.splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225.0)), Math.toRadians(225.0))
//
//                                                .strafeTo(new Vector2d(-58.25, -58.25))
//                                                .turnTo(Math.toRadians(225.0))//
//                                                .build(),
//                                        endAction()
//                                ),
//                                Update()
                        ),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE())),
                        //new SleepAction(1),
                        //reset the claw in order to grab next sample
                        new SequentialAction(
                                new InstantAction(() -> drivefinished = false),
                                new ParallelAction(
                                        armSub.armAction(List.of(() -> armSub.setUptarget(0))),
                                        armSub.armAction(List.of(() -> armSub.isUpAtTarget(50))),
                                        Update()
                                ),
                                new InstantAction(() -> drivefinished = true),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangTOP())),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE()))

                        )
                )
        );
    }

    public void sample1bad() {
        drivefinished = true;
        Actions.runBlocking(
                new SequentialAction(

//                        new SequentialAction(
//                                drive.actionBuilder(lastPose)
//                                        .turnTo(Math.toRadians(90.0))
//                                        .strafeTo(new Vector2d(-50, -33.5))//
//                                        //.lineToY(-30)
//                                        .build(),
//
//                                endAction()
//                        ),
//                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setPrimeBOTTOM())),
//                        new SleepAction(1),
//                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setClawCLOSE())),
//                        new SleepAction(1),
//                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setPrimeTOP())),
//                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangBOTTOM())),
//                        new SleepAction(1),
//                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawOPEN())),
//                        new SleepAction(1),
//                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setClawOPEN())),
//                        new InstantAction(() -> drivefinished = true),
//                        new ParallelAction(
//                                armSub.armAction(List.of(() -> armSub.setUptarget(2000))),
//                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setFREAKY()))
////                                new SequentialAction(
////                                        drive.actionBuilder(lastPose)
////                                                //.splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225.0)), Math.toRadians(225.0))
////
////                                                .strafeTo(new Vector2d(-58.25, -58.25))
////                                                .turnTo(Math.toRadians(225.0))//
////                                                .build(),
////                                        endAction()
////                                ),
////                                Update()
//
////                                new SequentialAction(
////                                        drive.actionBuilder(lastPose)
////                                                //.splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225.0)), Math.toRadians(225.0))
////
////                                                .strafeTo(new Vector2d(-58.25, -58.25))
////                                                .turnTo(Math.toRadians(225.0))//
////                                                .build(),
////                                        endAction()
////                                ),
////                                Update()
//                        ),
                       // clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE())),
                        //new SleepAction(1),
                        //reset the claw in order to grab next sample
//                        new SequentialAction(
//                                new InstantAction(() -> drivefinished = false),
//                                new ParallelAction(
//                                        armSub.armAction(List.of(() -> armSub.setUptarget(0))),
//                                        armSub.armAction(List.of(() -> armSub.isUpAtTarget(50))),
//                                        Update()
//                                ),
//                                new InstantAction(() -> drivefinished = true)
////                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangTOP())),
////                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE()))
//
//                        )
                )
        );
    }
    public void sample1R() {
        drivefinished = true;
        Actions.runBlocking(
                new SequentialAction(

                        new SequentialAction(
                                drive.actionBuilder(lastPose)
                                        .turnTo(Math.toRadians(90.0))
                                        .strafeTo(new Vector2d(-45, -34.5))//
                                        //.lineToY(-30)
                                        .build(),

                                endAction()
                        ),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setPrimeBOTTOM())),
                        new SleepAction(1),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setClawCLOSE())),
                        new SleepAction(1),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setPrimeTOP())),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangBOTTOM())),
                        new SleepAction(1),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawOPEN())),
                        new SleepAction(1),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setClawOPEN())),
                        new InstantAction(() -> drivefinished = true),
                        new ParallelAction(
                                armSub.armAction(List.of(() -> armSub.setUptarget(2000))),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setFREAKY())),
                                new SequentialAction(
                                        drive.actionBuilder(lastPose)
                                                //.splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225.0)), Math.toRadians(225.0))

                                                .strafeTo(new Vector2d(-58.25, -58.25))
                                                .turnTo(Math.toRadians(225.0))//
                                                .build(),
                                        endAction()
                                ),
                                Update()

//                                new SequentialAction(
//                                        drive.actionBuilder(lastPose)
//                                                //.splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225.0)), Math.toRadians(225.0))
//
//                                                .strafeTo(new Vector2d(-58.25, -58.25))
//                                                .turnTo(Math.toRadians(225.0))//
//                                                .build(),
//                                        endAction()
//                                ),
//                                Update()
                        ),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE())),
                        //new SleepAction(1),
                        //reset the claw in order to grab next sample
                        new SequentialAction(
                                new InstantAction(() -> drivefinished = false),
                                new ParallelAction(
                                        armSub.armAction(List.of(() -> armSub.setUptarget(150))),
                                        armSub.armAction(List.of(() -> armSub.isUpAtTarget(50))),
                                        Update()
                                ),
                                new InstantAction(() -> drivefinished = true),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangTOP())),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE()))

                        ), new SleepAction(1)
                )
        );
    }


    public void sample2() {
        drivefinished = true;
        Actions.runBlocking(
                new SequentialAction(

                        new SequentialAction(
                                drive.actionBuilder(lastPose)
                                        .turnTo(Math.toRadians(90.0))
                                        .strafeTo(new Vector2d(-60, -35))//
                                        //.lineToY(-30)
                                        .build(),

                                endAction()
                        ),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setPrimeBOTTOM())),
                        new SleepAction(1),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setClawCLOSE())),
                        new SleepAction(1),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setPrimeTOP())),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangBOTTOM())),
                        new SleepAction(1),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawOPEN())),
                        new SleepAction(1),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setClawOPEN())),
                        new InstantAction(() -> drivefinished = true),
                        new ParallelAction(
                                armSub.armAction(List.of(() -> armSub.setUptarget(2000))),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setFREAKY())),
                                new SequentialAction(
                                        drive.actionBuilder(lastPose)
                                                //.splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225.0)), Math.toRadians(225.0))

                                                .strafeTo(new Vector2d(-48, -30))
                                                .turnTo(Math.toRadians(225.0))//
                                                .build(),
                                        endAction()
                                ),
                                Update(),

                                new SequentialAction(
                                        drive.actionBuilder(lastPose)
                                                //.splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225.0)), Math.toRadians(225.0))

                                                .strafeTo(new Vector2d(-58.25, -58.25))
                                                .turnTo(Math.toRadians(225.0))//
                                                .build(),
                                        endAction()
                                ),
                                Update()
                        ),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE())),
                        //new SleepAction(1),
                        //reset the claw in order to grab next sample
                        new SequentialAction(
                                new InstantAction(() -> drivefinished = false),
                                new ParallelAction(
                                        armSub.armAction(List.of(() -> armSub.setUptarget(0))),
                                        armSub.armAction(List.of(() -> armSub.isUpAtTarget(50))),
                                        Update()
                                ),
                                new InstantAction(() -> drivefinished = true),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangTOP())),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE()))

                        ), new SleepAction(1)
                )
        );
    }



    public void sample2R() {
        drivefinished = true;
        Actions.runBlocking(
                new SequentialAction(

                        new SequentialAction(
                                drive.actionBuilder(lastPose)
                                        .turnTo(Math.toRadians(90.0))
                                        .strafeTo(new Vector2d(-57.5, -33))//
                                        //.lineToY(-30)
                                        .build(),

                                endAction()
                        ),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setPrimeBOTTOM())),
                        new SleepAction(1),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setClawCLOSE())),
                        new SleepAction(1),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setPrimeTOP())),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangBOTTOM())),
                        new SleepAction(1),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawOPEN())),
                        new SleepAction(1),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setClawOPEN())),
                        new InstantAction(() -> drivefinished = true),
                        new ParallelAction(
                                armSub.armAction(List.of(() -> armSub.setUptarget(2100))),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setFREAKY())),
                                new SequentialAction(
                                        drive.actionBuilder(lastPose)
                                                //.splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225.0)), Math.toRadians(225.0))

                                                .strafeTo(new Vector2d(-48, -30))
                                                .turnTo(Math.toRadians(225.0))//
                                                .build(),
                                        endAction()
                                ),
                                Update(),

                                new SequentialAction(
                                        drive.actionBuilder(lastPose)
                                                //.splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225.0)), Math.toRadians(225.0))

                                                .strafeTo(new Vector2d(-58.25, -58.25))
                                                .turnTo(Math.toRadians(225.0))//
                                                .build(),
                                        endAction()
                                ),
                                Update()
                        ),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE())),
                        //new SleepAction(1),
                        //reset the claw in order to grab next sample
                        new SequentialAction(
                                new InstantAction(() -> drivefinished = false),
                                new ParallelAction(
                                        armSub.armAction(List.of(() -> armSub.setUptarget(0))),
                                        armSub.armAction(List.of(() -> armSub.isUpAtTarget(50))),
                                        Update()
                                ),
                                new InstantAction(() -> drivefinished = true),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangTOP())),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE()))

                        ), new SleepAction(1)
                )
        );
    }

    public void placeYellowSample1() {
        drivefinished = true;
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                //drive to first sample
                                new InstantAction(() -> drivefinished = true),
                                drive.actionBuilder(lastPose) // this needs a sequential and a parallel??
                                        .strafeTo(new Vector2d(-48, -39))
                                        .turnTo(Math.toRadians(90.0))
                                        .lineToY(-37).build(),// end action?
                                endAction(),
                                //get ready for when over the sample
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setPrimeBOTTOM())),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangTOP())),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE())),
                                new InstantAction(() -> drivefinished = false),
                                new ParallelAction(
                                        armSub.armAction(List.of(() -> armSub.setUptarget(100))),
                                        Update()
                                ),
                                new InstantAction(() -> drivefinished = true),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setClawOPEN())),
                                new SequentialAction(
                                        //start transfer
                                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setClawCLOSE())),
                                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setPrimeTOP())),
                                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawOPEN())),
                                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setClawOPEN())),
                                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setPrimeBOTTOM()))
                                        //update?
                                ),
                                new SequentialAction(
                                        //set the top claw up
                                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangBOTTOM())),
                                        new ParallelAction(
                                                //set the arm up to the basket
                                                armSub.armAction(List.of(() -> armSub.setUptarget(2050))),
                                                new SequentialAction(
                                                        //drive to basket
                                                        new InstantAction(() -> drivefinished = true), //move this up to line 281
                                                        drive.actionBuilder(startPose)
                                                                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225.0)), Math.toRadians(225.0))
                                                                .build(),
                                                        endAction()

                                                ),
                                                //score the sample in the basket
                                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setFREAKY())),// could these two be auth that first claw action
                                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE()))
                                        ),
                                        //reset the claw to grab next sample
                                        new InstantAction(() -> drivefinished = false),
                                        new ParallelAction(
                                                armSub.armAction(List.of(() -> armSub.setUptarget(300))),
                                                Update()
                                        ),
                                        new InstantAction(() -> drivefinished = true),// if this needs to wait, it should be paired with a paralllel and update
                                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangTOP()))
                                )
                        ))
        );
    }


    public void placeYellowSample2() {
        drivefinished = true;
        Actions.runBlocking(
                new SequentialAction(

                        new ParallelAction(// sequence????????
                                //drive to second sample
                                new InstantAction(() -> drivefinished = true),//unnecessary
                                drive.actionBuilder(lastPose)
                                        .turnTo(Math.toRadians(90.0))
                                        .strafeToConstantHeading(new Vector2d(-57, -39))
                                        .build(),
                                endAction()
                        ),
                        new SequentialAction(// if you want it to wait use SleepAction() between
                                //set Bottom claw down
                                //set the claw closed to grab sample
                                //set the Bottom claw up
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setPrimeBOTTOM())),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setClawCLOSE())),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setPrimeTOP()))
                        ),
                        new SequentialAction( // same as a couple lines up
                                //set the top claw down
                                //grab the sample with top
                                //let go of the sample on bottom
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangTOP())),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawOPEN())),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setClawOPEN())),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setPrimeBOTTOM())),//
                                new SequentialAction(
                                        //set top claw up
                                        new InstantAction(() -> drivefinished = true),
                                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangBOTTOM())),
                                        new ParallelAction(
                                                //set arm up to basket
                                                armSub.armAction(List.of(() -> armSub.setUptarget(2100))),
                                                new SequentialAction(
                                                        //drive to basket
                                                        drive.actionBuilder(lastPose)
                                                                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225.0)), Math.toRadians(225.0))
                                                                .build(),
                                                        endAction()
                                                ),
                                                //set the top claw to score the sample
                                                //open top claw to do that
                                                //set back to top position to avoid from hitting basket
                                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setFREAKY())),// this should go higher correct? itll run at the same time as that drive
                                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE())),
                                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangBOTTOM()))

                                                //
                                                //backup??????
                                                //
                                        ),
                                        //set arm back down
                                        //set the top position down
                                        armSub.armAction(List.of(() -> armSub.setUptarget(100))),// if this needs to wait, it should be paired with a paralllel and update
                                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangTOP()))
                                )
                        )
                ));
    }


    public void placeYellowSample3() {
        drivefinished = true;
        Actions.runBlocking(
                new SequentialAction(


                        new ParallelAction(
                                //drive to third sample
                                new InstantAction(() -> drivefinished = true),//unnecessary
                                drive.actionBuilder(lastPose)


                                        .build()//endaction?
                        ),
                        new SequentialAction(
                                //set Bottom claw down
                                //set the claw closed to grab sample
                                //set the Bottom claw up
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setPrimeBOTTOM())),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setClawCLOSE())),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setPrimeTOP()))
                        ),
                        new SequentialAction(
                                //set the top claw down
                                //grab the sample with top
                                //let go of the sample on bottom
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangTOP())),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawOPEN())),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setClawOPEN())),
                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setPrimeBOTTOM())
                                ),
                                new SequentialAction(
                                        //set top claw up
                                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangBOTTOM())),
                                        new ParallelAction(
                                                //set arm up to basket
                                                armSub.armAction(List.of(() -> armSub.setUptarget(2100))),
                                                new SequentialAction(
                                                        //drive to basket
                                                        new InstantAction(() -> drivefinished = true),
                                                        drive.actionBuilder(lastPose)
                                                                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225.0)), Math.toRadians(225.0))
                                                                .build(),
                                                        endAction()

                                                ),
                                                //set the top claw to score the sample
                                                //open top claw to do that
                                                //set back to top position to avoid from hitting basket
                                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setFREAKY())),
                                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE())),
                                                clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangBOTTOM()))
                                                //
                                                //backup??????
                                                //
                                        ),
                                        //set arm back down
                                        //set the top position down
                                        armSub.armAction(List.of(() -> armSub.setUptarget(100))),// if this needs to wait, it should be paired with a paralllel and update
                                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangTOP()))
                                )

                        )
                ));
    }


    //Theoretical
    //
    //
    //
    public void grabSpikeSample() {
        drivefinished = true;
        Actions.runBlocking(
                new SequentialAction(
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangTOP())),
                        new ParallelAction(
                                armSub.armAction(List.of(() -> armSub.setUptarget(100))),
                                new SequentialAction(
                                        drive.actionBuilder(startPose)
                                                .setTangent(Math.toRadians(270))
                                                .splineToConstantHeading(new Vector2d(-48, -39), Math.toRadians(90.0))
                                                .build(),
                                        endAction()
                                ),
                                Update()
                        ),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangBOTTOM()))
                )
        );
    }

    public void grabSpikeSample2() {
        drivefinished = true;
        Actions.runBlocking(
                new SequentialAction(
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangTOP())),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setUClawCLOSE())),
                        new ParallelAction(
                                armSub.armAction(List.of(() -> armSub.setUptarget(100))),
                                new SequentialAction(
                                        drive.actionBuilder(startPose)
                                                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225.0)), Math.toRadians(225.0))

                                                .build(),
                                        endAction()//replace wiht endaction
                                ),
                                Update()
                        ),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangBOTTOM()))
                )
        );
    }

    public void grabSpikeSample3() {
        drivefinished = true;
        Actions.runBlocking(
                new SequentialAction(
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangTOP())),
                        new ParallelAction(
                                armSub.armAction(List.of(() -> armSub.setUptarget(100))),
                                new SequentialAction(
                                        drive.actionBuilder(startPose)

                                                .build(),
                                        armSub.armAction(List.of(() -> drivefinished = false))//replace
                                ),
                                Update()
                        ),
                        clawsub.clawAction(clawsub, List.of(() -> clawsub.setHangBOTTOM()))
                )
        );
    }
    //theoretical ^
    //            |

    //you can delete the next couple of funcs
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            leftfrontTarget = frontLeftMotor.getCurrentPosition() + moveCounts;
            rightfrontTarget = frontRightMotor.getCurrentPosition() + moveCounts;
            backrightTarget = backRightMotor.getCurrentPosition() + moveCounts;
            backleftTarget = backLeftMotor.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            frontLeftMotor.setTargetPosition(leftfrontTarget);
            frontRightMotor.setTargetPosition(rightfrontTarget);
            backRightMotor.setTargetPosition(backrightTarget);
            backLeftMotor.setTargetPosition(backleftTarget);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() &&
                    (frontLeftMotor.isBusy() && frontRightMotor.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     *
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        leftfrontSpeed = drive - turn;
        rightfrontSpeed = drive + turn;
        rightbackSpeed = drive + turn;
        leftbackSpeed = drive - turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftfrontSpeed), Math.abs(rightfrontSpeed));
        if (max > 1.0) {
            leftfrontSpeed /= max;
            rightfrontSpeed /= max;
            leftbackSpeed /= max;
            rightbackSpeed /= max;

        }

        frontLeftMotor.setPower(leftfrontSpeed);
        frontRightMotor.setPower(rightfrontSpeed);
        backRightMotor.setPower(rightbackSpeed);
        backLeftMotor.setPower(leftbackSpeed);
    }

    /**
     * Display the various control parameters while driving
     *
     * @param straight Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {


        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R", "%7d:%7d", leftfrontTarget, rightfrontTarget);
            telemetry.addData("Actual Pos L:R", "%7d:%7d", frontLeftMotor.getCurrentPosition(),
                    frontRightMotor.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr", "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds FLW : FRW", "%5.2f : %5.2f", leftfrontSpeed, rightfrontSpeed);
        telemetry.addData("Wheel Speeds BLW : BRW", "%5.2f : %5.2f", leftbackSpeed, rightbackSpeed);
        telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void sideWaysEncoderDrive(double speed,
                                     double inches) {//+=right //-=left
        int newFRTarget;
        int newFLTarget;
        int newBRTarget;
        int newBLTarget;
        if (opMode.opModeIsActive()) {
            if (inches < 0) {
                newFLTarget = frontLeftMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
                newBLTarget = backLeftMotor.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_Side);
                newFRTarget = frontRightMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
                newBRTarget = backRightMotor.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_Side);
                frontLeftMotor.setTargetPosition(newFLTarget);//actually backleft
                backLeftMotor.setTargetPosition(newBLTarget);//actually frontleft
                backRightMotor.setTargetPosition(newBRTarget);
                frontRightMotor.setTargetPosition(newFRTarget);//

            }
            if (inches > 0) {
                newFLTarget = frontLeftMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
                newBLTarget = backLeftMotor.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_Side);
                newFRTarget = frontRightMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
                newBRTarget = backRightMotor.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_Side);
                frontLeftMotor.setTargetPosition(newFLTarget);//actually backleft
                backLeftMotor.setTargetPosition(newBLTarget);//actually frontleft
                backRightMotor.setTargetPosition(newBRTarget);//makes go forward
                frontRightMotor.setTargetPosition(newFRTarget);
            }

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            backLeftMotor.setPower(Math.abs(speed));
            frontLeftMotor.setPower(Math.abs(speed));
            frontRightMotor.setPower(Math.abs(speed));
            backRightMotor.setPower(Math.abs(speed));
            while (opMode.opModeIsActive() &&
                    backRightMotor.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Running to", "%7d:%7d", frontLeftMotor.getCurrentPosition()
                        , backRightMotor.getCurrentPosition());
                telemetry.addData("Running to", "%7d:%7d", backLeftMotor.getCurrentPosition()
                        , frontRightMotor.getCurrentPosition());
                telemetry.addData("Currently at", "%7d:%7d",
                        frontLeftMotor.getCurrentPosition()
                        , backRightMotor.getCurrentPosition());
                telemetry.addData("Currently at", "%7d:%7d",
                        backLeftMotor.getCurrentPosition()
                        , frontRightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    //    public void scoresample(){
//        Actions.runBlocking(ParallelAction(
//                clawsub.setHangMIDDLE();
//
//                )
//
//        );
//    }
    class update implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            clawsub.update();
            armSub.update();
            telemetryPacket.put("encoder1", armSub.dualEncoder.getmost());
            return !armSub.isUpAtTarget(150) || drivefinished;

        }
    }

    Action Update() {
        return new update();

    }

    class end implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            lastPose = drive.localizer.getPose();
            drivefinished = false;
            return false;
        }
    }

    Action endAction() {
        return new end();
    }

    public void armextend(double maxDriveSpeed,
                          double distance, double heading) {

        // Ensure that the OpMode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNT_PER_INCH_ARM);
            armMotorTarget = armMotor1.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            armMotor1.setTargetPosition((int) armMotorTarget);

            armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            armMotor1.setPower(1);

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() &&
                    (armMotor1.isBusy())) {
                telemetry.addData("current pose", armMotor1.getCurrentPosition());
                telemetry.addData("targetPose", armMotorTarget);
                telemetry.update();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            armMotor1.setPower(0);
            armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}

