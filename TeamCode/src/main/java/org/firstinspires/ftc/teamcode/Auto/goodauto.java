package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.AutoHardware;



@Autonomous
public class goodauto extends LinearOpMode {
    AutoHardware robot = null;



    /* Declare OpMode members. */


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new AutoHardware(this, hardwareMap);

        // Initialize the drive system variables.
        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        //leftdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //backdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //frontdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", robot.getHeading());
            telemetry.update();
        }

        // Set the encoders for closed loop speed control, and reset the heading.

        // sideWaysEncoderDrive(1,30);
        //sideWaysEncoderDrive(1,-30);
        // Step through each leg of the path,
        //Notes:   Reverse movement is obtained by setting a negative distance (not speed)
        //holdHeading() is used after turns to let the heading stabilize
        //Add a sleep(2000) after any step to keep the telemetry data visible for review
        robot.clawsub.setClawCLOSE();
        robot.clawsub.update();
        sleep(500);
        robot.clawsub.setPrimeTOP();
        robot.clawsub.update();
        sleep(500);
        robot.turnToHeading(robot.TURN_SPEED, 0);
        robot.sideWaysEncoderDrive(1, 9);
        robot.clawsub.setUClawCLOSE();
        robot.clawsub.update();
        sleep(500);
        robot.clawsub.setHangTOP();
        robot.clawsub.update();
        sleep(500);
        robot.clawsub.setUClawOPEN();
        robot.clawsub.update();
        sleep(500);
        robot.clawsub.setClawOPEN();
        robot.clawsub.update();
        sleep(500);
        robot.clawsub.setHangBOTTOM();
        robot.clawsub.update();
        sleep(1000);
        robot.armSub.setUptarget(250);
        while (!robot.armSub.isUpAtTarget(50)) {
            robot.armSub.update();
        }
        sleep(500);
        robot.clawsub.setHangMIDDLE();
        robot.clawsub.update();
        sleep(500);
        robot.clawsub.setUClawCLOSE();
        robot.clawsub.update();
        sleep(50000);

        robot.driveStraight(robot.DRIVE_SPEED, 14, 0);
        sleep(1000);
        //robot.armextend(robot.ARM_EXTEND,3.99,0); // not in inches not final either
        //robot.clawsub.setClawCLOSE();
        robot.clawsub.update();
        sleep(750);
        // robot.clawsub.setPrimeMIDDLE();
        robot.clawsub.update();
        sleep(750);
        // robot.armextend(robot.ARM_EXTEND,-3.65,0);
        robot.clawsub.setPrimeTOP();
        robot.clawsub.update();
        robot.clawsub.setClawOPEN();
        robot.clawsub.update();
//        //parking code
        robot.driveStraight(robot.DRIVE_SPEED, -5, 0);
        sleep(500);
        robot.turnToHeading(robot.TURN_SPEED, 0);
        sleep(500);
        robot.sideWaysEncoderDrive(1, 51);
        sleep(500);
        //turnToHeading(TURN_SPEED,0);
        robot.driveStraight(robot.DRIVE_SPEED, 14, 0);
        //turnToHeading(TURN_SPEED,15);
        robot.armextend(robot.ARM_EXTEND, 1, 0);
        sleep(500);
        robot.clawsub.setClawOPEN();
        robot.clawsub.update();
        sleep(1000);
        robot.clawsub.setPrimeBOTTOM();
        robot.clawsub.update();
        sleep(1000);
        robot.clawsub.setClawCLOSE();
        robot.clawsub.update();
        sleep(1000);
        robot.clawsub.setPrimeTOP();
        robot.clawsub.update();

        sleep(500);
        robot.armextend(robot.ARM_EXTEND, -0.70, 0);
        robot.turnToHeading(robot.TURN_SPEED, 180);
        robot.clawsub.setPrimeMIDDLE();
        robot.clawsub.update();
        sleep(500);
        robot.clawsub.setClawOPEN();
        robot.clawsub.update();
        sleep(500);
        robot.driveStraight(robot.DRIVE_SPEED, -10, 180);
        robot.sideWaysEncoderDrive(1, -5);
        sleep(10000);
        robot.driveStraight(robot.DRIVE_SPEED, 10, 180);
        robot.clawsub.setPrimeBOTTOM();
        robot.clawsub.update();
        sleep(500);
        robot.clawsub.setClawCLOSE();
        robot.clawsub.update();
        sleep(500);
        robot.clawsub.setPrimeTOP();
        robot.clawsub.update();
        sleep(500);
        robot.turnToHeading(robot.TURN_SPEED, 0);
        robot.sideWaysEncoderDrive(1, -64.5);

        robot.buildtelemetry();


//        turnToHeading( TURN_SPEED, -90.0);
//        driveStraight(DRIVE_SPEED, 32.5, -90.0);      // Drive Forward 24"kk
//        turnToHeading( TURN_SPEED,  0.0);// Turn  CW to -45 Degrees
//        //driveStraight(DRIVE_SPEED,10,0);
//        //armextend(ARM_EXTEND,2,0);
//        driveStraight(DRIVE_SPEED, -1.0, 0.0);
        //end of parking
        //holdHeading( TURN_SPEED, -45.0, 0.5);   // Hold -45 Deg heading for a 1/2 second
        //driveStraight(DRIVE_SPEED, 22,-90.0);
        // driveStraight(DRIVE_SPEED, 17.0, -45.0);  // Drive Forward 17" at -45 degrees (12"x and 12"y)
        //turnToHeading( TURN_SPEED,  45.0);               // Turn  CCW  to  45 Degrees
        //holdHeading( TURN_SPEED,  45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second

        //driveStraight(DRIVE_SPEED, 17.0, 45.0);  // Drive Forward 17" at 45 degrees (-12"x and 12"y)
        //turnToHeading( TURN_SPEED,   0.0);               // Turn  CW  to 0 Degrees
        //holdHeading( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for 1 second

        //driveStraight(DRIVE_SPEED,-48.0, 0.0);  // Drive in Reverse 48" (should return to approx. staring position)

        //drivetosub(DRIVE_SPEED, -48.0, 0.0); //fix the numbers in this to see what it does.
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
     * Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance      Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                      0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                      If a relative angle is required, add/subtract from the current robotHeading.
     */

}