/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.Trig;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.List;

// Base class for all TrigBot autonomous programs

public abstract class TrigAutoBase extends LinearOpMode {

    /* Declare OpMode members. */
    public HardwareTrig robot = new HardwareTrig(telemetry, this);   // Use TrigBot's hardware
    public ElapsedTime runtime = new ElapsedTime();   // used for encoderDrive's timeout

    public static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    protected float DRIVE_SPEED = .7f;
    protected float ORIGINAL_DRIVE_SPEED = .4f;
    protected float TURN_SPEED = 0.2f;

    // Change everything related to DRIVE_SPEED. Preferably change DRIVE_SPEED to 1.0.

    protected boolean dpadUpPressed        = false;
    protected boolean dpadDownPressed      = false;
    protected boolean dpadRightPressed     = false;
    protected boolean dpadLeftPressed      = false;
    protected boolean leftBumperPressed = false;
    protected boolean rightBumperPressed = false;
    protected boolean aPressed = false;
    protected boolean yPressed = false;



    protected List<VuforiaTrackable> allTrackables;

    // define some abstract methods which actual autonomous code must implement
    public abstract void waitForDelay();
    public abstract void executeMovements();

    protected boolean useEncoderDrive = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        robot.eye.setPosition(0.0);
        robot.beaconLeft.setPosition(robot.LEFT_BEACON_DOWN);
        robot.beaconRight.setPosition(robot.RIGHT_BEACON_DOWN);
        // reset encoders
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // make sure the gyro is calibrated before continuing
        robot.gyro.calibrate();
        while (!isStopRequested() && robot.gyro.isCalibrating()) {
            if (gamepad1.x) {
                useEncoderDrive = true;
                break;
            }
            sleep(100);
            idle();
        }

        // NOTE: investigate doing this after start it it's quick....
      allTrackables =  robot.initializeTrackables();


        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {


            if (gamepad1.dpad_up && !dpadUpPressed) {
                dpadUpPressed = true;
                DRIVE_SPEED = DRIVE_SPEED + 0.05f;
            }
            if (!gamepad1.dpad_up) {
                dpadUpPressed = false;
            }


            if (gamepad1.dpad_down && !dpadDownPressed) {
                dpadDownPressed = true;
                DRIVE_SPEED = DRIVE_SPEED - 0.05f;
            }
            if (!gamepad1.dpad_down) {
                dpadDownPressed = false;
            }

            if (gamepad1.y && !yPressed) {
                yPressed = true;
                TURN_SPEED = TURN_SPEED + 0.02f;
            }
            if (!gamepad1.y) {
                yPressed = false;
            }


            if (gamepad1.a && !aPressed) {
                aPressed = true;
                TURN_SPEED = TURN_SPEED - 0.02f;
            }
            if (!gamepad1.a) {
                aPressed = false;
            }



            if (gamepad1.dpad_right && !dpadRightPressed) {
                dpadRightPressed = true;
                robot.P_DRIVE_COEFF = robot.P_DRIVE_COEFF + 0.001f;
            }
            if (!gamepad1.dpad_right) {
                dpadRightPressed = false;
            }

            if (gamepad1.dpad_left && !dpadLeftPressed) {
                dpadLeftPressed = true;
                robot.P_DRIVE_COEFF = robot.P_DRIVE_COEFF - 0.001f;
            }
            if (!gamepad1.dpad_left) {
                dpadLeftPressed = false;
            }


            if (gamepad1.left_bumper && !leftBumperPressed) {
                leftBumperPressed = true;
                robot.P_TURN_COEFF = robot.P_TURN_COEFF - 0.01f;
            }
            if (!gamepad1.left_bumper) {
                leftBumperPressed = false;
            }

            if (gamepad1.right_bumper && !rightBumperPressed) {
                rightBumperPressed = true;
                robot.P_TURN_COEFF = robot.P_TURN_COEFF + 0.01f;
            }
            if (!gamepad1.right_bumper) {
                rightBumperPressed = false;
            }

            telemetry.addData(">", "Robot Heading = %d", robot.gyro.getIntegratedZValue());
            telemetry.addData("DRIVE_SPEED", DRIVE_SPEED);
            telemetry.addData("TURN_SPEED", TURN_SPEED);
            telemetry.addData("P_DRIVE_COEFF", robot.P_DRIVE_COEFF);
            telemetry.addData("P_TURN_COEFF", robot.P_TURN_COEFF);

            telemetry.update();
            idle();
        }
        robot.beaconLeft.setPosition(robot.LEFT_BEACON_UP);
        robot.beaconRight.setPosition(robot.RIGHT_BEACON_UP);
        executeMovements();
    }


    public void simpleGyroTurn(double speed, double angle) {


        telemetry.addData(">", "Robot Heading = %d", robot.gyro.getIntegratedZValue());
        telemetry.update();

        if (angle > robot.gyro.getIntegratedZValue()) {
            robot.leftMotor.setPower(-1.0 * speed);
            robot.rightMotor.setPower(speed);


            while (opModeIsActive() && robot.gyro.getIntegratedZValue() < angle) {
                // Update telemetry & Allow time for other processes to run.
                telemetry.addData(">", "Robot Heading = %d", robot.gyro.getIntegratedZValue());
                telemetry.update();
            }

        } else {
            robot.rightMotor.setPower(-1.0 * speed);
            robot.leftMotor.setPower( speed);
            while (opModeIsActive() && robot.gyro.getIntegratedZValue() >  angle) {
                // Update telemetry & Allow time for other processes to run.
                telemetry.addData(">", "Robot Heading = %d", robot.gyro.getIntegratedZValue());
                telemetry.update();
            }
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

        }
    }

    // read the color sensor, and then activate the
    // proper servo to press the correct button based on our alliance color
    public void selectAndPressBeacon(boolean redTeam) {
           final double     TURN_SPEED              = 0.4;     // Nominal half speed for better accuracy.

            int dist = 12;
        if (robot.colorSensor.red() > robot.colorSensor.blue()) {
            robot.eye.setPosition(0.0);
            if (redTeam) {
                robot.beaconRight.setPosition(HardwareTrig.RIGHT_BEACON_DOWN);
                sleep(250);
                encoderDrive(TURN_SPEED, dist , dist, 5.0);
                robot.hammerBeacon(robot.beaconLeft, HardwareTrig.LEFT_BEACON_PARTIAL_DOWN, HardwareTrig.LEFT_BEACON_UP);
            } else {
                robot.beaconLeft.setPosition(HardwareTrig.LEFT_BEACON_DOWN);
                sleep(250);
                encoderDrive(TURN_SPEED, dist, dist, 5.0);
                robot.hammerBeacon(robot.beaconRight, HardwareTrig.RIGHT_BEACON_PARTIAL_DOWN, HardwareTrig.RIGHT_BEACON_UP);
            }
        } else {// we see blue
            robot.eye.setPosition(0.0);

            if (!redTeam) {
                robot.beaconRight.setPosition(HardwareTrig.RIGHT_BEACON_DOWN);
                sleep(250);
                encoderDrive(TURN_SPEED, dist, dist, 5.0);
                robot.hammerBeacon(robot.beaconLeft, HardwareTrig.LEFT_BEACON_PARTIAL_DOWN, HardwareTrig.LEFT_BEACON_UP);

            } else {
                robot.beaconLeft.setPosition(HardwareTrig.LEFT_BEACON_DOWN);
                sleep(250);
                encoderDrive(TURN_SPEED, dist, dist, 5.0);
                robot.hammerBeacon(robot.beaconRight, HardwareTrig.RIGHT_BEACON_PARTIAL_DOWN, HardwareTrig.RIGHT_BEACON_UP);

            }
        }


        telemetry.addData("Red  ", robot.colorSensor.red());
        telemetry.addData("blue  ", robot.colorSensor.blue());
        telemetry.update();

        encoderDrive(DRIVE_SPEED, -4, -4, 5);
        robot.beaconRight.setPosition(HardwareTrig.RIGHT_BEACON_UP);
        robot.beaconLeft.setPosition(HardwareTrig.LEFT_BEACON_UP);

    }
}
