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
package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Trig.HardwareTrig;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 * <p>
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * <p>
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Test: Systems Test", group = "Test")
//@Disabled
public class SystemsTest extends OpMode {
    public static final double COUNTS_PER_MOTOR_REV = 1440;

    /* Declare OpMode members. */
    HardwareTrig robot = new HardwareTrig(telemetry, null); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    double sweepSpeed = 0.0;
    final double SWEEP_SPEED = 0.001;                 // sets rate to move servo
    double flickerSpeed = 1.0;
    final double FLICKER_SPEED = 0.05;
    boolean firing = false;
    int xVal, yVal, zVal = 0;     // Gyro rate Values
    int gyroAngle = 0; // Gyro integrated heading
    int angleZ = 0;
    VectorF currentPosition = new VectorF(-304.8f, -1524f);
    VectorF targetPosition = new VectorF(-1549.4f, -304.8f);
    VectorF targetVector;

    private ElapsedTime runtime = new ElapsedTime();

    double DRIVE_SPEED = .8;


    // calculate the angular error between the currentHeading and our targetHeading
    // we do this by using the formula cos(t) = (v1 dot v2 )/ (mag(v1) * mag(v2))

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        // make sure the gyro is calibrated before continuing
        robot.gyro.calibrate();
        while (robot.gyro.isCalibrating()) {
            telemetry.addData("calibrating gyro...",true);
            telemetry.update();
        }
        telemetry.addData("Done calibrating gyro...",true);
        telemetry.update();

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        robot.beaconLeft.setPosition(HardwareTrig.LEFT_BEACON_DOWN);
        robot.beaconRight.setPosition(HardwareTrig.RIGHT_BEACON_DOWN);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void loop() {
        telemetry.addData("flickerValue",robot.flicker.getCurrentPosition());
        telemetry.addData("leftMotorValue",robot.leftMotor.getCurrentPosition());
        telemetry.addData("rightMotorValue",robot.rightMotor.getCurrentPosition());
        telemetry.addData("beaconLeft",robot.beaconLeft.getPosition());
        telemetry.addData("beaconRight",robot.beaconRight.getPosition());
        telemetry.addData("Clear", robot.colorSensor.alpha());
        telemetry.addData("Red  ", robot.colorSensor.red());
        telemetry.addData("Green", robot.colorSensor.green());
        telemetry.addData("Blue ", robot.colorSensor.blue());

        // get the x, y, and z values (rate of change of angle).
        xVal = robot.gyro.rawX();
        yVal = robot.gyro.rawY();
        zVal = robot.gyro.rawZ();

        // get the heading info.
        // the Modern Robotics' gyro sensor keeps
        // track of the current heading for the Z axis only.
        gyroAngle = robot.gyro.getHeading();
        int standardAngle = robot.convertGyroToStandardAngle(gyroAngle);
        angleZ  = robot.gyro.getIntegratedZValue();


        targetVector = targetPosition.subtracted(currentPosition);


        float headingX = robot.getXFromStandardAngle(standardAngle);
        float headingY = robot.getYFromStandardAngle(standardAngle);
        VectorF headingVector = new VectorF(headingX, headingY);


        telemetry.addData("GyroAngle", "Heading %03d", gyroAngle);
        telemetry.addData("Standard Angle", "%03d", standardAngle);
        telemetry.addData("CurrentPos", "%5.2f %5.2f", currentPosition.get(0), currentPosition.get(1));
        telemetry.addData("TargetPos", "%5.2f %5.2f", targetPosition.get(0), targetPosition.get(1));
        telemetry.addData("TargetVector", "%5.2f %5.2f", targetVector.get(0), targetVector.get(1));
        telemetry.addData("HeadingVector", "%5.2f %5.2f", headingX, headingY);


        telemetry.addData("TargetAngleError", robot.getHeadingError(headingVector, targetVector));
        telemetry.addData("angZ", "Int. Ang. %03d", angleZ);
        telemetry.addData("X", "X av. %03d", xVal);
        telemetry.addData("Y", "Y av. %03d", yVal);
        telemetry.addData("Z", "Z av. %03d", zVal);
        telemetry.update();
    }
}



