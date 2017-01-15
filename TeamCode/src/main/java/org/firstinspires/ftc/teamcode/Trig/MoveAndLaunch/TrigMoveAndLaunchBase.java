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
package org.firstinspires.ftc.teamcode.Trig.MoveAndLaunch;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Trig.HardwareTrig;
import org.firstinspires.ftc.teamcode.Trig.TrigAutoBase;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Disabled
public class TrigMoveAndLaunchBase extends TrigAutoBase {


    public void turnToBall() {}


    @Override
    public void doOpMode() {



        waitForDelay(); // call delay method, this class waits 0 sec, but may be overridden

        //Drive forward 2ft
        robot.gate.setPosition(GATE_DOWN_POSITION);
        encoderDrive(DRIVE_SPEED,  52.5,  52.5, 5.0);

        // fire first particle: turn flicker on to 100, wait half second, turn flicker off
        robot.flicker.setPower(1.00);
        sleep(500);
        robot.flicker.setPower(0);

        // load second ball:  turn sweep on, wait 5 sec, turn sweep off
        sleep(500);
        robot.gate.setPosition(GATE_UP_POSITION);
        robot.sweepMotor.setPower(.5);      // This is for the second ball
        sleep(2500);
        robot.sweepMotor.setPower(0);
        robot.gate.setPosition(GATE_DOWN_POSITION);

        // fire second particle: wait half sec, turn flicker on, wait half second, turn flicker off
        sleep(500);
        robot.flicker.setPower(1.00);
        sleep(500);
        robot.flicker.setPower(0);

        // knock ball off center platform: drive forward 1/2 foot and park
        sleep(500);
        encoderDrive(1.0,  18, 18, 4.0);  //ram
        encoderDrive(1.0, -4, -4, 4.0);  //back up

        encoderDrive(DRIVE_SPEED, 4, 4, 4.0); // go forward again

        turnToBall();

        encoderDrive(DRIVE_SPEED, 10,10,4);
        sleep(10000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


}
