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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.Trig.HardwareTrig;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Trig: Teleop Tank", group="Trig")
//@Disabled
public class TrigTeleopTank_Iterative extends OpMode{

    /* Declare OpMode members. */
    HardwareTrig robot       = new HardwareTrig(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    double          sweepSpeed  = 0.0 ;                  // Servo mid position
    final double SWEEP_SPEED = 0.001 ;                 // sets rate to move servo

    double          DRIVE_SPEED= .8;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)

        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;
        robot.leftMotor.setPower(right * DRIVE_SPEED);
        robot.rightMotor.setPower(left * DRIVE_SPEED);

        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad1.right_bumper)
            sweepSpeed += SWEEP_SPEED;
        else if (gamepad1.left_bumper)
            sweepSpeed -= SWEEP_SPEED;
        robot.sweepMotor.setPower(sweepSpeed);

        if (gamepad1.right_trigger > 0)
            sweepSpeed = .3;

        if (gamepad1.x) {
            sweepSpeed = (0.00);
        }

        if (gamepad1.a) {
            robot.flicker.setPower(10);
        } else if (gamepad1.y) {
            robot.flicker.setPower(-.08);
        } else {
            robot.flicker.setPower(0);

            if (gamepad1.dpad_right) {
                robot.button.setPosition(0.15);
            }
            if (gamepad1.dpad_left)
                robot.button.setPosition(.85);

            if (gamepad1.dpad_up) {
                robot.button.setPosition(.5);
                telemetry.addData("servo:", "center");
            }

            if (gamepad1.left_trigger > 0) {
                robot.arm1.setPower(1);
                robot.arm2.setPower(1);
            }
            else{
                robot.arm1.setPower(0);
                robot.arm2.setPower(0);
            }

        }
        // Use gamepad buttons to move the arm up (Y) and down (A)
        //     if (gamepad1.y)
        //         robot.armMotor.setPower(robot.ARM_UP_POWER);
        //     else if (gamepad1.a)
        //         robot.armMotor.setPower(robot.ARM_DOWN_POWER);
        //     else
        //         robot.armMotor.setPower(0.0);

        // Send telemetry message to signify robot running;
        telemetry.addData("sweepSpeed", "Offset = %.2f", sweepSpeed);
        telemetry.addData("left", "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        ;
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
