package org.firstinspires.ftc.teamcode.Trig.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Trig.HardwareTrig;

import java.lang.Math;

public abstract class TrigTeleopBase extends OpMode {

    HardwareTrig robot = new HardwareTrig(telemetry, null);
    double sweepSpeed = 0.0;                   // initial sweep speed
    final double SWEEP_SPEED_OFFSET = 0.001;   // amount to change sweep speed when changing it with trigger buttons
    final double DRIVE_SPEED = 1.0;

    final double LEFT_BEACON_DOWN = 1.0;
    final double LEFT_BEACON_UP = 0.0;
    final double RIGHT_BEACON_UP = 1.0;
    final double RIGHT_BEACON_DOWN = 0.0;

    boolean beaconLeftDown = true;
    boolean beaconRightDown = true;
    boolean beaconLeftEnabled = true;
    boolean beaconRightEnabled = true;
    private boolean firing = false;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);

        robot.beaconRight.setPosition(RIGHT_BEACON_UP);
        robot.beaconLeft.setPosition(LEFT_BEACON_UP);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Good luck Driver!", "TRISECTORS RULE");
    }

    public abstract void steer();

      /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // call abstract method to handle steering
        steer();


        // Use gamepad left & right Bumpers and x to adjust sweep speed
        if (gamepad1.right_bumper)
            sweepSpeed += SWEEP_SPEED_OFFSET;
        else if (gamepad1.left_bumper)
            sweepSpeed -= SWEEP_SPEED_OFFSET;
        if (gamepad1.right_trigger > 0)
            sweepSpeed = .3;
        if (gamepad1.x) {
            sweepSpeed = (0.00);
        }
        robot.sweepMotor.setPower(sweepSpeed);


        // manually control flicker with A and Y
        if (gamepad1.a) {
            robot.flicker.setPower(robot.flickerSpeed);
        } else if (gamepad1.y) {
            robot.flicker.setPower(robot.manualReverseFlickerSpeed);
        } else {
            robot.flicker.setPower(0);
        }

        // automatic flicker control with B
        if (gamepad1.b && !firing) {
            firing = true;
            robot.flickerFire();
        }
        if (!gamepad1.b) {
            firing = false;
        }

        // beaconLeft Control with dpad_left
        if (beaconLeftEnabled && gamepad1.dpad_left) {
            beaconLeftEnabled = false;
            if (beaconLeftDown) {
                beaconLeftDown = false;
                robot.beaconLeft.setPosition(LEFT_BEACON_UP);
            } else {
                beaconLeftDown = true;
                robot.beaconLeft.setPosition(LEFT_BEACON_DOWN);
            }
        } else if (!gamepad1.dpad_left) {
            beaconLeftEnabled = true;
        }


        // beaconRight Control with dpad_right
        if (beaconRightEnabled && gamepad1.dpad_right) {
            beaconRightEnabled = false;
            if (beaconRightDown) {
                beaconRightDown = false;
                robot.beaconRight.setPosition(RIGHT_BEACON_DOWN);
            } else {
                beaconRightDown = true;
                robot.beaconRight.setPosition(RIGHT_BEACON_UP);
            }
        } else if (!gamepad1.dpad_right) {
            beaconRightEnabled = true;
        }

        telemetry.addData("sweepSpeed", "Offset = %.2f", sweepSpeed);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}