package org.firstinspires.ftc.teamcode.Trig.Auto.AutoBeacon;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Trig.HardwareTrig;

/**
 * Created by FTC12073 on 2/8/17.
 */

@Autonomous(name = "TrigBot:AutoBeaconShoot Red ", group = "TrigBot")

public class AutoBeaconShootRed extends TrigAutoBeaconRed {
    public void executeMovements() {
        initializeTeam();

        //Drive forward using the gyro 30 inches.
        robot.gyroDrive(DRIVE_SPEED, 30.0, 0);

        // fire first particle
        robot.flickerFire();

        // load second ball:  turn sweep on, wait 2.5 sec, turn sweep off
        robot.sweepMotor.setPower(.4);
        sleep(3000);
        robot.sweepMotor.setPower(0);
        sleep(500);
        // fire second particle
        robot.flickerFire();
        robot.gyroTurn(TURN_SPEED, 65);
        robot.gyroDrive(DRIVE_SPEED, 35, 65);
        doBeaconPress(targetPositions[0], trackables[0], redTeam);

        //back up, turn and drive to second beacon position
        robot.gyroDrive(DRIVE_SPEED, -12, turnDirection);
        robot.gyroTurn(TURN_SPEED, farWallDirection);

        // arcTurn(DRIVE_SPEED, -1085, -3570)

        robot.gyroDrive(DRIVE_SPEED, 48, farWallDirection);
        robot.gyroTurn(TURN_SPEED, turnDirection);

        // reset location and beacon pressers
        lastLocation = null;
        robot.beaconLeft.setPosition(HardwareTrig.LEFT_BEACON_DOWN);
        robot.beaconRight.setPosition(HardwareTrig.RIGHT_BEACON_DOWN);

        // steer directly into the current beacon and press the correct button
        doBeaconPress(targetPositions[1], trackables[1], redTeam);

    }


    private void arcTurn(double speed, int leftOffset, int rightOffset) {
        int leftStart = robot.leftMotor.getCurrentPosition();
        int rightStart = robot.rightMotor.getCurrentPosition();

        int leftFinal = leftStart + leftOffset;
        int rightFinal = rightStart + rightOffset;

        // Set Target and Turn On RUN_TO_POSITION
        robot.leftMotor.setTargetPosition(leftFinal);
        robot.rightMotor.setTargetPosition(rightFinal);


        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (leftOffset < 0) {
            speed *= -1;
        }

        if (Math.abs(leftOffset) > Math.abs(rightOffset)) {
            float ratio = (float) rightOffset / (float) leftOffset;
            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(speed * ratio);
        } else {
            float ratio = (float) leftOffset / (float) rightOffset;
            robot.rightMotor.setPower(speed);
            robot.leftMotor.setPower(speed * ratio);
        }

        while (opModeIsActive() &&
                (robot.leftMotor.isBusy() || robot.rightMotor.isBusy())) {
            telemetry.update();
        }
    }

}
