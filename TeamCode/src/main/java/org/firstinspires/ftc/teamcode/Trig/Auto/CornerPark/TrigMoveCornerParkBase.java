package org.firstinspires.ftc.teamcode.Trig.Auto.CornerPark;

import org.firstinspires.ftc.teamcode.Trig.TrigAutoBase;

// base class for firing two particles then parking in the corner
public abstract class TrigMoveCornerParkBase extends TrigAutoBase {

    protected double initialAngle;

    protected double cornerAngle;

    public abstract void waitForDelay();

    public void turnToCorner() {
        robot.gyroTurn(ORIGINAL_DRIVE_SPEED/4, cornerAngle);
    }


    public abstract void initializeTeam();
    @Override
    public void executeMovements() {

        // set up alliance-specific variables
        initializeTeam();

        waitForDelay(); // call delay method, this class waits 0 sec, but may be overridden

        //Drive forward using the gyro 30 inches.
        if (useEncoderDrive) {
            encoderDrive(ORIGINAL_DRIVE_SPEED, 30.0, 30.0, initialAngle);
        } else {
            robot.gyroDrive(ORIGINAL_DRIVE_SPEED, 30.0, initialAngle);
        }
        // fire first particle
        robot.flickerFire();
        // load second ball:  turn boot on, wait 2.5 sec, turn boot down
        robot.sweepMotor.setPower(.4);
        sleep(3000);
        robot.sweepMotor.setPower(0);
        sleep(500);
        // fire second particle
        robot.flickerFire();
        // turn towards the corner vortex and park on it.
        sleep(500);
        turnToCorner();  //turn
        if (useEncoderDrive) {
            if (cornerAngle == -55) {
                // redTeam
                encoderDrive(DRIVE_SPEED * .5, 17.5, -17.5, 4.0);
            } else {
                encoderDrive(DRIVE_SPEED * .5,  -17.5, 17.5, 4.0);
            }

        } else {
            robot.gyroDrive(ORIGINAL_DRIVE_SPEED, -40.0, cornerAngle);  //back up into corner
        }
        telemetry.update();
    }
}