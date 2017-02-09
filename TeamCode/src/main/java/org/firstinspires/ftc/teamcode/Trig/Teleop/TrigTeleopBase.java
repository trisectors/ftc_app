package org.firstinspires.ftc.teamcode.Trig.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.EyewearDevice;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.Trig.Auto.AutoBeacon.TrigAutoBeaconBase;
import org.firstinspires.ftc.teamcode.Trig.HardwareTrig;

import java.lang.Math;
import java.util.List;

public abstract class TrigTeleopBase extends OpMode {

    HardwareTrig robot = new HardwareTrig(telemetry, null);
    double sweepSpeed = 0.0;                   // initial sweep speed
    final double SWEEP_SPEED_OFFSET = 0.001;   // amount to change sweep speed when changing it with trigger buttons
    final double DRIVE_SPEED = 1.0;

    final double LEFT_BEACON_UP = 0;
    final double RIGHT_BEACON_UP = 1.0;
    final double RIGHT_BEACON_DOWN = 0;
    final double LEFT_BEACON_DOWN = 1;


    boolean beaconLeftDown = true;
    boolean beaconRightDown = true;
    boolean beaconLeftEnabled = true;
    boolean beaconRightEnabled = true;
    private boolean firing = false;

    protected List<VuforiaTrackable> allTrackables;
    protected OpenGLMatrix lastLocation;
    protected VectorF trans;
    protected Orientation rot;
    protected boolean pressingBeacon;


    // use the vizualization targets to determine the robot's location, if possible.  This will set lastLocation.
    private void getVuforiaLocation() {
        for (VuforiaTrackable trackable : allTrackables) {
            /**
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            VuforiaTrackableDefaultListener listener = ((VuforiaTrackableDefaultListener) trackable.getListener());
            //telemetry.addData(trackable.getName(), listener.isVisible() ? "Visible" : "Not Visible");    //

            OpenGLMatrix robotLocationTransform = listener.getUpdatedRobotLocation();
            listener.getRobotLocation();

            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
                // Then you can extract the positions and angles using the getTranslation and getOrientation methods.
                trans = robotLocationTransform.getTranslation();
                rot = Orientation.getOrientation(robotLocationTransform, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
            }
        }
    }

    public boolean getLocation() {
        lastLocation = null;

        getVuforiaLocation();
        if (lastLocation == null)
            return false;
            return  true;

    }

    private class VuforiaErrorAngleCalculator implements HardwareTrig.ErrorAngleCalculator
    {
        VectorF targetPos;
        VuforiaTrackable trackable;
        VuforiaErrorAngleCalculator() {
        }

        public double getErrorAngle() {
            getLocation();

            // get current location from vuforia
            // get robot x/y
            float robotX = trans.get(0);
            float robotY = trans.get(1);
            // Robot bearing (in Cartesian system) position is defined by the standard Matrix z rotation
            return -rot.thirdAngle * 360.0f / (2.0f * (float) Math.PI);
        }
    }

    void driveIntoBeacon() {
        if (!getLocation())
            return;
        VuforiaErrorAngleCalculator errorAngleCalculator = new VuforiaErrorAngleCalculator();
        robot.gyroDrive(DRIVE_SPEED / 2, 24, errorAngleCalculator);
    }
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
        allTrackables =  robot.initializeTrackables();

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


        //Drive into beacon if left trigger is pressed
        if (gamepad1.left_trigger > 0 && !pressingBeacon) {
            pressingBeacon = true;
            driveIntoBeacon();
        }
        if (gamepad1.left_trigger == 0) {
            pressingBeacon = false;
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