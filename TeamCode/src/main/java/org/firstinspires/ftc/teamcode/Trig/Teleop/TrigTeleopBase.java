package org.firstinspires.ftc.teamcode.Trig.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.Trig.HardwareTrig;

import java.lang.Math;
import java.util.ArrayList;
import java.util.List;

public abstract class TrigTeleopBase extends OpMode {

    HardwareTrig robot = new HardwareTrig(telemetry, null);
    double sweepSpeed = 0.0;  // initial sweep speed
    final double SWEEP_SPEED_OFFSET = 0.001;   // amount to change sweep speed when changing it with trigger buttons
    double cbrdPos = 1.0;
    final double CBRD_POS_OFFSET = 0.01;
    double DRIVE_SPEED = 1.0;
    final double STANDARD_DRIVE_SPEED = 1.0;

    boolean hammerBothEnabled = true;
    private boolean firing = false;

    protected List<VuforiaTrackable> allTrackables = new ArrayList<>();
    protected OpenGLMatrix lastLocation;
    protected VectorF trans;
    protected Orientation rot;
    protected boolean pressingBeacon;


    @Override
    protected void preInit() {
        msStuckDetectInit = 7500;
        super.preInit();
    }


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
        telemetry.addData("Got location:", lastLocation != null);
        telemetry.update();
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
            double err= -rot.thirdAngle * 360.0f / (2.0f * (float) Math.PI);
            telemetry.addData("vuforia error:",err);
            telemetry.update();
            return err;
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
        robot.eye.setPosition(0.0);
        robot.beaconRight.setPosition(HardwareTrig.RIGHT_BEACON_DOWN);
        robot.beaconLeft.setPosition(HardwareTrig.LEFT_BEACON_DOWN);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Good luck Driver! TRISECTORS RULE! May the Force be with you! You are one with the force. The force is with you! Help us Trisector you're our only hope!  GOOD LUCK! :)  ");
    }

    public abstract void steer();

      /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        robot.beaconRight.setPosition(HardwareTrig.RIGHT_BEACON_UP);
        robot.beaconLeft.setPosition(HardwareTrig.LEFT_BEACON_UP);

        // call abstract method to handle steering
        steer();

        // control Cap ball retention device
        if (gamepad2.y) {
            cbrdPos = 0.0;
        } else if (gamepad2.x) {
            cbrdPos = 1.0;
        } else {
            if(gamepad2.b)
                cbrdPos -= CBRD_POS_OFFSET;
            else if (gamepad2.a)
                cbrdPos += CBRD_POS_OFFSET;
            cbrdPos = Range.clip(cbrdPos, 0.0, 1.0);
        }
        robot.cbrd.setPosition(cbrdPos);

        // enable half speed for better control
        if (gamepad2.dpad_up) {
            DRIVE_SPEED = STANDARD_DRIVE_SPEED;
        }
        if (gamepad2.dpad_down) {
            DRIVE_SPEED = STANDARD_DRIVE_SPEED / 2;
        }


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
        if (hammerBothEnabled && gamepad1.dpad_down) {
            hammerBothEnabled = false;
            robot.hammerBoth();

        } else if (!gamepad1.dpad_down) {
            hammerBothEnabled = true;
        }



        if(gamepad2.right_trigger > 0) {
            robot.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightSlide.setPower(-0.1);
        }
        else if(gamepad2.right_bumper){
            robot.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightSlide.setPower(1.0);
        }
        else {
            robot.rightSlide.setPower(0.0);
        }


        if(gamepad2.left_trigger > 0) {
            robot.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           robot.leftSlide.setPower(-0.1);
        }
        else if(gamepad2.left_bumper){
            robot.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftSlide.setPower(1.0);
        }
        else {
            robot.leftSlide.setPower(0.0);
        }





        telemetry.addData("sweepSpeed", "Offset = %.2f", sweepSpeed);
        telemetry.addData("cbrdPos", "Offset = %.2f", cbrdPos);
        telemetry.update();
    }




    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}