package org.firstinspires.ftc.teamcode.Trig;

import android.hardware.Sensor;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;


// This class defines the hardware for the trigbot and some methods which are useful in all trig opmodes
public class HardwareTrig {
    public static final double LEFT_BEACON_DOWN = 1.0;    /* Public OpMode members. */
    public static final double LEFT_BEACON_UP = 0.0;
    public static final double LEFT_BEACON_PARTIAL_DOWN = 0.2;
    public static final double RIGHT_BEACON_UP = 1.0;
    public static final double RIGHT_BEACON_DOWN = 0.0;
    public static final double RIGHT_BEACON_PARTIAL_DOWN = 0.8;
    public static final double COUNTS_PER_MOTOR_REV = 1440;
    public static final double DRIVE_GEAR_REDUCTION = 1;
    public static final double FLICKER_GEAR_REDUCTION = 2;
    public static final double COUNTS_PER_FLICKER_REVOLUTION = COUNTS_PER_MOTOR_REV / FLICKER_GEAR_REDUCTION;
    public static final double WHEEL_DIAMETER_INCHES = 4;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    
    public final double flickerSpeed = 1.0;
    public final double reverseFlickerSpeed = -0.4;
    public final double manualReverseFlickerSpeed = -.05;
    private final Telemetry telemetry;
    private final LinearOpMode opMode;


    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.015;     // Larger is more responsive, but also less stable

    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor sweepMotor = null;
    public DcMotor flicker = null;
    public Servo beaconLeft = null;
    public Servo beaconRight = null;
    public Servo gate = null;
    public ModernRoboticsI2cGyro gyro = null;
    public ColorSensor colorSensor = null;
    public ElapsedTime runtime = new ElapsedTime();   // used for encoderDrive's timeout

    public static final String TAG = "TrigBotTag";


    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HardwareTrig(Telemetry telemetry, LinearOpMode opMode) {
        this.telemetry = telemetry;
        this.opMode = opMode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor = hwMap.dcMotor.get("left_drive");
        rightMotor = hwMap.dcMotor.get("right_drive");
        sweepMotor = hwMap.dcMotor.get("sweep");
        flicker = hwMap.dcMotor.get("flicker");
        beaconLeft = hwMap.servo.get("beaconLeft");
        beaconRight = hwMap.servo.get("beaconRight");
        gate = hwMap.servo.get("gate");
        gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");
        colorSensor = hwMap.colorSensor.get("sensor_color");
        colorSensor.enableLed(false);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        sweepMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flicker.setDirection(DcMotorSimple.Direction.FORWARD);


        // Set all motors and servos to either zero power or a position
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sweepMotor.setPower(0);
        flicker.setPower(0);
        gate.setPosition(0);
        beaconLeft.setPosition(LEFT_BEACON_UP);
        beaconRight.setPosition(RIGHT_BEACON_UP);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sweepMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flicker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /***
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }


    public void flickerFire() {

        double timeoutS = 2.0;

        int startPosition = flicker.getCurrentPosition();

        // Determine new target position, and pass to motor controller
        int newTarget = flicker.getCurrentPosition() + (int) (1.5 * COUNTS_PER_FLICKER_REVOLUTION);
        int finalTarget = flicker.getCurrentPosition() + (int) (COUNTS_PER_FLICKER_REVOLUTION);
        flicker.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        flicker.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start backward motion.
        runtime.reset();
        flicker.setPower(Math.abs(flickerSpeed));

        // keep looping while we are still active, and there is time left
        while ((runtime.seconds() < timeoutS) && (flicker.isBusy())) {
            telemetry.addData("newTarget:", newTarget);
            telemetry.addData("flicker.currentPosition:", flicker.getCurrentPosition());
            telemetry.update();
        }

        // reset the timeout time and start backwards motion to return to initial position
        runtime.reset();
        flicker.setPower(Math.abs(reverseFlickerSpeed));
        flicker.setTargetPosition(finalTarget);

        // keep looping while we are still active, and there is time left
        while ((runtime.seconds() < timeoutS) && (flicker.isBusy())) {
            telemetry.addData("newTarget:", newTarget);
            telemetry.addData("flicker.currentPosition:", flicker.getCurrentPosition());
            telemetry.update();
        }


        // Stop all motion;
        flicker.setPower(0);
        // Turn off RUN_TO_POSITION
        waitForTick(100);
        flicker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int convertGyroToStandardAngle(int gyroAngle) {
        if (gyroAngle > 180)
            gyroAngle = gyroAngle - 360;
        return 90 - gyroAngle;
    }

    public float getXFromStandardAngle(int standardAngle) {
        return (float) Math.cos(Math.toRadians(standardAngle));
    }

    public float getYFromStandardAngle(int standardAngle) {
        return (float) Math.sin(Math.toRadians(standardAngle));
    }

    public double getHeadingError(VectorF currentHeading, VectorF targetHeading) {
        double dot = currentHeading.dotProduct(targetHeading);
        double rawError = Math.toDegrees(Math.acos(dot / (currentHeading.magnitude() * targetHeading.magnitude())));

        // get angle of error.
        // see: http://stackoverflow.com/questions/5188561/signed-angle-between-two-3d-vectors-with-same-origin-within-the-same-plane
        // NOTE: we may have to reverse < 0 here for > depending on rotation convention
        double cross = currentHeading.get(0) * targetHeading.get(1) - currentHeading.get(1) * targetHeading.get(0);
        if (cross < 0) {
            rawError = -rawError;
        }

        return rawError;
    }


    public List<VuforiaTrackable> initializeTrackables() {
        VuforiaLocalizer vuforia;

        // initialize vuforia library.  First we set up the license key and camera direction and create the vuforia object
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ASj98IX/////AAAAGeDoiG9tFUO7oBKP9PIwRbMP/FZQcvSH/tItsosVh/vnvrW/LhNBMiOrDdARozpBq6yQomYYJmMWT5kTS3b48Dy0+7yu/rEf0ATZCxAJ1sk9qBnuru/AwWiEs+fwaoFro0Br7OnViNLhp1Kh8zE6dnTYBoRTlAUKR3JkJBGWMSi9YMu7hwa0jmlMr3FvsE/xZt/j+rK2yNH3Qj91nhnU6pBwcoOh6crunuk/BUDnNt2+3kmCt+VhiFzgau/cMNOBfvgb/U4Tzw3WfAwJjEVHAZbmKpKLuiWu5QXnCi7C0HfefLkLe0BvWUo9EnFO5mTp/dMuw5X0lm2b67eP5snY2zsxceiOlBjSu+AXE5S5vhXt";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        // load this year's trackables images from resources
        VuforiaTrackables FTC_2016 = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        VuforiaTrackable wheels = FTC_2016.get(0);
        wheels.setName("Wheels");
        VuforiaTrackable tools = FTC_2016.get(1);
        tools.setName("Tools");
        VuforiaTrackable legos = FTC_2016.get(2);
        legos.setName("LEGOS");
        VuforiaTrackable gears = FTC_2016.get(3);
        gears.setName("Gears");

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(FTC_2016);

        // now, we begin specifying the location of each vision target on the field.
        float mmPerInch = 25.4f;
        float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels


        // Image locations:  Images start face up at the origin, so we have to rotate and translate them to their correct position.

        // Gears: rotate 90 deg around X, then 90 around Z
        // then translate half the field width in X and -1ft in Y.  -1*12*25.4=-304.8
        // center of image is 8.5in (paper width)/2 + 1 1/2 in (height image should be mounted off floor) 8.5/2+1.5*25.4=42.35
        OpenGLMatrix gearsLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth / 2, -304.8f, 42.35f)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        gears.setLocation(gearsLocationOnField);
        RobotLog.ii(TAG, "Gear Target=%s", format(gearsLocationOnField));

        // Tools: rotate 90 deg around X, then 90 around Z
        // then translate half the field width in X and +3ft in Y.  33*12*25.4=914.4
        // center of image is 8.5in (paper width)/2 + 1 1/2 in (height image should be mounted off floor) 8.5/2+1.5*25.4=42.35
        OpenGLMatrix toolsLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth / 2, 914.4f, 42.35f)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        tools.setLocation(toolsLocationOnField);
        RobotLog.ii(TAG, "Tools Target=%s", format(toolsLocationOnField));

        //Wheels: rotate 90 deg around X
        // then translate half the field width in Y, and +1 ft in X 1*12*25.4=304.8
        // center of image is 8.5in (paper width)/2 + 1 1/2 in (height image should be mounted off floor) 8.5/2+1.5*25.4=42.35
        OpenGLMatrix wheelsLocationOnField = OpenGLMatrix
                .translation(304.8f, mmFTCFieldWidth / 2, 42.35f)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        wheels.setLocation(wheelsLocationOnField);
        RobotLog.ii(TAG, "Wheel Target=%s", format(wheelsLocationOnField));

        //Legos: rotate 90 deg around X
        // then translate half the field width in Y, and -3 ft in X =3*12*25.4=-914.4
        // center of image is 8.5in (paper width)/2 + 1 1/2 in (height image should be mounted off floor) 8.5/2+1.5*25.4=42.35
        OpenGLMatrix legosLocationOnField = OpenGLMatrix
                .translation(-914.4f, mmFTCFieldWidth / 2, 42.35f)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        legos.setLocation(legosLocationOnField);
        RobotLog.ii(TAG, "Lego Target=%s", format(legosLocationOnField));


        // set phone location on robot
        // phone location by default is center of bot.
        // ours is 6.5 in towards the front, so +6.5*25.4 = 165.1mm translated in y.
        // the camera is also translated off the floor (z axis by about 5 in) 5*25.4=127
        // it appears the default orientation is in portrait mode aligned with the bot's axes, so our phone is rotated 90deg around x then 180 around y.
        // Yes...it's upside down!
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(0, 165.1f, 0)
                .translation(0, 0, 127.0f)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYX,
                        AngleUnit.DEGREES, 90, 180, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener) gears.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) tools.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) wheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) legos.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        /** Start tracking the data sets we care about. */
        FTC_2016.activate();
        return allTrackables;
    }
    

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }


    public interface ErrorAngleCalculator
    {
        public double getErrorAngle();
    };

    private class GyroErrorAngleCalculator implements ErrorAngleCalculator
    {
        double angle;

        GyroErrorAngleCalculator(double angle) {
            this.angle = angle;
        }

        public double getErrorAngle() {
            return getGyroAngleError(angle);
        }
    }

    public void gyroDrive(double speed,
                          double distance,
                          final double angle) {
        ErrorAngleCalculator errorAngleCalculator = new GyroErrorAngleCalculator(angle);

        gyroDrive(speed, distance, errorAngleCalculator);
    }


    public void gyroDrive (double speed,
                           double distance,
                           ErrorAngleCalculator errorAngleCalculator) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opMode == null || opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = leftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = rightMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while ((opMode == null || opMode.opModeIsActive()) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = errorAngleCalculator.getErrorAngle();
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftMotor.setPower(leftSpeed);
                rightMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        
        
        
        
    }
    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getGyroAngleError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftMotor.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getGyroAngleError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


}


