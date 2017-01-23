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

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.RobotLog;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
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
import org.firstinspires.ftc.teamcode.Trig.HardwareTrig;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name = "Concept: Vuforia Nav Test", group = "Concept")
//@Disabled
public class VuforiaNavTest extends LinearOpMode {

    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;
    VectorF targetPosition = new VectorF(-1549.4f, -304.8f);

    public List<VuforiaTrackable> initializeTrackables() {
        VuforiaLocalizer vuforia;

        // initialize vuforia library.  First we set up the license key and camera direction and create the vuforia object
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
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


    @Override
    public void runOpMode() {
        HardwareTrig robot = new HardwareTrig(telemetry);
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        // make sure the gyro is calibrated before continuing
        robot.gyro.calibrate();
        while (robot.gyro.isCalibrating()) {
            telemetry.addData("calibrating gyro...", true);
            telemetry.update();
        }
        telemetry.addData("Done calibrating gyro...", true);
        telemetry.update();

        // NOTE: investigate doing this after start it it's quick....
        List<VuforiaTrackable> allTrackables = initializeTrackables();
        waitForStart();

        VectorF trans = null;
        Orientation rot = null;
        while (opModeIsActive()) {
            // get the heading info.
            // the Modern Robotics' gyro sensor keeps
            // track of the current heading for the Z axis only.
            int gyroAngle = robot.gyro.getHeading();
            int standardAngle = robot.convertGyroToStandardAngle(gyroAngle);
            // angleZ  = robot.gyro.getIntegratedZValue();


            telemetry.addData("GyroAngle", "Heading %03d", gyroAngle);
            telemetry.addData("Standard Angle", "%03d", standardAngle);
            telemetry.addData("TargetPos", "%5.2f %5.2f", targetPosition.get(0), targetPosition.get(1));


            for (VuforiaTrackable trackable : allTrackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */
                VuforiaTrackableDefaultListener listener = ((VuforiaTrackableDefaultListener) trackable.getListener());
                telemetry.addData(trackable.getName(), listener.isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = listener.getUpdatedRobotLocation();
                listener.getRobotLocation();

                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                    // Then you can extract the positions and angles using the getTranslation and getOrientation methods.
                    trans = robotLocationTransform.getTranslation();
                    rot = Orientation.getOrientation(robotLocationTransform, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
                }
            }

            if (lastLocation != null) {
                float robotX = trans.get(0);
                float robotY = trans.get(1);

                // Robot bearing (in Cartesian system) position is defined by the standard Matrix z rotation
                // float robotBearing = rot.thirdAngle * 360.0f / (2.0f * (float) Math.PI);
                telemetry.addData("X", robotX);
                telemetry.addData("Y", robotY);
                // telemetry.addData("BEARING", robotBearing);
                VectorF currentPosition = new VectorF(robotX, robotY);
                VectorF targetVector = targetPosition.subtracted(currentPosition);
                float headingX = robot.getXFromStandardAngle(standardAngle);
                float headingY = robot.getYFromStandardAngle(standardAngle);
                VectorF headingVector = new VectorF(headingX, headingY);
                telemetry.addData("CurrentPos", "%5.2f %5.2f", currentPosition.get(0), currentPosition.get(1));
                telemetry.addData("TargetVector", "%5.2f %5.2f", targetVector.get(0), targetVector.get(1));
                telemetry.addData("HeadingVector", "%5.2f %5.2f", headingX, headingY);
            }


            telemetry.update();
        }
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}
