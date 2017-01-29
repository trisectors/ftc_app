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


@Autonomous(name = "Test: Vuforia Nav Test", group = "Test")
//@Disabled
public class VuforiaNavTest extends LinearOpMode {

    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;
    VectorF targetPosition = new VectorF(-1549.4f, -304.8f);

    @Override
    public void runOpMode() {
        HardwareTrig robot = new HardwareTrig(telemetry,this);
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
        List<VuforiaTrackable> allTrackables = robot.initializeTrackables();
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
                VectorF currentPosition = new VectorF(robotX, robotY, 0.0f);
                VectorF targetVector = targetPosition.subtracted(currentPosition);
                float headingX = robot.getXFromStandardAngle(standardAngle);
                float headingY = robot.getYFromStandardAngle(standardAngle);
                VectorF headingVector = new VectorF(headingX, headingY, 0.0f);
                telemetry.addData("CurrentPos", "%5.2f %5.2f", currentPosition.get(0), currentPosition.get(1));
                telemetry.addData("TargetVector", "%5.2f %5.2f", targetVector.get(0), targetVector.get(1));
                telemetry.addData("HeadingVector", "%5.2f %5.2f", headingX, headingY);
                telemetry.addData("Target angle error",robot.getHeadingError(headingVector,targetVector));
            }


            telemetry.update();
        }
    }

}
