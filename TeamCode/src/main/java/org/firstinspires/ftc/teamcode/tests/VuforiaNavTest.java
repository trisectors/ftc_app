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

import java.util.ArrayList;
import java.util.List;

/**
 * This OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 *
 * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * This example assumes a "diamond" field configuration where the red and blue alliance stations
 * are adjacent on the corner of the field furthest from the audience.
 * From the Audience perspective, the Red driver station is on the right.
 * The two vision target are located on the two walls closest to the audience, facing in.
 * The Stones are on the RED side of the field, and the Chips are on the Blue side.
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Autonomous(name="Concept: Vuforia Nav Test", group ="Concept")
//@Disabled
public class VuforiaNavTest extends LinearOpMode {

    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ASj98IX/////AAAAGeDoiG9tFUO7oBKP9PIwRbMP/FZQcvSH/tItsosVh/vnvrW/LhNBMiOrDdARozpBq6yQomYYJmMWT5kTS3b48Dy0+7yu/rEf0ATZCxAJ1sk9qBnuru/AwWiEs+fwaoFro0Br7OnViNLhp1Kh8zE6dnTYBoRTlAUKR3JkJBGWMSi9YMu7hwa0jmlMr3FvsE/xZt/j+rK2yNH3Qj91nhnU6pBwcoOh6crunuk/BUDnNt2+3kmCt+VhiFzgau/cMNOBfvgb/U4Tzw3WfAwJjEVHAZbmKpKLuiWu5QXnCi7C0HfefLkLe0BvWUo9EnFO5mTp/dMuw5X0lm2b67eP5snY2zsxceiOlBjSu+AXE5S5vhXt";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables FTC_2016 = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
        VuforiaTrackable wheels  = FTC_2016.get(0);
        wheels.setName("Wheels");
        VuforiaTrackable tools  = FTC_2016.get(1);
        tools.setName("Tools");
        VuforiaTrackable legos  = FTC_2016.get(2);
        legos.setName("LEGOS");
        VuforiaTrackable gears = FTC_2016.get(3);
        gears.setName("Gears");
        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(FTC_2016);

        /**
         * We use units of mm here because that's the recommended units of measurement for the
         * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
         *      <ImageTarget name="stones" size="247 173"/>
         * You don't *have to* use mm here, but the units here and the units used in the XML
         * target configuration files *must* correspond for the math to work out correctly.
         */
        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels


        initTargets(wheels, tools, legos, gears, mmFTCFieldWidth);

        // Update location if we have a new one.


//        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
//                .translation(mmBotWidth/2,0,0)
//                .multiplied(Orientation.getRotationMatrix(
//                        AxesReference.EXTRINSIC, AxesOrder.YZY,
//                        AngleUnit.DEGREES, -90, 0, 0));
//        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));
//
//        /**
//         * Let the trackable listeners we care about know where the phone is. We know that each
//         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
//         * we have not ourselves installed a listener of a different type.
//         */
//        ((VuforiaTrackableDefaultListener)gears.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
//        ((VuforiaTrackableDefaultListener)tools.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
//        ((VuforiaTrackableDefaultListener)wheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
//        ((VuforiaTrackableDefaultListener)legos.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

      waitForStart();

        /** Start tracking the data sets we care about. */
        FTC_2016.activate();

        VectorF trans= null;
        Orientation rot= null;
        while (opModeIsActive()) {

            for (VuforiaTrackable trackable : allTrackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */

                VuforiaTrackableDefaultListener listener = ((VuforiaTrackableDefaultListener) trackable.getListener());

                telemetry.addData(trackable.getName(), listener.isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = listener.getUpdatedRobotLocation();

                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;


                    // Then you can extract the positions and angles using the getTranslation and getOrientation methods.
                     trans = robotLocationTransform.getTranslation();
                     rot = Orientation.getOrientation(robotLocationTransform, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

                    // Robot position is defined by the standard Matrix translation (x and y)




                }
            }
            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastLocation != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                //telemetry.addData("Pos", format(lastLocation));

                float robotX = trans.get(0);
                float robotY = trans.get(1);

                // Robot bearing (in Cartesian system) position is defined by the standard Matrix z rotation
                float robotBearing = rot.thirdAngle * 360.0f / (2.0f*(float)Math.PI);

                telemetry.addData("X", robotX);
                telemetry.addData("Y", robotY);
                telemetry.addData("BEARING", robotBearing);
            }
            else {
                //telemetry.addData("Pos", "Unknown");


            }


            telemetry.update();
        }
    }

    public void initTargets(VuforiaTrackable wheels, VuforiaTrackable tools, VuforiaTrackable legos, VuforiaTrackable gears, float mmFTCFieldWidth) {
        OpenGLMatrix gearsLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(0, -304.8f, 0)
                .translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        gears.setLocation(gearsLocationOnField);
        RobotLog.ii(TAG, "Gear Target=%s", format(gearsLocationOnField));


        OpenGLMatrix toolsLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(0, 914.4f, 0)
                .translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        tools.setLocation(toolsLocationOnField);
        RobotLog.ii(TAG, "Tools Target=%s", format(toolsLocationOnField));



       /*
        * To place the Stones Target on the Blue Audience wall:
        * - First we rotate it 90 around the field's X axis to flip it upright
        * - Finally, we translate it along the Y axis towards the blue audience wall.
        */
        OpenGLMatrix wheelsLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(304.8f, 0, 0 )
                .translation(0, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        wheels.setLocation(wheelsLocationOnField);
        RobotLog.ii(TAG, "Wheel Target=%s", format(wheelsLocationOnField));

        OpenGLMatrix legosLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(-914.4f, 0, 0)
                .translation(0, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        legos.setLocation(legosLocationOnField);
        RobotLog.ii(TAG, "Lego Target=%s", format(legosLocationOnField));
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}
