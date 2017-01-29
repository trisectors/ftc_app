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
package org.firstinspires.ftc.teamcode.Trig.Auto.AutoBeacon;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.Trig.TrigAutoBase;

import java.util.ArrayList;
import java.util.List;

// base class for firing two particles then parking in the corner
@Autonomous(name = "TrigBot:Auto Beacon ", group = "TrigBot")
public  class TrigAutoBeacon extends TrigAutoBase {
    private static double MM_PER_INCH=25.4;
    public  void waitForDelay(){}

    VuforiaLocalizer vuforia;
    VectorF trans;
    Orientation rot;
    VectorF[] targetPositions = {
            new VectorF(-1537.4f, -304.8f, 0.0f),
            new VectorF(-1537.4f, 914.4f, 0.0f)         // Yes
    };

    VectorF targetPosition = null;
    double DRIVE_SPEED = 0.7f;
    double TURN_SPEED=DRIVE_SPEED/4;

    class NavParams
    {
        VectorF targetVector;
        double targetGyroAngle;
    }

    public NavParams getNavParams (final VectorF targetPosition) {
        NavParams navParams = new NavParams();

        do {
            //telemetry.addData("Getting location:","working...");
            //telemetry.update();
            getVuforiaLocation();
            if (lastLocation == null) {
                sleep(1000);
            }
        } while ((opModeIsActive() && lastLocation == null));

        // get current location from vuforia
        // get robot x/y
        float robotX = trans.get(0);
        float robotY = trans.get(1);


        int gyroZ = robot.gyro.getIntegratedZValue();
        int gyroH = robot.gyro.getHeading();
        int standardAngle = robot.convertGyroToStandardAngle(gyroH);
        VectorF currentPosition = new VectorF(robotX, robotY, 0.0f);
        navParams.targetVector = targetPosition.subtracted(currentPosition);
        float headingX = robot.getXFromStandardAngle(standardAngle);
        float headingY = robot.getYFromStandardAngle(standardAngle);

        VectorF headingVector = new VectorF(headingX, headingY, 0.0f);
        telemetry.addData("gyroZ/gyroH/std", "%03d / %03d / %03d", gyroZ, gyroH, standardAngle);
        telemetry.addData("Pos: Curr/Targ", "%5.2f %5.2f / %5.2f %5.2f", currentPosition.get(0), currentPosition.get(1), targetPosition.get(0), targetPosition.get(1));
        telemetry.addData("TargetVector(magnitude)", "%5.2f %5.2f (%5.2f)",
                navParams.targetVector.get(0), navParams.targetVector.get(1), navParams.targetVector.magnitude());
        double headingError = robot.getHeadingError(headingVector, navParams.targetVector);
        navParams.targetGyroAngle = gyroZ + headingError;



        telemetry.addData("headingError (deg)", robot.getHeadingError(headingVector, navParams.targetVector));
        telemetry.addData("target Angle", navParams.targetGyroAngle);
        telemetry.addData("HeadingVector", "%5.2f %5.2f", headingX, headingY);
        telemetry.update();
        return navParams;
    }

    public void doBeaconPress(VectorF targetPosition) {
        // get navigation parameters from vuforia
        NavParams navParams = this.getNavParams(targetPosition);
        // TURN TO TARGET POS.
        robot.gyroTurn(TURN_SPEED, navParams.targetGyroAngle);
        // Drive to target pos.
        robot.gyroDrive(DRIVE_SPEED, navParams.targetVector.magnitude()/25.4, navParams.targetGyroAngle);
        // Turn to wall
        robot.gyroTurn(TURN_SPEED,  85);
        //Press beacon
        selectAndPressBeacon(true);

    }

    OpenGLMatrix lastLocation = null;
    @Override
    public void executeMovements() {
        telemetry.log().setCapacity(10);

        waitForDelay();             // call delay method, this class waits 0 sec, but may be overridden

        // drive near beacon
        robot.gyroDrive(DRIVE_SPEED, 55, 30);
        robot.gyroTurn(TURN_SPEED, 90);

        doBeaconPress(targetPositions[0]);

        encoderDrive(DRIVE_SPEED, -24, -24, 5);
        //simpleGyroTurn(TURN_SPEED, -180);
        //robot.flickerFire();

/*
        //back up
        robot.gyroDrive(DRIVE_SPEED, -24, 90);  // back up
        robot.gyroTurn(TURN_SPEED,0);           // turn to front wall
        robot.gyroDrive(DRIVE_SPEED, 48, 0);    // drive to beacon
        robot.gyroTurn(TURN_SPEED,90);          // turn to beacon


        lastLocation = null;
        doBeaconPress(targetPositions[1]);
*/
    }

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

}
