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

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.Trig.HardwareTrig;
import org.firstinspires.ftc.teamcode.Trig.TrigAutoBase;


class NavParams {
    VectorF targetVector;
    double targetBearingAngle;
    double robotBearing;
    double headingError;
}


// base class for firing two particles then parking in the corner
public abstract class TrigAutoBeaconBase extends TrigAutoBase {
    private static double MM_PER_INCH = 25.4;

    protected VuforiaLocalizer vuforia;
    protected VectorF trans;
    protected Orientation rot;
    protected VectorF[] targetPositions = {
            new VectorF(-1625f, -304.8f, 0.0f),
            new VectorF(-1625f, 914.4f, 0.0f)
    };
    protected VuforiaTrackable[] trackables = new VuforiaTrackable[2];

    protected VectorF targetPosition = null;
    protected OpenGLMatrix lastLocation = null;
    protected boolean redTeam;
    protected int turnDirection;
    protected int farWallDirection;
    protected double initialAngle;

    protected int beaconDistanceOffset = 0;  // how much do we need to compensate for the beacon distance?


    public void waitForDelay() {}
    public abstract void initializeTeam();



    // use the vizualization targets to determine the robot's location, if possible.  This will set lastLocation.
    private void getVuforiaLocation(VuforiaTrackable trackable) {
    //    for (VuforiaTrackable trackable : allTrackables) {
            /**
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            VuforiaTrackableDefaultListener listener = ((VuforiaTrackableDefaultListener) trackable.getListener());
            //telemetry.addData(trackable.getName(), listener.isVisible() ? "Visible" : "Not Visible");    //

            OpenGLMatrix robotLocationTransform = listener.getUpdatedRobotLocation();
            // listener.getRobotLocation();

            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
                // Then you can extract the positions and angles using the getTranslation and getOrientation methods.
                trans = robotLocationTransform.getTranslation();
                rot = Orientation.getOrientation(robotLocationTransform, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
            }
    //    }
    }

    // use the gyro to get navigation parameters needed to drive to the given target position
    public NavParams getGyroNavParams(final VectorF targetPosition, VuforiaTrackable trackable) {
        NavParams navParams = new NavParams();

        do {
            //telemetry.addData("Getting location:","working...");
            //telemetry.update();
            getVuforiaLocation(trackable);
            if (lastLocation == null) {
                sleep(100);
            }
        } while ((opModeIsActive() && lastLocation == null));

        // get current location from vuforia
        // get robot x/y
        float robotX = trans.get(0);
        float robotY = trans.get(1);
        // Robot bearing (in Cartesian system) position is defined by the standard Matrix z rotation
        navParams.robotBearing = rot.thirdAngle * 360.0f / (2.0f * (float) Math.PI);


        // get current heading from gyro
        int gyroZ = robot.gyro.getIntegratedZValue();
        int gyroH = robot.gyro.getHeading();
        int standardAngle = robot.convertGyroToStandardAngle(gyroH);
        VectorF currentPosition = new VectorF(robotX, robotY, 0.0f);
        float headingX = robot.getXFromStandardAngle(standardAngle);
        float headingY = robot.getYFromStandardAngle(standardAngle);
        VectorF headingVector = new VectorF(headingX, headingY, 0.0f);

        navParams.targetVector = targetPosition.subtracted(currentPosition);
        navParams.headingError = robot.getHeadingError(headingVector, navParams.targetVector);
        navParams.targetBearingAngle = gyroZ + navParams.headingError;

        telemetry.addData("gyroZ/gyroH/std/bearing", "%03d / %03d / %03d / %5.3f", gyroZ, gyroH, standardAngle, navParams.robotBearing);
        telemetry.addData("Pos: Curr/Targ", "%5.2f %5.2f / %5.2f %5.2f", currentPosition.get(0), currentPosition.get(1), targetPosition.get(0), targetPosition.get(1));
        telemetry.addData("TargetVector(magnitude)", "%5.2f %5.2f (%5.2f)",
                navParams.targetVector.get(0), navParams.targetVector.get(1), navParams.targetVector.magnitude());
        telemetry.addData("headingError (deg)", navParams.headingError);
        telemetry.addData("target Angle", navParams.targetBearingAngle);
        telemetry.addData("HeadingVector", "%5.2f %5.2f", headingX, headingY);
        telemetry.update();
        return navParams;
    }

    // use the gyro to get navigation parameters needed to drive to the given target position
    public NavParams getVizNavParams(final VectorF targetPosition, VuforiaTrackable trackable) {
        NavParams navParams = new NavParams();

        do {
            //telemetry.addData("Getting location:","working...");
            //telemetry.update();
            getVuforiaLocation(trackable);
            if (lastLocation == null) {
                sleep(100);
            }
        } while ((opModeIsActive() && lastLocation == null));

        // get current location from vuforia
        // get robot x/y
        float robotX = trans.get(0);
        float robotY = trans.get(1);
        // Robot bearing (in Cartesian system) position is defined by the standard Matrix z rotation
        navParams.robotBearing = rot.thirdAngle * 360.0f / (2.0f * (float) Math.PI);


        // get current heading from vuforia
        VectorF currentPosition = new VectorF(robotX, robotY, 0.0f);
        int standardAngle = (int) navParams.robotBearing +90;
        float headingX = robot.getXFromStandardAngle(standardAngle);
        float headingY = robot.getYFromStandardAngle(standardAngle);
        VectorF headingVector = new VectorF(headingX, headingY, 0.0f);

        navParams.targetVector = targetPosition.subtracted(currentPosition);
        navParams.headingError = robot.getHeadingError(headingVector, navParams.targetVector);
        navParams.targetBearingAngle = navParams.robotBearing + navParams.headingError;



/*        telemetry.addData("bearing", "%5.3f", navParams.robotBearing);
        telemetry.addData("Pos: Curr/Targ", "%5.2f %5.2f / %5.2f %5.2f", currentPosition.get(0), currentPosition.get(1), targetPosition.get(0), targetPosition.get(1));
        //telemetry.addData("TargetVector(magnitude)", "%5.2f %5.2f (%5.2f)",
        //        navParams.targetVector.get(0), navParams.targetVector.get(1), navParams.targetVector.magnitude());
        telemetry.addData("headingError (deg)", navParams.headingError);
        telemetry.addData("target Angle", navParams.targetBearingAngle);
        //telemetry.addData("HeadingVector", "%5.2f %5.2f", headingX, headingY);
        //telemetry.update();
*/


        //safety check, if these are NaN, just go straight
        if (Double.isNaN(navParams.targetBearingAngle))
            navParams.targetBearingAngle = 0.0;
        if (Double.isNaN(navParams.headingError))
            navParams.headingError = 0.0;


        return navParams;
    }



    private class VuforiaErrorAngleCalculator implements HardwareTrig.ErrorAngleCalculator
    {
        VectorF targetPos;
            VuforiaTrackable trackable;
             VuforiaErrorAngleCalculator(VectorF targetPos, VuforiaTrackable trackable) {
            this.targetPos = targetPos;
            this.trackable = trackable;
        }

        public double getErrorAngle() {
            NavParams navParams = getVizNavParams(targetPos, trackable);
            return navParams.headingError;
        }

    }

    // From a position near the beacon, drive close to the viz target; then press the correct button
    public void doBeaconPress(VectorF targetPosition, VuforiaTrackable trackable, boolean redTeam) {

        NavParams np = getVizNavParams(targetPosition, trackable);
        VuforiaErrorAngleCalculator errorAngleCalculator = new VuforiaErrorAngleCalculator(targetPosition, trackable);
        robot.gyroDrive(DRIVE_SPEED/2, (np.targetVector.magnitude()/MM_PER_INCH)-beaconDistanceOffset, errorAngleCalculator);

        //Press beacon
        selectAndPressBeacon(redTeam);
    }


    public void followTarget(VectorF targetPosition, VuforiaTrackable trackable) {
        NavParams np = getVizNavParams(targetPosition, trackable);
        VuforiaErrorAngleCalculator errorAngleCalculator = new VuforiaErrorAngleCalculator(targetPosition, trackable);
        robot.gyroDrive(DRIVE_SPEED/2, 128, errorAngleCalculator);
    }

    public void doTest() {
        while(opModeIsActive()) {
            getVizNavParams(targetPositions[0],trackables[0]);
            if (gamepad1.x) {
                break;
            }
        }
        while(opModeIsActive()) {
            getVizNavParams(targetPositions[1],trackables[1]);
        }
    }


    @Override
    public void executeMovements() {
        waitForDelay();  // call delay method, base class waits 0 sec, but may be overridden in child class

        initializeTeam();

        if (gamepad1.a) {
            doTest();
        }
        if (gamepad1.b) {
            followTarget(targetPositions[0], trackables[0]);
            stop();
        }


        // drive 55 inches to get  near beacon
        robot.gyroDrive(DRIVE_SPEED, 55, initialAngle);
        robot.gyroTurn(TURN_SPEED, turnDirection); // turn to face the beacon

        // steer directly into the current beacon and press the correct button
        doBeaconPress(targetPositions[0], trackables[0], redTeam);

        //back up, turn and drive to second beacon position
        robot.gyroDrive(DRIVE_SPEED, -12, turnDirection);
        robot.gyroTurn(TURN_SPEED, farWallDirection);
        robot.gyroDrive(DRIVE_SPEED, 51, farWallDirection);
        robot.gyroTurn(TURN_SPEED, turnDirection);

        // reset location and beacon pressers
        lastLocation = null;

        // steer directly into the current beacon and press the correct button
        doBeaconPress(targetPositions[1], trackables[1], redTeam);
    }


}