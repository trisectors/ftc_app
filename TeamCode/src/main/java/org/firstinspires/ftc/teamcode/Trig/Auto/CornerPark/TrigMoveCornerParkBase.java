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
package org.firstinspires.ftc.teamcode.Trig.Auto.CornerPark;

import org.firstinspires.ftc.teamcode.Trig.TrigAutoBase;

// base class for firing two particles then parking in the corner
public abstract class TrigMoveCornerParkBase extends TrigAutoBase {

    public abstract void waitForDelay();

    public abstract void turnToCorner();

    @Override
    public void executeMovements() {


        waitForDelay(); // call delay method, this class waits 0 sec, but may be overridden


        //Drive forward 2ft
        encoderDrive(DRIVE_SPEED, 27.0, 27.0, 6.0);

        // fire first particle: turn flicker on to 100, wait half second, turn flicker off
        robot.flickerFire();
        // load second ball:  turn sweep on, wait 5 sec, turn sweep off
        sleep(500);

        robot.sweepMotor.setPower(.5);      // This is for the second ball
        sleep(2500);
        robot.sweepMotor.setPower(0);
        sleep(500);
        // fire second particle wait half sec, turn flicker on, wait half second, turn flicker off
        robot.flickerFire();

        encoderDrive(DRIVE_SPEED, 3.0, 3.0, 6.0);
        // turn towards the corner vortex and park on it.
        sleep(500);
        turnToCorner();  //turn
        encoderDrive(1.0, -40.0, -40.0, 4.0);  //back up into corner


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
