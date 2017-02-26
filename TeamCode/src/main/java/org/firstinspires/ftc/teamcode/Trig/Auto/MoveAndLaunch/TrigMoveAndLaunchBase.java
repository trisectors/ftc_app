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
package org.firstinspires.ftc.teamcode.Trig.Auto.MoveAndLaunch;

import org.firstinspires.ftc.teamcode.Trig.TrigAutoBase;


// base class for starting in the corner, firing two particles and then parking on the center vortex
public abstract class TrigMoveAndLaunchBase extends TrigAutoBase {

    public abstract void turnToBall();

    @Override
    public void executeMovements() {
        waitForDelay(); // call delay method, this class waits 0 sec, but may be overridden

        //Drive forward 42 inches.

        encoderDrive(ORIGINAL_DRIVE_SPEED, 45, 45, 6);

        // fire first particle: turn flicker on to 100, wait half second, turn flicker off
        robot.flickerFire();
        // load second ball:  turn boot on, wait 5 sec, turn boot down
        robot.sweepMotor.setPower(.4);
        sleep(3000);
        robot.sweepMotor.setPower(0);
        sleep(500);
        // fire second particle: wait half sec, turn flicker on, wait half second, turn flicker off
        robot.flickerFire();
        robot.beaconLeft.setPosition(1.0);
        robot.beaconRight.setPosition(0);
        // knock ball off center platform: drive forward 1/2 foot and park
        encoderDrive(ORIGINAL_DRIVE_SPEED, 16, 16, 4.0);  //ram

        encoderDrive(ORIGINAL_DRIVE_SPEED, -6, -6, 4.0);  //back up

        encoderDrive(.8, 6, 6, 4.0); // go forward again

       // turnToBall();

        encoderDrive(.8, 10, 10, 4);


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


}
