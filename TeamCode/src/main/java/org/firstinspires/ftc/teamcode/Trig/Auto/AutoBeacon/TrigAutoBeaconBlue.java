package org.firstinspires.ftc.teamcode.Trig.Auto.AutoBeacon;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

/**
 * Created by FTC12073 on 12/7/16.
 */


@Autonomous(name = "TrigBot:Beacon BLUE ", group = "TrigBot")
//@Disabled
public class TrigAutoBeaconBlue extends TrigAutoBeaconBase {

    public void waitForDelay() {}



    public void initializeTeam() {
        // half field width = 1803.4

        redTeam = false;
        turnDirection = 0;
        farWallDirection = 90;
        targetPositions[0] = new VectorF(304.8f, 1625f, 0.0f);
        targetPositions[1] = new VectorF(-914.4f, 1625f, 0.0f);
        trackables[0] = allTrackables.get(0);
        trackables[1] = allTrackables.get(2);
        initialAngle = 60;
        beaconDistanceOffset = 1; // we were getting 1 inch too close to the blue beacons.
        telemetry.addData("Say","Turn flicker side to the corner vortex. press init. Then turn the robot 60 deg.");
    }


}