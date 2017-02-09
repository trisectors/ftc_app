package org.firstinspires.ftc.teamcode.Trig.Auto.AutoBeacon;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Trig.Auto.CornerPark.TrigMoveCornerParkBase;

/**
 * Created by FTC12073 on 12/7/16.
 */


@Autonomous(name = "TrigBot:Beacon RED ", group = "TrigBot")
//@Disabled
public class TrigAutoBeaconRed extends TrigAutoBeaconBase {

    public void waitForDelay() {}



    public void initializeTeam() {
        redTeam=true;
        turnDirection = 90;
        farWallDirection = 0;
        targetPositions[0] = new VectorF(-1625f, -304.8f, 0.0f);
        targetPositions[1] = new VectorF(-1625f, 914.4f, 0.0f)  ;
        trackables[0] = allTrackables.get(3);
        trackables[1] = allTrackables.get(1);
        initialAngle = 30;
    }

}