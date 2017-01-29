package org.firstinspires.ftc.teamcode.Trig.Auto.MoveAndLaunch;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by FTC12073 on 12/7/16.
 */


@Autonomous(name = "TrigBot:Move and Launch Blue", group = "TrigBot")
//@Disabled
public class TrigMoveAndLaunchBlue extends TrigMoveAndLaunchBase {

    public void waitForDelay() {
    }

    public void turnToBall() {
        //blue side
        // encoderDrive(DRIVE_SPEED * .5, 4.25, -4.25, 4);  //turn to move ball off center
    }

}
