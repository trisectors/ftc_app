package org.firstinspires.ftc.teamcode.Trig.Auto.MoveAndLaunch;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by FTC12073 on 12/7/16.
 */


@Autonomous(name = "TrigBot:Move and Launch Red", group = "TrigBot")
//@Disabled
public class TrigMoveAndLaunchRed extends TrigMoveAndLaunchBase {

    public void waitForDelay() {
    }

    public void turnToBall() {
        //red side
        //encoderDrive(DRIVE_SPEED * .5, -3, 3, 4);  //turn to move ball off center
    }

}
