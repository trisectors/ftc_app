package org.firstinspires.ftc.teamcode.Trig.Auto.MoveAndLaunch;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by FTC12073 on 12/7/16.
 */


@Autonomous(name = "TrigBot:Move and Launch Blue 10s", group = "TrigBot")
//@Disabled
public class TrigMoveAndLaunchBlue10s extends TrigMoveAndLaunchBlue {

    public void waitForDelay() {
        sleep(10000);
    }

}