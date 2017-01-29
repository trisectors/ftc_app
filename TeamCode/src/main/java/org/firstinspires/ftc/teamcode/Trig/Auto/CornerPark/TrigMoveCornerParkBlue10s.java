package org.firstinspires.ftc.teamcode.Trig.Auto.CornerPark;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by FTC12073 on 12/7/16.
 */


@Autonomous(name = "TrigBot:Corner Park Blue 10s", group = "TrigBot")
//@Disabled
public class TrigMoveCornerParkBlue10s extends TrigMoveCornerParkBlue {

    public void waitForDelay() {
        sleep(10000);
    }

}
