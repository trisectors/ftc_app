package org.firstinspires.ftc.teamcode.Trig;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by FTC12073 on 12/7/16.
 */



@Autonomous(name="TrigBot:Move and launch (10s delay)", group="TrigBot")
//@Disabled
public class TrigMoveAndLaunchDelay10 extends TrigMoveAndLaunchNoDelay {

   public void waitForDelay() {
        sleep(10000);
    }

}
