package org.firstinspires.ftc.teamcode.Trig.Auto.CornerPark;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by FTC12073 on 12/7/16.
 */


@Autonomous(name = "TrigBot:Corner Park Blue ", group = "TrigBot")
//@Disabled
public class TrigMoveCornerParkBlue extends TrigMoveCornerParkBase {

    public void initializeTeam(){
        initialAngle = 90;
        cornerAngle = 145;
    }

    public void waitForDelay() {}


}
