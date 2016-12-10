package org.firstinspires.ftc.teamcode.Trig.CornerPark;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by FTC12073 on 12/7/16.
 */



@Autonomous(name="TrigBot:Trig Move Corner Park Blue ", group="TrigBot")
//@Disabled
public class TrigMoveCornerParkBlue extends TrigMoveCornerParkBase {

    public void  turnToCorner() {
        encoderDrive(DRIVE_SPEED * .5,  -16, 16  , 4.0);
    }

}
