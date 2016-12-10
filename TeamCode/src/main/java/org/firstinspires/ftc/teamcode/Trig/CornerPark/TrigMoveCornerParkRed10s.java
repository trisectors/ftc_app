package org.firstinspires.ftc.teamcode.Trig.CornerPark;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by FTC12073 on 12/7/16.
 */



@Autonomous(name="TrigBot:Trig Move Corne Park Red 10s ", group="TrigBot")
//@Disabled
public class TrigMoveCornerParkRed10s extends TrigMoveCornerParkBase {

    public void  turnToCorner() {
        encoderDrive(DRIVE_SPEED,  14, -14, 4.0);
    }
    public void waitForDelay() {sleep(10000);}

}
