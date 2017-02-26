package org.firstinspires.ftc.teamcode.Trig.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Trig.HardwareTrig;
import org.firstinspires.ftc.teamcode.Trig.Teleop.TrigTeleopArcade;

/**
 * Created by FTC12073 on 2/21/17.
 */
@TeleOp(name = "TrigBot:TeleopArcadeNoVis", group = "TrigBot")

public class TeleopArcadeNoVis extends TrigTeleopArcade {
    public void init() {
        robot.init(hardwareMap);

        robot.beaconRight.setPosition(HardwareTrig.RIGHT_BEACON_UP);
        robot.beaconLeft.setPosition(HardwareTrig.LEFT_BEACON_UP);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Good luck Driver! TRISECTORS RULE! May the Force be with you! You are one with the force. The force is with you! Help us Trisector you're our only hope!  GOOD LUCK! :)  ");
    }
}
