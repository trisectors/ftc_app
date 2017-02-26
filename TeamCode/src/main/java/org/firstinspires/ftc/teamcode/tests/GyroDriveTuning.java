package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Trig.TrigAutoBase;

/**
 * Created by FTC12073 on 2/8/17.
 */
@Autonomous(name = "TrigBot:Tune Drive ", group = "TrigBot")

public class GyroDriveTuning extends TrigAutoBase {
    @Override
    public void waitForDelay() {

    }

    @Override
    public void executeMovements() {
        robot.gyroDrive(DRIVE_SPEED, 120, 0);

    }
}
