package org.firstinspires.ftc.teamcode.Trig.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Trig.HardwareTrig;

/**
 * Created by FTC12073 on 12/7/16.
 */



@TeleOp(name="TrigBot:TeleopTank", group="TrigBot")
//@Disabled
public class TrigTeleopTank extends TrigTeleopBase {
    public void steer(){
        double left  = gamepad1.left_stick_y;
        double right = gamepad1.right_stick_y;
        robot.leftMotor.setPower(right * DRIVE_SPEED);
        robot.rightMotor.setPower(left * DRIVE_SPEED);
    }




}

