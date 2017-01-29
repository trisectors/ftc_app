package org.firstinspires.ftc.teamcode.Trig.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Trig.Teleop.TrigTeleopTank;

/**
 * Created by FTC12073 on 12/7/16.
 */


@TeleOp(name = "TrigBot:TeleopArcade", group = "TrigBot")
public class TrigTeleopArcade extends TrigTeleopBase {
    public void steer() {
        double turn = gamepad1.right_stick_x;
        double power = -gamepad1.right_stick_y;
        if (turn != 0.0 || power != 0) {
            robot.rightMotor.setPower((power - turn) * DRIVE_SPEED);
            robot.leftMotor.setPower((power + turn) * DRIVE_SPEED);

        } else {
            turn = gamepad1.left_stick_x;
            power = gamepad1.left_stick_y;
            if (turn != 0.0 || power != 0) {
                robot.rightMotor.setPower((power - turn) * DRIVE_SPEED);
                robot.leftMotor.setPower((power + turn) * DRIVE_SPEED);
            } else {
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
            }
        }

    }
}
