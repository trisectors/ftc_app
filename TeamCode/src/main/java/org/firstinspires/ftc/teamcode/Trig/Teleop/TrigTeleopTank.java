package org.firstinspires.ftc.teamcode.Trig.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TrigBot:TeleopTank", group = "TrigBot")
public class TrigTeleopTank extends TrigTeleopBase {
    public void steer() {
        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;
        robot.leftMotor.setPower(left * DRIVE_SPEED);
        robot.rightMotor.setPower(right * DRIVE_SPEED);
    }
}

