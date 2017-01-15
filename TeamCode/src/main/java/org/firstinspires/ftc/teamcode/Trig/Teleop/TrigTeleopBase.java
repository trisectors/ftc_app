package org.firstinspires.ftc.teamcode.Trig.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Trig.HardwareTrig;

public class TrigTeleopBase extends OpMode {
    
        /* Declare OpMode members. */
        HardwareTrig robot = new HardwareTrig(); // use the class created to define a Pushbot's hardware
        // could also use HardwarePushbotMatrix class.
        double sweepSpeed = 0.0;                  // Servo mid position
        final double SWEEP_SPEED = 0.001;                 // sets rate to move servo
        final double LEFT_BEACON_DOWN = 1.0;
        final double LEFT_BEACON_UP = 0.0;
    final double RIGHT_BEACON_UP = 1.0;
    final double RIGHT_BEACON_DOWN = 0.0;

    
        private ElapsedTime runtime = new ElapsedTime();
    
        double DRIVE_SPEED = .8;
    
        /*
         * Code to run ONCE when the driver hits INIT
         */
        @Override
        public void init() {

            /* Initialize the hardware variables.
             * The init() method of the hardware class does all the work here
             */
            robot.init(hardwareMap);
    
            // Send telemetry message to signify robot waiting;
            telemetry.addData("Say", "Hello Driver"); //
            robot.beaconRight.setPosition(RIGHT_BEACON_UP);
            robot.beaconLeft.setPosition(LEFT_BEACON_UP);
        }
    
        /*
         * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
         */
        @Override
        public void init_loop() {
        }
    
        /*
         * Code to run ONCE when the driver hits PLAY
         */
        @Override
        public void start() {
        }
    
        public void steer() {
    
        }
    
        boolean b_enabled = true;
        boolean beaconLeftDown = true;
        boolean beaconRightDown = true;
        boolean beaconLeftEnabled = true;
        boolean beaconRightEnabled = true;
    
        /*
         * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
         */
        @Override
        public void loop() {
            steer();

            double GATE_DOWN_POSITION = .7;
            double GATE_UP_POSITION = 0;
    /*
            if (gamepad1.b && b_enabled && robot.gate.getPosition() > .1) {
                robot.gate.setPosition(GATE_DOWN_POSITION);
                b_enabled = false;
            }
    
            if (gamepad1.b && b_enabled && robot.gate.getPosition() < .05) {
                robot.gate.setPosition(GATE_UP_POSITION);
                b_enabled = false;
            }
    
            if (!gamepad1.b) {
                b_enabled=true;
            }
    */
    
            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
    
    
            // Use gamepad left & right Bumpers and x to adjust sweep speed
            if (gamepad1.right_bumper)
                sweepSpeed += SWEEP_SPEED;
            else if (gamepad1.left_bumper)
                sweepSpeed -= SWEEP_SPEED;
            if (gamepad1.right_trigger > 0)
                sweepSpeed = .3;
            if (gamepad1.x) {
                sweepSpeed = (0.00);
            }
    
            // control gate automatically from sweepSpeed
            if (sweepSpeed == 0) {
                robot.gate.setPosition(GATE_DOWN_POSITION);
            } else {
                robot.gate.setPosition(GATE_UP_POSITION);
            }
            robot.sweepMotor.setPower(sweepSpeed);
    
    
            // control flicker
            if (gamepad1.a) {
                robot.flicker.setPower(1.00);
            } else if (gamepad1.y) {
                robot.flicker.setPower(-.075);
            } else {
                robot.flicker.setPower(0);
            }
    
            // beaconLeft Control
            if (beaconLeftEnabled && gamepad1.dpad_left) {
                beaconLeftEnabled = false;
                if (beaconLeftDown) {
                    beaconLeftDown = false;
                    robot.beaconLeft.setPosition(LEFT_BEACON_UP);           // Change these positions
                } else {
                    beaconLeftDown = true;
                    robot.beaconLeft.setPosition(LEFT_BEACON_DOWN);            // Change these positions
                }
    
            } else if (!gamepad1.dpad_left) {
                beaconLeftEnabled = true;
            }
    
    
            // beaconRight Control
            if (beaconRightEnabled && gamepad1.dpad_right) {
                beaconRightEnabled = false;
                if (beaconRightDown) {
                    beaconRightDown = false;
                    robot.beaconRight.setPosition(RIGHT_BEACON_DOWN);           // Change these positions
                } else {
                    beaconRightDown = true;
                    robot.beaconRight.setPosition(RIGHT_BEACON_UP);            // Change these positions
                }
    
            } else if (!gamepad1.dpad_right) {
                beaconRightEnabled = true;
            }


            telemetry.addData("beaconRight" , "Offset = %.2f", robot.beaconRight.getPosition());

            telemetry.addData("beaconLeft" , "Offset = %.2f", robot.beaconLeft.getPosition());
            telemetry.update();
    
    
              /*  if (gamepad1.dpad_right && robot.frontBeacon.getPosition() <= 0.15) {
                    double pos = robot.frontBeacon.getPosition();
                    pos -= 0.05;
                    robot.frontBeacon.setPosition(pos);
                    telemetry.addData("frontBeacon:",pos);
                } else if (gamepad1.dpad_right) {
                    robot.frontBeacon.setPosition(0.15);
                }
    
                if (gamepad1.dpad_left && robot.frontBeacon.getPosition() >= 0.85) {
                    double pos = robot.frontBeacon.getPosition();
                    pos += 0.05;
                    robot.frontBeacon.setPosition(pos);
                    telemetry.addData("frontBeacon:",pos);
                } else if (gamepad1.dpad_left) {
                    robot.frontBeacon.setPosition(.85);
                }
    
    
                if (gamepad1.dpad_up) {
                    robot.frontBeacon.setPosition(.5);
                    telemetry.addData("servo:", "center");
                }
    
    
                if (gamepad1.dpad_down)  {
                    robot.sideBeacon.setPosition(.5); // this is a guess modify as needed.
                } else {
                    robot.sideBeacon.setPosition(0);
                }
    
    
    
                if (gamepad1.left_trigger > 0) {
                    robot.arm1.setPower(1);
                    robot.arm2.setPower(1);
                }
                else{
                    robot.arm1.setPower(0);
                    robot.arm2.setPower(0);
                }
    */
    
            // Use gamepad buttons to move the arm up (Y) and down (A)
            //     if (gamepad1.y)
            //         robot.armMotor.setPower(robot.ARM_UP_POWER);
            //     else if (gamepad1.a)
            //         robot.armMotor.setPower(robot.ARM_DOWN_POWER);
            //     else
            //         robot.armMotor.setPower(0.0);
    
            // Send telemetry message to signify robot running;
            telemetry.addData("sweepSpeed", "Offset = %.2f", sweepSpeed);
            ;
        }
    
        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop() {
        }
    
    }