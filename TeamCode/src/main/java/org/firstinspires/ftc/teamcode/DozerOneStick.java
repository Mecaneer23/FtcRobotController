package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DozerOneStick extends LinearOpMode {

    private CRServo backLeft;
    private CRServo backRight;
    private Servo eyes;
    private Servo plowLeft;
    private Servo plowRight;

    @Override
    public void runOpMode() {
        float y;
        float clockwise;
        double bl;
        double br;
        int speed = 1;
        boolean slowmodeChanged = false;
        boolean shouldSlowmode = false;

        backLeft = hardwareMap.get(CRServo.class, "backLeft");
        backRight = hardwareMap.get(CRServo.class, "backRight");
        eyes = hardwareMap.get(Servo.class, "eyes");
        plowLeft = hardwareMap.get(Servo.class, "plowLeft");
        plowRight = hardwareMap.get(Servo.class, "plowRight");

        backLeft.setDirection(CRServo.Direction.REVERSE);
        plowLeft.setDirection(Servo.Direction.REVERSE);

        eyes.setPosition((double) 0.5);
        plowLeft.setPosition((double) 0.1);
        plowRight.setPosition((double) 0.1);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                clockwise = gamepad1.left_stick_x;
                y = -gamepad1.left_stick_y;

                if (gamepad1.dpad_up) {
                    y = (float) 1.0;
                } else if (gamepad1.dpad_down) {
                    y = (float) -1.0;
                }

                if (gamepad1.dpad_right) {
                    clockwise = (float) 1.0;
                } else if (gamepad1.dpad_left) {
                    clockwise = (float) -1.0;
                }

                if (gamepad1.back) {
                    clockwise = (float) -1.0;
                } else if (gamepad1.guide) {
                    clockwise = (float) 1.0;
                }

                bl = y + clockwise;
                br = y - clockwise;

                if (gamepad1.right_bumper) {
                    speed = 2;
                } else if (gamepad1.left_bumper) {
                    speed = 4;
                } else if (gamepad1.start) {
                    if (!slowmodeChanged) {
                        shouldSlowmode = !shouldSlowmode;
                        slowmodeChanged = true;
                    }
                } else {
                    if (slowmodeChanged) {
                        slowmodeChanged = false;
                    } else {
                        speed = 1;
                    }
                }
                if (shouldSlowmode) {
                    speed = 2;
                }

                bl /= speed;
                br /= speed;

                backLeft.setPower(bl);
                backRight.setPower(br);

                if (gamepad1.x) {
                    eyes.setPosition(0);
                    sleep(500);
                    eyes.setPosition((double) 1.0);
                    sleep(500);
                    eyes.setPosition((double) 0.5);
                }

                if (gamepad1.y) {
                    plowLeft.setPosition((double) 0.9);
                    plowRight.setPosition((double) 0.9);
                } else if (gamepad1.a) {
                    plowLeft.setPosition((double) 0.1);
                    plowRight.setPosition((double) 0.1);
                }

                telemetry.update();
            }
        }
    }
}