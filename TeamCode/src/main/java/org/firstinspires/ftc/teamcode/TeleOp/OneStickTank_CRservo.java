package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@Disabled
@TeleOp
public class OneStickTank_CRservo extends LinearOpMode {

    private CRServo backLeft;
    private CRServo backRight;

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

        backLeft.setDirection(CRServo.Direction.REVERSE);

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

                telemetry.update();
            }
        }
    }
}