/*
Control Scheme:
  Gamepad 1 - robot locomotion:
    left stick - xy position of robot
    right stick - rotation of robot
    right bumper 1/2 speed slowmode
    dpad - 1.0 power in any given direction
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Mecanum_Drive extends LinearOpMode {

    private static final double TETRIX_TICKS_PER_MOTOR_REV = 1440;
    private static final double ANDYMARK_TICKS_PER_MOTOR_REV = 1120;
    private static final double GOBILDA_TICKS_PER_MOTOR_REV = 537;
    private static final double PULSES_PER_REVOLUTION = GOBILDA_TICKS_PER_MOTOR_REV;
    private static final double WHEEL_DIAMETER_IN = 4;
    private static final double PULSES_PER_IN = PULSES_PER_REVOLUTION / (WHEEL_DIAMETER_IN * 3.1415);
    private static final double ROBOT_LENGTH_IN = 17;
    private static final double ROBOT_WIDTH_IN = 13;
    
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    @Override
    public void runOpMode() {
        float x;
        float y;
        float clockwise;
        double fl;
        double fr;
        double bl;
        double br;
        int speed = 1;
        boolean slowmodeChanged = false;
        boolean shouldSlowmode = false;
        
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            while (opModeIsActive()) {
                x = gamepad1.left_stick_x;
                y = -gamepad1.left_stick_y;
                clockwise = gamepad1.right_stick_x;

                if (gamepad1.dpad_up) {
                    y = (float) 1.0;
                } else if (gamepad1.dpad_down) {
                    y = (float) -1.0;
                }

                if (gamepad1.dpad_right) {
                    x = (float) 1.0;
                } else if (gamepad1.dpad_left) {
                    x = (float) -1.0;
                }

                if (gamepad1.back) {
                    clockwise = (float) -1.0;
                } else if (gamepad1.guide) {
                    clockwise = (float) 1.0;
                }

                fl = y + x + clockwise;
                fr = y - x - clockwise;
                bl = y - x + clockwise;
                br = y + x - clockwise;
                
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
                
                if (gamepad1.left_stick_button) {
                    if (gamepad1.y) {
                        turnHelper(0, 45);
                    } else if (gamepad1.b) {
                        turnHelper(0, 120);
                    } else if (gamepad1.a) {
                        turnHelper(0, 180);
                    } else {
                        turnHelper(0, 90);
                    }
                } else if (gamepad1.right_stick_button) {
                    if (gamepad1.y) {
                        turnHelper(1, 45);
                    } else if (gamepad1.b) {
                        turnHelper(1, 120);
                    } else if (gamepad1.a) {
                        turnHelper(1, 180);
                    } else {
                        turnHelper(1, 90);
                    }
                }
                
                fl /= speed;
                fr /= speed;
                bl /= speed;
                br /= speed; 
                    
                frontLeft.setPower(fl);
                frontRight.setPower(fr);
                backLeft.setPower(bl);
                backRight.setPower(br);
                    
                telemetry.update();
            }
        }
    }
    
    private void turnHelper(int direction, int degrees) {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int distance = (int) ((Math.sqrt(Math.pow(ROBOT_LENGTH_IN / 2.0, 2.0) + Math.pow(ROBOT_WIDTH_IN / 2.0, 2.0)) / 90.0) * degrees * PULSES_PER_IN)*2;
        if (direction == 0) { // left
            frontLeft.setTargetPosition(-distance);
            frontRight.setTargetPosition(distance);
            backLeft.setTargetPosition(-distance);
            backRight.setTargetPosition(distance);
        } else if (direction == 1) { // right
            frontLeft.setTargetPosition(distance);
            frontRight.setTargetPosition(-distance);
            backLeft.setTargetPosition(distance);
            backRight.setTargetPosition(-distance);
        }
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (direction == 0) { // left
            frontLeft.setPower((double) -1);
            frontRight.setPower((double) 1);
            backLeft.setPower((double) -1);
            backRight.setPower((double) 1);
        } else if (direction == 1) { // right
            frontLeft.setPower((double) 1);
            frontRight.setPower((double) -1);
            backLeft.setPower((double) 1);
            backRight.setPower((double) -1);
        }
        
        while (
            frontLeft.isBusy() &&
            frontRight.isBusy() &&
            backLeft.isBusy() &&
            backRight.isBusy()
        ) {
            // waiting for target position to be reached
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
