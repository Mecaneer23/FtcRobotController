package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.eventloop.opmode.LinearOpMode;

public class AutoBase {
    static DcMotor left_front, right_front, left_back, right_back;

    Telemetry telemetry;

    static double DRIVE_SPEED = 0.5;
    static double TURN_SPEED = 0.5;

    static final double TETRIX_TICKS_PER_MOTOR_REV = 1440;
    static final double ANDYMARK_TICKS_PER_MOTOR_REV = 1120;
    static final double GOBILDA_TICKS_PER_MOTOR_REV = 537;
    static final double PULSES_PER_REVOLUTION = GOBILDA_TICKS_PER_MOTOR_REV;
    static final double WHEEL_DIAMETER_IN = 4;
    static final double PULSES_PER_IN = PULSES_PER_REVOLUTION / (WHEEL_DIAMETER_IN * 3.1415);
    static double ROBOT_LENGTH_IN = 18;
    static double ROBOT_WIDTH_IN = 18;
    static double STRAFE_MULTIPLIER = 1.13;

    public void InitAuto(
        HardwareMap hardwareMap,
        String left_front_name,
        String right_front_name,
        String left_back_name,
        String right_back_name,
        Telemetry telemetry,
        double...len_width_speed_turnSpeed_strafeMultiplier
    ) {
        if (len_width_speed_turnSpeed_strafeMultiplier.length == 5) {
            ROBOT_LENGTH_IN = ROBOT_LENGTH_IN;
            ROBOT_WIDTH_IN = ROBOT_WIDTH_IN;
            DRIVE_SPEED = DRIVE_SPEED;
            TURN_SPEED = TURN_SPEED;
            STRAFE_MULTIPLIER = STRAFE_MULTIPLIER;
        }

        left_front = hardwareMap.get(DcMotor.class, left_front_name);
        right_front = hardwareMap.get(DcMotor.class, right_front_name);
        left_back = hardwareMap.get(DcMotor.class, left_back_name);
        right_back = hardwareMap.get(DcMotor.class, right_back_name);

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.FORWARD);
        left_back.setDirection(DcMotor.Direction.REVERSE);
        right_back.setDirection(DcMotor.Direction.FORWARD);

        this.telemetry = telemetry;

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private static void setRunToPosition() {
        left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private static void resetEncoders() {
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private static void goForward(int distance) {
        left_front.setTargetPosition(distance);
        right_front.setTargetPosition(distance);
        left_back.setTargetPosition(distance);
        right_back.setTargetPosition(distance);
    }

    private static void goBackward(int distance) {
        left_front.setTargetPosition(-distance);
        right_front.setTargetPosition(-distance);
        left_back.setTargetPosition(-distance);
        right_back.setTargetPosition(-distance);
    }

    private static void goLeft(int distance) {
        left_front.setTargetPosition(-distance);
        right_front.setTargetPosition(distance);
        left_back.setTargetPosition(distance);
        right_back.setTargetPosition(-distance);
    }

    private static void goRight(int distance) {
        left_front.setTargetPosition(distance);
        right_front.setTargetPosition(-distance);
        left_back.setTargetPosition(-distance);
        right_back.setTargetPosition(distance);
    }

    private static void goNW(int distance) {
        left_front.setTargetPosition(0);
        right_front.setTargetPosition(distance);
        left_back.setTargetPosition(distance);
        right_back.setTargetPosition(0);
    }

    private static void goNE(int distance) {
        left_front.setTargetPosition(distance);
        right_front.setTargetPosition(0);
        left_back.setTargetPosition(0);
        right_back.setTargetPosition(distance);
    }

    private static void goSW(int distance) {
        left_front.setTargetPosition(-distance);
        right_front.setTargetPosition(0);
        left_back.setTargetPosition(0);
        right_back.setTargetPosition(-distance);
    }

    private static void goSE(int distance) {
        left_front.setTargetPosition(0);
        right_front.setTargetPosition(-distance);
        left_back.setTargetPosition(-distance);
        right_back.setTargetPosition(0);
    }

    private static void goTurnLeft(int distance) {
        left_front.setTargetPosition(-distance);
        right_front.setTargetPosition(distance);
        left_back.setTargetPosition(-distance);
        right_back.setTargetPosition(distance);
    }

    private static void goTurnRight(int distance) {
        left_front.setTargetPosition(distance);
        right_front.setTargetPosition(-distance);
        left_back.setTargetPosition(distance);
        right_back.setTargetPosition(-distance);
    }

    private static void stopDriving() {
        left_front.setPower(0);
        right_front.setPower(0);
        left_back.setPower(0);
        right_back.setPower(0);
    }

    private static void setMotors(double power) {
        left_front.setPower(power);
        right_front.setPower(power);
        left_back.setPower(power);
        right_back.setPower(power);
    }

    public void driveForward(double distanceIN, double...MotorPower) {
        double Motor_Power;
        if (MotorPower.length == 1) {
            Motor_Power = MotorPower[0];
        } else {
            Motor_Power = DRIVE_SPEED;
        }
        resetEncoders();
        setRunToPosition();
        goForward((int) PULSES_PER_IN*distanceIN);
        setMotors(Motor_Power);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            idle();
        }
        stopDriving();
        sleep(100);
    }

    public void driveBackward(double distanceIN, double...MotorPower) {
        double Motor_Power;
        if (MotorPower.length == 1) {
            Motor_Power = MotorPower[0];
        } else {
            Motor_Power = DRIVE_SPEED;
        }
        resetEncoders();
        setRunToPosition();
        goBackward((int) PULSES_PER_IN*distanceIN);
        setMotors(Motor_Power);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            idle();
        }
        stopDriving();
        sleep(100);
    }

    public void strafeLeft(int distanceIN, double...MotorPower) {
        double Motor_Power;
        if (MotorPower.length == 1) {
            Motor_Power = MotorPower[0];
        } else {
            Motor_Power = DRIVE_SPEED;
        }
        resetEncoders();
        setRunToPosition();
        goLeft((int) Math.round(PULSES_PER_IN*distanceIN*STRAFE_MULTIPLIER));
        setMotors(Motor_Power);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            idle();
        }
        stopDriving();
        sleep(100);
    }

    public void strafeRight(int distanceIN, double...MotorPower) {
        double Motor_Power;
        if (MotorPower.length == 1) {
            Motor_Power = MotorPower[0];
        } else {
            Motor_Power = DRIVE_SPEED;
        }
        resetEncoders();
        setRunToPosition();
        goRight((int) Math.round(PULSES_PER_IN*distanceIN*STRAFE_MULTIPLIER));
        setMotors(Motor_Power);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            idle();
        }
        stopDriving();
        sleep(100);
    }

    public void strafeNW(double distanceIN, double...MotorPower) {
        double Motor_Power;
        if (MotorPower.length == 1) {
            Motor_Power = MotorPower[0];
        } else {
            Motor_Power = DRIVE_SPEED;
        }
        resetEncoders();
        setRunToPosition();
        goNW((int) PULSES_PER_IN*distanceIN);
        setMotors(Motor_Power);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            idle();
        }
        stopDriving();
        sleep(100);
    }

    public void strafeNE(double distanceIN, double...MotorPower) {
        double Motor_Power;
        if (MotorPower.length == 1) {
            Motor_Power = MotorPower[0];
        } else {
            Motor_Power = DRIVE_SPEED;
        }
        resetEncoders();
        setRunToPosition();
        goNE((int) PULSES_PER_IN*distanceIN);
        setMotors(Motor_Power);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            idle();
        }
        stopDriving();
        sleep(100);
    }

    public void strafeSW(double distanceIN, double...MotorPower) {
        double Motor_Power;
        if (MotorPower.length == 1) {
            Motor_Power = MotorPower[0];
        } else {
            Motor_Power = DRIVE_SPEED;
        }
        resetEncoders();
        setRunToPosition();
        goSW((int) PULSES_PER_IN*distanceIN);
        setMotors(Motor_Power);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            idle();
        }
        stopDriving();
        sleep(100);
    }

    public void strafeSE(double distanceIN, double...MotorPower) {
        double Motor_Power;
        if (MotorPower.length == 1) {
            Motor_Power = MotorPower[0];
        } else {
            Motor_Power = DRIVE_SPEED;
        }
        resetEncoders();
        setRunToPosition();
        goSE((int) PULSES_PER_IN*distanceIN);
        setMotors(Motor_Power);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            idle();
        }
        stopDriving();
        sleep(100);
    }

    public void turnLeft() {
        turnLeft(90);
    }

    public void turnRight() {
        turnRight(90);
    }

    public void turnLeft(int degrees, double...MotorPower) {
        double Motor_Power;
        if (MotorPower.length == 1) {
            Motor_Power = MotorPower[0];
        } else {
            Motor_Power = TURN_SPEED;
        }
        resetEncoders();
        setRunToPosition();
        double hypotenuse = (Math.sqrt(Math.pow(ROBOT_LENGTH_IN / 2.0, 2.0) + Math.pow(ROBOT_WIDTH_IN / 2.0, 2.0)));
        goTurnLeft((int) ((hypotenuse / 90.0) * degrees * PULSES_PER_IN)*2);
        setMotors(Motor_Power);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            idle();
        }
        stopDriving();
        sleep(100);
    }

    public void turnRight(int degrees, double...MotorPower) {
        double Motor_Power;
        if (MotorPower.length == 1) {
            Motor_Power = MotorPower[0];
        } else {
            Motor_Power = TURN_SPEED;
        }
        resetEncoders();
        setRunToPosition();
        double hypotenuse = (Math.sqrt(Math.pow(ROBOT_LENGTH_IN / 2.0, 2.0) + Math.pow(ROBOT_WIDTH_IN / 2.0, 2.0)));
        goTurnRight((int) ((hypotenuse / 90.0) * degrees * PULSES_PER_IN)*2);
        setMotors(Motor_Power);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            idle();
        }
        stopDriving();
        sleep(100);
    }
}

/*
    // During init:
    AutoBase auto = new AutoBase();
    auto.InitAuto(
        hardwareMap,
        front_left,
        front_right,
        back_left,
        back_right,
        telemetry,
    )

    // During run loop:
    auto.driveForward(12);
    auto.driveBackward(12);
    auto.strafeLeft(12);
    auto.strafeRight(12);
    auto.strafeNW(12);
    auto.strafeNE(12);
    auto.strafeSW(12);
    auto.strafeSE(12);
    auto.turnLeft(90);
    auto.turnRight(90);
*/
