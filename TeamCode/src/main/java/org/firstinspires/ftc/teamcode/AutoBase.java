package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.Callable;

public class AutoBase {
    static DcMotor left_front, right_front, left_back, right_back;

    Telemetry telemetry;
    LinearOpMode opMode;

    static final double TETRIX_TICKS_PER_MOTOR_REV = 1440;
    static final double ANDYMARK_TICKS_PER_MOTOR_REV = 1120;
    static final double GOBILDA_TICKS_PER_MOTOR_REV = 537;
    static final double PULSES_PER_REVOLUTION = GOBILDA_TICKS_PER_MOTOR_REV;
    static final double WHEEL_DIAMETER_IN = 4;
    static final double PULSES_PER_IN = PULSES_PER_REVOLUTION / (WHEEL_DIAMETER_IN * 3.1415);
    static double DRIVE_SPEED, TURN_SPEED, ROBOT_LENGTH_IN, ROBOT_WIDTH_IN, STRAFE_MULTIPLIER, DELAY_BETWEEN_METHODS;

    public AutoBase(
            LinearOpMode opMode,
            HardwareMap hardwareMap,
            String left_front_name,
            String right_front_name,
            String left_back_name,
            String right_back_name,
            Telemetry telemetry,
            double driveSpeed, // 1.0
            double turnSpeed, // 0.5
            double len, // 18
            double width, // 18
            double strafeMultiplier, // 1.13
            double delay // 100
    ) {
        ROBOT_LENGTH_IN = len;
        ROBOT_WIDTH_IN = width;
        DRIVE_SPEED = driveSpeed;
        TURN_SPEED = turnSpeed;
        STRAFE_MULTIPLIER = strafeMultiplier;
        DELAY_BETWEEN_METHODS = delay;

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
        this.opMode = opMode;

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    private void sleep(double milliseconds) {
        opMode.sleep((long) milliseconds);
    }

    private void idle() {
        opMode.idle();
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

    private static void setRunWithoutEncoders() {
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void goForward(Object distance) {
        left_front.setTargetPosition((int) distance);
        right_front.setTargetPosition((int) distance);
        left_back.setTargetPosition((int) distance);
        right_back.setTargetPosition((int) distance);
    }

    private void goBackward(int distance) {
        left_front.setTargetPosition(-distance);
        right_front.setTargetPosition(-distance);
        left_back.setTargetPosition(-distance);
        right_back.setTargetPosition(-distance);
    }

    private void goLeft(int distance) {
        left_front.setTargetPosition(-distance);
        right_front.setTargetPosition(distance);
        left_back.setTargetPosition(distance);
        right_back.setTargetPosition(-distance);
    }

    private void goRight(int distance) {
        left_front.setTargetPosition(distance);
        right_front.setTargetPosition(-distance);
        left_back.setTargetPosition(-distance);
        right_back.setTargetPosition(distance);
    }

    private void goNW(int distance) {
        left_front.setTargetPosition(0);
        right_front.setTargetPosition(distance);
        left_back.setTargetPosition(distance);
        right_back.setTargetPosition(0);
    }

    private void goNE(int distance) {
        left_front.setTargetPosition(distance);
        right_front.setTargetPosition(0);
        left_back.setTargetPosition(0);
        right_back.setTargetPosition(distance);
    }

    private void goSW(int distance) {
        left_front.setTargetPosition(-distance);
        right_front.setTargetPosition(0);
        left_back.setTargetPosition(0);
        right_back.setTargetPosition(-distance);
    }

    private void goSE(int distance) {
        left_front.setTargetPosition(0);
        right_front.setTargetPosition(-distance);
        left_back.setTargetPosition(-distance);
        right_back.setTargetPosition(0);
    }

    private void goTurnLeft(int distance) {
        left_front.setTargetPosition(-distance);
        right_front.setTargetPosition(distance);
        left_back.setTargetPosition(-distance);
        right_back.setTargetPosition(distance);
    }

    private void goTurnRight(int distance) {
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

    private void drive(goFunction direction, double distanceIN, double motorPower) {
        resetEncoders();
        setRunToPosition();
        direction.run((int) (PULSES_PER_IN*distanceIN));
        setMotors(motorPower);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            opMode.idle();
        }
        stopDriving();
        setRunWithoutEncoders();
        sleep(DELAY_BETWEEN_METHODS);
    }

    private interface goFunction {
        void run(int distanceIN);
    }

    public void driveForward(double distanceIN) {
        driveForward(distanceIN, DRIVE_SPEED);
    }

    public void driveForward(double distanceIN, double motorPower) {
        drive(this::goForward, distanceIN, motorPower);
    }

    public void driveBackward(double distanceIN) {
        driveBackward(distanceIN, DRIVE_SPEED);
    }

    public void driveBackward(double distanceIN, double motorPower) {
        drive(goBackward, distanceIN, motorPower);
    }

    public void strafeLeft(int distanceIN) {
        strafeLeft(distanceIN, DRIVE_SPEED);
    }

    public void strafeLeft(int distanceIN, double motorPower) {
        drive(goLeft, distanceIN, motorPower);
    }

    public void strafeRight(int distanceIN) {
        strafeRight(distanceIN, DRIVE_SPEED);
    }

    public void strafeRight(int distanceIN, double motorPower) {
        drive(goRight, distanceIN, motorPower);
    }

    public void strafeNW(double distanceIN) {
        strafeNW(distanceIN, DRIVE_SPEED);
    }

    public void strafeNW(double distanceIN, double motorPower) {
        drive(goNW, distanceIN, motorPower);
    }

    public void strafeNE(double distanceIN) {
        strafeNE(distanceIN, DRIVE_SPEED);
    }

    public void strafeNE(double distanceIN, double motorPower) {
        drive(goNE, distanceIN, motorPower);
    }

    public void strafeSW(double distanceIN) {
        strafeSW(distanceIN, DRIVE_SPEED);
    }

    public void strafeSW(double distanceIN, double motorPower) {
        drive(goSW, distanceIN, motorPower);
    }

    public void strafeSE(double distanceIN) {
        strafeSE(distanceIN, DRIVE_SPEED);
    }

    public void strafeSE(double distanceIN, double motorPower) {
        drive(goSE, distanceIN, motorPower);
    }

    static double TURN_CONSTANT = ((Math.sqrt(Math.pow(ROBOT_LENGTH_IN / 2.0, 2.0) + Math.pow(ROBOT_WIDTH_IN / 2.0, 2.0))) / 90.0) * 2.0;

    public void turnLeft() {
        turnLeft(90, TURN_SPEED);
    }

    public void turnLeft(int degrees) {
        turnLeft(degrees, TURN_SPEED);
    }

    public void turnLeft(int degrees, double motorPower) {
        drive(goTurnLeft, (int) (TURN_CONSTANT * degrees), motorPower);
    }

    public void turnRight() {
        turnRight(90, TURN_SPEED);
    }

    public void turnRight(int degrees) {
        turnRight(degrees, TURN_SPEED);
    }

    public void turnRight(int degrees, double motorPower) {
        drive(goTurnRight, (int) (TURN_CONSTANT * degrees), motorPower);
    }
}

/*
    // During init:
    AutoBase auto = new AutoBase(
        this,
        hardwareMap,
        "frontLeft",
        "frontRight",
        "backLeft",
        "backRight",
        telemetry,
        1,
        1,
        18,
        18,
        1.13,
        100
    );

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
