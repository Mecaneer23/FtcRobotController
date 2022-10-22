package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoBase {
    static DcMotor frontLeft, frontRight, backLeft, backRight;

    Telemetry telemetry;
    LinearOpMode opMode;

    static final double PULSES_PER_REVOLUTION = 384.5; // 435 rpm goBilda 5202
    static final double WHEEL_DIAMETER_IN = 3.77953; // 96 mm
    static final double PULSES_PER_IN = PULSES_PER_REVOLUTION / (WHEEL_DIAMETER_IN * Math.PI);
    static double DRIVE_SPEED, TURN_SPEED, STRAFE_MULTIPLIER, DELAY_BETWEEN_METHODS, TURN_CONSTANT;
    static boolean USE_PID;
    double kP, kI, kD;

    public AutoBase(
            LinearOpMode opMode,
            HardwareMap hardwareMap,
            String left_front_name,
            String right_front_name,
            String left_back_name,
            String right_back_name,
            Telemetry telemetry,
            double driveSpeed, // 1.0 power
            double turnSpeed, // 0.5 power
            double lengthInches, // front-back axle to axle
            double widthInches, // left-right wheel center to wheel center
            double strafeMultiplier, // 1.13 units
            double delay, // 100 ms
            boolean usePID, // https://www.ctrlaltftc.com/the-pid-controller/tuning-methods-of-a-pid-controller
            double kP,
            double kI,
            double kD
    ) {
        DRIVE_SPEED = driveSpeed;
        TURN_SPEED = turnSpeed;
        STRAFE_MULTIPLIER = strafeMultiplier;
        DELAY_BETWEEN_METHODS = delay;
        USE_PID = usePID;
        TURN_CONSTANT = (Math.PI * Math.sqrt((Math.pow(lengthInches / 2.0, 2.0) + Math.pow(widthInches / 2.0, 2.0)) / 2.0)) / 90.0;

        frontLeft = hardwareMap.get(DcMotor.class, left_front_name);
        frontRight = hardwareMap.get(DcMotor.class, right_front_name);
        backLeft = hardwareMap.get(DcMotor.class, left_back_name);
        backRight = hardwareMap.get(DcMotor.class, right_back_name);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        this.telemetry = telemetry;
        this.opMode = opMode;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    private void drive(goFunction direction, double distanceIN, double motorPower) {
        ElapsedTime timer = new ElapsedTime();
        double proportional, integral = 0, derivative, pid, prevError = 0, totalTicks = PULSES_PER_IN * distanceIN;
        resetEncoders();
        direction.run((int) totalTicks);
        if (USE_PID) {
            setRunWithoutEncoders();
        } else {
            setRunToPosition();
        }
        setMotors(motorPower);
        while (
                frontLeft.isBusy() &&
                        frontRight.isBusy() &&
                        backLeft.isBusy() &&
                        backRight.isBusy()
        ) {
            if (USE_PID) {
                proportional = totalTicks - frontLeft.getCurrentPosition();
                integral += proportional * timer.seconds();
                derivative = (proportional - prevError) / timer.seconds();
                pid = (kP * proportional) + (kI * integral) + (kD * derivative);
                setMotors(Math.min(pid, motorPower));
                prevError = proportional;
                timer.reset();
            } else {
//                opMode.idle();
                setMotors(-4.0 * motorPower / Math.pow(totalTicks, 2.0) * Math.pow(totalTicks / 2.0 - frontLeft.getCurrentPosition(), 2.0) + motorPower);
            }
        }
        stopDriving();
        setRunWithoutEncoders();
        opMode.sleep((long) DELAY_BETWEEN_METHODS);
    }

    private interface goFunction {
        void run(int distanceIN);
    }

    private static void setRunToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private static void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private static void setRunWithoutEncoders() {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void goForward(Object distance) {
        frontLeft.setTargetPosition((int) distance);
        frontRight.setTargetPosition((int) distance);
        backLeft.setTargetPosition((int) distance);
        backRight.setTargetPosition((int) distance);
    }

    private void goBackward(Object distance) {
        frontLeft.setTargetPosition(-(int) distance);
        frontRight.setTargetPosition(-(int) distance);
        backLeft.setTargetPosition(-(int) distance);
        backRight.setTargetPosition(-(int) distance);
    }

    private void goLeft(Object distance) {
        frontLeft.setTargetPosition(-(int) distance);
        frontRight.setTargetPosition((int) distance);
        backLeft.setTargetPosition((int) distance);
        backRight.setTargetPosition(-(int) distance);
    }

    private void goRight(Object distance) {
        frontLeft.setTargetPosition((int) distance);
        frontRight.setTargetPosition(-(int) distance);
        backLeft.setTargetPosition(-(int) distance);
        backRight.setTargetPosition((int) distance);
    }

    private void goNW(Object distance) {
        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition((int) distance);
        backLeft.setTargetPosition((int) distance);
        backRight.setTargetPosition(0);
    }

    private void goNE(Object distance) {
        frontLeft.setTargetPosition((int) distance);
        frontRight.setTargetPosition(0);
        backLeft.setTargetPosition(0);
        backRight.setTargetPosition((int) distance);
    }

    private void goSW(Object distance) {
        frontLeft.setTargetPosition(-(int) distance);
        frontRight.setTargetPosition(0);
        backLeft.setTargetPosition(0);
        backRight.setTargetPosition(-(int) distance);
    }

    private void goSE(Object distance) {
        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(-(int) distance);
        backLeft.setTargetPosition(-(int) distance);
        backRight.setTargetPosition(0);
    }

    private void goTurnLeft(Object distance) {
        frontLeft.setTargetPosition(-(int) distance);
        frontRight.setTargetPosition((int) distance);
        backLeft.setTargetPosition(-(int) distance);
        backRight.setTargetPosition((int) distance);
    }

    private void goTurnRight(Object distance) {
        frontLeft.setTargetPosition((int) distance);
        frontRight.setTargetPosition(-(int) distance);
        backLeft.setTargetPosition((int) distance);
        backRight.setTargetPosition(-(int) distance);
    }

    private static void stopDriving() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private static void setMotors(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
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
        drive(this::goBackward, distanceIN, motorPower);
    }

    public void strafeLeft(double distanceIN) {
        strafeLeft(distanceIN, DRIVE_SPEED);
    }

    public void strafeLeft(double distanceIN, double motorPower) {
        drive(this::goLeft, distanceIN * STRAFE_MULTIPLIER, motorPower);
    }

    public void strafeRight(double distanceIN) {
        strafeRight(distanceIN, DRIVE_SPEED);
    }

    public void strafeRight(double distanceIN, double motorPower) {
        drive(this::goRight, distanceIN * STRAFE_MULTIPLIER, motorPower);
    }

    public void strafeNW(double distanceIN) {
        strafeNW(distanceIN, DRIVE_SPEED);
    }

    public void strafeNW(double distanceIN, double motorPower) {
        drive(this::goNW, distanceIN, motorPower);
    }

    public void strafeNE(double distanceIN) {
        strafeNE(distanceIN, DRIVE_SPEED);
    }

    public void strafeNE(double distanceIN, double motorPower) {
        drive(this::goNE, distanceIN, motorPower);
    }

    public void strafeSW(double distanceIN) {
        strafeSW(distanceIN, DRIVE_SPEED);
    }

    public void strafeSW(double distanceIN, double motorPower) {
        drive(this::goSW, distanceIN, motorPower);
    }

    public void strafeSE(double distanceIN) {
        strafeSE(distanceIN, DRIVE_SPEED);
    }

    public void strafeSE(double distanceIN, double motorPower) {
        drive(this::goSE, distanceIN, motorPower);
    }

    public void turnLeft() {
        turnLeft(90, TURN_SPEED);
    }

    public void turnLeft(int degrees) {
        turnLeft(degrees, TURN_SPEED);
    }

    public void turnLeft(int degrees, double motorPower) {
        drive(this::goTurnLeft, (int) (TURN_CONSTANT * degrees), motorPower);
    }

    public void turnRight() {
        turnRight(90, TURN_SPEED);
    }

    public void turnRight(int degrees) {
        turnRight(degrees, TURN_SPEED);
    }

    public void turnRight(int degrees, double motorPower) {
        drive(this::goTurnRight, (int) (TURN_CONSTANT * degrees), motorPower);
    }
}