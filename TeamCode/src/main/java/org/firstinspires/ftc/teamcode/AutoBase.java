package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.util.Locale;

/**
 * AutoPathController is an abstraction of the drivetrain. Use this for Autos
 * where a predefined path needs to be executed.
 */
public class AutoBase {
    static DcMotor left_front, right_front, left_back, right_back;

    Telemetry telemetry;

    // https://first-tech-challenge.github.io/SkyStone/com/qualcomm/hardware/bosch/BNO055IMUImpl.html
    static BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    // Orientation relative to signing the top of the field.
    // https://github.com/acmerobotics/road-runner/blob/master/gui/src/main/resources/field.png
    double currentAngle;

    static final ElapsedTime runtime = new ElapsedTime();

    // Drive speed constants
    static double DRIVE_SPEED = 1.0;
    static double TURN_SPEED = 1.0;

    // static final double HEADING_THRESHOLD = 1; // As tight as we can make it with an integer gyro
    // static final double P_TURN_COEFF = 0.1; // Larger is more responsive, but also less stable
    // static final double P_DRIVE_COEFF = 0.15; // Larger is more responsive, but also less stable
    // static final double LEFT_DAMPING_CONSTANT = 0.885;
    // static final double RIGHT_DAMPING_CONSTANT = 0.885;
    // static final double DISTANCE_TO_ANGLE_CONSTANT = 1.2;

    // Motor encoder configuration constant
    static final double TETRIX_TICKS_PER_MOTOR_REV = 1440;
    static final double ANDYMARK_TICKS_PER_MOTOR_REV = 1120;
    static final double GOBILDA_TICKS_PER_MOTOR_REV = 537;
    static final double PULSES_PER_REVOLUTION = GOBILDA_TICKS_PER_MOTOR_REV;
    static final double WHEEL_DIAMETER_IN = 4;
    static final double PULSES_PER_IN = PULSES_PER_REVOLUTION / (WHEEL_DIAMETER_IN * 3.1415);
    static double ROBOT_LENGTH_IN = 18;
    static double ROBOT_WIDTH_IN = 18;

    public void InitAuto(
        HardwareMap hardwareMap,
        String left_front_name,
        String right_front_name,
        String left_back_name,
        String right_back_name,
        Telemetry telemetry,
        double...len_width_speed_turnSpeed
    ) {
        if (len_width_speed_turnSpeed.length == 4) {
            ROBOT_LENGTH_IN = len_width_speed_turnSpeed[0] != 0 ? len_width_speed_turnSpeed[0] : ROBOT_LENGTH_IN;
            ROBOT_WIDTH_IN = len_width_speed_turnSpeed[1] != 0 ? len_width_speed_turnSpeed[1] : ROBOT_WIDTH_IN;
            DRIVE_SPEED = len_width_speed_turnSpeed[2] != 0 ? len_width_speed_turnSpeed[2] : DRIVE_SPEED;
            TURN_SPEED = len_width_speed_turnSpeed[3] != 0 ? len_width_speed_turnSpeed[3] : TURN_SPEED;
        }

        initIMU(hardwareMap);

        left_front = hardwareMap.get(DcMotor.class, left_front_name);
        right_front = hardwareMap.get(DcMotor.class, right_front_name);
        left_back = hardwareMap.get(DcMotor.class, left_back_name);
        right_back = hardwareMap.get(DcMotor.class, right_back_name);

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right_front.setDirection(DcMotor.Direction.REVERSE);
        right_back.setDirection(DcMotor.Direction.REVERSE);

        this.telemetry = telemetry;

        ZeroPowerBehaviorBRAKE();
    }

    // public void update() {
    //     angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    //     gravity = imu.getGravity();
    //     telemetry.addData("Angles: ", angles.toString());
    //     telemetry.addData("Gravity: ", gravity.toString());
    // }

    private static void initIMU(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    private static void ZeroPowerBehaviorBRAKE() {
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private static void setRunUsingEncoders() {
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    }

    private static void setRunWithoutEncoders() {
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    }

    private static void setRunToPosition() {
        left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private static void resetEncoders() {
        // method may be stopandreset, but I'm not sure
        left_front.setMode(DcMotor.RunMode.RESET_ENCODERS);
        right_front.setMode(DcMotor.RunMode.RESET_ENCODERS);
        left_back.setMode(DcMotor.RunMode.RESET_ENCODERS);
        right_back.setMode(DcMotor.RunMode.RESET_ENCODERS);
    }

    private static void setTargetPositionsForward(int distance) {
        left_front.setTargetPosition(distance);
        right_front.setTargetPosition(distance);
        left_back.setTargetPosition(distance);
        right_back.setTargetPosition(distance);
    }

    private static void setTargetPositionsBackward(int distance) {
        left_front.setTargetPosition(-distance);
        right_front.setTargetPosition(-distance);
        left_back.setTargetPosition(-distance);
        right_back.setTargetPosition(-distance);
    }

    private static void setTargetPositionsLeft(int distance) {
        left_front.setTargetPosition(-distance);
        right_front.setTargetPosition(distance);
        left_back.setTargetPosition(distance);
        right_back.setTargetPosition(-distance);
    }

    private static void setTargetPositionsRight(int distance) {
        left_front.setTargetPosition(distance);
        right_front.setTargetPosition(-distance);
        left_back.setTargetPosition(-distance);
        right_back.setTargetPosition(distance);
    }

    private static void setTargetPositionsNW(int distance) {
        left_front.setTargetPosition(0);
        right_front.setTargetPosition(distance);
        left_back.setTargetPosition(distance);
        right_back.setTargetPosition(0);
    }

    private static void setTargetPositionsNE(int distance) {
        left_front.setTargetPosition(distance);
        right_front.setTargetPosition(0);
        left_back.setTargetPosition(0);
        right_back.setTargetPosition(distance);
    }

    private static void setTargetPositionsSW(int distance) {
        left_front.setTargetPosition(-distance);
        right_front.setTargetPosition(0);
        left_back.setTargetPosition(0);
        right_back.setTargetPosition(-distance);
    }

    private static void setTargetPositionsSE(int distance) {
        left_front.setTargetPosition(0);
        right_front.setTargetPosition(-distance);
        left_back.setTargetPosition(-distance);
        right_back.setTargetPosition(0);
    }

    private static void setTargetPositionsTurnLeft(int distance) {
        left_front.setTargetPosition(-distance);
        right_front.setTargetPosition(distance);
        left_back.setTargetPosition(-distance);
        right_back.setTargetPosition(distance);
    }

    private static void setTargetPositionsTurnRight(int distance) {
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

    private static void goForward(double MotorPower) {
        left_front.setPower(MotorPower);
        right_front.setPower(MotorPower);
        left_back.setPower(MotorPower);
        right_back.setPower(MotorPower);
    }

    private static void goBackward(double MotorPower) {
        left_front.setPower(-MotorPower);
        right_front.setPower(-MotorPower);
        left_back.setPower(-MotorPower);
        right_back.setPower(-MotorPower);
    }

    private static void goLeft(double MotorPower) {
        left_front.setPower(-MotorPower);
        right_front.setPower(MotorPower);
        left_back.setPower(MotorPower);
        right_back.setPower(-MotorPower);
    }

    private static void goRight(double MotorPower) {
        left_front.setPower(MotorPower);
        right_front.setPower(-MotorPower);
        left_back.setPower(-MotorPower);
        right_back.setPower(MotorPower);
    }

    private static void goNW(double MotorPower) {
        left_front.setPower(0);
        right_front.setPower(MotorPower);
        left_back.setPower(MotorPower);
        right_back.setPower(0);
    }

    private static void goNE(double MotorPower) {
        left_front.setPower(MotorPower);
        right_front.setPower(0);
        left_back.setPower(0);
        right_back.setPower(MotorPower);
    }

    private static void goSW(double MotorPower) {
        left_front.setPower(-MotorPower);
        right_front.setPower(0);
        left_back.setPower(0);
        right_back.setPower(-MotorPower);
    }

    private static void goSE(double MotorPower) {
        left_front.setPower(0);
        right_front.setPower(-MotorPower);
        left_back.setPower(-MotorPower);
        right_back.setPower(0);
    }

    private static void goTurnLeft(double MotorPower) {
        left_front.setPower(-MotorPower);
        right_front.setPower(MotorPower);
        left_back.setPower(-MotorPower);
        right_back.setPower(MotorPower);
    }

    private static void goTurnRight(double MotorPower) {
        left_front.setPower(MotorPower);
        right_front.setPower(-MotorPower);
        left_back.setPower(MotorPower);
        right_back.setPower(-MotorPower);
    }

    public void driveBackward(int distanceIN, double...MotorPower) {
        double Motor_Power;
        if (MotorPower.length == 1) {
            Motor_Power = MotorPower[0];
        } else {
            Motor_Power = DRIVE_SPEED;
        }
        resetEncoders();
        setTargetPositionsForward((int) Math.round(PULSES_PER_IN));
        setRunToPosition();
        goForward(Motor_Power);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            // waiting for target position to be reached
        }
        stopDriving();
        setRunUsingEncoders();
    }

    public void driveForward(int distanceIN, double...MotorPower) {
        double Motor_Power;
        if (MotorPower.length == 1) {
            Motor_Power = MotorPower[0];
        } else {
            Motor_Power = DRIVE_SPEED;
        }
        resetEncoders();
        setTargetPositionsBackward((int) Math.round(PULSES_PER_IN));
        setRunToPosition();
        goBackward(Motor_Power);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            // waiting for target position to be reached
        }
        stopDriving();
        setRunUsingEncoders();
    }

    public void strafeLeft(int distanceIN, double...MotorPower) {
        double Motor_Power;
        if (MotorPower.length == 1) {
            Motor_Power = MotorPower[0];
        } else {
            Motor_Power = DRIVE_SPEED;
        }
        resetEncoders();
        setTargetPositionsLeft((int) Math.round(PULSES_PER_IN));
        setRunToPosition();
        goLeft(Motor_Power);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            // waiting for target position to be reached
        }
        stopDriving();
        setRunUsingEncoders();
    }

    public void strafeRight(int distanceIN, double...MotorPower) {
        double Motor_Power;
        if (MotorPower.length == 1) {
            Motor_Power = MotorPower[0];
        } else {
            Motor_Power = DRIVE_SPEED;
        }
        resetEncoders();
        setTargetPositionsRight((int) Math.round(PULSES_PER_IN));
        setRunToPosition();
        goRight(Motor_Power);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            // waiting for target position to be reached
        }
        stopDriving();
        setRunUsingEncoders();
    }

    public void strafeNW(int distanceIN, double...MotorPower) {
        double Motor_Power;
        if (MotorPower.length == 1) {
            Motor_Power = MotorPower[0];
        } else {
            Motor_Power = DRIVE_SPEED;
        }
        resetEncoders();
        setTargetPositionsNW((int) Math.round(PULSES_PER_IN));
        setRunToPosition();
        goNW(Motor_Power);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            // waiting for target position to be reached
        }
        stopDriving();
        setRunUsingEncoders();
    }

    public void strafeNE(int distanceIN, double...MotorPower) {
        double Motor_Power;
        if (MotorPower.length == 1) {
            Motor_Power = MotorPower[0];
        } else {
            Motor_Power = DRIVE_SPEED;
        }
        resetEncoders();
        setTargetPositionsNE((int) Math.round(PULSES_PER_IN));
        setRunToPosition();
        goNE(Motor_Power);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            // waiting for target position to be reached
        }
        stopDriving();
        setRunUsingEncoders();
    }

    public void strafeSW(int distanceIN, double...MotorPower) {
        double Motor_Power;
        if (MotorPower.length == 1) {
            Motor_Power = MotorPower[0];
        } else {
            Motor_Power = DRIVE_SPEED;
        }
        resetEncoders();
        setTargetPositionsSW((int) Math.round(PULSES_PER_IN));
        setRunToPosition();
        goSW(Motor_Power);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            // waiting for target position to be reached
        }
        stopDriving();
        setRunUsingEncoders();
    }

    public void strafeSE(int distanceIN, double...MotorPower) {
        double Motor_Power;
        if (MotorPower.length == 1) {
            Motor_Power = MotorPower[0];
        } else {
            Motor_Power = DRIVE_SPEED;
        }
        resetEncoders();
        setTargetPositionsSE((int) Math.round(PULSES_PER_IN));
        setRunToPosition();
        goSE(Motor_Power);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            // waiting for target position to be reached
        }
        stopDriving();
        setRunUsingEncoders();
    }

    public void turnLeft(int degrees, double...MotorPower) {
        double Motor_Power;
        if (MotorPower.length == 1) {
            Motor_Power = MotorPower[0];
        } else {
            Motor_Power = TURN_SPEED;
        }
        int hypotenuse = (int) Math.sqrt(Math.pow(ROBOT_LENGTH_IN / 2, 2) + Math.pow(ROBOT_WIDTH_IN / 2, 2));
        resetEncoders();
        setTargetPositionsTurnLeft((hypotenuse / 90) * degrees);
        setRunToPosition();
        goTurnLeft(Motor_Power);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            // waiting for target position to be reached
        }
        stopDriving();
        setRunUsingEncoders();
    }

    public void turnRight(int degrees, double...MotorPower) {
        double Motor_Power;
        if (MotorPower.length == 1) {
            Motor_Power = MotorPower[0];
        } else {
            Motor_Power = TURN_SPEED;
        }
        int hypotenuse = (int) Math.sqrt(Math.pow(ROBOT_LENGTH_IN / 2, 2) + Math.pow(ROBOT_WIDTH_IN / 2, 2));
        resetEncoders();
        setTargetPositionsTurnRight((hypotenuse / 90) * degrees);
        setRunToPosition();
        goTurnRight(Motor_Power);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            // waiting for target position to be reached
        }
        stopDriving();
        setRunUsingEncoders();
    }
}

/*
    // During init:
    AutoBase auto = new AutoBase();
    auto.InitAuto()

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