package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp
public class Gyro_Mecanum extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private BNO055IMU imu;

    @Override
    public void runOpMode() {
        float x;
        float y;
        double forward;
        float clockwise;
        double right;
        double fl;
        double fr;
        double bl;
        double br;
        float gyro_getHeading;

        frontLeft = hardwareMap.dcMotor.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.dcMotor.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.dcMotor.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.dcMotor.get(DcMotor.class, "backRight");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        IMU_Parameters = new BNO055IMU.Parameters();
        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(IMU_Parameters);
        telemetry.addData("Status", "IMU initialized");
        telemetry.update();
        while (!IMU_Calibrated()) {
            telemetry.addData("Don't ", "start yet...");
            telemetry.update();
            sleep(1000);
        }
        telemetry.addData("Status", "Calibration Complete");
        telemetry.update();
 
        waitForStart();
        if (opModeIsActive()) {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            while (opModeIsActive()) {
                x = gamepad1.left_stick_x;
                y = gamepad1.left_stick_y;

                gyro_getHeading = imu.getAngularOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.XYZ,
                    AngleUnit.DEGREES
                ).thirdAngle;
                telemetry.addData("gyro_AngularOrientation", gyro_getHeading);
                
                // this section is all testing
                telemetry.update()
                sleep(1000)
                // I don't know if the following line works/returns anything... it may just be an in place function
                telemetry.addData("gyro_getPosition", imu.getPosition());
                telemetry.addData("gyro_AngularOrientation", gyro_getHeading);

                // what is the number at the end? Maybe a calibration thing?
                sin = Math.sin(-gyro_getHeading * 0.0174533);
                cos = Math.cos(-gyro_getHeading * 0.0174533);
                
                forward = x * sin + y * cos;
                clockwise = gamepad1.right_stick_x;
                right = x * cos - y * sin;

                fl = forward - clockwise - right;
                fr = forward + clockwise - right;
                bl = forward - clockwise + right;
                br = forward + clockwise + right;

                frontLeft.setPower(fl);
                frontRight.setPower(fr);
                backLeft.setPower(bl);
                backRight.setPower(br);
                
                telemetry.update();
            }
        }
    }

    private boolean IMU_Calibrated() {
        telemetry.addData("IMU Calibration Status", imu.getCalibrationStatus());
        telemetry.addData("Gyro Calibrated", imu.isGyroCalibrated().toString());
        telemetry.addData("System Status", imu.getSystemStatus().toString());
        return imu.isGyroCalibrated();
    }
}