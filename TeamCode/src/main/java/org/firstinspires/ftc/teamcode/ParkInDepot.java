package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.AutoBase;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous
public class ParkInDepot extends LinearOpMode {
    
    private DcMotor CarouselSpinner;

    @Override
    public void runOpMode() {
        CarouselSpinner = hardwareMap.get(DcMotor.class, "CarouselSpinner");
        CarouselSpinner.setDirection(DcMotor.Direction.REVERSE);
        CarouselSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        AutoBase auto = new AutoBase();
        auto.InitAuto(
            hardwareMap,
            "frontLeft",
            "frontRight",
            "backLeft",
            "backRight",
            telemetry,
            (double) 18, //tbd
            (double) 18, //tbd
            (double) 0.5,
            (double) 0.5
        );
        waitForStart();
        if (opModeIsActive()) {
            auto.strafeLeft(3);
            sleep(100);
            CarouselSpinner.setPower((float) 0.35);
            sleep(5000);
            CarouselSpinner.setPower(0);
            auto.driveForward(18);
            auto.strafeRight(22);
            auto.driveForward(8);
            auto.strafeLeft(4);
        }
    }
}