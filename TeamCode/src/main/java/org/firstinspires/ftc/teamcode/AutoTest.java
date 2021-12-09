package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.AutoBase;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutoTest extends LinearOpMode {
    
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
            (double) 17, //tbd
            (double) 13, //tbd
            (double) 0.5,
            (double) 0.5
        );
        waitForStart();
        if (opModeIsActive()) {
            auto.driveForward(24);
        }
    }
}
