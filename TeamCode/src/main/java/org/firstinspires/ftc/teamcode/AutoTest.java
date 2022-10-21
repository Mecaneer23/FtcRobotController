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
        CarouselSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        AutoBase auto = new AutoBase(
            this,
            hardwareMap,
            "frontLeft",
            "frontRight",
            "backLeft",
            "backRight",
            telemetry,
            1,
            0.5,
            17,
            13,
            1.13,
            100,
            false,
            0,
            0,
            0
        );
        waitForStart();
        if (opModeIsActive()) {
            auto.driveForward(36);
            auto.turnLeft(90);
            auto.strafeLeft(36);
            auto.driveBackward(24);
            auto.strafeNW(24);
            auto.turnRight(450);
        }
    }
}
