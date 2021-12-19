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

        AutoBase auto = new AutoBase();
        auto.InitAuto(
            hardwareMap,
            "frontLeft",
            "frontRight",
            "backLeft",
            "backRight",
            telemetry,
            (double) 17,
            (double) 13,
            0.5,
            0.5
        );
        waitForStart();
        if (opModeIsActive()) {
            auto.driveForward(12);
            sleep(100);
            auto.driveBackward(12);
            sleep(100);
            auto.strafeLeft(12);
            sleep(100);
            auto.strafeRight(12);
            sleep(100);
            auto.strafeNW(12);
            sleep(100);
            auto.strafeSE(12);
            sleep(100);
            auto.strafeNE(12);
            sleep(100);
            auto.strafeSW(12);
            sleep(100);
            auto.turnLeft(90);
            sleep(100);
            auto.turnRight(90);
            sleep(100);
        }
    }
}
