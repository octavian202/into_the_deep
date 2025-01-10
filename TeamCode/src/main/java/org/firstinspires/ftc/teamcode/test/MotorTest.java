package org.firstinspires.ftc.teamcode.test;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "motors test", group = "test")
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MotorEx frontLeft = new MotorEx(hardwareMap, "lf");
        MotorEx frontRight = new MotorEx(hardwareMap, "rf");
        MotorEx backRight = new MotorEx(hardwareMap, "rb");
        MotorEx backLeft = new MotorEx(hardwareMap, "lb");

        frontLeft.setInverted(true);
        frontRight.setInverted(true);
        backLeft.setInverted(true);
        backRight.setInverted(true);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.cross) {
                backLeft.set(1d);
            } else {
                backLeft.set(0d);
            }
            if (gamepad1.square) {
                frontLeft.set(1d);
            } else {
                frontLeft.set(0d);
            }
            if (gamepad1.triangle) {
                frontRight.set(1d);
            } else {
                frontRight.set(0d);
            }
            if (gamepad1.circle) {
                backRight.set(1d);
            } else {
                backRight.set(0d);
            }
        }
    }
}
