package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

@Autonomous(name = "auto", group = ".")
public class OnlyInit extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Arm arm = new Arm(hardwareMap);
        arm.init();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("ceva", "ceva");
            telemetry.update();
        }

    }
}
