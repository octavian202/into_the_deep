package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Extension;

@TeleOp(name = "reset extension encoder", group = "z")
public class ResetExtensionEncoder extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor extension = hardwareMap.get(DcMotor.class, "edr");
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("encoder ", "reset");
            telemetry.update();
        }
    }
}
