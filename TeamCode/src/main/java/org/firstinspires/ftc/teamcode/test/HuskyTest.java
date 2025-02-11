package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Huskylens;

import java.util.List;

@TeleOp(name = "HuskyTest", group = "test")
public class HuskyTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CommandScheduler.getInstance().reset();

        Huskylens husky = new Huskylens(hardwareMap);

        telemetry.addData(">>", husky.knock() ? "apasa start ca sa pornesti" : "problema pl");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<HuskyLens.Block> blocks = husky.getBlocks();
            if (blocks != null) {
                for (HuskyLens.Block block : blocks) {
                    int id = husky.getBlockId(block);
                    int width = husky.getBlockWidth(block);
                    int height = husky.getBlockHeight(block);
                    int x = husky.getBlockX(block);
                    int y = husky.getBlockY(block);
                    String pos = husky.getBlockPosition(block);

                    telemetry.addData("block", "id=" + id + "  size=" + width + "x" + height + "  pos=" + pos);
                }
                telemetry.update();
            }

            CommandScheduler.getInstance().run();
        }
    }
}
