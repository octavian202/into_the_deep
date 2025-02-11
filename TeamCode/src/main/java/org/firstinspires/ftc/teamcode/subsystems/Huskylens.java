package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

@Config
public class Huskylens extends SubsystemBase {

    private HuskyLens huskylens;
    private ElapsedTime elapsedTime;
    private List<HuskyLens.Block> myHuskyLensBlocks;
    private FtcDashboard dashboard;

    public Huskylens(HardwareMap hardwareMap) {
        huskylens = hardwareMap.get(HuskyLens.class, "huskylens");
        dashboard = FtcDashboard.getInstance();

        huskylens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        elapsedTime = new ElapsedTime();
    }

    public boolean knock() {
        return huskylens.knock();
    }

    public List<HuskyLens.Block> getBlocks() {
        return myHuskyLensBlocks;
    }

    public int getBlockId(HuskyLens.Block block) {
        return block.id;
    }

    public int getBlockWidth(HuskyLens.Block block) {
        return block.width;
    }

    public int getBlockHeight(HuskyLens.Block block) {
        return block.height;
    }

    public int getBlockX(HuskyLens.Block block) {
        return block.x;
    }

    public int getBlockY(HuskyLens.Block block) {
        return block.y;
    }

    public String getBlockPosition(HuskyLens.Block block) {
        return block.x + ", " + block.y;
    }

    @Override
    public void periodic() {
        myHuskyLensBlocks = Arrays.asList(huskylens.blocks());
    }
}
