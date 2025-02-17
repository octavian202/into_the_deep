package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.*;

public class PedroDrivetrain extends SubsystemBase {

    public Follower follower;
    public final Pose startPose = new Pose(0, 0, 0);

    public PedroDrivetrain(HardwareMap hardwareMap) {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        follower.startTeleopDrive();
    }

    @Override
    public void periodic() {
        follower.update();
    }

    public void drive(double x, double y, double rx) {
        follower.setTeleOpMovementVectors(y, -x, -rx);
    }

    public void auto() {
        follower.breakFollowing();
        follower.resumePathFollowing();
    }



}
