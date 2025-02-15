package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = 0.002;
        TwoWheelConstants.strafeTicksToInches = -0.002;
        TwoWheelConstants.forwardY = -0.945;
        TwoWheelConstants.strafeX = 7.205;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "est";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "ascend";
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);

    }
}




