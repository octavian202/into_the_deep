package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Extension extends SubsystemBase {


    public static int HORIZONTAL_LIMIT = 13000;
    Motor left, right;
    Motor.Encoder encoder;

    public Extension(HardwareMap hardwareMap) {
        left = new Motor(hardwareMap, "est");
        right = new Motor(hardwareMap, "edr");

        left.setRunMode(Motor.RunMode.RawPower);
        right.setRunMode(Motor.RunMode.RawPower);

        left.setInverted(true);

        encoder = right.encoder;
        encoder.setDirection(Motor.Direction.REVERSE);
    }

    public void set(double power) {
        left.set(power);
        right.set(power);
    }

    public int getPosition() {
        return encoder.getPosition();
    }
}
