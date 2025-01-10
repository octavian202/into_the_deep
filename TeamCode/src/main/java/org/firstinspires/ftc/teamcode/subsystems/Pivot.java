package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Pivot extends SubsystemBase {

    public final double ANGLE_IN_TICK = 0.0439453125;
    public static double KP = 0.02, KI = 0d, KD = 0.001, KF = 0.005;
    private PIDController pidController;
    Motor motor;
    Motor.Encoder encoder;
    public static double targetAngle = 100;
    public static double angle = 100;
    public static double STARTING_ANGLE = 110;

    public Pivot(HardwareMap hardwareMap) {
        motor = new Motor(hardwareMap, "pivot");
        encoder = motor.encoder;
        encoder.reset();
        pidController = new PIDController(KP, KI, KD);
    }

    public double getAngle() {
        return angle;
    }

    public void set(double power) {

        if (power >= -0.1 && angle >= 98) {
            motor.set(0.05);
            return;
        }

        power *= 0.7;
        motor.set(power);

    }

    public void resetAngle() {
        STARTING_ANGLE = 0;
        encoder.reset();
    }

    @Override
    public void periodic() {
        angle = STARTING_ANGLE + ANGLE_IN_TICK * encoder.getPosition();
    }
}
