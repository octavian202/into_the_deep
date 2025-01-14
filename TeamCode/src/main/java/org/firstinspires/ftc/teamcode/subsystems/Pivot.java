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
    public static double KP = 0.025, KI = 0d, KD = 0.003, KF = 0.12;
    private PIDController pidController;
    Motor motor;
    Motor.Encoder encoder;
    public static double targetAngle = 95;
    public static double angle = 95;
    public static double STARTING_ANGLE = 95;

    public Pivot(HardwareMap hardwareMap) {
        motor = new Motor(hardwareMap, "pivot");
        encoder = motor.encoder;
        encoder.reset();
        motor.setInverted(true);
        pidController = new PIDController(KP, KI, KD);

        targetAngle = 95;
        angle = 95;
        STARTING_ANGLE = 95;
    }

    public void setAngledStart() {
        targetAngle = 40;
        angle = 40;
        STARTING_ANGLE = 40;
    }

    public double getAngle() {
        return angle;
    }

    public void set(double power) {

        motor.set(power);

    }

    public void resetAngleHorizontal() {
        STARTING_ANGLE = 0;
        encoder.reset();
    }

    public void resetAngleVertical() {
        STARTING_ANGLE = 95;
        encoder.reset();
    }

    @Override
    public void periodic() {
        pidController.setPID(KP, KI, KD);
        angle = STARTING_ANGLE - ANGLE_IN_TICK * encoder.getPosition();

        double output = pidController.calculate(angle, targetAngle) + KF * Math.cos(Math.toRadians(angle));
        this.set(output);

    }

    public void goDown() {
        targetAngle = 0;
    }
    public void goUp() {
        targetAngle = 95;
    }
}
