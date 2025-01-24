package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.nio.file.ClosedWatchServiceException;

@Config
public class Extension extends SubsystemBase {


    public static final int HIGH_BASKET = 47000, HIGH_RUNG = 17000;
    public static double KP = 0.0006, KI = 0, KD = 0.00001, KG = 0.09;
    public static int HORIZONTAL_LIMIT = 18000, LOWER_LIMIT = 0;
    Motor left, right;
    Motor.Encoder encoder;
    PIDController pidController;
    private int target = 0;

    public Extension(HardwareMap hardwareMap) {
        left = new Motor(hardwareMap, "est");
        right = new Motor(hardwareMap, "edr");

        left.setRunMode(Motor.RunMode.RawPower);
        right.setRunMode(Motor.RunMode.RawPower);

        left.setInverted(true);

        encoder = right.encoder;
//        encoder.reset();

        pidController = new PIDController(KP, KI, KD);
        pidController.setTolerance(400);
    }

    public void set(double power) {
        left.set(power);
        right.set(power);
    }

    public void resetEncoder() {
        encoder.reset();
    }

    public int getPosition() {
        return encoder.getPosition();
    }

    public void setTarget(int newTarget) {
        if (Pivot.targetAngle <= 40) {
            newTarget = Math.min(newTarget, HORIZONTAL_LIMIT);
            LOWER_LIMIT = 2224;
        } else {
            newTarget = Math.min(newTarget, HIGH_BASKET);
            LOWER_LIMIT = 0;
        }
        newTarget = Math.max(newTarget, LOWER_LIMIT);
        target = newTarget;
    }

    public int getTarget() {return target;}

    public void goHighBasket() {
        this.setTarget(HIGH_BASKET);
    }

    public void goHighRung() {
        this.setTarget(HIGH_RUNG);
    }
    public void goDown() {
        this.setTarget(LOWER_LIMIT);
    }

    public boolean isBusy() {
        return (Math.abs(getPosition() - getTarget()) > 400);
    }

    @Override
    public void periodic() {
        pidController.setPID(KP, KI, KD);
        this.setTarget(target);

        double output = pidController.calculate(this.getPosition(), target) + KG * Math.sin(Math.toRadians(Pivot.targetAngle));
        this.set(output);
    }
}
