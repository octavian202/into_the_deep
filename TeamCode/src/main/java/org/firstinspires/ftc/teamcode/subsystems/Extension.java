package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Extension extends SubsystemBase {


    public static final int HIGH_BASKET = 47000, HIGH_CHAMBER = 8900, LOW_BASKET = 18000;
    public static double KP = 0.0005, KI = 0, KD = 0.000015, KG = 0.16;
    public static double ASCEND_KP = 0.002;
    public static int HORIZONTAL_LIMIT = 18000, LOWER_LIMIT = 0;
    Motor left, right, ascend;
    Servo shifter;
    Motor.Encoder encoder;
    PIDController pidController;
    PController ascendPController;
    private int target = 0;
    public boolean isAscending = false;

    public Extension(HardwareMap hardwareMap) {
        left = new Motor(hardwareMap, "est");
        right = new Motor(hardwareMap, "edr");
        ascend = new Motor(hardwareMap, "ascend");

        shifter = hardwareMap.get(Servo.class, "shifter");

        left.setRunMode(Motor.RunMode.RawPower);
        right.setRunMode(Motor.RunMode.RawPower);
        ascend.setRunMode(Motor.RunMode.RawPower);

        left.setInverted(true);

        encoder = right.encoder;
//        encoder.reset();

        pidController = new PIDController(KP, KI, KD);
//        pidController.setTolerance(400);

        ascendPController = new PController(ASCEND_KP);

        disengageAscend();
    }

    public void set(double power) {
        if (!isAscending) {
            left.set(power);
            right.set(power);
        } else {
            ascend.set(power);
        }
    }

    public void engageAscend() {

        left.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        left.set(0d);
        right.set(0d);

        ascend.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        isAscending = true;
        shifter.setPosition(0.15);
    }

    public void disengageAscend() {

        left.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        ascend.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        ascend.set(0d);

        isAscending = false;
        shifter.setPosition(0.01);
    }

    public void resetEncoder() {
        encoder.reset();
    }

    public int getPosition() {
        return encoder.getPosition();
    }

    public void setTarget(int newTarget) {
        if (Pivot.targetAngle <= 25) {
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

    public void goLowBasket() {
        this.set(LOW_BASKET);
    }

    public void goHighChamber() {
        this.setTarget(HIGH_CHAMBER);
    }
    public void goDown() {
        this.setTarget(LOWER_LIMIT);
    }

    public boolean isBusy() {
        return (Math.abs(getPosition() - getTarget()) > 1300);
    }

    @Override
    public void periodic() {
        pidController.setPID(KP, KI, KD);
        this.setTarget(target);

        double output;
        if (!isAscending) {
            output = pidController.calculate(this.getPosition(), target) + KG * Math.sin(Math.toRadians(Pivot.targetAngle));
        } else {
            output = ascendPController.calculate(this.getPosition(), target);
        }
        this.set(output);
    }
}
