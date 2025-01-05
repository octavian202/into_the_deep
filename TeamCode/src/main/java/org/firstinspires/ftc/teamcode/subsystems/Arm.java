package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm extends SubsystemBase {

    Servo arm, wrist;

    public Arm(HardwareMap hardwareMap) {
        arm = hardwareMap.get(Servo.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");

        this.init();
    }

    public void intakeSpecimen() {
        arm.setPosition(1.0);
        wrist.setPosition(0.95);
    }

    public void intakeOverSubmersible() {
        arm.setPosition(0.63);
        wrist.setPosition(1.0);
    }

    public void intake() {
        arm.setPosition(0.57);
        wrist.setPosition(1.0);
    }

    public void outtakeSpecimen() {
        arm.setPosition(0.54);
        wrist.setPosition(0.5);
    }

    public void outtakeSample() {
        arm.setPosition(0.54);
        wrist.setPosition(0.65);
    }

    public void set(double armPos, double wristPos) {
        arm.setPosition(armPos);
        wrist.setPosition(wristPos);
    }

    public void init() {
        arm.setPosition(0.95);
        wrist.setPosition(0.5);
    }
}