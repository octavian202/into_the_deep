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

    }


    public void initInDimensions() {
        arm.setPosition(1.0);
        wrist.setPosition(0.5);
    }

    public void intakeOverSubmersible() {
        arm.setPosition(0.31);
        wrist.setPosition(0.97);
    }

    public void intakeAuto() {
        arm.setPosition(0.36);
        wrist.setPosition(0.97);
    }

    public void intakeSample() {
        arm.setPosition(0.26);
        wrist.setPosition(0.97);
    }
    public void intakeSpecimen() {
        arm.setPosition(0.6);
        wrist.setPosition(0.85);
    }


    public void outtakeSample() {
        arm.setPosition(0.26);
        wrist.setPosition(0.5);
    }

    public void outtakeSpecimen() {
        arm.setPosition(0.71);
        wrist.setPosition(0.85);
    }

    public void idle() {
        arm.setPosition(0.2);
        wrist.setPosition(0.65);
    }

    public void set(double armPos, double wristPos) {
        arm.setPosition(armPos);
        wrist.setPosition(wristPos);
    }
}
