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

    /*public void intakeSpecimen() {
        arm.setPosition(1.0);
        wrist.setPosition(0.95);
    }*/

    public void initInDimensions() {
        arm.setPosition(1.0);
        wrist.setPosition(0.5);
    }

    public void intakeOverSubmersible() {
        arm.setPosition(0.5);
        wrist.setPosition(0.83);
    }

    public void intakeSample() {
        arm.setPosition(0.3);
        wrist.setPosition(0.85);
    }
    public void intakeSpecimen() {
        arm.setPosition(0.34);
        wrist.setPosition(0.83);
    }


    public void outtake() {
        arm.setPosition(0.4);
        wrist.setPosition(0.41);
    }

    public void outtakeSpecimen() {
        arm.setPosition(0.15);
        wrist.setPosition(0.29);
    }

    public void set(double armPos, double wristPos) {
        arm.setPosition(armPos);
        wrist.setPosition(wristPos);
    }
}
