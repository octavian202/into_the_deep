package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Gripper extends SubsystemBase {

    public static double OPEN = 0.43, CLOSED = 0.83;
    public static double DEFAULT = 0.51, LEFT = 0.66, RIGHT = 0.36;
    Servo servo, roll;
    public boolean isOpen = false;

    public Gripper(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "gripper");
        roll = hardwareMap.get(Servo.class, "roll");
//        this.close();

        this.close();
        this.turnDefault();
    }

    public void set(double gripperPos, double rollPos) {
        servo.setPosition(gripperPos);
        roll.setPosition(rollPos);
    }

    public void close() {
        servo.setPosition(CLOSED);
        isOpen = false;
    }

    public void open() {
        servo.setPosition(OPEN);
        isOpen = true;
    }

    public void aliniat() { roll.setPosition(0.2); }
    public void turnDefault() {
        roll.setPosition(DEFAULT);
    }
    public void turnLeft() {
        roll.setPosition(LEFT);
    }
    public void turnRight() {
        roll.setPosition(RIGHT);
    }

}
