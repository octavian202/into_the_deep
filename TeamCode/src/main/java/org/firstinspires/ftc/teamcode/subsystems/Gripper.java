package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Gripper extends SubsystemBase {

    public static double OPEN = 0.54, CLOSED = 0.91;
    public static double LEFT = 1.02, RIGHT = 0.38;
            ;
    Servo servo, roll;
    public boolean isOpen = false;

    public Gripper(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "gripper");
        roll = hardwareMap.get(Servo.class, "roll");

        this.close();
//        this.turn(0);
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

    public void turn(double angle) { // -90 – 90 degrees
        double turnPercentage = (angle + 90) / 180;
        double position = (RIGHT - LEFT) * turnPercentage + LEFT;
        roll.setPosition(position);
    }

    public void aliniat() { turn(90); }
    public void turnLeft() {
        roll.setPosition(LEFT);
    }
    public void turnRight() {
        roll.setPosition(RIGHT);
    }

}
