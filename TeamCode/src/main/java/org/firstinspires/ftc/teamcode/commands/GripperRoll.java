package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

import java.util.function.Supplier;

public class GripperRoll extends CommandBase {

    Gripper gripper;
    Supplier<Double> turnSupplier; // -1 – 1

    public GripperRoll(Gripper gripper, Supplier<Double> turnSupplier) {
        this.gripper = gripper;
        this.turnSupplier = turnSupplier;

//        addRequirements(gripper);
    }

    @Override
    public void execute() {
        double turn = turnSupplier.get() * 90;
        gripper.turn(turn);
    }

    @Override
    public boolean isFinished() {
        return (Pivot.angle >= 45);
    }

}
