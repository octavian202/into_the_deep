package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.function.Supplier;

public class DriveRobotCentric extends CommandBase {

    Drivetrain drivetrain;
    Supplier<Double> x, y, rx;

    public DriveRobotCentric(Drivetrain drivetrain, Supplier<Double> x, Supplier<Double> y, Supplier<Double> rx) {
        this.drivetrain = drivetrain;
        this.x = x;
        this.y = y;
        this.rx = rx;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.driveRobotCentric(x.get(), y.get(), rx.get(), 1d);
    }
}
