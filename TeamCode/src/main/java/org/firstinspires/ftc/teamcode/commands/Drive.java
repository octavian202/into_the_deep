package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.PedroDrivetrain;

import java.util.function.Supplier;

public class Drive extends CommandBase {

    private PedroDrivetrain drivetrain;
    private Supplier<Double> xSupplier, ySupplier, rxSupplier, speedSupplier;

    public Drive(PedroDrivetrain pedroDrivetrain, Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> rxSupplier, Supplier<Double> speedSupplier) {
        this.drivetrain = pedroDrivetrain;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rxSupplier = rxSupplier;
        this.speedSupplier = speedSupplier;

        addRequirements(pedroDrivetrain);
    }

    @Override
    public void execute() {
        double x = xSupplier.get(), y = ySupplier.get(), rx = rxSupplier.get(), speed = speedSupplier.get();

        x *= speed;
        y *= speed;
        rx *= speed;

        drivetrain.drive(x, y, rx);

    }

}
