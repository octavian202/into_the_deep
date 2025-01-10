package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Extension;

import java.util.function.Supplier;

@Config
public class ManualExtension extends CommandBase {

    Extension extension;
    Supplier<Double> powerSupplier, angleSupplier;
    public static double KF = 0.05;

    public ManualExtension(Extension extension, Supplier<Double> powerSupplier, Supplier<Double> angle) {
        this.extension = extension;
        this.powerSupplier = powerSupplier;
        this.angleSupplier = angle;

        addRequirements(extension);
    }

    @Override
    public void execute() {
        double angle = angleSupplier.get();
        double power = powerSupplier.get();

        if (angle <= 85 && power > 0 && extension.getPosition() >= Extension.HORIZONTAL_LIMIT) {
            power = 0;
        }

        extension.set(power + KF * Math.abs(Math.sin(Math.toRadians(angle))));
    }

}
