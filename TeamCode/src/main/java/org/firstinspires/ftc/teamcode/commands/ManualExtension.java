package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Extension;

import java.util.function.Supplier;

@Config
public class ManualExtension extends CommandBase {

    Extension extension;
    Supplier<Double> directionSupplier;
    public static double step = 2000;

    public ManualExtension(Extension extension, Supplier<Double> directionSupplier) {
        this.extension = extension;
        this.directionSupplier = directionSupplier;

        addRequirements(extension);
    }

    @Override
    public void execute() {
        int newTarget = (int)((double)extension.getTarget() + step * directionSupplier.get());
        extension.setTarget(newTarget);
    }

}
