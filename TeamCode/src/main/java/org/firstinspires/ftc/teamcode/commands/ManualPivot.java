package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Pivot;

import java.util.function.Supplier;

@Config
public class ManualPivot extends CommandBase {

    Pivot pivot;
    public static double step = 1;
    Supplier<Double> directionSupplier;

    public ManualPivot(Pivot pivot, Supplier<Double> supplier) {
        this.pivot = pivot;
        this.directionSupplier = supplier;

        addRequirements(pivot);
    }

    @Override
    public void execute() {
        int newAngleTarget = (int)(Pivot.targetAngle + step * directionSupplier.get());
        Pivot.targetAngle = newAngleTarget;

    }

}
