package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

@Config
public class AutoArmControl extends CommandBase {
    public static double THRESHOLD = 500;
    private Arm arm;
    private Extension extension;

    public AutoArmControl(Arm arm, Extension extension) {
        this.arm = arm;
        this.extension = extension;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        if (extension.getTarget() < Extension.HIGH_BASKET - THRESHOLD || Math.abs(extension.getPosition() - extension.getTarget()) > THRESHOLD) {
            arm.outtakeSpecimen();
        } else {
            arm.outtakeSample();
        }
    }

    @Override
    public boolean isFinished() {
        return (Pivot.targetAngle <= 40);
    }

}
