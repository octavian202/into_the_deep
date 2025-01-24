package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

@Config
public class AutoArmControl extends CommandBase {
    public static double THRESHOLD = 800;
    private Arm arm;
    private Extension extension;
    private Gripper gripper;

    public AutoArmControl(Arm arm, Gripper gripper, Extension extension) {
        this.arm = arm;
        this.extension = extension;
        this.gripper = gripper;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        if (extension.getTarget() < Extension.HIGH_BASKET - THRESHOLD || Math.abs(extension.getPosition() - extension.getTarget()) > THRESHOLD) {
            arm.outtakeSpecimen();
            gripper.turn(182);
        } else {
            arm.outtakeSample();
            gripper.turn(90);
        }
    }

    @Override
    public boolean isFinished() {
        return (Pivot.targetAngle <= 40);
    }

}
