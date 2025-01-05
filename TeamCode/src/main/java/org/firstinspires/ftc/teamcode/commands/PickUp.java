package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

public class PickUp extends SequentialCommandGroup {

    public PickUp(Arm arm, Gripper gripper) {
        addCommands(
                new InstantCommand(gripper::open, gripper),
                new InstantCommand(arm::intake, arm),
                new WaitCommand(400),
                new InstantCommand(gripper::close, gripper),
                new WaitCommand(200),
                new InstantCommand(arm::intakeOverSubmersible, arm)
        );

        addRequirements(arm, gripper);
    }

}
