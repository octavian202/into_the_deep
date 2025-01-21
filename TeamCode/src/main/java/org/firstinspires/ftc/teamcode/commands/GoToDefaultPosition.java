package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

public class GoToDefaultPosition extends SequentialCommandGroup {

    public GoToDefaultPosition(Pivot pivot, Arm arm) {
        addCommands(
                new WaitCommand(1000),
                new InstantCommand(pivot::goUp),
                new InstantCommand(arm::outtakeSample),
                new WaitCommand(1000),
                new InstantCommand(pivot::resetAngleVertical)
        );

        addRequirements(pivot, arm);
    }

}
