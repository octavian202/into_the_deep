package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

public class GoToDefaultPosition extends SequentialCommandGroup {

    public GoToDefaultPosition(Pivot pivot) {
        addCommands(
                new WaitCommand(100),
                new InstantCommand(pivot::goUp),
                new WaitCommand(700),
                new InstantCommand(pivot::resetAngleVertical)
        );

        addRequirements(pivot);
    }

}
