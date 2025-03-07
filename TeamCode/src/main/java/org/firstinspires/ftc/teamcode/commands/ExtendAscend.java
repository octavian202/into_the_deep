package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Extension;

public class ExtendAscend extends SequentialCommandGroup {

    public ExtendAscend(Extension extension) {
        addCommands(
                new InstantCommand(() -> extension.setTarget(29000)),
                new WaitCommand(1000),
                new InstantCommand(extension::engageAscend),
                new WaitCommand(2000)
        );
    }
}
