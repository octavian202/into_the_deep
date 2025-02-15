package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

public class Ascend extends SequentialCommandGroup {

    public Ascend(Extension extension) {
        addCommands(
                new InstantCommand(() -> extension.setTarget(11000)),
                new WaitCommand(1500),
                new InstantCommand(() -> Pivot.targetAngle = 35)
        );
    }

}
