package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveRobotCentric;
import org.firstinspires.ftc.teamcode.commands.GripperRoll;
import org.firstinspires.ftc.teamcode.commands.ManualPivot;
import org.firstinspires.ftc.teamcode.commands.PickUpSample;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

import java.util.List;
import java.util.function.Supplier;

@TeleOp(name = "solo", group = ".")
public class Solo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        CommandScheduler.getInstance().reset();

        GamepadEx gp = new GamepadEx(gamepad1);

        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);
        Extension extension = new Extension(hardwareMap);
        Gripper gripper = new Gripper(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        Trigger pivotIsUp = new Trigger(() -> pivot.getAngle() >= 45);
        Trigger pivotIsDown = new Trigger(() -> pivot.getAngle() < 45);
        Trigger extended = new Trigger(() -> extension.getTarget() >= 10000 && !extension.isBusy());
        Trigger retracted = new Trigger(() -> extension.getTarget() < 10000 && !extension.isBusy());


        drivetrain.setDefaultCommand(new DriveRobotCentric(drivetrain, gp::getLeftX, gp::getLeftY, gp::getRightX));

        pivot.resetAngleVertical();

        gp.getGamepadButton(GamepadKeys.Button.B).and(new Trigger(() -> extension.getPosition() <= 10000)).whenActive(new ConditionalCommand(
                new InstantCommand(pivot::goDown, pivot),
                new InstantCommand(pivot::goUp, pivot),
                () -> pivot.getAngle() >= 45
        ));
        Supplier<Double> pivotControl = () -> {
            if (Math.abs(gp.getRightY()) > 0.6) {
                return -gp.getRightY();
            }
            return 0d;
        };
        pivot.setDefaultCommand(new ManualPivot(pivot, pivotControl));

        gp.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new ConditionalCommand(
                new InstantCommand(gripper::close),
                new InstantCommand(gripper::open),
                () -> gripper.isOpen
        ));


        gp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ConditionalCommand(
                new InstantCommand(extension::goHighBasket),
                new ConditionalCommand(
                        new InstantCommand(extension::goDown),
                        new InstantCommand(extension::goHighChamber),
                        pivotIsDown::get
                ),
                () -> extension.getPosition() <= 10000
        ));

        gp.getGamepadButton(GamepadKeys.Button.X).and(pivotIsDown).whenActive(new PickUpSample(arm, gripper));

        pivotIsUp.whenActive(extension::goHighChamber);
        pivotIsDown.whenActive(extension::goDown);
        pivotIsDown.whenActive(new GripperRoll(gripper, () -> gp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));

        pivotIsUp.and(extended).whenActive(arm::outtakeSample);
        pivotIsUp.and(extended).whenActive(() -> gripper.turn(90));
        pivotIsUp.and(new Trigger(() -> extension.isBusy() && extension.getPosition() >= 10000)).whenActive(arm::idle);

        pivotIsUp.and(retracted).whenActive(arm::outtakeSpecimen);
        pivotIsUp.and(retracted).whenActive(() -> gripper.turn(180));

        pivotIsDown.and(extended).whenActive(arm::intakeOverSubmersible);
        pivotIsDown.and(extended).whenActive(gripper::open);

        pivotIsDown.and(new Trigger(() -> extension.getTarget() <= 10000)).whenActive(arm::intakeSpecimen);

        waitForStart();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            telemetry.addData("pos", extension.getPosition());
            telemetry.addData("angle", pivot.getAngle());
            telemetry.addData("angle target", Pivot.targetAngle);
            telemetry.update();
        }
    }
}
