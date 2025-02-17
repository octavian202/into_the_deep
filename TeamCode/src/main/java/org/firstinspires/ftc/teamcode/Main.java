package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.Ascend;
import org.firstinspires.ftc.teamcode.commands.Drive;
import org.firstinspires.ftc.teamcode.commands.DriveRobotCentric;
import org.firstinspires.ftc.teamcode.commands.ExtendAscend;
import org.firstinspires.ftc.teamcode.commands.GripperRoll;
import org.firstinspires.ftc.teamcode.commands.ManualPivot;
import org.firstinspires.ftc.teamcode.commands.PickUpSample;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.PedroDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

import java.util.List;
import java.util.function.Supplier;

@TeleOp(name = "main", group = ".")
public class Main extends LinearOpMode {

//    private final Pose scorePose = new Pose(39, 70, Math.toRadians(180));
//    private final Pose scoreControlPose1 = new Pose(23, 65, Math.toRadians(180));
//    private final Pose scoreControlPose2 = new Pose(35, 73, Math.toRadians(180));
//    private final Pose scoreControlPose3 = new Pose(35, 73, Math.toRadians(180));
//    private final Pose startPose = new Pose(20, 27, Math.toRadians(180));
//    private final Pose pickupPose = new Pose(22, 27, Math.toRadians(180));
//    private final Pose pickupControlPose = new Pose(35, 27, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        CommandScheduler.getInstance().reset();

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        PedroDrivetrain pedroDrivetrain = new PedroDrivetrain(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);
        Extension extension = new Extension(hardwareMap);
        Gripper gripper = new Gripper(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        Trigger pivotIsUp = new Trigger(() -> pivot.getAngle() >= 45);
        Trigger pivotIsDown = new Trigger(() -> pivot.getAngle() < 45);
        Trigger extended = new Trigger(() -> extension.getTarget() >= 10000 && !extension.isBusy());
        Trigger retracted = new Trigger(() -> extension.getTarget() < 10000 && !extension.isBusy());


        Supplier<Double> speedSupplier = () -> 1.0 - gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 0.7;
        pedroDrivetrain.setDefaultCommand(new Drive(pedroDrivetrain, gp1::getLeftX, gp1::getLeftY, () -> gp1.getRightX() * 0.6, speedSupplier));


        gp1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(pivot::resetAngleVertical);
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(pivot::resetAngleHorizontal);
        pivot.resetAngleVertical();

        gp2.getGamepadButton(GamepadKeys.Button.B).and(new Trigger(() -> extension.getPosition() <= 10000)).whenActive(new ConditionalCommand(
                new InstantCommand(pivot::goDown, pivot),
                new InstantCommand(pivot::goUp, pivot),
                () -> pivot.getAngle() >= 45
        ));
        pivot.setDefaultCommand(new ManualPivot(pivot, gp2::getRightY));

        gp2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new ConditionalCommand(
                new InstantCommand(gripper::close),
                new InstantCommand(gripper::open),
                () -> gripper.isOpen
        ));


        gp2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ConditionalCommand(
                new InstantCommand(extension::goHighBasket),
                new ConditionalCommand(
                        new InstantCommand(extension::goDown),
                        new InstantCommand(extension::goHighChamber),
                        pivotIsDown::get
                ),
                () -> extension.getPosition() <= 17000
        ));



        gp2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new ConditionalCommand(
                new InstantCommand(() -> extension.setTarget(16000)),
                new ConditionalCommand(
                        new InstantCommand(extension::goDown),
                        new InstantCommand(extension::goHighChamber),
                        pivotIsDown::get
                ),
                () -> extension.getPosition() <= 10000
        ));

        gp2.getGamepadButton(GamepadKeys.Button.DPAD_UP).and(pivotIsDown).whenActive(new InstantCommand(arm::intakeSpecimen, arm));
        gp2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).and(pivotIsDown).whenActive(arm::intakeOverSubmersible);
        gp2.getGamepadButton(GamepadKeys.Button.X).and(pivotIsDown).whenActive(new PickUpSample(arm, gripper));

        gp2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new ExtendAscend(extension));
        gp2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new Ascend(extension));

        gp2.getGamepadButton(GamepadKeys.Button.Y).and(pivotIsUp).and(retracted).whenActive(() -> extension.setTarget(8000));

        (new Trigger(() -> extension.isAscending)).whileActiveContinuous(arm::outtakeSpecimen);

        pivotIsUp.whenActive(extension::goHighChamber);
        pivotIsDown.whenActive(extension::goDown);
        pivotIsDown.whenActive(new GripperRoll(gripper, gp2::getLeftX));

        pivotIsUp.and(new Trigger(() -> extension.getTarget() >= 10000)).whenActive(arm::outtakeSample);
        pivotIsUp.and(new Trigger(() -> extension.getTarget() >= 10000)).whenActive(() -> gripper.turn(90));


//        pivotIsUp.and(extended).whenActive(arm::outtakeSample);
//        pivotIsUp.and(extended).whenActive(() -> gripper.turn(90));
//        pivotIsUp.and(new Trigger(() -> extension.isBusy() && extension.getTarget() >= 10000)).whenActive(arm::idle);

        pivotIsUp.and(retracted).whenActive(arm::outtakeSpecimen);
        pivotIsUp.and(retracted).whenActive(() -> gripper.turn(180));

        pivotIsDown.and(extended).whenActive(arm::intakeOverSubmersible);
        pivotIsDown.and(extended).whenActive(gripper::open);

        pivotIsDown.and(new Trigger(() -> extension.getTarget() <= 10000)).whileActiveContinuous(arm::intakeSpecimen);

        waitForStart();

        arm.outtakeSpecimen();
        extension.goHighChamber();
        gripper.turn(180);

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();




            telemetry.addData("pos", extension.getPosition());
            telemetry.addData("angle", pivot.getAngle());
            telemetry.addData("angle target", Pivot.targetAngle);
            telemetry.update();
        }
    }
}
