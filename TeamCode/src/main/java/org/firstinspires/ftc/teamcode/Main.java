package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.SpecimenAuto;
import org.firstinspires.ftc.teamcode.commands.Ascend;
import org.firstinspires.ftc.teamcode.commands.Drive;
import org.firstinspires.ftc.teamcode.commands.ExtendAscend;
import org.firstinspires.ftc.teamcode.commands.GripperRoll;
import org.firstinspires.ftc.teamcode.commands.ManualPivot;
import org.firstinspires.ftc.teamcode.commands.PickUpSample;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.PedroDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

import java.util.List;
import java.util.function.Supplier;

@TeleOp(name = "main", group = ".")
public class Main extends LinearOpMode {

    PedroDrivetrain pedroDrivetrain;
    Pivot pivot;
    Gripper gripper;
    Arm arm;
    Extension extension;
    AutoState autoState;

    Timer pathTimer = new Timer();

    private final Pose startPose = new Pose(20, 27, Math.toRadians(180));
    private final Pose scorePose = new Pose(37.5, 70, Math.toRadians(180));
    private final Pose scoreControlPose1 = new Pose(23, 65, Math.toRadians(180));
    private final Pose scoreControlPose2 = new Pose(35, 73, Math.toRadians(180));
    private final Pose scoreControlPose3 = new Pose(35, 73, Math.toRadians(180));

    private final Pose pickupPose = new Pose(21, 27, Math.toRadians(180));
    private final Pose pickupControlPose = new Pose(35, 27, Math.toRadians(180));

    private PathChain scorePreload, grabPickup, scorePickup;

    private void buildPaths() {
        scorePreload = pedroDrivetrain.follower.pathBuilder()
                .addBezierLine(new Point(startPose), new Point(scorePose))
                .setZeroPowerAccelerationMultiplier(4.0)
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        grabPickup = pedroDrivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickupControlPose), new Point(pickupPose)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scorePickup = pedroDrivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupPose), new Point(scoreControlPose1), new Point(scoreControlPose2), new Point(scoreControlPose3), new Point(scorePose)))
                .setZeroPowerAccelerationMultiplier(4.0)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private void updateAuto() {
        switch (autoState) {
            case Manual:
                pedroDrivetrain.follower.startTeleopDrive();
                setPathState(AutoState.Nothing);

                break;
            case StartAuto:
                gripper.close();
                pedroDrivetrain.follower.setPose(startPose);
                scorePreload = pedroDrivetrain.follower.pathBuilder()
                        .addBezierLine(new Point(startPose), new Point(scorePose))
                        .setZeroPowerAccelerationMultiplier(4.0)
                        .setConstantHeadingInterpolation(startPose.getHeading())
                        .build();
                pedroDrivetrain.follower.followPath(scorePreload, false);
                pedroDrivetrain.auto();
                setPathState(AutoState.ScorePreload);

                break;
            case ScorePreload:
                if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    pivot.goUp();
//                    pedroDrivetrain.follower.followPath(scorePreload);
                    grabPickup = pedroDrivetrain.follower.pathBuilder()
                            .addPath(new BezierCurve(new Point(scorePose), new Point(pickupControlPose), new Point(pickupPose)))
                            .setConstantHeadingInterpolation(Math.toRadians(180))
                            .build();
                    setPathState(AutoState.Score);
                }

                break;
            case Score:
                if (!pedroDrivetrain.follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.8) {
                    gripper.open();
                    extension.goDown();
                    pivot.goDown();
                    pedroDrivetrain.follower.followPath(grabPickup, true);
                    setPathState(AutoState.ScoreToPickup);
                }

                break;
            case ScoreToPickup:

                if (pathTimer.getElapsedTimeSeconds() >= 1.0 && !pedroDrivetrain.follower.isBusy()) {
                    scorePickup = pedroDrivetrain.follower.pathBuilder()
                            .addPath(new BezierCurve(new Point(pickupPose), new Point(scoreControlPose1), new Point(scoreControlPose2), new Point(scoreControlPose3), new Point(scorePose)))
                            .setZeroPowerAccelerationMultiplier(4.0)
                            .setConstantHeadingInterpolation(Math.toRadians(180))
                            .build();
                    setPathState(AutoState.Pickup);
                }

                break;

            case Pickup:
                double pickupTime = pathTimer.getElapsedTimeSeconds();
                double waitTime = 0.4;
                if (pickupTime >= waitTime && pickupTime <= waitTime + 0.3) {
                    extension.setTarget(Extension.LOWER_LIMIT + 4000);
                } else if (waitTime + 0.8 < pickupTime && pickupTime <= waitTime + 0.9) {
                    gripper.close();
                } else if (pickupTime > waitTime + 1.0) {
                    pivot.goUp();

                    pedroDrivetrain.follower.followPath(scorePickup, false);
                    setPathState(AutoState.Score);
                }

                break;
        }
    }

    void setPathState(AutoState state) {
        autoState = state;
        pathTimer.resetTimer();
    }

    public enum AutoState {
        ScorePreload,
        Pickup,
        Score,
        ScoreToPickup,
        Manual,
        StartAuto,
        Nothing
    }

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        CommandScheduler.getInstance().reset();

        autoState = AutoState.Manual;

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        pedroDrivetrain = new PedroDrivetrain(hardwareMap);
        pivot = new Pivot(hardwareMap);
        extension = new Extension(hardwareMap);
        gripper = new Gripper(hardwareMap);
        arm = new Arm(hardwareMap);

        buildPaths();

        Trigger pivotIsUp = new Trigger(() -> pivot.getAngle() >= 45);
        Trigger pivotIsDown = new Trigger(() -> pivot.getAngle() < 45);
        Trigger extended = new Trigger(() -> extension.getTarget() >= 10000 && !extension.isBusy());
        Trigger retracted = new Trigger(() -> extension.getTarget() < 10000 && !extension.isBusy());


        Supplier<Double> speedSupplier = () -> 1.0 - gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 0.7;
        pedroDrivetrain.setDefaultCommand(new Drive(pedroDrivetrain, gp1::getLeftX, gp1::getLeftY, () -> gp1.getRightX() * 0.6, speedSupplier));


        gp1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(pivot::resetAngleVertical);
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(pivot::resetAngleHorizontal);
        pivot.resetAngleVertical();

        gp2.getGamepadButton(GamepadKeys.Button.B).and(new Trigger(() -> extension.getTarget() <= 13000)).whenActive(new ConditionalCommand(
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

        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> setPathState(AutoState.StartAuto));
        gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> setPathState(AutoState.Manual));

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

            updateAuto();


            telemetry.addData("pos", extension.getPosition());
            telemetry.addData("angle", pivot.getAngle());
            telemetry.addData("angle target", Pivot.targetAngle);
            telemetry.update();
        }
    }
}
