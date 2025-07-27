package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.GoToDefaultPosition;
import org.firstinspires.ftc.teamcode.commands.PickUpSample;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.*;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.vision.SampleCamera;

import java.util.List;

@Config
@Autonomous(name = "camera auto test", group = "test")
public class CameraAutoTest extends LinearOpMode {
    public static double DISTANCE = 3600, POSITION = 1.8, ZERO_POWER_ACCELERATION = 2.0;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private PathState pathState;
    private Extension extension;
    private Arm arm;
    private Pivot pivot;
    private Gripper gripper;
    private SampleCamera camera;
    private GoToDefaultPosition goToDefaultPosition;

    private PathChain adjustmentPath;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case Start:
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    pivot.goDown();
                    if (Pivot.angle <= 40) {
                        arm.intakeSubmersibleAuto();
                        extension.setTarget(Extension.LOWER_LIMIT + 4000);
                        gripper.turn(0);
                        gripper.open();
                    }

                    if (Pivot.angle <= 20) {
                        if (!extension.isBusy()) {
                            setPathState(PathState.Collecting);
                        }
                    }
                }
                break;
            case ScoreToSubmersible:
                if (Pivot.angle <= 40) {
                    arm.intakeSubmersibleAuto();
                    extension.setTarget(Extension.LOWER_LIMIT + 8000);
                    gripper.turn(0);
                    gripper.open();
                }

                if(!extension.isBusy() && Pivot.angle <= 20) {
                    setPathState(PathState.Collecting);
                }

                break;

            case Collecting:

                org.opencv.core.Point sample = camera.getPosition();
                double distanceFromCenter = sample.x * sample.x + sample.y * sample.y;

                if (distanceFromCenter <= 0.15 && (sample.x != 0.0 && sample.y != 0.0 && camera.getOrientation() != 0.0)) {
//                    gripper.turn(camera.getOrientation());

                    follower.holdPoint(follower.getPose());
                    extension.setTarget(extension.getPosition());

//                    setPathState(SamplePlusAuto.PathState.Pickup);
                    break;
                } else if (pathTimer.getElapsedTimeSeconds() >= 0.3) {

                    telemetry.addData("orientation", camera.getOrientation());
                    telemetry.addData("pos", camera.getPosition().toString());
                    telemetry.update();

                    double xAdjustment = sample.x * POSITION;
                    follower.holdPoint(new Pose(follower.getPose().getX() + xAdjustment, follower.getPose().getY(), Math.toRadians(-90)));
                    adjustmentPath = follower.pathBuilder()
                            .addBezierLine(new Point(follower.getPose()), new Point(follower.getPose().getX() + xAdjustment, follower.getPose().getY()))
                            .setConstantHeadingInterpolation(Math.toRadians(-90))
                            .setZeroPowerAccelerationMultiplier(ZERO_POWER_ACCELERATION)
                            .build();

                    double extensionTarget = extension.getPosition() + sample.y * DISTANCE;

                    follower.followPath(adjustmentPath, true);
                    extension.setTarget((int)extensionTarget);

                    setPathState(PathState.Collecting);
                }
                break;
        }
    }

    public void setPathState(PathState pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(-90)));

        camera = new SampleCamera(hardwareMap);

        arm = new Arm(hardwareMap);

        pivot = new Pivot(hardwareMap);
        pivot.setAngledStart();

        gripper = new Gripper(hardwareMap);
        gripper.close();

        extension = new Extension(hardwareMap);
        extension.goHighChamber();

        goToDefaultPosition = new GoToDefaultPosition(pivot);

        Trigger pivotIsUp = new Trigger(() -> pivot.getAngle() >= 45);
        Trigger pivotIsDown = new Trigger(() -> pivot.getAngle() < 45);

        pivotIsUp.whenActive(() -> gripper.turn(90));
        pivotIsUp.and(new Trigger(() -> extension.isBusy())).whenActive(arm::idle);
        pivotIsUp.and(new Trigger(() -> !extension.isBusy())).whenActive(arm::outtakeSample);

        waitForStart();


        goToDefaultPosition.schedule();
        PathChain path = follower.pathBuilder()
                .addBezierLine(new Point(follower.getPose()), new Point( new Pose(1, 0, Math.toRadians(-90))))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();


        follower.followPath(path, false);

        arm.idle();

        opmodeTimer.resetTimer();
        setPathState(PathState.Start);

        while (opModeIsActive()) {

            CommandScheduler.getInstance().run();

            follower.update();
            autonomousPathUpdate();

        }
    }

    enum PathState {
        Start,
        Pickup,
        ScoreToSubmersible,
        Collecting,
        Done
    }
}