package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AutoArmControl;
import org.firstinspires.ftc.teamcode.commands.GoToDefaultPosition;
import org.firstinspires.ftc.teamcode.commands.PickUpSample;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.*;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.vision.SampleCamera;

import java.util.List;
//import pedroPathing.constants.LConstants;

@Autonomous(name = "sample+ auto", group = ".")
public class SamplePlusAuto extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Telemetry telemetryA;

    private PathState pathState;
    private Extension extension;
    private Arm arm;
    private Pivot pivot;
    private Gripper gripper;
    private SampleCamera camera;
    private GoToDefaultPosition goToDefaultPosition;
    //    private AutoArmControl autoArmControl;
    private PickUpSample pickUpSample;
    private int scoredSamples = 0;

//      Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
//      (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
    // This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>


    private final Pose startPose = new Pose(6.5, 111.5, Math.toRadians(-90));

    private final Pose scorePose = new Pose(21, 127, Math.toRadians(-45));

    private final Pose pickup1Pose = new Pose(23, 123.5, Math.toRadians(0));

    private final Pose pickup2Pose = new Pose(23, 130, Math.toRadians(0));

    private final Pose pickup3Pose = new Pose(25, 130.5, Math.toRadians(30));

    private final Pose parkPose = new Pose(52, -20, Math.toRadians(90));

    private final Pose parkControlPose = new Pose(52, -20, Math.toRadians(90));

    private final Pose submersiblePose = new Pose(65, 96, Math.toRadians(-90));
    private final Pose submersibleControlPose = new Pose(65, 120);

    private PathChain scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, submersiblePickup, scoreSubmersiblePickup;

    public void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addBezierCurve(new Point(startPose), new Point(scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        submersiblePickup = follower.pathBuilder()
                .addBezierCurve(new Point(scorePose), new Point(submersibleControlPose), new Point(submersiblePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), submersiblePose.getHeading())
                .build();

        scoreSubmersiblePickup = follower.pathBuilder()
                .addBezierCurve(new Point(submersiblePose), new Point(submersibleControlPose), new Point(scorePose))
                .setLinearHeadingInterpolation(submersiblePose.getHeading(), scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addBezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case StartToBasket:
                if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                    extension.goHighBasket();
                }
                if (!follower.isBusy()) { // verifica daca a ajuns la cos
                    setPathState(SamplePlusAuto.PathState.ExtendingScore);
                }
                break;
            case ExtendingScore: // e deja la basket si pivot sus, extensia se ridica

                if (extension.getPosition() >= 45000) {
                    setPathState(SamplePlusAuto.PathState.Scoring);
                }

                break;
            case Scoring:
                double scoringTime = pathTimer.getElapsedTimeSeconds();
                if (scoringTime <= 0.1) {
                    gripper.open();
                } else if (0.5 < scoringTime) {
                    scoredSamples++;
                    extension.goDown();
                    gripper.turn(0);
                    arm.intakeAuto();
                    setPathState(SamplePlusAuto.PathState.RetractingScore);
                }
                break;
            case RetractingScore:
                if (extension.getPosition() <= 50000) { // in timp ce coboara incepe sa se miste catre sample uri
                    if (scoredSamples == 1) {
                        follower.followPath(grabPickup1, true);
                    } else if (scoredSamples == 2) {
                        follower.followPath(grabPickup2, true);
                    } else if (scoredSamples == 3) {
                        follower.followPath(grabPickup3, true);
                    } else {
                        follower.followPath(submersiblePickup, false);
                    }
//                    else {
//                        follower.followPath(park);
//                    }
                    pivot.goDown();
                    if (scoredSamples <= 3) {
                        setPathState(SamplePlusAuto.PathState.BasketToPickup);
                    } else {
                        setPathState(PathState.ScoreToSubmersible);
                    }
                }
                break;
            case BasketToPickup:
                if (Pivot.angle <= 40) {
                    arm.intakeAuto();
                    extension.setTarget(Extension.HORIZONTAL_LIMIT - 3000);
                    gripper.turn(0);
                    gripper.open();
                }

                if(!follower.isBusy() && Pivot.angle <= 20 && pathTimer.getElapsedTimeSeconds() >= 0.9) {
                    if (scoredSamples == 3) {
                        gripper.turn(30);
                    } else {
                        gripper.turn(0);
                    }
                    if (!extension.isBusy()) {
                        setPathState(SamplePlusAuto.PathState.Pickup);
                    }
                }
                break;
            case Pickup:
                double pickupTime = pathTimer.getElapsedTimeSeconds();
                if (pickupTime >= 0.2 && pickupTime <= 0.3) { // abia a ajuns la sample
                    pickUpSample.schedule();
                } else if (0.8 <= pickupTime && pickupTime < 0.9) {
                    extension.goDown();
                } else if (1.0 <= pickupTime && pickupTime < 1.3) {
                    pivot.goUp();
                    if (Pivot.angle >= 70) {
                        extension.goHighBasket();
                    }
                } else if (pickupTime >= 1.4) {
                    extension.goHighBasket();
                    setPathState(SamplePlusAuto.PathState.PickupToBasket);
                    if (scoredSamples == 1) {
                        follower.followPath(scorePickup1, true);
                    } else if (scoredSamples == 2) {
                        follower.followPath(scorePickup2, true);
                    } else if (scoredSamples == 3) {
                        follower.followPath(scorePickup3, true);
                    }
                }
                break;
            case PickupToBasket:
                if (Pivot.angle >= 80) {
                    extension.goHighBasket();
                    if(!follower.isBusy()) {
                        setPathState(SamplePlusAuto.PathState.ExtendingScore);
                    }
                }

                break;

            case ScoreToSubmersible:
                if (Pivot.angle <= 40) {
                    arm.intakeSubmersibleAuto();
                    extension.setTarget(Extension.LOWER_LIMIT + 4000);
                    gripper.turn(0);
                    gripper.open();
                }

                if(!follower.isBusy() && Pivot.angle <= 20 && pathTimer.getElapsedTimeSeconds() >= 0.9) {
                    camera.resumeReading();
                    setPathState(PathState.Collecting);
                }

                break;

            case Collecting:

                gripper.turn(camera.getOrientation());

                org.opencv.core.Point sample = camera.getPosition();
                double distanceFromCenter = sample.x * sample.x + sample.y * sample.y;
                if (distanceFromCenter <= 0.05) {
                    camera.stopReading();
                    setPathState(PathState.Pickup);
                } else {
                    telemetry.addData("orientation", camera.getOrientation());
                    telemetry.addData("point", sample.toString());
                    telemetry.update();

                    follower.turn(Math.toRadians(sample.y * 8.0), true);
                    extension.setTarget((int)(extension.getPosition() - sample.x * 200.0));
                }

                break;
//            case Park:
//                if(!follower.isBusy()) {
//                    setPathState(PathState.Done);
//                }
//                break;
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

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

//        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        camera = new SampleCamera(hardwareMap);

        arm = new Arm(hardwareMap);

        pivot = new Pivot(hardwareMap);
        pivot.setAngledStart();

        gripper = new Gripper(hardwareMap);
        gripper.close();

        extension = new Extension(hardwareMap);
        extension.goHighChamber();

        goToDefaultPosition = new GoToDefaultPosition(pivot);
//        autoArmControl = new AutoArmControl(arm, gripper, extension);
        pickUpSample = new PickUpSample(arm, gripper);

        Trigger pivotIsUp = new Trigger(() -> pivot.getAngle() >= 45);
        Trigger pivotIsDown = new Trigger(() -> pivot.getAngle() < 45);

//        pivotIsUp.whenActive(autoArmControl);
        pivotIsUp.whenActive(() -> gripper.turn(90));
        pivotIsUp.and(new Trigger(() -> extension.isBusy())).whenActive(arm::idle);
        pivotIsUp.and(new Trigger(() -> !extension.isBusy())).whenActive(arm::outtakeSample);

        waitForStart();

        goToDefaultPosition.schedule();

        arm.idle();

        opmodeTimer.resetTimer();
        follower.followPath(scorePreload, true);
        setPathState(PathState.StartToBasket);

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            follower.update();
            autonomousPathUpdate();

            if (opmodeTimer.getElapsedTimeSeconds() >= 28.0) {
                setPathState(PathState.Park);
                park = follower.pathBuilder()
                    .addBezierLine(new Point(follower.getPose()), new Point(parkPose))
                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), parkPose.getHeading())
                    .build();
                follower.followPath(park);
                extension.goDown();
                pivot.goUp();
            }

            follower.telemetryDebug(telemetryA);

//            telemetry.addData("path state", pathState);
//            telemetry.addData("x", follower.getPose().getX());
//            telemetry.addData("y", follower.getPose().getY());
//            telemetry.addData("heading", follower.getPose().getHeading());
//            telemetry.addData("extension", extension.getPosition());
//            telemetry.addData("target", extension.getTarget());
//            telemetry.addData("busy", extension.isBusy());
//            telemetry.update();
        }
    }

    enum PathState {
        StartToBasket,
        ExtendingScore,
        Scoring,
        RetractingScore,
        BasketToPickup,
        Pickup,
        PickupToBasket,
        Park,
        ScoreToSubmersible,
        SubmersibleToScore,
        Collecting,
        Done
    }
}