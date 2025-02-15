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

import java.util.List;
//import pedroPathing.constants.LConstants;

@Autonomous(name = "sample auto", group = ".")
public class SampleAuto extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Telemetry telemetryA;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private PathState pathState;

    private Extension extension;
    private Arm arm;
    private Pivot pivot;
    private Gripper gripper;
    private GoToDefaultPosition goToDefaultPosition;
//    private AutoArmControl autoArmControl;
    private PickUpSample pickUpSample;
    private int scoredSamples = 0;

//      Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
//      (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     // This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>


    private final Pose startPose = new Pose(-11.2, -8, Math.toRadians(-90));

    private final Pose scorePose = new Pose(5, 9, Math.toRadians(-45));

    private final Pose pickup1Pose = new Pose(13, 10, Math.toRadians(0));

    private final Pose pickup2Pose = new Pose(13, 17, Math.toRadians(0));

    private final Pose pickup3Pose = new Pose(16, 22, Math.toRadians(30));

    private final Pose parkPose = new Pose(52, -20, Math.toRadians(90));

    private final Pose parkControlPose = new Pose(52, -20, Math.toRadians(90));

    private PathChain scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    public void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addBezierLine(new Point(startPose), new Point(scorePose))
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
                    setPathState(PathState.ExtendingScore);
                }
                break;
            case ExtendingScore: // e deja la basket si pivot sus, extensia se ridica

                if (!extension.isBusy()) {
                    setPathState(PathState.Scoring);
                }

//                if (pathTimer.getElapsedTimeSeconds() <= 0.1) {
//                    extension.setTarget(Extension.HIGH_BASKET);
//                } else if (pathTimer.getElapsedTimeSeconds() >= 0.4 && !extension.isBusy()) {
//                    setPathState(PathState.Scoring);
//                }
                break;
            case Scoring:
                double scoringTime = pathTimer.getElapsedTimeSeconds();
                if (scoringTime >= 0.4 && scoringTime <= 0.5) { // timp de cand a ajuns sus ca sa se invarta bratu
                    gripper.open();
                } else if (0.7 < scoringTime) {
                    scoredSamples++;
                    extension.goDown();
                    setPathState(PathState.RetractingScore);
                }
                break;
            case RetractingScore:
                if (extension.getPosition() <= 13000) { // in timp ce coboara incepe sa se miste catre sample uri
                    if (scoredSamples == 1) {
                        follower.followPath(grabPickup1, true);
                    } else if (scoredSamples == 2) {
                        follower.followPath(grabPickup2, true);
                    } else if (scoredSamples == 3) {
                        follower.followPath(grabPickup3, true);
                    }
//                    else {
//                        follower.followPath(park);
//                    }
                    if (scoredSamples <= 3) {
                        pivot.goDown();
                        setPathState(PathState.BasketToPickup);
                    } else {
                        setPathState(PathState.Park);
                    }
                }
                break;
            case BasketToPickup:
                if (Pivot.angle <= 40) {
                    arm.intakeAuto();
                    gripper.turn(0);
                    gripper.open();
                }

                if(!follower.isBusy() && Pivot.angle <= 20 && pathTimer.getElapsedTimeSeconds() >= 0.9) {
                    extension.setTarget(Extension.HORIZONTAL_LIMIT - 3000);
                    if (scoredSamples == 3) {
                        gripper.turn(30);
                    } else {
                        gripper.turn(0);
                    }
                    if (!extension.isBusy()) {
                        setPathState(PathState.Pickup);
                    }
                }
                break;
            case Pickup:
                double pickupTime = pathTimer.getElapsedTimeSeconds();
                if (pickupTime >= 0.2 && pickupTime <= 0.3) { // abia a ajuns la sample
                    pickUpSample.schedule();
                } else if (0.8 <= pickupTime && pickupTime < 0.9) {
                    extension.goDown();
                } else if (1.0 <= pickupTime) {
                    pivot.goUp();
                    setPathState(PathState.PickupToBasket);
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
                        setPathState(PathState.ExtendingScore);
                    }
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

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

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
        Done
    }
}