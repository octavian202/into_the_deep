package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

@Autonomous(name = "specimen auto", group = ".")
public class SpecimenAuto extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private PathState pathState;

    private Extension extension;
    private Arm arm;
    private Pivot pivot;
    private Gripper gripper;
    private GoToDefaultPosition goToDefaultPosition;
    private AutoArmControl autoArmControl;
    private PickUpSample pickUpSample;
    private int scoredSpecimens = 0;
    private int collectedSamples = 0;

    private final Pose startPose = new Pose(-14, 3, Math.toRadians(180));

    private final Pose scorePose = new Pose(15, 3, Math.toRadians(180));
//    private final Pose scoreControlPose = new Pose(23, -5, Math.toRadians(180));


    private final Pose pickupPose = new Pose(2, -30, Math.toRadians(180));
    private final Pose pickupControlPose = new Pose(32, -41, Math.toRadians(180));

    private final Pose parkPose = new Pose(4, -42, Math.toRadians(90));
//    private final Pose parkControlPose = new Pose(4, -42, Math.toRadians(90));

    private final Pose pickupSamplePose1 = new Pose(17, -32, Math.toRadians(-30));
    private final Pose pickupSamplePose2 = new Pose(17, -44, Math.toRadians(-30));
    private final Pose pickupSamplePose3 = new Pose(17, -53, Math.toRadians(-30));

    private final Pose dropSamplePose1 = new Pose(12, -32, Math.toRadians(-120));
    private final Pose dropSamplePose2 = new Pose(12, -45, Math.toRadians(-120));

//    private final Pose dropSamplePose3 = new Pose(20, -40, Math.toRadians(-60));


    private PathChain scorePreload, park;
    private PathChain grabPickup, scorePickup, grabSample1, dropSample1, grabSample2, dropSample2, grabSample3, dropSample3;

    public void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addBezierLine(new Point(startPose), new Point(scorePose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        grabPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickupPose)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scorePickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupPose), new Point(scorePose)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        grabSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickupSamplePose1)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupSamplePose1.getHeading())
                .build();

        dropSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupSamplePose1), new Point(dropSamplePose1)))
                .setLinearHeadingInterpolation(pickupSamplePose1.getHeading(), dropSamplePose1.getHeading())
                .build();

        grabSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropSamplePose1), new Point(pickupSamplePose2)))
                .setLinearHeadingInterpolation(dropSamplePose1.getHeading(), pickupSamplePose2.getHeading())
                .build();

        dropSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupSamplePose2), new Point(dropSamplePose2)))
                .setLinearHeadingInterpolation(pickupSamplePose2.getHeading(), dropSamplePose2.getHeading())
                .build();

        grabSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropSamplePose2), new Point(pickupSamplePose3)))
                .setLinearHeadingInterpolation(dropSamplePose2.getHeading(), pickupSamplePose3.getHeading())
                .build();

        dropSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupSamplePose3), new Point(pickupPose)))
                .setLinearHeadingInterpolation(pickupSamplePose3.getHeading(), pickupPose.getHeading())
                .build();


        park = follower.pathBuilder()
                .addBezierLine(new Point(scorePose), new Point(parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case ScorePreload:
                if (pathTimer.getElapsedTimeSeconds() >= 0.6) {
                    extension.goHighChamber();
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.Scoring);
                }
                break;
            case Scoring:

                // e facut deja

                if (pathTimer.getElapsedTimeSeconds() <= 0.1) {
                    gripper.open();
                } else if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    scoredSpecimens++;

                    if (scoredSpecimens == 1) {
                        follower.followPath(grabSample1, true);
                        pivot.goDown();
                        extension.goDown();
                        setPathState(PathState.GoToSample);
                    } else if (scoredSpecimens < 5) {
                        pivot.goDown();
                        follower.followPath(grabPickup, true);
                        setPathState(PathState.ScoreToPickup);
                    } else {
                        follower.followPath(park);
                        setPathState(PathState.Park);
                    }
                }
                break;
//            case ScoreToPickup:
//                if(!follower.isBusy() && Pivot.angle <= 20) {
//                    setPathState(PathState.Pickup);
//                }
//                break;
//            case Pickup:
//                double pickupTime = pathTimer.getElapsedTimeSeconds();
//                double waitTime = 0.8;
//                if (pickupTime >= waitTime && pickupTime <= waitTime + 0.8) {
//                    extension.setTarget(Extension.LOWER_LIMIT + 4000);
//                } else if (waitTime + 0.8 < pickupTime && pickupTime <= waitTime + 1.0) {
//                    gripper.close();
//                } else if (pickupTime > waitTime + 1.0) {
//                    pivot.goUp();
//                    follower.followPath(scorePickup, true);
//                    setPathState(PathState.PickupToScore);
//                }
//                break;
//            case PickupToScore:
//                if (Pivot.angle >= 50) {
//                    extension.goHighChamber();
//                }
//                if(!follower.isBusy()) {
//                    setPathState(PathState.Scoring);
//                }
//                break;
            case GoToSample:

                if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    arm.intakeAuto();
                    gripper.turn(-30);
                }
                if (!follower.isBusy()) {
                    extension.goHighBasket();

                    if (!extension.isBusy()) {
                        pickUpSample.schedule();
                        collectedSamples++;
                        setPathState(PathState.PickupSample);
                    }
                }
                break;
            case PickupSample:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    if (collectedSamples == 1) {
                        follower.followPath(dropSample1, true);
                    } else if (collectedSamples == 2) {
                        follower.followPath(dropSample2, true);
                    } else {
                        follower.followPath(dropSample3, true);
                    }
                    extension.goDown();
                    setPathState(PathState.DropoffSample);
                }
                break;
            case DropoffSample:
                if (!follower.isBusy()) {
                    gripper.open();
                    if (collectedSamples == 1) {
                        follower.followPath(grabSample2);
                        setPathState(PathState.GoToSample);
                    } else if (collectedSamples == 2) {
                        follower.followPath(grabSample3);
                        setPathState(PathState.GoToSample);
                    } else if (pathTimer.getElapsedTimeSeconds() >= 1.2) {
                        arm.intakeSpecimen();
//                        setPathState(PathState.Pickup);
                    }
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
        autoArmControl = new AutoArmControl(arm, gripper, extension);
        pickUpSample = new PickUpSample(arm, gripper);

        Trigger pivotIsUp = new Trigger(() -> pivot.getAngle() >= 45);
        Trigger pivotIsDown = new Trigger(() -> pivot.getAngle() < 45);

        pivotIsUp.whenActive(arm::outtakeSpecimen);
        pivotIsUp.whenActive(() -> gripper.turn(182));

        pivotIsDown.whenActive(arm::intakeSpecimen);
        pivotIsDown.whenActive(() -> gripper.turn(0));

        waitForStart();

        goToDefaultPosition.schedule();

        opmodeTimer.resetTimer();

        follower.followPath(scorePreload, true);
        setPathState(PathState.ScorePreload);

        arm.outtakeSpecimen();
        gripper.close();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            follower.update();
            autonomousPathUpdate();

            // Feedback to Driver Hub
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
        ScorePreload,
        ScoreToPickup,
        Pickup,
        PickupToScore,
        Scoring,
        Park,
        PickupSample,
        DropoffSample,
        GoToSample,
        Done
    }
}