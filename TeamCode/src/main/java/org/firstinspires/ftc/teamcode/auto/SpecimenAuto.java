package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    private Telemetry telemetryA;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private PathState pathState;

    private Extension extension;
    private Arm arm;
    private Pivot pivot;
    private Gripper gripper;
    private GoToDefaultPosition goToDefaultPosition;
    private int scoredSpecimens = 0;
    private int collectedSamples = 0;

    private final Pose startPose = new Pose(8.5, 65, Math.toRadians(180));
    private final Pose scorePreloadControlPose = new Pose(37, 65, Math.toRadians(180));
    private final Pose scorePreloadPose = new Pose(38, 65, Math.toRadians(180));



    private final Pose scorePose = new Pose(37.5, 70, Math.toRadians(180));
    private final Pose scoreControlPose1 = new Pose(23, 65, Math.toRadians(180));
    private final Pose scoreControlPose2 = new Pose(35, 73, Math.toRadians(180));
    private final Pose scoreControlPose3 = new Pose(35, 73, Math.toRadians(180));

    private final Pose pickupPose = new Pose(21, 27, Math.toRadians(180));
    private final Pose pickupControlPose = new Pose(35, 27, Math.toRadians(180));

    private final Pose parkPose = new Pose(20, 70, Math.toRadians(0));
//    private final Pose parkControlPose = new Pose(4, -42, Math.toRadians(90));

    private final Pose dropSamplePose1 = new Pose(28, 26, Math.toRadians(180));
    private final Pose dropSamplePose2 = new Pose(28, 14, Math.toRadians(180));
    private final Pose dropSamplePose3 = new Pose(19, 9, Math.toRadians(180));

    private final Pose samplePose1 = new Pose(56, 26, Math.toRadians(180));
    private final Pose samplePose2 = new Pose(56, 14, Math.toRadians(180));
    private final Pose samplePose3 = new Pose(56, 9, Math.toRadians(180));


    private PathChain scorePreload, park;
    private PathChain scorePickup1, scorePickup2, scorePickup3, grabPickup, grabPickup1, grabPickup2, grabPickup3, scorePickup, collectSample1, collectSample2, collectSample3, samplesToPickup;

    public void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addBezierLine(new Point(startPose), new Point(scorePose))
                .setZeroPowerAccelerationMultiplier(4.0)
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        grabPickup = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickupControlPose), new Point(pickupPose)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickupControlPose), new Point(pickupPose)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickupControlPose), new Point(pickupPose)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickupControlPose), new Point(pickupPose)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scorePickup = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(dropSamplePose3), new Point(scoreControlPose1), new Point(scoreControlPose2), new Point(scoreControlPose3), new Point(scorePose)))
                .setZeroPowerAccelerationMultiplier(4.0)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupPose), new Point(scoreControlPose1), new Point(scoreControlPose2), new Point(scoreControlPose3), new Point(scorePose)))
                .setZeroPowerAccelerationMultiplier(4.0)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupPose), new Point(scoreControlPose1), new Point(scoreControlPose2), new Point(scoreControlPose3), new Point(scorePose)))
                .setZeroPowerAccelerationMultiplier(4.0)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupPose), new Point(scoreControlPose1), new Point(scoreControlPose2), new Point(scoreControlPose3), new Point(scorePose)))
                .setZeroPowerAccelerationMultiplier(4.0)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        collectSample1 = follower.pathBuilder()
                .addBezierCurve(new Point(scorePose), new Point(new Pose(20, 25)), new Point(new Pose(60, 45)),  new Point(samplePose1))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(4.0)
                .addBezierCurve(new Point(samplePose1), new Point(dropSamplePose1), new Point(dropSamplePose1))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(7.0)
                .build();

        collectSample2 = follower.pathBuilder()
                .addBezierCurve(new Point(dropSamplePose1), new Point(new Pose(60, 24)), new Point(samplePose2))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(4.0)
                .addBezierCurve(new Point(samplePose2), new Point(dropSamplePose2), new Point(dropSamplePose2))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(6.0)
                .build();

        collectSample3 = follower.pathBuilder()
                .addBezierCurve(new Point(dropSamplePose2), new Point(new Pose(56, 14)), new Point(samplePose3))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3.0)
                .addBezierCurve(new Point(samplePose3), new Point(dropSamplePose3), new Point(dropSamplePose3))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(4.0)
                .build();

        samplesToPickup = follower.pathBuilder()
                .addBezierCurve(new Point(dropSamplePose3), new Point(pickupPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        park = follower.pathBuilder()
                .addBezierLine(new Point(scorePose), new Point(parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case ScorePreload:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    extension.goHighChamber();
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.Scoring);
                }
                break;
            case Scoring:

                // e facut deja

                gripper.open();
                extension.goDown();

                scoredSpecimens++;

                if (scoredSpecimens == 1) {
//                    pivot.goDown();
                    extension.goDown();
                    follower.followPath(collectSample1, false);
                    setPathState(PathState.CollectSample);
                } else if (scoredSpecimens < 5) {
//                    pivot.goDown();
                    extension.goDown();

                    if (scoredSpecimens == 2) {
                        follower.followPath(grabPickup, true);
                    } else if (scoredSpecimens == 3) {
                        follower.followPath(grabPickup1, true);
                    } else if (scoredSpecimens == 4) {
                        follower.followPath(grabPickup2, true);
                    }

                    setPathState(PathState.ScoreToPickup);
                } else {
                    follower.followPath(park);
                    setPathState(PathState.Park);
                }

//                if (pathTimer.getElapsedTimeSeconds() <= 0.1) {
//                    gripper.open();
//                } else if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
//                }
                break;
            case ScoreToPickup:
                if (pathTimer.getElapsedTimeSeconds() >= 0.2) {
                    pivot.goDown();
                }
                if(!follower.isBusy() && Pivot.angle <= 20 && pathTimer.getElapsedTimeSeconds() >= 0.3) {
                    setPathState(PathState.Pickup);
                }
                break;
            case Pickup:
                double pickupTime = pathTimer.getElapsedTimeSeconds();
                double waitTime = 0.2;
                if (pickupTime >= waitTime && pickupTime <= waitTime + 0.3) {
                    extension.setTarget(Extension.LOWER_LIMIT + 4000);
                } else if (waitTime + 0.8 < pickupTime && pickupTime <= waitTime + 0.9) {
                    gripper.close();
                } else if (pickupTime > waitTime + 1.0) {

                    pivot.goUp();

                    if (scoredSpecimens == 1) {
                        follower.followPath(scorePickup, true);
                    } else if (scoredSpecimens == 2) {
                        follower.followPath(scorePickup1, true);
                    } else if (scoredSpecimens == 3) {
                        follower.followPath(scorePickup2, true);
                    } else if (scoredSpecimens == 4) {
                        follower.followPath(scorePickup3, true);
                    }
                    setPathState(PathState.PickupToScore);
                }
                break;
            case PickupToScore:
                if (Pivot.angle >= 60) {
                    extension.goHighChamber();
                }
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.4) {
                    setPathState(PathState.Scoring);
                }
                break;
            case CollectSample:

                if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                    pivot.goDown();
                    arm.intakeSpecimen();
                    gripper.open();
                }

                if (pathTimer.getElapsedTimeSeconds() >= 0.4 && !follower.isBusy()) {
                    collectedSamples++;

                    if (collectedSamples == 1) {
                        follower.followPath(collectSample2, false);
                        setPathState(PathState.CollectSample);
                    } else if (collectedSamples == 2) {
                        follower.followPath(collectSample3, true);
                        setPathState(PathState.CollectSample);
                    } else {
                        pivot.goDown();
                        arm.intakeSpecimen();
                        setPathState(PathState.Pickup);
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

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

//        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        arm = new Arm(hardwareMap);
        pivot = new Pivot(hardwareMap);
        gripper = new Gripper(hardwareMap);
        extension = new Extension(hardwareMap);

        pivot.setAngledStart();
        gripper.close();

        goToDefaultPosition = new GoToDefaultPosition(pivot);

        Trigger pivotIsUp = new Trigger(() -> pivot.getAngle() >= 20);
        Trigger pivotIsDown = new Trigger(() -> pivot.getAngle() < 20);

        pivotIsUp.whenActive(arm::outtakeSpecimen);
        pivotIsUp.whenActive(() -> gripper.turn(180));

        pivotIsDown.whenActive(arm::intakeSpecimen);
        pivotIsDown.whenActive(() -> gripper.turn(0));

        waitForStart();

        opmodeTimer.resetTimer();

        follower.followPath(scorePreload);
        setPathState(PathState.ScorePreload);

        goToDefaultPosition.schedule();

        arm.outtakeSpecimen();
        gripper.close();
        gripper.turn(180);
        extension.setTarget(3000);

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            follower.update();
            autonomousPathUpdate();


//            follower.telemetryDebug(telemetryA);

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
        CollectSample,
        GoToSample,
        Done
    }
}