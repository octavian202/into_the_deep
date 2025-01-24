package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
    private int scoredSamples = 0;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(4, -2, Math.toRadians(-90));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(12, 14, Math.toRadians(-45));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(13, 10, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(13, 19, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(15, 15.5, Math.toRadians(30));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(52, -20, Math.toRadians(90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(52, -20, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case StartToBasket:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    setPathState(PathState.ExtendingScore);
                }
                break;
            case ExtendingScore: // e deja la basket si pivot sus

                if (pathTimer.getElapsedTimeSeconds() <= 0.1) {
                    extension.setTarget(Extension.HIGH_BASKET);
                } else if (pathTimer.getElapsedTimeSeconds() >= 0.4 && !extension.isBusy()) {
                    setPathState(PathState.Scoring);
                }
                break;
            case Scoring:
                double scoringTime = pathTimer.getElapsedTimeSeconds();
                if (scoringTime >= 0.4 && scoringTime <= 1.0) {
                    gripper.open();
                } else if (1.0 < scoringTime) {
                    scoredSamples++;
                    extension.setTarget(Extension.LOWER_LIMIT);
                    setPathState(PathState.RetractingScore);
                }
                break;
            case RetractingScore:
                if (!extension.isBusy()) {

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
                if (Pivot.angle <= 30) {
                    arm.intakeOverSubmersible();
                    gripper.turn(0);
                }
                if(!follower.isBusy() && Pivot.angle <= 20 && pathTimer.getElapsedTimeSeconds() >= 1.7) {
                    extension.setTarget(Extension.HORIZONTAL_LIMIT - 4000);
                    gripper.open();
                    if (scoredSamples == 3) {
                        gripper.turn(30);
                    } else {
                        gripper.turn(0);
                    }
                    setPathState(PathState.Pickup);
                }
                break;
            case Pickup:
                double pickupTime = pathTimer.getElapsedTimeSeconds();
                if (pickupTime >= 1 && pickupTime <= 1.1) {
                    pickUpSample.schedule();
                } else if (1.8 <= pickupTime && pickupTime < 2.3) {
                    extension.setTarget(Extension.LOWER_LIMIT);
                } else if (pickupTime >= 2.3) {
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
                if(!follower.isBusy() && Pivot.angle >= 80) {
                    setPathState(PathState.ExtendingScore);
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

//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CommandScheduler.getInstance().reset();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        arm = new Arm(hardwareMap);
//        arm.intakeSample();

        pivot = new Pivot(hardwareMap);
        pivot.setAngledStart();

        gripper = new Gripper(hardwareMap);
        gripper.close();

        extension = new Extension(hardwareMap);
        extension.setTarget(2300);

        goToDefaultPosition = new GoToDefaultPosition(pivot);
        autoArmControl = new AutoArmControl(arm, gripper, extension);
        pickUpSample = new PickUpSample(arm, gripper);

        Trigger pivotIsUp = new Trigger(() -> pivot.getAngle() >= 45);
        Trigger pivotIsDown = new Trigger(() -> pivot.getAngle() < 45);

        pivotIsUp.whenActive(autoArmControl);

        waitForStart();

        goToDefaultPosition.schedule();
//        autoArmControl.schedule();

        opmodeTimer.resetTimer();
        follower.followPath(scorePreload, true);
        setPathState(PathState.StartToBasket);

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            follower.update();
            autonomousPathUpdate();

            // Feedback to Driver Hub
//            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("extension", extension.getPosition());
            telemetry.addData("target", extension.getTarget());
            telemetry.addData("busy", extension.isBusy());
            telemetry.update();
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