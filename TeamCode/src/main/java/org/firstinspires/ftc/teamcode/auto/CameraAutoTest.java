package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.controller.PIDController;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

@Config
@Autonomous(name = "camera auto test", group = ".")
public class CameraAutoTest extends LinearOpMode {

    public static double TURN_KP = 1.4, TURN_KD = 0.0, TURN_KF = 0.0;
    public static double EXTENSION_KP = 1.4, EXTENSION_KD = 0.0, EXTENSION_KF = 0.0;
    public static double DISTANCE = 600;
    PIDFController turnPIDF, extensionPIDF;

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
            case Pickup:
                double pickupTime = pathTimer.getElapsedTimeSeconds();
                if (pickupTime >= 0.2 && pickupTime <= 0.3) { // abia a ajuns la sample
                    pickUpSample.schedule();
                } else if (0.8 <= pickupTime && pickupTime < 0.9) {
                    extension.goDown();
                } else if (1.0 <= pickupTime && pickupTime < 1.3) {
                    pivot.goUp();
                    if (Pivot.angle >= 80) {
                        extension.goHighBasket();
                    }
                } else if (pickupTime >= 1.4) {
                    extension.goHighBasket();
                    setPathState(PathState.Done);

                }
                break;


            case ScoreToSubmersible:
                if (Pivot.angle <= 40) {
                    arm.intakeSubmersibleAuto();
                    extension.setTarget(Extension.LOWER_LIMIT + 8000);
                    gripper.turn(0);
                    gripper.open();
                    camera.resumeReading();
                }

                if(!extension.isBusy() && Pivot.angle <= 20) {
                    setPathState(PathState.Collecting);
                }

                break;

            case Collecting:

                if (pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    org.opencv.core.Point sample = camera.getPosition();
                    double distanceFromCenter = sample.x * sample.x + sample.y * sample.y;
                    if (distanceFromCenter <= 0.1 && (sample.x != 0.0 && sample.y != 0.0 && camera.getOrientation() != 0.0)) {
                        gripper.turn(camera.getOrientation());
                        follower.holdPoint(follower.getPose());
//                        camera.stopReading();
                        setPathState(PathState.Pickup);
                    } else {
                        telemetry.addData("reading", camera.reading);
                        telemetry.addData("orientation", camera.getOrientation());
                        telemetry.addData("point", sample.toString());
                        telemetry.update();

                        double angleTarget = turnPIDF.calculate(0, Math.toRadians(sample.x * 25.0));
                        double extensionTarget = extension.getPosition() + extensionPIDF.calculate(extension.getPosition(), extension.getPosition() + sample.y * DISTANCE);

                        follower.turn(angleTarget, true);
                        extension.setTarget((int)extensionTarget);
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


        turnPIDF = new PIDFController(TURN_KP, 0D, TURN_KD, TURN_KF);
        extensionPIDF = new PIDFController(EXTENSION_KP, 0D, EXTENSION_KD, EXTENSION_KF);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

//        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose());

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
        PathChain path = follower.pathBuilder()
                        .addBezierLine(new Point(new Pose()), new Point( new Pose(1, 1, 0)))
                .setConstantHeadingInterpolation(0)
                                .build();


        follower.followPath(path, false);

        arm.idle();

        opmodeTimer.resetTimer();
        setPathState(PathState.Start);

        while (opModeIsActive()) {

            turnPIDF.setPIDF(TURN_KP, 0D, TURN_KD, TURN_KF);
            extensionPIDF.setPIDF(EXTENSION_KP, 0D, EXTENSION_KD, EXTENSION_KF);

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
        Start,
        Pickup,
        ScoreToSubmersible,
        Collecting,
        Done
    }
}