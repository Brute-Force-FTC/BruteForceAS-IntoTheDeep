package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import org.firstinspires.ftc.teamcode.Utilities.BackClaw;
import org.firstinspires.ftc.teamcode.Utilities.Claw;
import org.firstinspires.ftc.teamcode.Utilities.Extension;
import org.firstinspires.ftc.teamcode.Utilities.Intake;
import org.firstinspires.ftc.teamcode.Utilities.SlideKits;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "BucketNext", group = "Autonomous")
public class BucketNext extends PedroOpMode {
    public BucketNext() {
        super(Claw.INSTANCE, SlideKits.INSTANCE, BackClaw.INSTANCE, Extension.INSTANCE);
    }

    private Timer pathTimer, actionTimer, opmodeTimer;



    private int pathState, slideState;


    private final Pose startPose = new Pose(9, 80, Math.toRadians(0));

    private final Pose scorePreload = new Pose(35, 80, Math.toRadians(0));

    private final Pose pickupBlock1 = new Pose(29.4, 124+ Math.toRadians(180));
    private final Pose pickupBlock1Control = new Pose(14, 99, Math.toRadians(180));

    private final Pose dropoffBlock1 = new Pose(17.5, 132, Math.toRadians(135));

    private final Pose pickupBlock2 = new Pose(27.5, 136, Math.toRadians(180));

    private final Pose pickupBlock3 = new Pose(28, 134, Math.toRadians(230));


    private PathChain scoreSpecimen1, block1, block2, block3, scoreBlock1, scoreBlock2, scoreBlock3;

    public void buildPaths() {

        scoreSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePreload)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .setPathEndTimeoutConstraint(500)
                .build();

        block1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePreload), new Point(pickupBlock1Control), new Point(pickupBlock1)))
                .setLinearHeadingInterpolation(scorePreload.getHeading(), pickupBlock1.getHeading())
                .setPathEndTimeoutConstraint(500)
                .build();

        scoreBlock1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupBlock1), new Point(dropoffBlock1)))
                .setLinearHeadingInterpolation(pickupBlock1.getHeading(), dropoffBlock1.getHeading())
                .setPathEndTimeoutConstraint(500)
                .build();

        block2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropoffBlock1), new Point(pickupBlock2)))
                .setConstantHeadingInterpolation(pickupBlock2.getHeading())
                .setPathEndTimeoutConstraint(500)
                .build();

        scoreBlock2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupBlock2), new Point(dropoffBlock1)))
                .setLinearHeadingInterpolation(pickupBlock2.getHeading(), dropoffBlock1.getHeading())
                .setPathEndTimeoutConstraint(500)
                .build();

        block3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropoffBlock1), new Point(pickupBlock3)))
                .setLinearHeadingInterpolation(dropoffBlock1.getHeading(), pickupBlock3.getHeading())
                .setPathEndTimeoutConstraint(500)
                .build();
    }

    public Command autoPathUpdate() {
        return new SequentialGroup(
                SlideKits.INSTANCE.getResetEncoders(),
                BackClaw.INSTANCE.setUp(),
                Extension.INSTANCE.in(),
                new Delay(1.0),
                new ParallelGroup(
                        new FollowPath(scoreSpecimen1),
                        SlideKits.INSTANCE.middle()
                        ),
                new Delay(0.5),
                SlideKits.INSTANCE.high(),
                BackClaw.INSTANCE.prepare(),
                new Delay(1.0),
                new ParallelGroup(
                        new FollowPath(block1),
                        SlideKits.INSTANCE.low(),
                        Claw.INSTANCE.prepare()
                ),
                new Delay(1.0),
                Claw.INSTANCE.pickUp(),
                new Delay(1.0),
                new ParallelGroup(
                        Intake.INSTANCE.transfer(),
                        new FollowPath(scoreBlock1),
                        SlideKits.INSTANCE.high()
                        ),
                new Delay(1.0),
                new ParallelGroup(
                        new FollowPath(block2),
                        SlideKits.INSTANCE.low(),
                        Claw.INSTANCE.prepare()
                ),
                new Delay(1.0),
                Claw.INSTANCE.pickUp(),
                new Delay(1.0),
                new ParallelGroup(
                        Intake.INSTANCE.transfer(),
                        new FollowPath(scoreBlock2),
                        SlideKits.INSTANCE.high()
                ),
                new Delay(1.0),
                new ParallelGroup(
                        new FollowPath(block3),
                        SlideKits.INSTANCE.low(),
                        Claw.INSTANCE.prepare()
                ),
                new Delay(1.0),
                Claw.INSTANCE.pickUp()
                );
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void onUpdate() {

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("isBusy", follower.isBusy());
        telemetry.addData("isStuck", follower.isRobotStuck());
        telemetry.addData("isCooked", follower.isPinpointCooked());
        telemetry.addData("slideKitPosition", SlideKits.INSTANCE.motor.getCurrentPosition());
        telemetry.addData("slideKitTargetPosition", SlideKits.INSTANCE.controller.getTarget());
        telemetry.update();
    }

    @Override
    public void onInit() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }


    @Override
    public void onStartButtonPressed() {
        autoPathUpdate().invoke();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void onStop() {
    }
}

