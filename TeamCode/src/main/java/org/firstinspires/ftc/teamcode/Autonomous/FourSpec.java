package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import org.firstinspires.ftc.teamcode.NextFTC.FollowPathWithSpeed;
import org.firstinspires.ftc.teamcode.Utilities.BackClaw;
import org.firstinspires.ftc.teamcode.Utilities.Claw;
import org.firstinspires.ftc.teamcode.Utilities.Extension;
import org.firstinspires.ftc.teamcode.Utilities.Intake;
import org.firstinspires.ftc.teamcode.Utilities.SlideKits;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "FourSpec😪🤤🥴😵💦🫳", group = "Autonomous")
public class FourSpec extends PedroOpMode {
    public FourSpec() {
        super(Claw.INSTANCE, SlideKits.INSTANCE, BackClaw.INSTANCE, Extension.INSTANCE, Intake.INSTANCE);
    }

    private final Pose startPose = new Pose(9.5, 60, Math.toRadians(0));

    private final Pose scorePreload = new Pose(38, 60, Math.toRadians(0));

    private final Pose moveBlock1 = new Pose(50, 44, Math.toRadians(180));
    private final Pose moveBlock1Control1 = new Pose(20, 36, Math.toRadians(180));

    private final Pose parkBlock1 = new Pose(18, 27, Math.toRadians(180));
    private final Pose parkBlock1Control = new Pose(65, 30.5, Math.toRadians(180));

    private final Pose moveBlock2 = new Pose(55, 36, Math.toRadians(180));
    private final Pose moveBlock2Control = new Pose(57, 6.5, Math.toRadians(180));

    private final Pose pickupSpec1 = new Pose(8, 32, Math.toRadians(180));

    private final Pose scoreSpec = new Pose(38, 68, Math.toRadians(0));
    private final Pose scoreSpec2 = new Pose(38, 75, Math.toRadians(0));
    private final Pose scoreSpec3 = new Pose(38, 65, Math.toRadians(0));
    private final Pose scoreSpecControl = new Pose(4, 75, Math.toRadians(0));

    private final Pose positionSpec = new Pose(15, 42, Math.toRadians(180));

    private final Pose pickupSpec = new Pose(9, 42, Math.toRadians(180));

    private final Pose controlSpec = new Pose(9, 70, Math.toRadians(0));
    private final Pose controlSpec2 = new Pose(32, 72, Math.toRadians(0));
    private final Pose controlSpec3 = new Pose(32, 72, Math.toRadians(0));

    private final Pose park = new Pose(30, 65, Math.toRadians(0));


    private PathChain scoreSpecimen1, block1, pushBlock1, block2, pushBlock2, scoreSpecimen42, parkBot,
            scoreSpecimen2, prepSpec3, spec3, scoreSpecimen3, scoreSpecimen32, scoreSpecimen4, scoreSpecimen22;

    public void buildPaths() {

        scoreSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePreload)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .setPathEndTimeoutConstraint(500)
                .build();

        block1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePreload), new Point(moveBlock1Control1), new Point(moveBlock1)))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        pushBlock1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(moveBlock1), new Point(parkBlock1Control), new Point(parkBlock1)))
                .setConstantHeadingInterpolation(moveBlock1.getHeading())
                .build();

        block2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(parkBlock1), new Point(moveBlock2)))
                .setConstantHeadingInterpolation(parkBlock1.getHeading())
                .build();

        pushBlock2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(moveBlock2), new Point(moveBlock2Control), new Point(pickupSpec1)))
                .setConstantHeadingInterpolation(moveBlock2.getHeading())
                .setPathEndTimeoutConstraint(500)
                .build();

        scoreSpecimen2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupSpec1), new Point(controlSpec), new Point(scoreSpec)))
                .setLinearHeadingInterpolation(pickupSpec1.getHeading(), scoreSpec.getHeading(), 0.9)
                .build();

        scoreSpecimen22 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(controlSpec), new Point(scoreSpec)))
                .setConstantHeadingInterpolation(scoreSpec.getHeading())
                .setPathEndTimeoutConstraint(500)
                .build();

        prepSpec3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSpec), new Point(positionSpec)))
                .setLinearHeadingInterpolation(scoreSpec.getHeading(), positionSpec.getHeading())
                .build();

        spec3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(positionSpec), new Point(pickupSpec)))
                .setConstantHeadingInterpolation(positionSpec.getHeading())
                .build();

        scoreSpecimen3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupSpec), new Point(controlSpec), new Point(scoreSpec2)))
                //scoreSpec2
                .setLinearHeadingInterpolation(pickupSpec.getHeading(), scoreSpec2.getHeading(), 0.9)
                .build();

        scoreSpecimen32 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(controlSpec2), new Point(scoreSpec2)))
                //scoreSpec2
                .setConstantHeadingInterpolation(scoreSpec.getHeading())
                .setPathEndTimeoutConstraint(500)
                .build();

        scoreSpecimen4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupSpec), new Point(controlSpec), new Point(scoreSpec3)))
                //scoreSpec3
                .setLinearHeadingInterpolation(pickupSpec.getHeading(), scoreSpec3.getHeading(), 0.9)
                .build();

        scoreSpecimen42 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(controlSpec3), new Point(scoreSpec3)))
                //scoreSpec2
                .setConstantHeadingInterpolation(scoreSpec.getHeading())
                .setPathEndTimeoutConstraint(500)
                .build();

        parkBot = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSpec3), new Point(park)))
                //scoreSpec3
                .setConstantHeadingInterpolation(scoreSpec3.getHeading())
                .build();

    }

    public Command autoPathUpdate() {
        return new SequentialGroup(
                SlideKits.INSTANCE.getResetEncoders(),
                BackClaw.INSTANCE.setUp(),
                BackClaw.INSTANCE.spinT(),
                Extension.INSTANCE.in(),
                new Delay(0.3),

                new ParallelGroup(
                        new FollowPathWithSpeed(scoreSpecimen1, true, 0.85),
                        SlideKits.INSTANCE.auto()
                ),
                BackClaw.INSTANCE.clip(),

                new Delay(0.3),

                new ParallelGroup(
                        new FollowPath(block1),
                        BackClaw.INSTANCE.prepare()
                ),

                new ParallelGroup(
                        new FollowPath(pushBlock1),
                        SlideKits.INSTANCE.low(),
                        Claw.INSTANCE.zero()
                ),

                new FollowPath(block2),

                new FollowPathWithSpeed(pushBlock2, true, 0.75),
                new Delay(0.4),

                BackClaw.INSTANCE.setUp(),
                new Delay(0.1),

                new ParallelGroup(
                        new FollowPath(scoreSpecimen2),
                        BackClaw.INSTANCE.spinT(),
                        SlideKits.INSTANCE.auto2()
                ),

                BackClaw.INSTANCE.clip(),

                new Delay(0.3),

                new ParallelGroup(
                        new FollowPath(prepSpec3),
                        SlideKits.INSTANCE.low(),
                        BackClaw.INSTANCE.prepare()
                ),

                new FollowPath(spec3),

                BackClaw.INSTANCE.setUp(),
                new Delay(0.1),

                new ParallelGroup(
                        new FollowPath(scoreSpecimen3),
                        BackClaw.INSTANCE.spinT(),
                        SlideKits.INSTANCE.auto2()
                ),

                BackClaw.INSTANCE.clip(),

                new Delay(0.3),

                new ParallelGroup(
                        new FollowPath(prepSpec3),
                        SlideKits.INSTANCE.low(),
                        BackClaw.INSTANCE.prepare()
                ),

                new FollowPath(spec3),

                BackClaw.INSTANCE.setUp(),
                new Delay(0.1),

                new ParallelGroup(
                        new FollowPath(scoreSpecimen4),
                        BackClaw.INSTANCE.spinT(),
                        SlideKits.INSTANCE.auto2()
                ),

                BackClaw.INSTANCE.clip(),

                new Delay(0.3),

                Intake.INSTANCE.drop(),

                new Delay(0.2),

                new ParallelGroup(
                        SlideKits.INSTANCE.low()
                )

        );
    }

    @Override
    public void onUpdate() {

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

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        autoPathUpdate().invoke();
    }

}

