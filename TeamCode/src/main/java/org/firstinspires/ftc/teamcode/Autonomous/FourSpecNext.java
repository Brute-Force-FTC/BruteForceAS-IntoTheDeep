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
import org.firstinspires.ftc.teamcode.Utilities.SlideKits;


import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "FourSpecNext😂😘💀☠️👲", group = "Autonomous")
public class FourSpecNext extends PedroOpMode {
    public FourSpecNext() {
        super(Claw.INSTANCE, SlideKits.INSTANCE, BackClaw.INSTANCE, Extension.INSTANCE);
    }

    /** Start Pose of the robot */
    private final Pose startPose = new Pose(9.5, 60, Math.toRadians(0));
    // x from 9 to 10

    /** Position to score the preload on the high chamber */
    private final Pose scorePreload = new Pose(36, 60, Math.toRadians(0));
    // x from 36 to 37

    /** Position to get ready to move the first block, and the control position */
    private final Pose moveBlock1 = new Pose(50, 44, Math.toRadians(180));
    private final Pose moveBlock1Control1 = new Pose(20, 36, Math.toRadians(180));

    /** Position to move the block into the human player station, adn the control position */
    private final Pose parkBlock1 = new Pose(18, 27, Math.toRadians(180));
    private final Pose parkBlock1Control = new Pose(65, 30.5, Math.toRadians(180));

    /** Position to get ready to move the second block, and the control position */
    private final Pose moveBlock2 = new Pose(50, 35, Math.toRadians(180));
    private final Pose moveBlock2Control = new Pose(57, 6.5, Math.toRadians(180));

    /** Position to pickup the first spec from the human player station */
    private final Pose pickupSpec1 = new Pose(7.5, 27, Math.toRadians(180));

    /** Position to score all specs on the high chamber, and the control position */
    private final Pose scoreSpec = new Pose(36, 69.8, Math.toRadians(0));
    private final Pose scoreSpec2 = new Pose(36, 73, Math.toRadians(0));
    private final Pose scoreSpecControl = new Pose(4, 75, Math.toRadians(0));

    /** Position to get ready to pickup all specs from the human player station */
    private final Pose positionSpec = new Pose(15, 42, Math.toRadians(180));

    /** Position to pickup all specs from the human player station */
    private final Pose pickupSpec = new Pose(9, 42, Math.toRadians(180));


    /* PathChains we are going to put our poses into */
    private PathChain scoreSpecimen1, block1, pushBlock1, block2, pushBlock2,
            scoreSpecimen2, prepSpec3, spec3, scoreSpecimen3, scoreSpecimen4;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
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
                .addPath(new BezierCurve(new Point(pickupSpec1), new Point (scoreSpecControl), new Point(scoreSpec)))
                .setLinearHeadingInterpolation(pickupSpec1.getHeading(), scoreSpec.getHeading())
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
                .addPath(new BezierCurve(new Point(pickupSpec), new Point(scoreSpecControl), new Point(scoreSpec)))
                .setLinearHeadingInterpolation(pickupSpec.getHeading(), scoreSpec.getHeading())
                .setPathEndTimeoutConstraint(500)
                .build();

        scoreSpecimen4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupSpec), new Point(scoreSpecControl), new Point(scoreSpec2)))
                .setLinearHeadingInterpolation(pickupSpec.getHeading(), scoreSpec.getHeading())
                .setPathEndTimeoutConstraint(500)
                .build();

    }

    /*public Command autoPathUpdate() {
        return new SequentialGroup(
                // Sets up the horizontal slide kits and back Claw before moving forward
                SlideKits.INSTANCE.getResetEncoders(),
                BackClaw.INSTANCE.setUp(),
                Extension.INSTANCE.in(),
                new Delay(1.0),

                // Moves forward while moving the slide kits up
                new ParallelGroup(
                        new FollowPath(scoreSpecimen1),
                        SlideKits.INSTANCE.toMiddle()
                        ),
                new Delay(0.5),

                // Moves the slide kits up and lets go of the spec for it to clip on
                SlideKits.INSTANCE.toHigh(),
                BackClaw.INSTANCE.prepare(),

                // Moves to push block1
                new FollowPath(block1),

                // Pushes block1 into the human player station while moving the slide kits down
                new ParallelGroup(
                        new FollowPath(pushBlock1),
                        SlideKits.INSTANCE.toLow()
                ),

                // Moves to push block2
                new FollowPath(block2),

                // Pushes block2 into human player station and gets ready to pick up spec2
                new FollowPath(pushBlock2),
                new Delay(0.25),

                // Picks up spec2
                BackClaw.INSTANCE.pickUp(),
                new Delay(0.1),

                // Moves slide kits up, spins the claw, and goes to score spec2
                new ParallelGroup(
                        new FollowPath(scoreSpecimen2),
                        BackClaw.INSTANCE.spinT(),
                        SlideKits.INSTANCE.toMiddler()
                ),
                new Delay(0.2),

                // Moves the slide kits up and lets go of the spec for it to clip on
                SlideKits.INSTANCE.toHigher(),
                BackClaw.INSTANCE.dropOff(),
                new Delay(0.1),

                // Gets ready to pick up spec3 while moving the slide kits down and spinning the claw
                new ParallelGroup(
                        new FollowPath(prepSpec3),
                        SlideKits.INSTANCE.toLow(),
                        BackClaw.INSTANCE.spinB()
                ),

                // Latches onto spec3
                new FollowPath(spec3),
                new Delay(0.15),

                // Picks up spec3
                BackClaw.INSTANCE.pickUp(),
                new Delay(0.1),

                // Goes to score spec3, moves slide kits up, and spins the claw
                new ParallelGroup(
                        new FollowPath(scoreSpecimen3),
                        BackClaw.INSTANCE.spinT(),
                        SlideKits.INSTANCE.toMiddler()
                ),
                new Delay(0.2),

                // Moves slide kits up and opens the back claw for the spec to clip on
                SlideKits.INSTANCE.toHigher(),
                BackClaw.INSTANCE.dropOff(),
                new Delay(0.1),

                // Gets ready to pick up spec4 while moving the slide kits down and spinning the claw
                new ParallelGroup(
                        new FollowPath(prepSpec3),
                        SlideKits.INSTANCE.toLow(),
                        BackClaw.INSTANCE.spinB()
                ),

                // Latches onto spec4
                new FollowPath(spec3),
                new Delay(0.15),

                // Picks up spec4
                BackClaw.INSTANCE.pickUp(),
                new Delay(0.1),

                // Goes to score spec4, moves slide kits up, and spins the claw
                new ParallelGroup(
                        new FollowPath(scoreSpecimen3),
                        BackClaw.INSTANCE.spinT(),
                        SlideKits.INSTANCE.toMiddler()
                ),
                new Delay(0.2),

                // Moves slide kits up and opens the back claw for the spec to clip on
                SlideKits.INSTANCE.toHigher(),
                BackClaw.INSTANCE.zero(),
                new Delay(0.1),

                // Goes back to park while moving the slide kits down and spinning the claw
                new ParallelGroup(
                        SlideKits.INSTANCE.toLow(),
                        Claw.INSTANCE.close()
                )
        );
    }*/


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
        //autoPathUpdate().invoke();
    }

    @Override
    public void onStop() {
    }
}

