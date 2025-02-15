package org.firstinspires.ftc.teamcode.Autonomous;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.NanoTimer;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.Utilities.BruteForceRobot;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "FiveSpec", group = "Autonomous")
@Disabled
public class FiveSpec extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    ExecutorService executor = Executors.newSingleThreadExecutor();
    double HSKout = 0.35;
    double HSKin = -0.2;
    double Copen = -1;
    double Cclose = 0;
    double CBopen = 0;
    double CBclose = 1;
    double CStop = 1;
    double CSbot = -0.42;



    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState, slideState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 60, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePreload = new Pose(37.2, 60, Math.toRadians(0));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose moveBlock1 = new Pose(72, 34.32, Math.toRadians(180));
    private final Pose moveBlock1Control1 = new Pose(15.5, 25.34, Math.toRadians(180));
    private final Pose moveBlock1Control2 = new Pose(77.61, 40.15, Math.toRadians(180));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose parkBlock1 = new Pose(18, 27, Math.toRadians(180));
    private final Pose parkBlock1Control = new Pose(65, 30.5, Math.toRadians(180));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose moveBlock2 = new Pose(63, 32, Math.toRadians(180));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose parkBlock2 = new Pose(18, 18.62, Math.toRadians(180));
    private final Pose parkBlock2Control = new Pose(61.5, 16, Math.toRadians(180));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose moveBlock3 = new Pose(63.3, 24.7, Math.toRadians(180));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose parkBlock3 = new Pose(13.45, 15.48, Math.toRadians(180));
    private final Pose parkBlock3Control = new Pose(51, 10, Math.toRadians(180));

    private final Pose pickupSpec1 = new Pose(9.75, 19, Math.toRadians(180));
    private final Pose scoreSpec1 = new Pose(38, 69.8, Math.toRadians(0));
    private final Pose scoreSpec1Control = new Pose(4, 75, Math.toRadians(0));
    private final Pose positionSpec = new Pose(13.45, 42, Math.toRadians(180));
    private final Pose pickupSpec = new Pose(9.75, 42, Math.toRadians(180));
    private final Pose scoreSpec2 = new Pose(37.2, 70, Math.toRadians(0));
    private final Pose scoreSpec3 = new Pose(37.2, 75, Math.toRadians(0));
    private final Pose scoreSpec4 = new Pose(37.2, 80, Math.toRadians(0));
    private final Pose parkPose = new Pose(13, 37, Math.toRadians(0));

    //end of code i wrote

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain scoreSpecimen1, block1, pushBlock1, block2, pushBlock2, block3, pushBlock3, spec2,
            scoreSpecimen2, prepSpec3, spec3, scoreSpecimen3, prepSpec4, spec4, scoreSpecimen4, prepSpec5,
            spec5, scoreSpecimen5, park;

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
        //scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        //scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePreload)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        block1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePreload), new Point(moveBlock1Control1), new Point(moveBlock1Control2), new Point(moveBlock1)))
                .setLinearHeadingInterpolation(scorePreload.getHeading(), moveBlock1.getHeading())
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
                .addPath(new BezierCurve(new Point(moveBlock2), new Point(parkBlock2Control), new Point(parkBlock2)))
                .setConstantHeadingInterpolation(moveBlock2.getHeading())
                .build();

        block3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(parkBlock2), new Point(moveBlock3)))
                .setConstantHeadingInterpolation(parkBlock2.getHeading())
                .build();


        pushBlock3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(moveBlock3), new Point(parkBlock3Control), new Point(parkBlock3)))
                .setConstantHeadingInterpolation(moveBlock3.getHeading())
                .build();

        spec2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(parkBlock3), new Point(pickupSpec1)))
                .setConstantHeadingInterpolation(parkBlock3.getHeading())
                .build();

        scoreSpecimen2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupSpec1), new Point (scoreSpec1Control), new Point(scoreSpec1)))
                .setLinearHeadingInterpolation(pickupSpec1.getHeading(), scoreSpec1.getHeading())
                .setPathEndTimeoutConstraint(500)
                .build();

        prepSpec3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSpec1), new Point(positionSpec)))
                .setLinearHeadingInterpolation(scoreSpec1.getHeading(), positionSpec.getHeading())
                .build();

        spec3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(positionSpec), new Point(pickupSpec)))
                .setConstantHeadingInterpolation(positionSpec.getHeading())
                .build();

        scoreSpecimen3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupSpec), new Point(scoreSpec1Control), new Point(scoreSpec1)))
                .setLinearHeadingInterpolation(pickupSpec.getHeading(), scoreSpec1.getHeading())
                .setPathEndTimeoutConstraint(500)
                .build();

        prepSpec4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSpec2), new Point(positionSpec)))
                .setLinearHeadingInterpolation(scoreSpec2.getHeading(), positionSpec.getHeading())
                .build();

        spec4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(positionSpec), new Point(pickupSpec)))
                .setConstantHeadingInterpolation(positionSpec.getHeading())
                .build();

        scoreSpecimen4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupSpec), new Point(scoreSpec3)))
                .setLinearHeadingInterpolation(pickupSpec.getHeading(), scoreSpec3.getHeading())
                .build();

        prepSpec5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSpec3), new Point(positionSpec)))
                .setLinearHeadingInterpolation(scoreSpec3.getHeading(), positionSpec.getHeading())
                .build();

        spec5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(positionSpec), new Point(pickupSpec)))
                .setConstantHeadingInterpolation(positionSpec.getHeading())
                .build();

        scoreSpecimen5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupSpec), new Point(scoreSpec4)))
                .setLinearHeadingInterpolation(pickupSpec.getHeading(), scoreSpec4.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSpec4), new Point(parkPose)))
                .setConstantHeadingInterpolation(scoreSpec4.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        //park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        //park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void slideKitUp() throws InterruptedException {
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);
        Timer slideTimer = new Timer();
        robot.verticalSlideKit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.verticalSlideKit.setTargetPosition(-1400);
        robot.verticalSlideKit.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.verticalSlideKit.setPower(-1);
        while (
                robot.verticalSlideKit.isBusy()
        )
        {
            robot.clawBack.setPosition(CBclose);
            robot.clawBackRotateRight.setPosition(0.45);
            robot.clawBackSpin.setPosition(CStop);
        }
        robot.verticalSlideKit.setPower(-0.0042); //0.0042
        robot.verticalSlideKit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.horizontalSlideKitRight.setPosition(HSKin);
    }

    public void slideKitMid() throws InterruptedException {
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);
        robot.verticalSlideKit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.verticalSlideKit.setTargetPosition(-1300);
        robot.verticalSlideKit.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.verticalSlideKit.setPower(-1);
        while (
                robot.verticalSlideKit.isBusy()
        )
        {
            robot.clawBack.setPosition(CBclose);
            robot.clawBackRotateRight.setPosition(0.45);
            robot.clawBackSpin.setPosition(CStop);
        }
        robot.verticalSlideKit.setPower(-0.0042); //0.0042
        robot.verticalSlideKit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.horizontalSlideKitRight.setPosition(HSKin);
    }

    public void slideKitDown() throws InterruptedException {
        BruteForceRobot robot1 = new BruteForceRobot(hardwareMap);
        robot1.verticalSlideKit.setTargetPosition(-2300);
        robot1.verticalSlideKit.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot1.verticalSlideKit.setPower(-1);
        while (
                robot1.verticalSlideKit.isBusy()
        )
        {
        }
        robot1.verticalSlideKit.setPower(0);
        robot1.clawBack.setPosition(CBopen); //drop off
        sleep(1000);
        robot1.clawBackSpin.setPosition(CSbot); //claw spin bottom
        robot1.clawBackRotateRight.setPosition(0.85);
        robot1.verticalSlideKit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void slideKitZero() throws InterruptedException {
        BruteForceRobot robot1 = new BruteForceRobot(hardwareMap);
        robot1.verticalSlideKit.setTargetPosition(0);
        robot1.verticalSlideKit.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot1.verticalSlideKit.setPower(1);
        while (
                robot1.verticalSlideKit.isBusy()
        )
        {
        }
        robot1.verticalSlideKit.setPower(0);
        robot1.verticalSlideKit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void pickUP() throws InterruptedException {
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);
        robot.clawBackRotateRight.setPosition(0.75); //pick up
        sleep(500);
        robot.clawBack.setPosition(CBclose);
        robot.clawBackRotateRight.setPosition(0.45);
        sleep(1000);
        robot.clawBackSpin.setPosition(CStop); //claw spin top
    }

    public void slide() throws InterruptedException {
        Timer slideTimer = new Timer();
        switch(slideState) {
            case 1:
                slideTimer.resetTimer();
                slideState++;
                break;
            case 2:
                slideKitUp();
                slideState++;
                break;
        }
    }


    public void autonomousPathUpdate() throws InterruptedException {

        switch (pathState) {
            case 0:
                slideKitUp();
                follower.followPath(scoreSpecimen1);
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    slideKitDown();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(block1,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    slideKitZero();
                    follower.followPath(pushBlock1);
                    setPathState(3);
                }
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(block2);
                    setPathState(4);
                }
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(pushBlock2);
                    setPathState(5);
                }
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(block3);
                    setPathState(6);
                }
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(pushBlock3);
                    setPathState(7);
                }
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(spec2);
                    setPathState(8);
                }
            case 8:
                if(!follower.isBusy()) {
                    pickUP();
                    slideKitMid();
                    follower.followPath(scoreSpecimen2);
                    setPathState(9);
                }
            case 9:
                if(!follower.isBusy()) {
                    slideKitDown();
                    follower.followPath(prepSpec3);
                    setPathState(10);
                }
            case 10:
                if(!follower.isBusy()) {
                    slideKitZero();
                    follower.followPath(spec3);
                    setPathState(11);
                }
            case 11:
                if(!follower.isBusy()) {
                    pickUP();
                    slideKitMid();
                    follower.followPath(scoreSpecimen3);
                    setPathState(12);
                }
            case 12:
                if(!follower.isBusy()) {
                    slideKitDown();
                    follower.followPath(prepSpec3);
                    setPathState(13);
                }
            case 13:
                if(!follower.isBusy()) {
                    slideKitZero();
                    follower.followPath(spec3);
                    setPathState(14);
                }
            case 14:
                if(!follower.isBusy()) {
                    pickUP();
                    slideKitMid();
                    follower.followPath(scoreSpecimen3);
                    setPathState(15);
                }
            case 15:
                if(!follower.isBusy()) {
                    slideKitDown();
                    follower.followPath(prepSpec3);
                    setPathState(16);
                }
            case 16:
                if(!follower.isBusy()) {
                    slideKitZero();
                    follower.followPath(spec3);
                    setPathState(17);
                }
            case 17:
                if(!follower.isBusy()) {
                    pickUP();
                    slideKitMid();
                    follower.followPath(scoreSpecimen3);
                    setPathState(18);
                }
            case 18:
                if(!follower.isBusy()) {
                    slideKitDown();
                    //follower.followPath(park);
                    setPathState(19);
                }
            case 19:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {


        // These loop the movements of the robot
        follower.update();
        /*try {
            slide();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }*/
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("isBusy", follower.isBusy());
        telemetry.addData("pathState", pathState);
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

