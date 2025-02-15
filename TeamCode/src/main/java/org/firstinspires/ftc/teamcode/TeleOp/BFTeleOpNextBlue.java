package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.CommandManager;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx;
import com.rowanmcalpin.nextftc.ftc.gamepad.Joystick;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.pedro.DriverControlled;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import org.firstinspires.ftc.teamcode.Utilities.BackClaw;
import org.firstinspires.ftc.teamcode.Utilities.BruteForceRobot;
import org.firstinspires.ftc.teamcode.Utilities.Claw;
import org.firstinspires.ftc.teamcode.Utilities.Extension;
import org.firstinspires.ftc.teamcode.Utilities.Intake;
import org.firstinspires.ftc.teamcode.Utilities.Lights;
import org.firstinspires.ftc.teamcode.Utilities.Limelight;
import org.firstinspires.ftc.teamcode.Utilities.SlideKits;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp
public class BFTeleOpNextBlue extends PedroOpMode {

    public BFTeleOpNextBlue() {
        super(Claw.INSTANCE, BackClaw.INSTANCE, SlideKits.INSTANCE, Intake.INSTANCE, Extension.INSTANCE, Lights.INSTANCE, Limelight.INSTANCE);
    }
    private final Pose startPose = new Pose(36, 69.8, Math.toRadians(0));

    /** Position to score all specs on the high chamber, and the control position */
    private final Pose scoreSpec = new Pose(36, 69.8, Math.toRadians(0));
    private final Pose scoreSpecControl = new Pose(4, 75, Math.toRadians(0));

    /** Position to get ready to pickup all specs from the human player station */
    private final Pose positionSpec = new Pose(13.45, 42, Math.toRadians(180));

    /** Position to pickup all specs from the human player station */
    private final Pose pickupSpec = new Pose(9, 42, Math.toRadians(180));

    private PathChain prepSpec3, spec3, scoreSpecimen3;

    public void buildPaths() {

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
    }


    private Limelight3A limelight;


    @Override
    public void onInit() {
        Lights.INSTANCE.green();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        SlideKits.INSTANCE.motor.setCurrentPosition(0);

        gamepadManager.getGamepad1().getLeftBumper().setPressedCommand( () -> { return new InstantCommand( () -> {
            gamepadManager.getGamepad1().getLeftStick().setProfileCurve(input -> 0.3f*input);
            gamepadManager.getGamepad1().getRightStick().setProfileCurve(input -> 0.3f*input); return null;
        });});
        gamepadManager.getGamepad1().getLeftBumper().setReleasedCommand( () -> { return new InstantCommand( () -> {
            gamepadManager.getGamepad1().getLeftStick().setProfileCurve(input -> 1.0f*input);
            gamepadManager.getGamepad1().getRightStick().setProfileCurve(input -> 1.0f*input); return null;
        });});
        gamepadManager.getGamepad1().getRightBumper().setPressedCommand( () -> { return new InstantCommand( () -> {
            gamepadManager.getGamepad1().getLeftStick().setProfileCurve(input -> 0.3f*input);
            gamepadManager.getGamepad1().getRightStick().setProfileCurve(input -> 0.3f*input); return null;
        });});
        gamepadManager.getGamepad1().getRightBumper().setReleasedCommand( () -> { return new InstantCommand( () -> {
            gamepadManager.getGamepad1().getLeftStick().setProfileCurve(input -> 1.0f*input);
            gamepadManager.getGamepad1().getRightStick().setProfileCurve(input -> 1.0f*input); return null;
        });});

        CommandManager.INSTANCE.scheduleCommand(new DriverControlled(gamepadManager.getGamepad1(), false, false, true, true));

        // Horizontal Slide Kit Button Control
        gamepadManager.getGamepad2().getLeftBumper().setPressedCommand(Extension.INSTANCE::out);
        gamepadManager.getGamepad2().getRightBumper().setPressedCommand(Extension.INSTANCE::in);

        // Transfer from Claw to BackClaw
        //gamepadManager.getGamepad1().getB().setPressedCommand(Intake.INSTANCE::transfer);

        // Claw Commands
        gamepadManager.getGamepad1().getDpadUp().setPressedCommand(Claw.INSTANCE::zero);
        gamepadManager.getGamepad1().getA().setPressedCommand(Claw.INSTANCE::prepare);
        gamepadManager.getGamepad1().getY().setPressedCommand(Claw.INSTANCE::prepareR);
        gamepadManager.getGamepad1().getX().setPressedCommand(Claw.INSTANCE::pickUp);
        gamepadManager.getGamepad2().getDpadLeft().setPressedCommand(Claw.INSTANCE::open);
        gamepadManager.getGamepad2().getDpadRight().setPressedCommand(Claw.INSTANCE::close);

        gamepadManager.getGamepad1().getLeftTrigger().setPressedCommand(value -> Extension.INSTANCE.in());
        gamepadManager.getGamepad1().getRightTrigger().setPressedCommand(value -> Extension.INSTANCE.out());

        // BackClaw Commands
        //gamepadManager.getGamepad2().getA().setPressedCommand(BackClaw.INSTANCE::pickUp);
        gamepadManager.getGamepad2().getA().setPressedCommand( () ->
                new SequentialGroup(
                        BackClaw.INSTANCE.setUp(),
                        SlideKits.INSTANCE.middle(),
                        BackClaw.INSTANCE.spinT()
                ));
        //gamepadManager.getGamepad2().getX().setPressedCommand(BackClaw.INSTANCE::prepare);
        gamepadManager.getGamepad2().getX().setPressedCommand( () ->
                new SequentialGroup(
                        BackClaw.INSTANCE.clip(),
                        new Delay(0.5),
                        BackClaw.INSTANCE.prepare(),
                        new Delay(0.5),
                        SlideKits.INSTANCE.low()
                ));

        // BackClaw Open
        gamepadManager.getGamepad2().getDpadDown().setPressedCommand(Intake.INSTANCE::drop);

        // Slide Kits
        gamepadManager.getGamepad2().getY().setPressedCommand(SlideKits.INSTANCE::middle);
        gamepadManager.getGamepad2().getB().setPressedCommand(SlideKits.INSTANCE::low);
        gamepadManager.getGamepad2().getRightStick().getButton().setPressedCommand(SlideKits.INSTANCE::clip);

        // Reset 💀⚡
        gamepadManager.getGamepad2().getLeftStick().getButton().setPressedCommand(Claw.INSTANCE::reset);

        // Lights
        gamepadManager.getGamepad1().getDpadRight().setPressedCommand(Lights.INSTANCE::policeCar);

        gamepadManager.getGamepad2().getLeftStick().setHeldCommand((pos) -> SlideKits.INSTANCE.move(pos.getSecond()));

        // AutoPath Run
        //gamepadManager.getGamepad2().getDpadUp().setPressedCommand( () -> { return autoPathUpdate(); } );

    }

    @Override
    public void onUpdate() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Lights.INSTANCE.green().invoke();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
            }
        }

        if (gamepad1.a){
            limelight.pipelineSwitch(0);
            telemetry.addLine("Switched pipeline to 0, straight blue");
        } else if (gamepad1.y) {
            limelight.pipelineSwitch(1);
            telemetry.addLine("Switched pipeline to 1, right blue");
        }

        /*if ((gamepad2.left_stick_y != 0) ) {
            SlideKits.INSTANCE.getDefaultCommand().stop(false);
            slideKit.setPower(-gamepad2.left_stick_y);
        }*/

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("slideKitPosition", SlideKits.INSTANCE.motor.getCurrentPosition());
        telemetry.addData("slideKitTargetPosition", SlideKits.INSTANCE.controller.getTarget());
        telemetry.addData( "gamepadManager.getGamepad2().getLeftStick().getYAxis().getValue()", gamepadManager.getGamepad2().getLeftStick().getYAxis().getValue());
        telemetry.update();
    }
}
