package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Utilities.BruteForceRobot;
import org.firstinspires.ftc.teamcode.Utilities.SlideKits;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import com.pedropathing.util.Constants;


@TeleOp
public class BFTeleOp extends LinearOpMode{

    private Follower follower;
    private final Pose startPose = new Pose(36, 69.8, Math.toRadians(0));


    double rsy1 = 0;
    double rsx1 = 0;
    double lsx1 = 0;
    double lsy1 = 0;
    double rsy2 = 0;
    double rsx2 = 0;
    double lsx2 = 0;
    double lsy2 = 0;

    double TRIGGER_THRESHOLD = 0.5;
    double globalAngle, power = .30, correction;
    double HSKout = 0.675;
    double HSKin = 0.4;
    double Copen = 0;
    double Cclose = 0.5;
    double CBopen = 0.5;
    double CBclose = 1;
    double CStop = 1;
    double CSbot = 0.29;

    public BFTeleOp() throws Exception {
        RobotLog.d("Starting TeleOp");
    }




    @Override
    public void runOpMode() throws InterruptedException {
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);

        //robot.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //robot.rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.verticalSlideKit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.verticalSlideKit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        lsy1 = gamepad1.left_stick_y;
        lsx1 = gamepad1.left_stick_x;
        rsy1 = gamepad1.right_stick_y;
        rsx1 = gamepad1.right_stick_x;
        lsy2 = gamepad2.left_stick_y;
        lsx2 = gamepad2.left_stick_x;
        rsy2 = gamepad2.right_stick_y;
        rsx2 = gamepad2.right_stick_x;

        Thread slideThread = new slideThread();
        Thread intakeThread = new intakeThread();


        waitForStart();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);


        slideThread.start();
        intakeThread.start();

        follower.startTeleopDrive();


        try {
            while (opModeIsActive()) {
                //Main Thread

                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
                follower.update();

                telemetry.addData("X", follower.getPose().getX());
                telemetry.addData("Y", follower.getPose().getY());
                telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
                telemetry.addData("slideKitPosition", SlideKits.INSTANCE.motor.getCurrentPosition());
                telemetry.addData("slideKitTargetPosition", SlideKits.INSTANCE.controller.getTarget());

                telemetry.addData("Horizontal Slide Kit Left Power", robot.horizontalSlideKitRight.getPosition());

                telemetry.addData("FrontLeft Power", robot.leftFront.getPower());
                telemetry.addData("FrontRight Power", robot.rightFront.getPower());
                telemetry.addData("BackLeft Power", robot.leftRear.getPower());
                telemetry.addData("BackRight Power", robot.rightRear.getPower());

                telemetry.addData("Claw Power", robot.claw.getPosition());
                telemetry.addData("Claw Back Power", robot.clawBack.getPosition());
                telemetry.addData("Claw Rotate Power", robot.clawRotate.getPosition());
                telemetry.addData("Claw Arm Power", robot.clawArm.getPosition());
                telemetry.addData("Claw Back Rotate Power", robot.clawBackRotateRight.getPosition());
                telemetry.addData("Claw Back Spin Power", robot.clawBackSpin.getPosition());

                telemetry.addData("Vertical Slide Kit Power", robot.verticalSlideKit.getPower());
                telemetry.addData("Vertical Slide Kit Position", robot.verticalSlideKit.getCurrentPosition());

                telemetry.addData("Status", "Running");
                telemetry.update();



                idle();
            }
        }
        catch(Exception e) {RobotLog.d(e.getMessage());}

        slideThread.interrupt();
        intakeThread.interrupt();
    }

    private class slideThread extends Thread {
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);

        public slideThread() {
            this.setName("slideThread");
            RobotLog.d("%s", this.getName());
        }

        @Override
        public void run() {
            try {
                while (!isInterrupted()) {
                    //Slide Kit Thread

                    lsy1 = gamepad1.left_stick_y;
                    lsx1 = gamepad1.left_stick_x;
                    rsy1 = gamepad1.right_stick_y;
                    rsx1 = gamepad1.right_stick_x;
                    lsy2 = gamepad2.left_stick_y;
                    lsx2 = gamepad2.left_stick_x;
                    rsy2 = gamepad2.right_stick_y;
                    rsx2 = gamepad2.right_stick_x;

                    int posv_h =0;
                    posv_h = robot.verticalSlideKit.getCurrentPosition();
                    // vertical max is ___

                    if ((gamepad2.left_stick_y != 0) ) {
                        robot.verticalSlideKit.setPower(lsy2);
                    }
                    else {
                        robot.verticalSlideKit.setPower(-0.001); //0.0042
                    }

                    if (gamepad2.left_stick_button) {
                        int newSlideKitTarget = -1300;
                        robot.verticalSlideKit.setTargetPosition(newSlideKitTarget);
                        robot.verticalSlideKit.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.verticalSlideKit.setPower(-1);
                        while (
                                robot.verticalSlideKit.isBusy()
                        )
                        {
                            telemetry.addData("its busy lil bro", 69);

                            telemetry.update();
                        }
                        robot.verticalSlideKit.setPower(0);
                        robot.verticalSlideKit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }

                    if (gamepad2.right_stick_button) {
                        int newSlideKitTarget = -2200;
                        robot.verticalSlideKit.setTargetPosition(newSlideKitTarget);
                        robot.verticalSlideKit.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.verticalSlideKit.setPower(-1);
                        while (
                                robot.verticalSlideKit.isBusy()
                        )
                        {
                        }
                        robot.verticalSlideKit.setPower(0);
                        robot.clawBack.setPosition(CBopen); //drop off
                        sleep(1000);
                        robot.clawBackSpin.setPosition(CSbot); //claw spin bottom
                        robot.clawBackRotateRight.setPosition(0.925);
                        robot.verticalSlideKit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }


                    if ((gamepad2.left_bumper)) {
                        robot.horizontalSlideKitRight.setPosition(HSKout);
                    } else if(gamepad2.right_bumper){
                        robot.horizontalSlideKitRight.setPosition(HSKin);
                    }

                    idle();
                }
            }
            catch (Exception e) {e.printStackTrace();}
        }
    }

    private class intakeThread extends Thread {
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);

        public intakeThread() {
            this.setName("intakeThread");
            RobotLog.d("%s", this.getName());
        }

        @Override
        public void run() {
            try {
                while (!isInterrupted()) {
                    //Intake Thread

                    lsy1 = gamepad1.left_stick_y;
                    lsx1 = gamepad1.left_stick_x;
                    rsy1 = gamepad1.right_stick_y;
                    rsx1 = gamepad1.right_stick_x;
                    lsy2 = gamepad2.left_stick_y;
                    lsx2 = gamepad2.left_stick_x;
                    rsy2 = gamepad2.right_stick_y;
                    rsx2 = gamepad2.right_stick_x;

                    if (gamepad1.b) {
                        robot.clawBackSpin.setPosition(CStop);
                        robot.clawBack.setPosition(CBopen);
                        robot.claw.setPosition(Cclose);
                        robot.horizontalSlideKitRight.setPosition(HSKin);
                        sleep(300);
                        robot.clawBackRotateRight.setPosition(0.1);
                        robot.clawArm.setPosition(0.6);
                        robot.clawRotate.setPosition(1);
                        sleep(1000);
                        robot.clawArm.setPosition(0.325);
                        sleep(500);
                        robot.clawBack.setPosition(CBclose);
                        robot.claw.setPosition(Copen);
                        sleep(500);
                        robot.clawArm.setPosition(0.5);
                        robot.clawRotate.setPosition(0.5);
                        robot.clawBackRotateRight.setPosition(0.75);
                    }

                    if(gamepad1.dpad_up){
                        robot.clawArm.setPosition(0.5);
                        sleep(100);
                        robot.clawRotate.setPosition(0.5);
                    }

                    if (gamepad1.a) {
                        robot.clawRotate.setPosition(0);
                        robot.clawArm.setPosition(0.7875);
                        robot.claw.setPosition(Copen);
                    }

                    if (gamepad1.x) {
                        robot.clawRotate.setPosition(0);
                        robot.clawArm.setPosition(1);
                        sleep(300);
                        robot.claw.setPosition(Cclose);
                        sleep(300);
                        robot.clawRotate.setPosition(0.05);
                        robot.clawArm.setPosition(0.575);
                    }

                    if (gamepad2.x) {
                        robot.clawBack.setPosition(CBopen); //drop off
                        robot.clawBackSpin.setPosition(CSbot); //claw spin bottom
                        robot.clawBackRotateRight.setPosition(0.925);
                    }

                    if (gamepad2.a) {
                        robot.clawBackRotateRight.setPosition(0.875); //pick up
                        sleep(500);
                        robot.clawBack.setPosition(CBclose);
                        robot.clawBackRotateRight.setPosition(0.725);
                        sleep(1000);
                        robot.clawBackSpin.setPosition(CStop); //claw spin top
                    }

                    /* Custom Controls for Recalibration
                    robot.clawRotate.setPower(lsx2);
                    robot.clawArm.setPower(rsx2);
                    robot.clawBackRotate.setPower(rsy2);

                    if (gamepad2.left_bumper) {
                        robot.clawBackRotate.setPower(-0.8);
                    }

                    if (gamepad2.right_bumper) {
                        robot.clawArm.setPower(-0.35);
                        robot.clawRotate.setPower(1);
                    }
                    */

                    if (gamepad2.dpad_right) {
                    robot.claw.setPosition(Cclose);
                    } else if (gamepad2.dpad_left) {
                    robot.claw.setPosition(Copen);
                    }

                    if (gamepad2.dpad_up) {
                        robot.clawBack.setPosition(CBclose);
                    } else if (gamepad2.dpad_down) {
                        robot.clawBack.setPosition(CBopen);
                    }

                    idle();
                }
            }
            catch (Exception e) {e.printStackTrace();}
        }
    }
}
