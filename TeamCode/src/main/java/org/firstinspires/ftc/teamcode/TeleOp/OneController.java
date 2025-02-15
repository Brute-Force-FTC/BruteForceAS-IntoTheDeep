package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Utilities.BruteForceRobot;

@TeleOp
public class OneController extends LinearOpMode{

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

    public OneController() throws Exception {
        RobotLog.d("Starting TeleOp");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);

        //robot.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //robot.rightBack.setDirection(DcMotorSimple.Direction.REVERSE);


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

        slideThread.start();
        intakeThread.start();

        robot.resetAngle(globalAngle);
        correction = robot.checkDirectionF(globalAngle, telemetry);

        try {
            while (opModeIsActive()) {
                //Main Thread

                double y = -gamepad1.left_stick_y; // Remember, this is reversed!
                double x = gamepad1.left_stick_x; // this is strafing
                double rx = gamepad1.right_stick_x;

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when
                // at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double leftFrontPower = (y + x - rx) / denominator;
                double leftRearPower = (y - x - rx) / denominator;
                double rightFrontPower = -(y - x + rx) / denominator;
                double rightRearPower = -(y + x + rx) / denominator;

                if (!gamepad1.y) {
                    robot.leftFront.setPower(leftFrontPower);
                    robot.leftRear.setPower(leftRearPower);
                    robot.rightFront.setPower(rightFrontPower);
                    robot.rightRear.setPower(rightRearPower);
                } else {
                    robot.leftFront.setPower(leftFrontPower*0.33);
                    robot.leftRear.setPower(leftRearPower*0.33);
                    robot.rightFront.setPower(rightFrontPower*0.33);
                    robot.rightRear.setPower(rightRearPower*0.33);
                }

                telemetry.addData("Horizontal Slide Kit Left Power", robot.horizontalSlideKitRight.getPosition());

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

                    if ((gamepad1.right_stick_y != 0) ) {
                        robot.verticalSlideKit.setPower(rsy1);
                    }
                    else {
                        robot.verticalSlideKit.setPower(-0.004);
                    }

                    if ((gamepad1.right_bumper)) {
                        robot.horizontalSlideKitRight.setPosition(0.35); //out
                    } else if(gamepad1.left_bumper){
                        robot.horizontalSlideKitRight.setPosition(-0.2); //in
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
                        robot.clawBackSpin.setPosition(1);
                        robot.clawBack.setPosition(0);
                        robot.claw.setPosition(0);
                        robot.horizontalSlideKitRight.setPosition(-0.2);
                        sleep(300);
                        robot.clawBackRotateRight.setPosition(-0.8);
                        robot.clawArm.setPosition(0.2);
                        robot.clawRotate.setPosition(1);
                        sleep(1000);
                        robot.clawArm.setPosition(-0.35);
                        sleep(500);
                        robot.clawBack.setPosition(1);
                        robot.claw.setPosition(-1);
                        sleep(500);
                        robot.clawArm.setPosition(0);
                        robot.clawRotate.setPosition(0);
                        robot.clawBackRotateRight.setPosition(0.5);
                    }

                    if (gamepad1.a) {
                        robot.clawRotate.setPosition(-0.95);
                        robot.clawArm.setPosition(0.575);
                        robot.claw.setPosition(-1);
                    }

                    if (gamepad1.x) {
                        robot.clawRotate.setPosition(-1);
                        robot.clawArm.setPosition(1);
                        sleep(300);
                        robot.claw.setPosition(0);
                        sleep(300);
                        robot.clawRotate.setPosition(-0.9);
                        robot.clawArm.setPosition(0.15);
                    }

                    if (gamepad1.right_stick_button) {
                        robot.clawBack.setPosition(0); //drop off
                        sleep(2000);
                        robot.clawBackSpin.setPosition(-0.42); //claw spin bottom
                        robot.clawBackRotateRight.setPosition(0.85);
                    }

                    if (gamepad1.left_stick_button) {
                        robot.clawBackRotateRight.setPosition(0.75); //pick up
                        sleep(500);
                        robot.clawBack.setPosition(1);
                        robot.clawBackRotateRight.setPosition(0.5);
                        sleep(2000);
                        robot.clawBackSpin.setPosition(1); //claw spin top
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

                    if (gamepad1.dpad_right) {
                    robot.claw.setPosition(0); //claw close
                    } else if (gamepad1.dpad_left) {
                    robot.claw.setPosition(-1); //claw open
                    }

                    if (gamepad1.dpad_up) {
                        robot.clawBack.setPosition(1); //claw back close
                    } else if (gamepad1.dpad_down) {
                        robot.clawBack.setPosition(0); //claw back open
                    }

                    idle();
                }
            }
            catch (Exception e) {e.printStackTrace();}
        }
    }
}
