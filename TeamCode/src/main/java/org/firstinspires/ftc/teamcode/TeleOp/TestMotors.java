package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Utilities.BruteForceRobot;

@TeleOp
public class TestMotors extends LinearOpMode{

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


    @Override
    public void runOpMode() {
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


        waitForStart();

        robot.resetAngle(globalAngle);
        correction = robot.checkDirectionF(globalAngle, telemetry);

        robot.limelight.pipelineSwitch(0);
        robot.limelight.start();

            while (opModeIsActive()) {
                //Main Thread

                LLResult result = robot.limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid()) {
                        telemetry.addData("tx", result.getTx());
                        telemetry.addData("ty", result.getTy());
                    }
                }

                if (gamepad1.a) {
                    //robot.leftFront.setPower(1); //frontright
                } else if (gamepad1.b) {
                    //robot.leftRear.setPower(1); //backleft
                } else if (gamepad1.x) {
                    //robot.rightFront.setPower(1); //verticalslidekit
                } else if (gamepad1.y) {
                    //robot.rightRear.setPower(1); //backright
                } else if (gamepad1.right_bumper) {
                    //robot.verticalSlideKit.setPower(1); //frontleft
                } else if (gamepad1.left_bumper) {
                    //robot.horizontalSlideKitRight.setPosition(1); //clawarm
                } else if (gamepad1.dpad_up) {
                    //robot.claw.setPosition(1); //clawbackrotate
                } else if (gamepad1.dpad_down) {
                    //robot.clawRotate.setPosition(1); //clawback
                } else if (gamepad1.dpad_right) {
                    robot.clawArm.setPosition(1); //claw
                } else if (gamepad1.dpad_left) {
                    //robot.clawBack.setPosition(1); //clawbackspin
                } else if (gamepad1.right_stick_button) {
                    //robot.clawBackRotateRight.setPosition(1); //horizontalslidekit
                } else if (gamepad1.left_stick_button) {
                    //robot.clawBackSpin.setPosition(1); //clawrotate
                } else {
                    //robot.leftFront.setPower(0);
                    //robot.leftRear.setPower(0);
                    //robot.rightFront.setPower(0);
                    //robot.rightRear.setPower(0);
                    //robot.verticalSlideKit.setPower(0);
                    //robot.horizontalSlideKitRight.setPosition(0);
                    //robot.claw.setPosition(0);
                    //robot.clawRotate.setPosition(0);
                    robot.clawArm.setPosition(0);
                    //robot.clawBack.setPosition(0);
                    //robot.clawBackRotateRight.setPosition(0);
                    //robot.clawBackSpin.setPosition(0);
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


            }
    }
}
