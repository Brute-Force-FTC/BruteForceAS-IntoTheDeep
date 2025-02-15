package org.firstinspires.ftc.teamcode.Utilities;

import static com.rowanmcalpin.nextftc.ftc.OpModeData.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

import java.util.Set;

public class Limelight extends Subsystem {

    public static final Limelight INSTANCE = new Limelight();
    private Limelight() { }

    public Limelight3A limelight;

    public String limelightName = "limelight";

    public Command startLimelight = new LambdaCommand(
            () -> false,
            () -> {
                limelight.setPollRateHz(100);
                limelight.start();
                return null;
            }
    );

    public Command switchPipelineCommand = new LambdaCommand(
            () -> false, // Command never "finishes" on its own
            () -> {
                limelight.pipelineSwitch(0);
                return null;
            }
    );

    public Command trackTargetCommand = new LambdaCommand(
            () -> false, // Keep running until explicitly stopped
            () -> {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.update();
                };
                return null;
            }
    );

    public Command stopLimelightCommand = new LambdaCommand(
            () -> true, // Finishes immediately
            () -> {
                limelight.stop();
                return null;
            }, // Stops polling
            () -> {return null;}, // No updates needed
            interrupted -> {
                telemetry.addData("Limelight", "Stopped");
                telemetry.update();
                return null;
            },
            Set.of(this),
            true
    );




    @Override
    public void initialize() {
        limelight = OpModeData.INSTANCE.getHardwareMap().get(Limelight3A.class, limelightName);
    }
}
