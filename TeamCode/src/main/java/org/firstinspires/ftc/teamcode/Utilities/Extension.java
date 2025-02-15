package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class Extension extends Subsystem {

    public static final Extension INSTANCE = new Extension();
    private Extension() { }

    public Servo horizontalSlideKitRight;
    public Servo horizontalSlideKitLeft;

    public String horizontalSlideKitRightName = "horizontalSlideKitRight";
    public String horizontalSlideKitLeftName = "horizontalSlideKitLeft";

    double HSKRin = 0.55;
    double HSKLin = 0.55;
    double HSKRout = 0.3;
    double HSKLout = 0.3;


    public Command out() {
        return new ParallelGroup(
                new ServoToPosition(horizontalSlideKitRight,
                        HSKRout,
                        this),
                new ServoToPosition(horizontalSlideKitLeft,
                        HSKLout,
                        this)
        );
    }

    public Command in() {
        return new ParallelGroup(
                new ServoToPosition(horizontalSlideKitRight,
                        HSKRin,
                        this),
                new ServoToPosition(horizontalSlideKitLeft,
                        HSKLin,
                        this)
        );
    }

    @Override
    public void initialize() {
        horizontalSlideKitRight = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, horizontalSlideKitRightName);
        horizontalSlideKitLeft = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, horizontalSlideKitLeftName);
    }
}
