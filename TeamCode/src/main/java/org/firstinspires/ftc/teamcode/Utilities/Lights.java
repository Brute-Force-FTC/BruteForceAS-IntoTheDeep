package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.core.control.coefficients.PIDCoefficients;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

public class Lights extends Subsystem {

    public static final Lights INSTANCE = new Lights();
    private Lights() { }

    public Servo light1;
    public Servo light2;

    public String light1Name = "light1";
    public String light2Name = "light2";

    double black = 0;
    double red = 0.28;
    double orange = 0.33;
    double yellow = 0.38;
    double sage = 0.44;
    double green = 0.5;
    double azure = 0.55;
    double blue = 0.61;
    double indigo = 0.66;
    double violet = 0.72;
    double white = 1;

    @Override
    public Command getDefaultCommand() {
        return new ParallelGroup(
                new ServoToPosition(light1,
                        red,
                        this),
                new ServoToPosition(light2,
                        red,
                        this)
        );
    }

    public Command policeCar() {
        return new SequentialGroup(
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.3)
        );
    }


    public Command change() {
        return new SequentialGroup(
                new ParallelGroup(
                        new ServoToPosition(light1,
                                black,
                                this),
                        new ServoToPosition(light2,
                                black,
                                this)
                ),
                new Delay(0.25),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                red,
                                this),
                        new ServoToPosition(light2,
                                red,
                                this)
                ),
                new Delay(0.25),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                orange,
                                this),
                        new ServoToPosition(light2,
                                orange,
                                this)
                ),
                new Delay(0.25),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                yellow,
                                this),
                        new ServoToPosition(light2,
                                yellow,
                                this)
                ),
                new Delay(0.25),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                sage,
                                this),
                        new ServoToPosition(light2,
                                sage,
                                this)
                ),
                new Delay(0.25),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                green,
                                this),
                        new ServoToPosition(light2,
                                green,
                                this)
                ),
                new Delay(0.25),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                azure,
                                this),
                        new ServoToPosition(light2,
                                azure,
                                this)
                ),
                new Delay(0.25),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                blue,
                                this),
                        new ServoToPosition(light2,
                                blue,
                                this)
                ),
                new Delay(0.25),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                indigo,
                                this),
                        new ServoToPosition(light2,
                                indigo,
                                this)
                ),
                new Delay(0.25),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                violet,
                                this),
                        new ServoToPosition(light2,
                                violet,
                                this)
                ),
                new Delay(0.25),
                new ParallelGroup(
                        new ServoToPosition(light1,
                                white,
                                this),
                        new ServoToPosition(light2,
                                white,
                                this)
                )
        );
    }

    public Command black() {
        return new ParallelGroup(
                new ServoToPosition(light1,
                        black,
                        this),
                new ServoToPosition(light2,
                        black,
                        this)
        );
    }

    public Command red() {
        return new ParallelGroup(
                new ServoToPosition(light1,
                        red,
                        this),
                new ServoToPosition(light2,
                        red,
                        this)
        );
    }

    public Command orange() {
        return new ParallelGroup(
                new ServoToPosition(light1,
                        orange,
                        this),
                new ServoToPosition(light2,
                        orange,
                        this)
        );
    }

    public Command yellow() {
        return new ParallelGroup(
                new ServoToPosition(light1,
                        yellow,
                        this),
                new ServoToPosition(light2,
                        yellow,
                        this)
        );
    }

    public Command sage() {
        return new ParallelGroup(
                new ServoToPosition(light1,
                        sage,
                        this),
                new ServoToPosition(light2,
                        sage,
                        this)
        );
    }

    public Command green() {
        return new ParallelGroup(
                new ServoToPosition(light1,
                        green,
                        this),
                new ServoToPosition(light2,
                        green,
                        this)
        );
    }

    public Command azure() {
        return new ParallelGroup(
                new ServoToPosition(light1,
                        azure,
                        this),
                new ServoToPosition(light2,
                        azure,
                        this)
        );
    }

    public Command blue() {
        return new ParallelGroup(
                new ServoToPosition(light1,
                        blue,
                        this),
                new ServoToPosition(light2,
                        blue,
                        this)
        );
    }

    public Command indigo() {
        return new ParallelGroup(
                new ServoToPosition(light1,
                        indigo,
                        this),
                new ServoToPosition(light2,
                        indigo,
                        this)
        );
    }

    public Command violet() {
        return new ParallelGroup(
                new ServoToPosition(light1,
                        violet,
                        this),
                new ServoToPosition(light2,
                        violet,
                        this)
        );
    }

    public Command white() {
        return new ParallelGroup(
                new ServoToPosition(light1,
                        white,
                        this),
                new ServoToPosition(light2,
                        white,
                        this)
        );
    }


    @Override
    public void initialize() {
        light1 = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, light1Name);
        light2 = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, light2Name);
    }
}
