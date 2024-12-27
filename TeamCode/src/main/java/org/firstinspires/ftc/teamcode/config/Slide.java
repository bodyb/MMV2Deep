package org.firstinspires.ftc.teamcode.config;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Slide {

    public PIDController controller;
    Gamepad gamepad;
    public static double p=0.006,i=0,d=0.00004,f=0.00006;
    public DcMotorEx slide, leftPivot, rightPivot;
    public final int maxPos = 3250;
    public final int base = 10;
    public final int lowBasket = 10;
    public final int highBasket = 3100;
    public final int lowBar = 10;
    public final int highBar = 1450;
    public final int unclip = 1250;


    public int target = 10;
    public Slide(HardwareMap map) {
        slide = map.get(DcMotorEx.class, "slide");
        leftPivot = map.get(DcMotorEx.class, "leftPivot");
        rightPivot = map.get(DcMotorEx.class, "rightPivot");
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);
    }

    public class SlideToPosition implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            int pos = slide.getCurrentPosition();
            double pid = controller.calculate(pos, target);
            double ff = pos * f;
            double power = pid + ff;
            slide.setPower(power);
            return true;
        }
    }
    public Action slideToPosition() {
        return new SlideToPosition();
    }

    public class setHighBasket implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            target = highBasket;
            return false;
        }
    }

    public Action HighBasket() {
        return new setHighBasket();
    }

    public class setHighBar implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            target = highBar;
            return false;
        }
    }

    public Action HighBar() {
        return new setHighBar();
    }

    public class setBase implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            target = base;
            return false;
        }
    }

    public Action Base() {
        return new setBase();
    }

    public class setUnclipCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            target = unclip;
            return false;
        }
    }

    public Action UnClip() {
        return new setUnclipCl();
    }

    public class RestEncoderCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            return false;
        }
    }

    public Action ResetEncoder() {
        return new RestEncoderCl();
    }

    /*public class CentralLift implements Action {
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            slide.setPower(gamepad2.left_stick_y);
            return true;
        }
    }*/

    public class checkSlideClass implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (slide.getCurrentPosition() > 200) {
                target = base;
            }
            return false;
        }
    }

    public Action checkSlide() {
        return new checkSlideClass();
    }

    public void setCentralLift(double Pow) {
        slide.setPower(Pow);
    }

    public void setTargetPosition (int position) {
        target = position;
    }

    public void movePovit (double power) {
        leftPivot.setPower(power);
        rightPivot.setPower(-power);
    }

}