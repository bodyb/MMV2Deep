package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Pivot {

    public PIDController controller;
    public static double p=-0.04,i=0,d=0,f=0;
    public static double f1R=-0.02, f2R=-0.01, f3R=-0.006, f4R=-0.0022, f5R=-0.0016, f6R=-0.0008, f7R=0, f8R=0, f9R=0.0005; //Retracted
    public static double f1H=-0.02, f2H=-0.01, f3H=-0.006, f4H=-0.0022, f5H=-0.0016, f6H=-0.0008, f7H=0, f8H=0, f9H=0.0005; //Half
    public static double f1F=-0.02, f2F=-0.01, f3F=-0.006, f4F=-0.0022, f5F=-0.0016, f6F=-0.0008, f7F=0, f8F=0, f9F=0.0005; //Full
    public DcMotorEx leftPivot, rightPivot;
    public static boolean motorReverse = false, motor2Reverse = true;
    public static int MAX_POS = 180;

    public final double ticksPerRotation = 288;

    public static int target = 0;
    public int targetPivot = 0;
    public static int pivotDrop, pivotHover;
    public static double pivotDegree, startDegree;
    public static boolean slideRetracted, slideHalf, slideFull;
    public String slideState = "Retracted";

    public Pivot(HardwareMap map) {
        leftPivot = map.get(DcMotorEx.class, "leftPivot");
        rightPivot = map.get(DcMotorEx.class, "rightPivot");

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        if (motorReverse) {rightPivot.setDirection(DcMotorSimple.Direction.REVERSE);}
        rightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (motor2Reverse) {leftPivot.setDirection(DcMotorSimple.Direction.REVERSE);}
        leftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public class pivotToPositionCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (target > MAX_POS)
                target = MAX_POS;
            if (target < 0) {
                target = 0;
            }
            slideState = Slide.checkLength();
            switch(slideState){
                case "Retracted":
                    slideRetracted = true;
                    slideHalf = false;
                    slideFull = false;
                case "Half":
                    slideRetracted = false;
                    slideHalf = true;
                    slideFull = false;
                case "Full":
                    slideRetracted = false;
                    slideHalf = false;
                    slideFull = true;
                case "None":
                    slideRetracted = false;
                    slideHalf = false;
                    slideFull = false;
                    f = 0;
            }
            controller.setPID(p, i, d);
            int pos = rightPivot.getCurrentPosition();
            if (pos < 0 && slideRetracted) {f = 0;}
            if (0 <= pos && pos <= 20 && slideRetracted) {f = f1R;}
            if (20 < pos && pos <= 40 && slideRetracted) {f = f2R;}
            if (40 < pos && pos <= 60 && slideRetracted) {f = f3R;}
            if (60 < pos && pos <= 80 && slideRetracted) {f = f4R;}
            if (80 < pos && pos <= 100 && slideRetracted) {f = f5R;}
            if (100 < pos && pos <= 120 && slideRetracted) {f = f6R;}
            if (120 < pos && pos <= 140 && slideRetracted) {f = f7R;}
            if (140 < pos && pos <= 160 && slideRetracted) {f = f8R;}
            if (160 < pos && pos <= 180 && slideRetracted) {f = f9R;}
            if (pos < 180 && slideRetracted) {f = 0;}
            if (pos < 0 && slideHalf) {f = 0;}
            if (0 <= pos && pos <= 20 && slideHalf) {f = f1H;}
            if (20 < pos && pos <= 40 && slideHalf) {f = f2H;}
            if (40 < pos && pos <= 60 && slideHalf) {f = f3H;}
            if (60 < pos && pos <= 80 && slideHalf) {f = f4H;}
            if (80 < pos && pos <= 100 && slideHalf) {f = f5H;}
            if (100 < pos && pos <= 120 && slideHalf) {f = f6H;}
            if (120 < pos && pos <= 140 && slideHalf) {f = f7H;}
            if (140 < pos && pos <= 160 && slideHalf) {f = f8H;}
            if (160 < pos && pos <= 180 && slideHalf) {f = f9H;}
            if (pos < 180 && slideHalf) {f = 0;}
            if (pos < 0 && slideFull) {f = 0;}
            if (0 <= pos && pos <= 20 && slideFull) {f = f1F;}
            if (20 < pos && pos <= 40 && slideFull) {f = f2F;}
            if (40 < pos && pos <= 60 && slideFull) {f = f3F;}
            if (60 < pos && pos <= 80 && slideFull) {f = f4F;}
            if (80 < pos && pos <= 100 && slideFull) {f = f5F;}
            if (100 < pos && pos <= 120 && slideFull) {f = f6F;}
            if (120 < pos && pos <= 140 && slideFull) {f = f7F;}
            if (140 < pos && pos <= 160 && slideFull) {f = f8F;}
            if (160 < pos && pos <= 180 && slideFull) {f = f9F;}
            if (pos < 180 && slideFull) {f = 0;}
            double pid = controller.calculate(pos, target);
            double ff = pos * f;
            double power = pid + ff;
            rightPivot.setPower(power);
            leftPivot.setPower(power);
            return true;
        }
    }
    public Action pivotToPosition() {return new pivotToPositionCl();}

    public class setPivotCl implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            target = targetPivot;
            return false;
        }
    }

    public Action setPivot(int pos) {
        targetPivot = pos;
        return new setPivotCl();
    }

    public void pivotPositionTele() {
        if (target > MAX_POS)
            target = MAX_POS;
        if (target < 0) {
            target = 0;
        }
        slideState = Slide.checkLength();
        switch(slideState){
            case "Retracted":
                slideRetracted = true;
                slideHalf = false;
                slideFull = false;
            case "Half":
                slideRetracted = false;
                slideHalf = true;
                slideFull = false;
            case "Full":
                slideRetracted = false;
                slideHalf = false;
                slideFull = true;
            case "None":
                slideRetracted = false;
                slideHalf = false;
                slideFull = false;
                f = 0;
        }
        controller.setPID(p, i, d);
        int pos = rightPivot.getCurrentPosition();
        if (pos < 0 && slideRetracted) {f = 0;}
        if (0 <= pos && pos <= 20 && slideRetracted) {f = f1R;}
        if (20 < pos && pos <= 40 && slideRetracted) {f = f2R;}
        if (40 < pos && pos <= 60 && slideRetracted) {f = f3R;}
        if (60 < pos && pos <= 80 && slideRetracted) {f = f4R;}
        if (80 < pos && pos <= 100 && slideRetracted) {f = f5R;}
        if (100 < pos && pos <= 120 && slideRetracted) {f = f6R;}
        if (120 < pos && pos <= 140 && slideRetracted) {f = f7R;}
        if (140 < pos && pos <= 160 && slideRetracted) {f = f8R;}
        if (160 < pos && pos <= 180 && slideRetracted) {f = f9R;}
        if (pos < 180 && slideRetracted) {f = 0;}
        if (pos < 0 && slideHalf) {f = 0;}
        if (0 <= pos && pos <= 20 && slideHalf) {f = f1H;}
        if (20 < pos && pos <= 40 && slideHalf) {f = f2H;}
        if (40 < pos && pos <= 60 && slideHalf) {f = f3H;}
        if (60 < pos && pos <= 80 && slideHalf) {f = f4H;}
        if (80 < pos && pos <= 100 && slideHalf) {f = f5H;}
        if (100 < pos && pos <= 120 && slideHalf) {f = f6H;}
        if (120 < pos && pos <= 140 && slideHalf) {f = f7H;}
        if (140 < pos && pos <= 160 && slideHalf) {f = f8H;}
        if (160 < pos && pos <= 180 && slideHalf) {f = f9H;}
        if (pos < 180 && slideHalf) {f = 0;}
        if (pos < 0 && slideFull) {f = 0;}
        if (0 <= pos && pos <= 20 && slideFull) {f = f1F;}
        if (20 < pos && pos <= 40 && slideFull) {f = f2F;}
        if (40 < pos && pos <= 60 && slideFull) {f = f3F;}
        if (60 < pos && pos <= 80 && slideFull) {f = f4F;}
        if (80 < pos && pos <= 100 && slideFull) {f = f5F;}
        if (100 < pos && pos <= 120 && slideFull) {f = f6F;}
        if (120 < pos && pos <= 140 && slideFull) {f = f7F;}
        if (140 < pos && pos <= 160 && slideFull) {f = f8F;}
        if (160 < pos && pos <= 180 && slideFull) {f = f9F;}
        if (pos < 180 && slideFull) {f = 0;}
        double pid = controller.calculate(pos, target);
        double ff = pos * f;
        double power = pid + ff;
        rightPivot.setPower(power);
        leftPivot.setPower(power);
    }

    public void setTargetPivotPosition(int pos) {
        target = pos;
    }

    public void setManual(double power) {
        rightPivot.setPower(power * 0.2);
        leftPivot.setPower(power * 0.2);
    }

    public double calcDegree(){
        double pos = rightPivot.getCurrentPosition();
        //
        return pivotDegree;
    }
}

/*
    public void pivotPositionTele() {
        if(target > MAX_POS)
            target = MAX_POS;
        if(target < 0) {
            target = 0;
        }
        controller.setPID(p,i,d);
        int pos = rightPivot.getCurrentPosition();
        if (pos < 0) {f = 0;}
        if (0 <= pos && pos <= 20) {f = f1;}
        if (20 < pos && pos <= 40) {f = f2;}
        if (40 < pos && pos <= 60) {f = f3;}
        if (60 < pos && pos <= 80) {f = f4;}
        if (80 < pos && pos <= 100) {f = f5;}
        if (100 < pos && pos <= 120) {f = f6;}
        if (120 < pos && pos <= 140) {f = f7;}
        if (140 < pos && pos <= 160) {f = f8;}
        if (160 < pos && pos <= 180) {f = f9;}
        if (pos < 180) {f=0;}
        double pid = controller.calculate(pos, target);
        double ff = pos * f;
        double power = pid + ff;
        rightPivot.setPower(power);
        leftPivot.setPower(power);
    }
 */