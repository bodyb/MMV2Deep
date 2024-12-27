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
@Config
public class Pivot {

    public PIDController controller;
    public static double p=-0.04,i=0,d=0,f=0;
    public static double f1=-0.02, f2=-0.01, f3=-0.006, f4=-0.0022, f5=-0.0016, f6=-0.0008, f7=0, f8=0, f9=0.0005;
    public DcMotorEx leftPivot, rightPivot;
    public static boolean motorReverse = false, motor2Reverse = true;
    public static int MAX_POS = 180;

    public final double ticksPerRotation = 288;

    public static int target = 0;
    public int targetPivot = 0;

    public Pivot() {
        leftPivot = hardwareMap.get(DcMotorEx.class, "leftPivot");
        rightPivot = hardwareMap.get(DcMotorEx.class, "rightPivot");

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
            return true;
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
    }
}