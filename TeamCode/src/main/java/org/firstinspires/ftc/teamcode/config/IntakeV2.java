package org.firstinspires.ftc.teamcode.config;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class IntakeV2 {

    public Servo wristLeft, wristRight, claw, gate;
    public DcMotorEx cycler;

    public static double targetClawPosition, targetWristPosition, targetClawPosition2, targetWristPosition2, gateTargetPosition, gateTargetPosition2;
    public int cycleDirection, cycleDirection2;
    public static double wristIntake, WristDrop, wristHide, openClaw, closeClaw;
    public static double openGate, closeGate;

    public IntakeV2(HardwareMap map) {
        wristLeft = map.servo.get("wristLeft");
        wristRight = map.servo.get("wristRight");
        claw = map.servo.get("claw");
        cycler = map.get(DcMotorEx.class,"cycler");
    }

    public class moveClawCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(targetClawPosition);
            return false;
        }
    }
    public Action moveClaw() {
        return new moveClawCl();
    }
    public class setClawCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            targetClawPosition = targetClawPosition2;
            return false;
        }
    }
    public Action setClaw(double pos) {
        targetClawPosition2 = pos;
        return new setClawCl();
    }
    public class moveWristCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wristLeft.setPosition(targetWristPosition);
            wristRight.setPosition(targetWristPosition);
            return false;
        }
    }
    public Action moveWrist(){
        return new moveWristCl();
    }
    public class setWristCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            targetWristPosition = targetWristPosition2;
            return false;
        }
    }
    public Action setWrist(double pos) {
        targetWristPosition2 = pos;
        return new setWristCl();
    }

    public void teleWrist(double pos) {
        wristLeft.setPosition(pos);
        wristRight.setPosition(pos);
    }
    //Cycler
    public class moveCyclerCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            cycler.setPower(cycleDirection);
            return false;
        }
    }

    public Action moveCycler() {return new moveCyclerCl();}

    public class setCyclerCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            cycleDirection = cycleDirection2;
            return true;
        }
    }

    public Action setCycler(int direction){
        cycleDirection2 = direction;
        return new setCyclerCl();
    }

    public void cycleCycler(int direction) {
        cycler.setPower(direction);
    }

    public class moveGateCl implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            gate.setPosition(gateTargetPosition);
            return true;
        }
    }
    public Action moveGate() {return new moveGateCl();}

    public class setGateCl implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            gateTargetPosition = gateTargetPosition2;
            return false;
        }
    }
    public Action setGate(double pos) {
        gateTargetPosition2 = pos;
        return new setGateCl();
    }

    public void teleGate(double pos){
        gate.setPosition(pos);
    }
}
