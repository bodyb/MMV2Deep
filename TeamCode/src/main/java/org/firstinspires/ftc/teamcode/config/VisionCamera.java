package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;


import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.core.Mat;

public class VisionCamera {
    VisionPortal visionPortal;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public VisionCamera(HardwareMap map, Telemetry tele) {
        hardwareMap = map;
        telemetry = tele;
        visionPortal = new VisionPortal.Builder()
                .setCamera(map.get(WebcamName.class, "Webcam1"))
                //.setAutoStartStreamOnBuild(true)
                .build();


    }
}
