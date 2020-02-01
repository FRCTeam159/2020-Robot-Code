/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cameras extends SubsystemBase {
  /**
   * Creates a new Cameras.
   */
  public static UsbCamera camera1;
  public static UsbCamera camera2;
  VideoSink server;
  public static boolean is_Front = true;
  NetworkTableEntry autotarget;
  static public CvSource outputStream;
  CvSink cvSink = null;
  static Mat mat;

  public Cameras() {
    final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    final NetworkTable table = inst.getTable("targetdata");
    autotarget = table.getEntry("autotarget");
    camera1 = CameraServer.getInstance().startAutomaticCapture(0);
    camera2 = CameraServer.getInstance().startAutomaticCapture(1);
    server = CameraServer.getInstance().addSwitchedCamera("SwitchedCamera");
    camera1.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
    camera2.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
    camera1.setFPS(15);
    camera1.setResolution(320, 240);
    camera2.setFPS(15);
    camera2.setResolution(320, 240);
    //outputStream = CameraServer.getInstance().putVideo("CameraView", 320, 240);
    setFront();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setFront(){
    server.setSource(camera1);
    cvSink = CameraServer.getInstance().getVideo(camera1);
    is_Front = true;
  }
  public void setBack(){
    server.setSource(camera2);
    cvSink = CameraServer.getInstance().getVideo(camera2);
    is_Front = false;
  }
  public static boolean isFront(){
    return is_Front;
  }
  public void setBrightness(){
    if (autotarget.getBoolean(false)) {
      Cameras.camera1.setBrightness(1);
      Cameras.camera1.setExposureManual(1);
    } else {
      Cameras.camera1.setBrightness(50);
      Cameras.camera1.setExposureManual(10);
    }
  }
  /*
  public void writeVideo(){
    mat = new Mat();
    cvSink = CameraServer.getInstance().getVideo();
    if (cvSink.grabFrame(mat) == 0) {
      // Send the output the error.
      outputStream.notifyError(cvSink.getError());
      // skip the rest of the current iteration
      return;
    }
    Mat m2=mat.clone();
    outputStream.putFrame(m2);
  } 
  */
}
