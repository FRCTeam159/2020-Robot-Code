/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.*;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.CvSink;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.GripPipeline;

/**
 * Add your docs here.
 */
public class VisionProcess extends Thread {
  // static UsbCamera camera1;
  public static double targetWidth = 34.0; // physical width of target (inches)
  public static double targetHeight = 4.0; // physical height of target (inches)
  public static double distanceToFillWidth = 27.0; // measured distance where target just fills screen (width)
  public static double distanceToFillHeight = 6.25; // measured distance where target just fills screen (height)
  public static double cameraFovW = 2 * Math.atan(0.5 * targetWidth / distanceToFillWidth); // 41.8 degrees
  public static double cameraFovH = 2 * Math.atan(0.5 * targetHeight / distanceToFillHeight); // 33.0 degrees
  public static double hTarget = 0.0;
  public static double vTarget = 0.0;
  // public static double hToll = 1.0;
  // public static double vToll = 10.0;
  public static boolean isAtTarget = false;
  public static double imageWidth = 320;
  public static double imageHeight = 240;
  public double xOff;
  // multiply these factors by target screen projection (pixels) to get distance
  double distanceFactorWidth = 0.5 * targetWidth * imageWidth / Math.tan(cameraFovW / 2.0);
  double distanceFactorHeight = 0.5 * targetHeight * imageHeight / Math.tan(cameraFovH / 2.0);
  // multiply these factors by target center offset (pixels) to get horizontal and
  // vertical angle offsets
  double angleFactorWidth = Math.toDegrees(cameraFovW) / imageWidth;
  double angleFactorHeight = Math.toDegrees(cameraFovH) / imageHeight;
  // expected width/height ratio
  double targetAspectRatio = targetWidth / targetHeight;
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("targetdata");
  NetworkTableEntry hAngle;
  NetworkTableEntry vAngle;
  NetworkTableEntry distance;
  NetworkTableEntry haveTarget;
  NetworkTableEntry atTarget;
  NetworkTableEntry autotarget;

  public void init() {
    // camera1 = CameraServer.getInstance().startAutomaticCapture("Targeting", 0);
    // camera1.setFPS(15);
    // camera1.setResolution(320, 240);
    // camera1.setBrightness(1);
    // camera1.setExposureManual(1);
    SmartDashboard.putNumber("Targets", 0);
    // SmartDashboard.putNumber("H distance", 0);
    SmartDashboard.putNumber("Target distance", 1);
    SmartDashboard.putNumber("V angle", 0);
    SmartDashboard.putNumber("H angle", 0);
    // SmartDashboard.putNumber("Aspect Ratio", 0);

    // SmartDashboard.putNumber("Angle", 0);
    SmartDashboard.putBoolean("Show HSV", false);
    SmartDashboard.putBoolean("onTarget", false);
    SmartDashboard.putBoolean("autotarget", false);
    System.out.println("fov H:" + Math.toDegrees(cameraFovH) + " W:" + Math.toDegrees(cameraFovW));
    System.out.println("Expected Target Aspect ratio:" + round10(targetAspectRatio));
  }

  Point center(Rect r) {
    double cx = r.tl().x + 0.5 * r.width;
    double cy = r.tl().y + 0.5 * r.height;
    return new Point(cx, cy);
  }

  double round10(double x) {
    return Math.round(x * 10 + 0.5) / 10.0;
  }

  public double horizontalTweek(){
    double a = 10.5/19.9; // fractional Max xOff given maOff
     xOff  = -a*(RobotContainer.driveTrain.getHeading());
    xOff = Robot.coerce(-10.5, 10.5, xOff);
    double aOff = angleFactorWidth*xOff;
    return aOff;
  }
  public double verticalTweek(){
    return 0.0;
  }
  public void log(){
    SmartDashboard.putNumber("xOff", xOff);
    double hoff = hAngle.getDouble(0);
    double dw = distance.getDouble(0);
    double voff = vAngle.getDouble(0);

    SmartDashboard.putNumber("Target distance", round10(dw));
    SmartDashboard.putNumber("H angle", round10(hoff));
    SmartDashboard.putNumber("V angle", round10(voff));
    SmartDashboard.putBoolean("onTarget", isAtTarget);
  }

  public void run() {
    GripPipeline grip = new GripPipeline();
    CvSink cvSink = CameraServer.getInstance().getVideo(Cameras.camera1);
    CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 320, 240);
    Mat mat = new Mat();
    ArrayList<Rect> rects = new ArrayList<Rect>();
    // TODO: use a network tables data structure to pass target params to Robot
    // Program


    distance = table.getEntry("tDistance");
    vAngle = table.getEntry("vAngle");
    hAngle = table.getEntry("hAngle");
    haveTarget = table.getEntry("targets");
    atTarget = table.getEntry("atTarget");
    autotarget = table.getEntry("autotarget");

    while (true) {
      try {
        Thread.sleep(20);
      } catch (InterruptedException ex) {
        System.out.println("exception)");
      }
      autotarget = table.getEntry("autotarget");
      // frontCamera = table.getEntry("frontcamera");
      // boolean targetcam=frontCamera.getBoolean(true);
  
      if(!autotarget.getBoolean(false)){
        isAtTarget = false;
        haveTarget.setBoolean(false);
        atTarget.setBoolean(false);
        SmartDashboard.putBoolean("onTarget", isAtTarget);
        SmartDashboard.putNumber("Targets",0);
      //  continue; 
      } 
      cvSink = CameraServer.getInstance().getVideo();
      if (cvSink.grabFrame(mat) == 0) {
        // Send the output the error.
        outputStream.notifyError(cvSink.getError());
        // skip the rest of the current iteration
        continue;
      }
      
      
      /*
       * autotarget = table.getEntry("autotarget");
       * 
       * 
       */
      grip.process(mat);
      Boolean show_hsv = SmartDashboard.getBoolean("Show HSV", false);
      if (show_hsv) {
        Mat hsv = grip.hsvThresholdOutput(); // display HSV image
        hsv.copyTo(mat);
      }
      ArrayList<MatOfPoint> contours = grip.filterContoursOutput();
      rects.clear();
      double max_area = 0;
      Rect biggest = null;
      // find the bounding boxes of all targets
      for (int i = 0; i < contours.size(); i++) {
        MatOfPoint contour = contours.get(i);
        Rect r = Imgproc.boundingRect(contour);
        double area = r.area();
        if (area > max_area) {
          biggest = r;
          max_area = area;
        }
        rects.add(r);
      }
      // calculate distance to target
      // - using ht
      SmartDashboard.putNumber("Targets", rects.size());
      if (biggest != null) {
        double dh = distanceFactorHeight / biggest.height;
        double dw = distanceFactorWidth / biggest.width;
        Point ctr = center(biggest);
        double hoff = angleFactorWidth * (ctr.x - 0.5 * imageWidth);
        hoff += horizontalTweek();
        double voff = -angleFactorHeight * (ctr.y - 0.5 * imageHeight); // invert y !
        hAngle.setDouble(hoff);
        vAngle.setDouble(voff);
        distance.setDouble(dw);
        haveTarget.setBoolean(true);
        atTarget.setBoolean(isAtTarget);
        // SmartDashboard.putNumber("H distance", round10(dh));


        if (Math.abs(hoff - hTarget) <= Targeting.hToll && Math.abs(voff - vTarget) <= Targeting.vToll) {
          isAtTarget = true;
        } else {
          isAtTarget = false;
        }
        hAngle.setDouble(hoff);
        vAngle.setDouble(voff);
        distance.setDouble(dw);
        haveTarget.setBoolean(true);
        atTarget.setBoolean(isAtTarget);
        log();
        // SmartDashboard.putNumber("Aspect Ratio",
        // round10((double)(biggest.width)/biggest.height));

      } else {
        haveTarget.setBoolean(false);
      }
      for (int i = 0; i < rects.size(); i++) {
        Rect r = rects.get(i);
        Point tl = r.tl();
        Point br = r.br();
        double width  = br.x - tl.x;
        double xVal = tl.x + 0.5*(width);
        double xTweek = (xOff * width)/targetWidth;
        Point xPoint = new Point(xVal + xTweek, tl.y);
        if (r == biggest){
          Imgproc.rectangle(mat, tl, br, new Scalar(255.0, 255.0, 0.0), 2);
          Imgproc.drawMarker(mat, xPoint, new Scalar(0, 0 , 255), Imgproc.MARKER_CROSS, 35 , 2 , 8);
        }else
          Imgproc.rectangle(mat, tl, br, new Scalar(255.0, 255.0, 255.0), 1);
      }
      
      outputStream.putFrame(mat);

    }
  }
}
