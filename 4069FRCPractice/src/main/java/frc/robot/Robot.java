// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;


public class Robot extends TimedRobot {

  private UsbCamera camera;
  private MjpegServer server1;

  private CvSink cvSink;
  private MjpegServer server2;

  private Mat mat;


  CvSource outputStream;

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private Timer timer;
  
  private CANSparkMax right, left, rightMaster, rightSlave, leftMaster, leftSlave;

  private DifferentialDrive robotDrive1010, robotDrive4066;

  private Encoder leftEnc, rightEnc;

  private double averageEncoderPulses;

  private XboxController con1, con2;

  private double speed, rotation;

  private Robot robot;

  @Override
  public void robotInit() {
    this.robot = robot;
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    camera = new UsbCamera("USB Camera 0", 0);
    server1 = new MjpegServer("serve_USB Camera 0", 1181);
    server1.setSource(camera);

    cvSink = new CvSink("opencv_USB Camera 0");
    cvSink.setSource(camera);

    outputStream = new CvSource("blur", PixelFormat.kMJPEG, 640, 480, 30);
    server2 = new MjpegServer("Serve_blur", 1182);
    server2.setSource(outputStream);

    //mat = new Mat();

    CameraServer.startAutomaticCapture();

    timer = new Timer();
    timer.reset();

    right = new CANSparkMax(Constants.RIGHT_ID, MotorType.kBrushless);
    left = new CANSparkMax(Constants.LEFT_ID, MotorType.kBrushless);

    rightMaster = new CANSparkMax(Constants.RIGHT_MASTER_ID, MotorType.kBrushless);
    leftMaster = new CANSparkMax(Constants.LEFT_MASTER_ID, MotorType.kBrushless);

    rightSlave = new CANSparkMax(Constants.RIGHT_SLAVE_ID, MotorType.kBrushless);
    leftSlave = new CANSparkMax(Constants.LEFT_SLAVE_ID, MotorType.kBrushless);

    rightSlave.follow(rightMaster);
    leftSlave.follow(leftMaster);

    leftMaster.setInverted(true);

    robotDrive1010 = new DifferentialDrive(leftMaster, rightMaster);
    robotDrive4066 = new DifferentialDrive(left, right);

    leftEnc = new Encoder(Constants.leftA, Constants.leftB);
    rightEnc = new Encoder(Constants.rightA, Constants.rightB);

    leftEnc.reset();
    rightEnc.reset();

    con1 = new XboxController(Constants.co1Port);
    con2 = new XboxController(Constants.con2Port);

    Thread visionThread = new Thread(() ->{
      camera = CameraServer.startAutomaticCapture();
      camera.setResolution(640, 480);

      cvSink = CameraServer.getVideo();
      outputStream = CameraServer.putVideo("rectangle", 640, 480);
      mat = new Mat();

      while(!Thread.interrupted()){
        if(cvSink.grabFrame(mat) == 0){
          outputStream.notifyError(cvSink.getError());
          continue;
        }
        Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);

        outputStream.putFrame(mat);
      }
    });
    visionThread.setDaemon(true);;
    visionThread.start();
  }
  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
      //1010
      averageEncoderPulses = leftEnc.getDistance() / -rightEnc.getDistance();

      if(averageEncoderPulses < 3000){
        robotDrive1010.arcadeDrive(0.3, 0);
      }
      else{
        robotDrive1010.stopMotor();
      }
      //4066
      if (timer.get() < 2){
        robotDrive4066.arcadeDrive(0.3, 0);
      }
    }
  


  @Override
  public void teleopInit() {

  }

 
  @Override
  public void teleopPeriodic() {
    speed = con1.getLeftTriggerAxis() - con1.getRightTriggerAxis();
    rotation  = con1.getLeftX();
    //1010
    robotDrive1010.arcadeDrive(speed, rotation);
    //4066
    robotDrive4066.arcadeDrive(speed, rotation);
  }

  
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
