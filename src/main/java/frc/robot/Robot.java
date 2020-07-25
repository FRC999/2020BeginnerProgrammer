/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// Testing sound

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import edu.wpi.first.wpilibj.PWMVictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.AnalogInput;

import com.kauailabs.navx.frc.*;

// Music-specific
import java.util.ArrayList;
import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final WPI_TalonSRX driveLeftFrontTalon = new WPI_TalonSRX(1);
  private final WPI_TalonSRX driveLeftBackTalon = new WPI_TalonSRX(2);
  private final WPI_TalonSRX driveRightFrontTalon = new WPI_TalonSRX(3);
  private final WPI_TalonSRX driveRightBackTalon = new WPI_TalonSRX(4);

  //private final WPI_TalonFX exampleTalonFX1 = new WPI_TalonFX(30);
  //private final WPI_TalonFX exampleTalonFX2 = new WPI_TalonFX(31);

  SpeedControllerGroup leftGroup = new SpeedControllerGroup( driveLeftFrontTalon, driveLeftBackTalon);
  SpeedControllerGroup rightGroup = new SpeedControllerGroup( driveRightFrontTalon, driveRightBackTalon);


  private final DifferentialDrive m_robotDrive
      = new DifferentialDrive( leftGroup,  rightGroup);
  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();

  private AHRS navxSensors; 

  private int firstUltrasonicSensorPort = 1;
  private double ultrasonicInchesConversionFactor = 0.125;
  private AnalogInput firstUltrasonicSensor = new AnalogInput(firstUltrasonicSensorPort);
  
  public Compressor compressor = new Compressor();
  public DoubleSolenoid doubleSolenoid = new DoubleSolenoid(4,5);

  public int solenoidForwardButton = 1;
  public int solenoidReverseButton = 2;
  public int solenoidOffButton = 3;


  // Music-specific
  Orchestra _orchestra;
  TalonFX [] _fxes =  { new TalonFX(30), new TalonFX(31) };
  String[] _songs = new String[] {
    "song1.chrp",
    "song2.chrp",
    "song3.chrp",
    "song4.chrp",
    "song5.chrp",
    "song6.chrp",
    "song7.chrp",
    "song8.chrp",
    "song9.chrp", /* the remaining songs play better with three or more FXs */
    "song10.chrp",
    "song11.chrp",
  };
  int _songSelection = 0;
  int _timeToPlayLoops = 0;
  Joystick _joy;
  int _lastButton = 0;
  int _lastPOV = 0;
  int getButton() {
    for (int i = 1; i < 9; ++i) {
        if (_joy.getRawButton(i)) {
            return i;
        }
    }
    return 0;
  }
  void LoadMusicSelection(int offset)
    {
        /* increment song selection */
        _songSelection += offset;
        /* wrap song index in case it exceeds boundary */
        if (_songSelection >= _songs.length) {
            _songSelection = 0;
        }
        if (_songSelection < 0) {
            _songSelection = _songs.length - 1;
        }
        /* load the chirp file */
        _orchestra.loadMusic(_songs[_songSelection]); 
        /* print to console */
        SmartDashboard.putString("Music", "Song selected is: " + _songs[_songSelection] + ".  Press left/right on d-pad to change.");
        
        /* schedule a play request, after a delay.  
            This gives the Orchestra service time to parse chirp file.
            If play() is called immedietely after, you may get an invalid action error code. */
        _timeToPlayLoops = 10;
    }
    // end music-specific

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    driveLeftFrontTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

  //exampleTalonFX1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    SmartDashboard.putNumber("Calum", 5);

    //exampleTalonFX2.follow(exampleTalonFX1);
  
    try {
      navxSensors = new AHRS(SPI.Port.kMXP);
    }
    catch (final Exception exception) {
       System.out.print("Call security!!");
    }
//compressor.setClosedLoopControl(true);

    // Music-specific
    ArrayList<TalonFX> _instruments = new ArrayList<TalonFX>();
    for (int i = 0; i < _fxes.length; ++i) {
      _instruments.add   (_fxes[i]);
    }
    _orchestra = new Orchestra(_instruments);
        _joy = new Joystick(0);

  }
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("MagnetX", navxSensors.getRawMagX());
    SmartDashboard.putNumber("Left Encoder",driveLeftFrontTalon.getSelectedSensorPosition());
    //SmartDashboard.putNumber("TalonFX 1 Encoder",exampleTalonFX1.getSelectedSensorPosition());
    //SmartDashboard.putNumber("TalonFX 2 Encoder",exampleTalonFX2.getSelectedSensorPosition());

    SmartDashboard.putNumber("Joystick y",m_stick.getY());
    double ultrasonicDistanceInches = firstUltrasonicSensor.getValue()*ultrasonicInchesConversionFactor;
    SmartDashboard.putNumber("Ultrasonic Distance", ultrasonicDistanceInches);
    //SmartDashboard.putNumber("MagnetX", navxSensors.getRawMagX());  
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
    driveLeftFrontTalon.setSelectedSensorPosition(0);
  //  compressor.setClosedLoopControl(true);
  }

  
  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
  if (driveLeftFrontTalon.getSelectedSensorPosition() <= 9000) {
      m_robotDrive.arcadeDrive(-0.5, 0.0); // drive forwards half speed
    } else {
      m_robotDrive.stopMotor(); // stop robot
    
  }
    

  }


  // test1
  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    compressor.setClosedLoopControl(false);

    // Music-specific
    LoadMusicSelection(0);

  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
    // exampleTalonFX1.set(m_stick.getZ());
/*
    if (m_stick.getRawButton(solenoidForwardButton))
        doubleSolenoid.set(Value.kForward);
        if (m_stick.getRawButton(solenoidReverseButton))
        doubleSolenoid.set(Value.kReverse);
        if (m_stick.getRawButton(solenoidOffButton))
        doubleSolenoid.set(Value.kOff);
*/


    // Music-specific

    int btn = getButton();
    int currentPOV = _joy.getPOV();
    if (_timeToPlayLoops > 0) {
      --_timeToPlayLoops;
      if (_timeToPlayLoops == 0) {
          /* scheduled play request */
          System.out.println("Auto-playing song.");
          _orchestra.play();
      }
    }
    if (_lastButton != btn) {
      _lastButton = btn;
      switch (btn) {
          case 1: /* toggle play and paused */
              if (_orchestra.isPlaying()) {
                  _orchestra.pause();
                  System.out.println("Song paused");
              }  else {
                  _orchestra.play();
                  System.out.println("Playing song...");
              }
              break;
              
          case 2: /* toggle play and stop */
              if (_orchestra.isPlaying()) {
                  _orchestra.stop();
                  System.out.println("Song stopped.");
              }  else {
                  _orchestra.play();
                  System.out.println("Playing song...");
              }
              break;
          case 5:
              /* increment song selection */
              LoadMusicSelection(+1);
              break;
          case 6:
              /* decrement song selection */
              LoadMusicSelection(-1);
              break;

            }
    }


  }



  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledInit() {
    m_robotDrive.arcadeDrive(0,0);
//    compressor.setClosedLoopControl(false);
  }

}
