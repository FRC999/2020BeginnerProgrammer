/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// This is a test
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

  private final WPI_TalonFX exampleTalonFX1 = new WPI_TalonFX(30);
  private final WPI_TalonFX exampleTalonFX2 = new WPI_TalonFX(31);

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

  final static public int wheelDiameter = 4;  // wheel diameter, inches
  final static public int clicksPerTurn = 4096;  // encoder clicks per turn; one turn = Pi*Diameter inches
  final static public int distanceBetweenWheels = 32;  // Distance between left and right wheels, inches
  
  public int homeworkState = 0;
  // 0 - drive normally
  // 1 - button was pressed; ignore everything until special task is done
  public int homeworkStateStep = 0;
  // 0 - turn
  // 1 - drive forward
  public int homeworkStateStepBegin = 0;
  // 0 - did not start yet
  // 1 - step started
  public int homeworkEncoderStart;  // remembers the encoder value before starting to move

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */

  @Override
  public void robotInit() {
    driveLeftFrontTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

  exampleTalonFX1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    SmartDashboard.putNumber("Calum", 5);

    exampleTalonFX2.follow(exampleTalonFX1);
  
    try {
      navxSensors = new AHRS(SPI.Port.kMXP);
    }
    catch (final Exception exception) {
       System.out.print("Call security!!");
    }
//compressor.setClosedLoopControl(true);


  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("MagnetX", navxSensors.getRawMagX());
    SmartDashboard.putNumber("Left Encoder",driveLeftFrontTalon.getSelectedSensorPosition());
    SmartDashboard.putNumber("TalonFX 1 Encoder",exampleTalonFX1.getSelectedSensorPosition());
    SmartDashboard.putNumber("TalonFX 2 Encoder",exampleTalonFX2.getSelectedSensorPosition());

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
    // m_timer.get() < 2.0
    //
    // Change to check encoder settings instead
    //
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
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {

    if ( this.homeworkState == 0 ) {
      m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
      exampleTalonFX1.set(m_stick.getZ());

      if (m_stick.getRawButton(8)) {
        this.homeworkState = 1;
        this.homeworkStateStep = 0;
        this.homework1();
      }
    } else {
      // special task
      this.homework1();
    }
    
/*
    if (m_stick.getRawButton(solenoidForwardButton))
        doubleSolenoid.set(Value.kForward);
        if (m_stick.getRawButton(solenoidReverseButton))
        doubleSolenoid.set(Value.kReverse);
        if (m_stick.getRawButton(solenoidOffButton))
        doubleSolenoid.set(Value.kOff);
*/
    }

    public void homeworkTurn(int degrees) {     // turn the robot, degrees

        double endTurnDistance;
        endTurnDistance = ( Math.abs(degrees) /360)*Math.PI* distanceBetweenWheels ; //distance inches the robot needs to go to turn "degrees"
        double totalClicks = (clicksPerTurn / wheelDiameter) * endTurnDistance ; // how many clicks I need to get through to complete the turn

        if (homeworkStateStepBegin == 0) {
          homeworkEncoderStart = driveLeftFrontTalon.getSelectedSensorPosition() ;
          m_robotDrive.arcadeDrive( 0 , (degrees > 0)? 0.5 : -0.5 );
          homeworkStateStepBegin = 1;
        } else { // turing already; check encoder to see if I am done
          if ( Math.abs( driveLeftFrontTalon.getSelectedSensorPosition() - homeworkEncoderStart) >= totalClicks ) // turn is done!!!
          {
            m_robotDrive.arcadeDrive( 0 , 0 ); //stop the robot!!
            homeworkStateStep ++ ;  // go to the next step
            homeworkStateStepBegin = 0;  // reset the Begit status back to 0; we did not start turning yet
          } else {
            m_robotDrive.arcadeDrive( 0 , (degrees > 0)? 0.5 : -0.5 );
          }
        }
    }

    public void homeworkGoForward(int distance) { // distance is in ft
    }

    public void homework1() {
      
      switch (homeworkStateStep) {
        case 0:
          this.homeworkTurn(45);
          break;
        case 1:
          this.homeworkGoForward(10);
          break;
      }

      // this.homeworkState = 0; // eventually

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
