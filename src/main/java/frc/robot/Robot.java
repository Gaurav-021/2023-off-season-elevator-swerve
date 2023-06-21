// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class Robot extends TimedRobot {

  public static CTREConfigs ctreConfigs;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  public Pigeon2 gyro;


  XboxController driver_Controller = new XboxController(0);
  XboxController operator_controller = new XboxController(1);
  boolean operator_controller_A_button;
  boolean operator_controller_Y_button;
  boolean operator_controller_X_button;
  boolean operator_controller_B_button;
  boolean brake_mode_enabled = false; 
  double driver_controller_L_X_Axis;
  double driver_controller_L_Y_Axis;
  double driver_controller_R_X_Axis;
  double driver_controller_R_Y_Axis;
  int driver_controller_POV_button;
  double speedScale = 0.85;

  
  long lastnano_time = 0;
  Timer m_timeToButtonPress = new Timer();
  Timer chargedStationTimer = new Timer();
  Arm m_arm;

  double[] autonomousSwerveCommands = {0,0,0};

  @Override
  public void robotInit() {

    ctreConfigs = new CTREConfigs();
    m_robotContainer = new RobotContainer();

    gyro = new Pigeon2(Constants.Swerve.pigeonID);
    gyro.configFactoryDefault();
    zeroGyro();

        
  }
 
  @Override
  public void robotPeriodic() {
    
    // CommandScheduler.getInstance().run();


    // SmartDashboard.putNumber("SwerveDistanceX", getSwerveDistanceX());
    // SmartDashboard.putNumber("SwerveDistanceY", getSwerveDistanceY());
    // SmartDashboard.putNumber("Roll", gyro.getRoll());
    // SmartDashboard.putNumber("Pitch", gyro.getPitch());
    // SmartDashboard.putNumber("Yaw", gyro.getYaw());

    //SmartDashboard.putNumber("getVerticalElevatorPosition", m_arm.getVerticalElevatorPosition());

    // SmartDashboard.putNumber("operator controller", operator_controller.getLeftY());


    // if (driver_Controller.getPOV() == 0){
    //   m_robotContainer.updateSwerveParameters(new Translation2d(0, 2), 0, true);
    // }
    // else if (driver_Controller.getPOV() == 90){
    //   m_robotContainer.updateSwerveParameters(new Translation2d(2, 0), 0, true);

    // }
    // else if (driver_Controller.getPOV() == 270){
    //   m_robotContainer.updateSwerveParameters(new Translation2d(-2, 0), 0, true);

    // }
    // else if (driver_Controller.getPOV() == 180){
    //   m_robotContainer.updateSwerveParameters(new Translation2d(0, -2), 0, true);
    // }
    // else {
    //   m_robotContainer.updateSwerveParameters(new Translation2d(0, 0), 0, false);

    // }  

   }

  @Override
  public void autonomousInit() {
    m_robotContainer = new RobotContainer();

  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_arm = new Arm();
  }

  @Override
  public void teleopPeriodic() {
    getControllerStates();
    SmartDashboard.putNumber("encoder value", -m_arm.vertical_elevator_motor_1.getSelectedSensorPosition());
    
    if (operator_controller_A_button){
      m_arm.move_vertical_elevator_to_pos(-1000);

    }
    else if (operator_controller_B_button || operator_controller_X_button){
      m_arm.move_vertical_elevator_to_pos(-30000);
    }
    else if (operator_controller_Y_button){
      m_arm.move_vertical_elevator_to_pos(-50000);
    }
    else {
      m_arm.move_vertical_elevator(operator_controller.getLeftY());
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  public void getControllerStates() {
    operator_controller_A_button = operator_controller.getAButton();
    operator_controller_B_button = operator_controller.getBButton();
    operator_controller_X_button = operator_controller.getXButton();
    operator_controller_Y_button = operator_controller.getYButton();
    // brake_mode_enabled = driver_controller.getStartButton() && driver_controller.getBackButton();
    int m_buttonPressCount = 0;
    double obtainedButtonTime = 0;
    if(operator_controller.getStartButton() && operator_controller.getBackButton()) {
      m_buttonPressCount++;
      if(m_buttonPressCount == 1) {
        m_timeToButtonPress.start();
      }
      obtainedButtonTime = m_timeToButtonPress.get();
    }
      if(obtainedButtonTime > 0.5) {
        brake_mode_enabled = !brake_mode_enabled;
        m_timeToButtonPress.stop();
        m_timeToButtonPress.reset();
    }
  }
  

  public double getSwerveDistanceX(){
    return m_robotContainer.s_Swerve.swerveOdometry.getPoseMeters().getX();
  }
  public double getSwerveDistanceY(){
    return m_robotContainer.s_Swerve.swerveOdometry.getPoseMeters().getY();
  }

  public void zeroGyro(){
    gyro.setYaw(0);
  }

  public Rotation2d getYaw() {
      return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
  }

}
