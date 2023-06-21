package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Timer;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Arm {
    
    // I don't think this has a use.
    double arm_length;
    double target_vertical_elevator_position_command, current_horizontal_elevator_position_command;
    
    // Used to time how long actions take.
    Timer m_time = new Timer();

    // Used to help time/document/run any actions.
    boolean action_finished = false;

    // Motors
    //CANSparkMax intake_motor_1 = new CANSparkMax(Constants.INTAKE_MOTOR_1_ID, MotorType.kBrushless);
    //CANSparkMax intake_motor_2 = new CANSparkMax(Constants.INTAKE_MOTOR_2_ID, MotorType.kBrushless);
    WPI_TalonFX vertical_elevator_motor_1 = new WPI_TalonFX(Constants.VERTICAL_ELEVATOR_MOTOR_1_ID, "rio"); 
    WPI_TalonFX vertical_elevator_motor_2 = new WPI_TalonFX(Constants.VERTICAL_ELEVATOR_MOTOR_2_ID, "rio");
    //WPI_TalonFX horizontal_elevator_motor = new WPI_TalonFX(Constants.HORIZONTAL_ELEVATOR_MOTOR_ID, "rio");

    public Arm() { // constructor
        // elevator motor setups
        vertical_elevator_motor_1.configFactoryDefault();
        vertical_elevator_motor_1.configFactoryDefault();
        
        vertical_elevator_motor_1.setInverted(false);
        vertical_elevator_motor_1.setSensorPhase(false);
        vertical_elevator_motor_1.setNeutralMode(NeutralMode.Coast);

        vertical_elevator_motor_2.setInverted(false);        
        vertical_elevator_motor_2.setSensorPhase(false);
        vertical_elevator_motor_2.setNeutralMode(NeutralMode.Coast);

        
        vertical_elevator_motor_1.config_kP(0, 0.035, 30);
        vertical_elevator_motor_1.config_kI(0, 0.0, 30);
        vertical_elevator_motor_1.config_kD(0, 0.0, 30);
        vertical_elevator_motor_1.config_kF(0, 0.0, 30);
        vertical_elevator_motor_1.configClosedloopRamp(0.25);
// :]

        vertical_elevator_motor_2.config_kP(0, 0.035, 30);
        vertical_elevator_motor_2.config_kI(0, 0.0, 30);
        vertical_elevator_motor_2.config_kD(0, 0.0, 30);
        vertical_elevator_motor_2.config_kF(0, 0.0, 30);
        vertical_elevator_motor_2.configClosedloopRamp(0.25);


        vertical_elevator_motor_1.configAllowableClosedloopError(0, 500, 30);
        vertical_elevator_motor_2.configAllowableClosedloopError(0, 500, 30);

        vertical_elevator_motor_1.follow(vertical_elevator_motor_2);

        target_vertical_elevator_position_command = vertical_elevator_motor_2.getSelectedSensorPosition();

        // horizontal_elevator_motor.setInverted(false);        
        // horizontal_elevator_motor.setSensorPhase(false);
        // horizontal_elevator_motor.setNeutralMode(NeutralMode.Coast);

        // horizontal_elevator_motor.config_kP(0, 0.2, 30);
        // horizontal_elevator_motor.config_kI(0, 0.0, 30);
        // horizontal_elevator_motor.config_kD(0, 0.0, 30);
        // horizontal_elevator_motor.config_kF(0, 0.0, 30);
    }

    public void move_vertical_elevator(double controller_input){

        if (Math.abs(controller_input) > 0.1){
            target_vertical_elevator_position_command = target_vertical_elevator_position_command + Constants.maxElevatorIncrement*controller_input;
            if (target_vertical_elevator_position_command >= 0){
                target_vertical_elevator_position_command = 0;
            }
            else if (target_vertical_elevator_position_command <= Constants.maxElevatorValue) {
                target_vertical_elevator_position_command = Constants.maxElevatorValue;
            }
            // vertical_elevator_motor_2.set(ControlMode.Position, target_vertical_elevator_position_command);
            move_vertical_elevator_to_pos(target_vertical_elevator_position_command);
        }
    }

    public void move_vertical_elevator_to_pos(double input){
        target_vertical_elevator_position_command = input;
        vertical_elevator_motor_2.set(ControlMode.Position, input);

    }

    public double getVerticalElevatorPosition() {
        return vertical_elevator_motor_2.getSelectedSensorPosition();
        
    }

/*     public void recalibrateElevatorPositions() {
        vertical_elevator_motor_1.setSelectedSensorPosition(?, 0, 30);
        vertical_elevator_motor_2.setSelectedSensorPosition(?, 0, 30);
        horizontal_elevator_motor.setSelectedSensorPosition(?, 0, 30);
    } */


};