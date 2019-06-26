/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // Motors
  public static final int DRIVETRAIN_RIGHT_MOTOR1_CAN_ID = 0;
  public static final int DRIVETRAIN_RIGHT_MOTOR2_CAN_ID = 1;
  public static final int DRIVETRAIN_RIGHT_MOTOR3_CAN_ID = 4;

  public static final int DRIVETRAIN_LEFT_MOTOR1_CAN_ID = 15;
  public static final int DRIVETRAIN_LEFT_MOTOR2_CAN_ID = 14;
  public static final int DRIVETRAIN_LEFT_MOTOR3_CAN_ID = 11;
}
