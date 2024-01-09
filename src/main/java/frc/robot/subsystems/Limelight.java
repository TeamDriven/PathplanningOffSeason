// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  public static final String LIMELIGHT = "limelight";
  public static double tx = 0;
  public static double ty = 0;
  public static double ta = 0;
  public static double ts = 0;

  /** Creates a new ExampleSubsystem. */
  public Limelight() {
    updateLimeLight();

    turnOffLimelight();
  }

  public void turnOnLimelight() {
    NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("camMode").setNumber(0);
    setLights(0);
  }

  public void turnOffLimelight() {
    NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("camMode").setNumber(1);
    setLights(1);
  }

  public Command setLightsCommand(int lightMode) {
    return new Command() {
        @Override
        public void initialize() {
            setLights(lightMode);
        }
    };
  }

  // 0 for default, 1 for off, 2 for blink, 3 for on
  public void setLights(int lightMode) {
    NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("ledMode").setNumber(lightMode);
  } 

  public Command setLimelightPipelineCommand(int pipelineNumber) {
    return new Command() {
        @Override
        public void initialize() {
            setLimelightPipeline(pipelineNumber);
        }
    };
  }

  // 0 for Apriltags, 1 for Retroreflective Tape
  public void setLimelightPipeline(int pipelineNumber) {
    NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("pipeline").setNumber(pipelineNumber);
  }

  public void updateLimeLight() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(LIMELIGHT);
    tx = table.getEntry("tx").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    ta = table.getEntry("ta").getDouble(0);
    ts = table.getEntry("ts").getDouble(0);

    // table.putValue("ledMode", 3);

    // final ShuffleboardTab tab = Shuffleboard.getTab(LIMELIGHT);
    // SmartDashboard.putNumber("ta", ta);
    // SmartDashboard.putNumber("ts", ts);
    // SmartDashboard.putNumber("tx", tx);
    // SmartDashboard.putNumber("ty", ty);
  }

  public void resetLimelight() {
    tx = 0;
    ty = 0;
    ta = 0;
    ts = 0;

    // SmartDashboard.putNumber("ta", ta);
    // SmartDashboard.putNumber("ts", ts);
    // SmartDashboard.putNumber("tx", tx);
    // SmartDashboard.putNumber("ty", ty);
    // SmartDashboard.putNumber("Apriltag id", -1);

    turnOffLimelight();
  }

  public double getApriltagID() {
    double id = NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("tid").getDouble(0);
    // SmartDashboard.putNumber("Apriltag id", id);
    return id;
    // return 1;
  }

  public Command getPieceCommand(int pipelineNumber) {
    return new Command() {
        @Override
        public void initialize() {
            turnOnLimelight();
            setLimelightPipeline(pipelineNumber);
        }

        @Override
        public void execute() {
            getPiece();
        }

        @Override
        public void end(boolean interrupted) {
            turnOffLimelight();
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    };
  }

  public void getPiece() {
    System.out.println(NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("tclass").getString(null));
  }

  /**
   * Finds the pose of the robot when detecting 3D april tags
   * @return An array of doubles in the order of X, Y, Z, Pitch, Yaw, Roll
   */
  public double[] getRobotPose() {
    double[] pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    try{
      pose = NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("botpose").getDoubleArray(pose);
      // SmartDashboard.putNumber("PoseX", pose[0]);
      // SmartDashboard.putNumber("PoseY", pose[1]);
      // SmartDashboard.putNumber("PoseZ", pose[2]);
      // SmartDashboard.putNumber("PosePitch", pose[3]);
      // SmartDashboard.putNumber("PoseYaw", pose[4]);
      // SmartDashboard.putNumber("PoseRoll", pose[5]);
    } catch(ArrayIndexOutOfBoundsException e) {
      // System.out.println("No 3D April tag detected");
    }
    return pose;
    // return null;
  }

  public double getTX() {
    return tx;
    // return 1;
  }

  public double getTY() {
    return ty;
    // return 1;
  }

  public double getTA() {
    return ta;
    // return 1;
  }

  public double getTS() {
    return ts;
    // return 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
