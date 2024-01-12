package frc.robot.commands.limelight;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

public class LineUpWithCube extends Command {
    private final double maxSpeed = 6.0;

    private PIDController m_drivePIDController = new PIDController(0.01, 0.0, 0.0);
    private final Limelight m_Limelight;
    private final CommandSwerveDrivetrain m_Drivetrain;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private double ySpeed;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LineUpWithCube(CommandSwerveDrivetrain drivetrain, Limelight limelight) {
    m_Limelight = limelight;
    m_Drivetrain = drivetrain;
    addRequirements(drivetrain, limelight);

    m_drivePIDController.setTolerance(1);
    m_drivePIDController.setSetpoint(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Limelight.turnOnLimelight();
    m_Limelight.setLimelightPipeline(4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Limelight.updateLimeLight();

    // System.out.println(m_Limelight.getTX());
    ySpeed = m_drivePIDController.calculate(m_Limelight.getTX());
    // if (m_drivePIDController.atSetpoint()) {
    //   ySpeed = 0;
    // }
    // System.out.println(ySpeed);
    m_Drivetrain.applyRequest(() -> drive.withVelocityY(ySpeed * maxSpeed)).execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Limelight.turnOffLimelight();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.println(m_limelight.getTX());
    // if (m_drivePIDController.atSetpoint()) {
    //   return true;
    // }
    return false;
  }
}
