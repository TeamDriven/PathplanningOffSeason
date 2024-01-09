package frc.robot.commands.drivetrain;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

public class LineUpWithCubePath extends Command {
    private final double maxSpeed = 6.0;

    private PIDController m_xPIDController = new PIDController(0.01, 0.0, 0.0);
    private PIDController m_yPIDController = new PIDController(0.01, 0.0, 0.0);
    private PIDController m_rotPIDController = new PIDController(0.01, 0.0, 0.0);

    private final Limelight m_Limelight;
    private final CommandSwerveDrivetrain m_Drivetrain;

    private final String autoPath;
    private final int index;

    private double xGoal;
    private double rotGoal;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private double xSpeed;
    private double ySpeed;
    private double rotSpeed;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LineUpWithCubePath(CommandSwerveDrivetrain drivetrain, Limelight limelight, String autoPath, int index) {
    m_Limelight = limelight;
    m_Drivetrain = drivetrain;
    this.autoPath = autoPath;
    this.index = index;
    addRequirements(drivetrain, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var state = m_Drivetrain.getEndPath(autoPath, index);
    xGoal = state.positionMeters.getX();
    rotGoal = state.heading.getDegrees();

    m_xPIDController.setTolerance(0.2);
    m_xPIDController.setSetpoint(xGoal);

    m_yPIDController.setTolerance(1);
    m_yPIDController.setSetpoint(0);

    m_rotPIDController.setTolerance(1);
    m_rotPIDController.setSetpoint(rotGoal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Limelight.updateLimeLight();

    xSpeed = m_xPIDController.calculate(m_Drivetrain.getPose().getX());
    ySpeed = m_yPIDController.calculate(m_Limelight.getTX());
    rotSpeed = m_rotPIDController.calculate(m_Drivetrain.getRotation().getDegrees());

    m_Drivetrain.applyRequest(() -> drive.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(rotSpeed)).execute();
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
