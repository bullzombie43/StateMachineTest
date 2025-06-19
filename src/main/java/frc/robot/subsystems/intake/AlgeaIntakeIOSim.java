package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;

/** Add your docs here. */
public class AlgeaIntakeIOSim implements AlgeaIntakeIO {
  private final IntakeSimulation intakeSimulation;
  private final AbstractDriveTrainSimulation drivetrainSimulation;
  private double setpointDegrees;
  private double rollerVelocityRPS;

  public AlgeaIntakeIOSim(AbstractDriveTrainSimulation driveTrainSimulation) {
    setpointDegrees = 0.0;
    rollerVelocityRPS = 0.0;

    this.drivetrainSimulation = driveTrainSimulation;

    // Create an IntakeSimulation for Algea
    this.intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Algae",
            driveTrainSimulation,
            Meters.of(0.6), // Adjust dimensions as needed
            Meters.of(0.15), // Adjust dimensions as needed
            IntakeSimulation.IntakeSide.BACK,
            1);
  }

  @Override
  public void updateInputs(AlgeaIntakeIO.AlgeaIntakeIOInputs inputs) {
    // Simulated inputs would be updated here
    inputs.pivotPositionDegrees = setpointDegrees; // Just set The pivot directly to the position
    inputs.pivotVelocityDegreesPerSec = 0.0; // Simulated velocity
    inputs.pivotVoltage = 0.0; // Simulated voltage
    inputs.pivotCurrent = 0.0; // Simulated current
    inputs.pivotTemperature = 30.0; // Simulated temperature
    inputs.motorConnected = true; // Assume motor is connected in simulation

    inputs.rollerVelocityRPS = rollerVelocityRPS; // Simulated roller velocity
    inputs.rollerVoltage = 0.0; // Simulated roller voltage
    inputs.rollerCurrent = 0.0; // Simulated roller current
    inputs.rollerTemperature = 30.0; // Simulated roller temperature
    inputs.rollerMotorConnected = true; // Assume roller motor is connected in simulation
    inputs.hasAlgea = hasAlgea();
  }

  @Override
  public void setSetpointDegrees(double setpointInDegrees) {
    this.setpointDegrees = setpointInDegrees;
  }

  @Override
  public void setPositionDegrees(double position) {
    this.setpointDegrees = position; // Directly set the position in simulation
  }

  @Override
  public void setVoltage(double voltage) {
    // In simulation, we might not need to do anything here
  }

  @Override
  public void setCurrent(double current) {
    // In simulation, we might not need to do anything here
  }

  @Override
  public void setGains(
      double kP, double kI, double kD, double kS, double kA, double kV, double kG) {

    IntakeConstants.algeaPivotGains.updateGains(kP, kI, kD, kS, kV, kA, kG);
  }

  @Override
  public void runRollerForward() {
    rollerVelocityRPS = IntakeConstants.algeaRollerForwardSpeed;
    intakeSimulation.startIntake();
  }

  @Override
  public void runRollerReverse() {
    rollerVelocityRPS = IntakeConstants.algeaRollerReverseSpeed;
    intakeSimulation.stopIntake();

    if (intakeSimulation.getGamePiecesAmount() > 0) {
      if (intakeSimulation.getGamePiecesAmount() > 0) {
        SimulatedArena.getInstance()
            .addGamePiece(
                new ReefscapeAlgaeOnField(
                    drivetrainSimulation
                        .getSimulatedDriveTrainPose()
                        .transformBy(new Transform2d(-0.6, 0, new Rotation2d()))
                        .getTranslation()));
      }
    }

    intakeSimulation.setGamePiecesCount(0);
  }

  @Override
  public void stopRoller() {
    rollerVelocityRPS = 0.0;
    intakeSimulation.stopIntake();
  }

  public boolean hasAlgea() {
    return intakeSimulation.getGamePiecesAmount() > 0;
  }

  @Override
  public void setHasAlgea(boolean hasAlgea) {
    if (!hasAlgea) intakeSimulation.setGamePiecesCount(0);
    else intakeSimulation.setGamePiecesCount(1);
  }
}
