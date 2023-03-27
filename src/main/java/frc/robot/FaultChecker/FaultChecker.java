package frc.robot.FaultChecker;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.northstar.AprilTagVisionIO;
import frc.robot.subsystems.vision.visionTest.CameraPoseTester;

public class FaultChecker {
	private int cameraTestingId = 5;
	private CameraPoseTester cameraPoseTester;
	private AprilTagVisionIO leftCamera;
	private AprilTagVisionIO centerCamera;
	private AprilTagVisionIO rightCamera;

	public FaultChecker(CameraPoseTester cameraPoseTester, AprilTagVisionIO leftCamera , AprilTagVisionIO centerCamera, AprilTagVisionIO rightCamera) {
		this.cameraPoseTester = cameraPoseTester;
		this.leftCamera = leftCamera;
		this.centerCamera = centerCamera;
		this.rightCamera = rightCamera;
		setupCameraTester();
	}

	/**
     * Used to start the camera tester
     * 
     * @param tagId
     */
    public void setupCameraTester() {
        cameraPoseTester.startAlignmentCheck(leftCamera.getIdentifier(), centerCamera.getIdentifier(), cameraTestingId);
        cameraPoseTester.startAlignmentCheck(rightCamera.getIdentifier(), centerCamera.getIdentifier(), cameraTestingId);
    }

	/**
	 * Used to update and retrive results form the camera tester
	 * 
	 * @return a pair with the left vs center in the first and the right vs center
	 *         in the second
	 */
	public Pair<CameraPoseTester.AlignmentTypes, CameraPoseTester.AlignmentTypes> updateCameraTester() {
		cameraPoseTester.update();
		return new Pair<CameraPoseTester.AlignmentTypes, CameraPoseTester.AlignmentTypes>(
				cameraPoseTester.updateAlignmentCheck(RobotContainer.leftCamera.getIdentifier(),
						RobotContainer.centerCamera.getIdentifier(), cameraTestingId),
				cameraPoseTester.updateAlignmentCheck(RobotContainer.rightCamera.getIdentifier(),
						RobotContainer.centerCamera.getIdentifier(), cameraTestingId));
	}

	public void update() {
		Pair<CameraPoseTester.AlignmentTypes, CameraPoseTester.AlignmentTypes> cameraAlignments = updateCameraTester();
		Logger.getInstance().recordOutput("FaultChecker/LeftVsCenter", cameraAlignments.getFirst().toString());
		Logger.getInstance().recordOutput("FaultChecker/RightVsCenter", cameraAlignments.getSecond().toString());
	}
}
