package SubSystems.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.Comparator;

import SubSystems.Drive.DriveSubsystem;
import SubSystems.Drive.TileCoordinate;
import Constants.FieldConstants;

/**
 * AprilTag-based localization system for tile-based navigation.
 * Uses AprilTag detection to determine robot's current tile position on the
 * field.
 * 
 * @author Pedro Pathing Team
 * @version 2.0
 */
public class AprilTagNavigator {

    private DriveSubsystem driveSubsystem;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Telemetry telemetry;

    // AprilTag field positions and specs are centralized in FieldConstants

    // Detection parameters optimized for DECODE field
    private final double MIN_DETECTION_DISTANCE = 6.0; // Minimum distance for reliable detection
    private final double MAX_DETECTION_DISTANCE = 120.0; // Maximum distance for detection (increased for goal tags)
    private final double MIN_DETECTION_CONFIDENCE = 0.6; // Minimum confidence threshold (slightly lowered for goal
                                                         // tags)

    public AprilTagNavigator(DriveSubsystem driveSubsystem, HardwareMap hardwareMap, Telemetry telemetry) {
        this.driveSubsystem = driveSubsystem;
        this.telemetry = telemetry;

        // Initialize AprilTag vision for DECODE field
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11) // DECODE uses 36h11 tag family
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Get the best AprilTag detection for localization
     * 
     * @return Best AprilTag detection or null if none found
     */
    public AprilTagDetection getBestDetection() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty())
            return null;

        // Filter detections by distance and confidence
        return detections.stream()
                .filter(tag -> tag.ftcPose.range >= MIN_DETECTION_DISTANCE)
                .filter(tag -> tag.ftcPose.range <= MAX_DETECTION_DISTANCE)
                .filter(tag -> tag.decisionMargin >= MIN_DETECTION_CONFIDENCE)
                .min(Comparator.comparingDouble(tag -> tag.ftcPose.range)) // Closest reliable detection
                .orElse(null);
    }

    /**
     * Get all valid AprilTag detections
     * 
     * @return List of valid detections
     */
    public List<AprilTagDetection> getAllDetections() {
        return aprilTag.getDetections().stream()
                .filter(tag -> tag.ftcPose.range >= MIN_DETECTION_DISTANCE)
                .filter(tag -> tag.ftcPose.range <= MAX_DETECTION_DISTANCE)
                .filter(tag -> tag.decisionMargin >= MIN_DETECTION_CONFIDENCE)
                .collect(java.util.stream.Collectors.toList());
    }

    /**
     * Get field position of an AprilTag by its ID
     * 
     * @param tagId AprilTag ID
     * @return Field position as {x, y, heading} or null if not found
     */
    private double[] getAprilTagFieldPosition(int tagId) {
        for (double[] position : FieldConstants.APRILTAG_POSITIONS) {
            if ((int) position[0] == tagId) {
                return new double[] { position[1], position[2], position[3] };
            }
        }
        return null;
    }

    /**
     * Calculate robot's field position based on AprilTag detection
     * 
     * @param detection AprilTag detection
     * @return Robot's field position as TileCoordinate, or null if calculation
     *         fails
     */
    public TileCoordinate calculateRobotPosition(AprilTagDetection detection) {
        if (detection == null)
            return null;

        double[] tagFieldPos = getAprilTagFieldPosition(detection.id);
        if (tagFieldPos == null) {
            telemetry.addData("AprilTag Error", "Unknown tag ID: %d", detection.id);
            return null;
        }

        // Get relative position from robot to tag
        double relativeX = detection.ftcPose.x; // Left/Right from robot
        double relativeY = detection.ftcPose.y; // Forward/Backward from robot
        double relativeYaw = Math.toRadians(detection.ftcPose.yaw); // Robot's heading relative to tag

        // Convert to field coordinates
        // Tag field position
        double tagX = tagFieldPos[0];
        double tagY = tagFieldPos[1];
        double tagHeading = Math.toRadians(tagFieldPos[2]);

        // Calculate robot's field position
        // First, rotate relative position by tag's heading
        double cosTag = Math.cos(tagHeading);
        double sinTag = Math.sin(tagHeading);

        double rotatedX = relativeX * cosTag - relativeY * sinTag;
        double rotatedY = relativeX * sinTag + relativeY * cosTag;

        // Then add to tag's field position
        double robotX = tagX + rotatedX;
        double robotY = tagY + rotatedY;

        // Calculate robot's field heading
        double robotHeading = tagHeading + relativeYaw;

        // Create and return tile coordinate
        TileCoordinate robotPos = new TileCoordinate(robotX, robotY);

        // Update drive subsystem with calculated position and heading
        if (driveSubsystem != null) {
            driveSubsystem.setPosition(robotPos);
            driveSubsystem.setHeading(robotHeading);
        }

        return robotPos;
    }

    /**
     * Update robot's position using AprilTag localization
     * 
     * @return True if position was successfully updated, false otherwise
     */
    public boolean updateRobotPosition() {
        AprilTagDetection bestDetection = getBestDetection();
        if (bestDetection == null) {
            if (telemetry != null) {
                telemetry.addData("AprilTag Localization", "No valid detections");
            }
            return false;
        }

        TileCoordinate robotPos = calculateRobotPosition(bestDetection);
        if (robotPos == null) {
            if (telemetry != null) {
                telemetry.addData("AprilTag Localization", "Position calculation failed");
            }
            return false;
        }

        if (telemetry != null) {
            telemetry.addData("AprilTag Localization", "Success");
            telemetry.addData("Detected Tag", "ID: %d, Range: %.1f", bestDetection.id, bestDetection.ftcPose.range);
            telemetry.addData("Robot Position", robotPos.getTilePosition());
            telemetry.addData("Robot Tab", robotPos.getTabPosition());
            telemetry.addData("Field Position", "X: %.1f, Y: %.1f", robotPos.getX(), robotPos.getY());
        }

        return true;
    }

    /**
     * Get robot's current tile position based on AprilTag localization
     * 
     * @return Current tile position or null if not localized
     */
    public TileCoordinate getCurrentTilePosition() {
        if (updateRobotPosition()) {
            return driveSubsystem.getCurrentPosition();
        }
        return null;
    }

    /**
     * Check if robot is currently localized using AprilTags
     * 
     * @return True if localized, false otherwise
     */
    public boolean isLocalized() {
        return getBestDetection() != null;
    }

    /**
     * Get detailed localization information
     * 
     * @return Localization info string
     */
    public String getLocalizationInfo() {
        List<AprilTagDetection> detections = getAllDetections();
        if (detections.isEmpty()) {
            return "No AprilTag detections";
        }

        StringBuilder info = new StringBuilder();
        info.append(String.format("Detected %d tags: ", detections.size()));

        for (AprilTagDetection detection : detections) {
            info.append(String.format("ID%d(%.1f) ", detection.id, detection.ftcPose.range));
        }

        TileCoordinate currentPos = getCurrentTilePosition();
        if (currentPos != null) {
            info.append(String.format("- Position: %s", currentPos.getTilePosition()));
        }

        return info.toString();
    }

    /**
     * Update telemetry with AprilTag localization information
     */
    public void updateLocalizationTelemetry() {
        if (telemetry == null)
            return;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("AprilTag Detections", detections.size());

        if (detections.isEmpty()) {
            telemetry.addData("Localization Status", "No detections");
            return;
        }

        AprilTagDetection best = getBestDetection();
        if (best != null) {
            telemetry.addData("Best Detection", "ID: %d, Range: %.1f, Confidence: %.2f",
                    best.id, best.ftcPose.range, best.decisionMargin);
        }

        TileCoordinate currentPos = getCurrentTilePosition();
        if (currentPos != null) {
            telemetry.addData("Robot Tile", currentPos.getTilePosition());
            telemetry.addData("Robot Tab", currentPos.getTabPosition());
            telemetry.addData("Field Position", "X: %.1f, Y: %.1f", currentPos.getX(), currentPos.getY());
            telemetry.addData("Robot Heading", "%.1f°", Math.toDegrees(driveSubsystem.getCurrentHeading()));
        } else {
            telemetry.addData("Localization Status", "Failed to calculate position");
        }
    }

    /**
     * Close the vision portal
     */
    public void closeVision() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Get the AprilTag processor for advanced usage
     * 
     * @return AprilTag processor instance
     */
    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTag;
    }

    /**
     * Get the vision portal for advanced usage
     * 
     * @return Vision portal instance
     */
    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    // ==================== DECODE FIELD SPECIFIC METHODS ====================

    /**
     * Check if the detected AprilTag is from the blue alliance goal
     * 
     * @param detection AprilTag detection
     * @return True if blue alliance goal (Tag ID 20)
     */
    public boolean isBlueAllianceGoal(AprilTagDetection detection) {
        return detection != null && detection.id == 20;
    }

    /**
     * Check if the detected AprilTag is from the red alliance goal
     * 
     * @param detection AprilTag detection
     * @return True if red alliance goal (Tag ID 24)
     */
    public boolean isRedAllianceGoal(AprilTagDetection detection) {
        return detection != null && detection.id == 24;
    }

    /**
     * Check if the detected AprilTag is from the obelisk (not recommended for
     * navigation)
     * 
     * @param detection AprilTag detection
     * @return True if obelisk tag (IDs 21, 22, 23)
     */
    public boolean isObeliskTag(AprilTagDetection detection) {
        return detection != null && (detection.id == 21 || detection.id == 22 || detection.id == 23);
    }

    /**
     * Get the alliance color of the detected AprilTag
     * 
     * @param detection AprilTag detection
     * @return "Blue", "Red", "Obelisk", or "Unknown"
     */
    public String getAllianceColor(AprilTagDetection detection) {
        if (detection == null)
            return "Unknown";

        if (isBlueAllianceGoal(detection))
            return "Blue";
        if (isRedAllianceGoal(detection))
            return "Red";
        if (isObeliskTag(detection))
            return "Obelisk";

        return "Unknown";
    }

    /**
     * Get the best detection from alliance goals only (excludes obelisk tags)
     * 
     * @return Best alliance goal detection or null if none found
     */
    public AprilTagDetection getBestAllianceGoalDetection() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty())
            return null;

        // Filter for alliance goals only (exclude obelisk tags)
        return detections.stream()
                .filter(tag -> tag.id == 20 || tag.id == 24) // Only blue and red alliance goals
                .filter(tag -> tag.ftcPose.range >= MIN_DETECTION_DISTANCE)
                .filter(tag -> tag.ftcPose.range <= MAX_DETECTION_DISTANCE)
                .filter(tag -> tag.decisionMargin >= MIN_DETECTION_CONFIDENCE)
                .min(Comparator.comparingDouble(tag -> tag.ftcPose.range)) // Closest reliable detection
                .orElse(null);
    }

    /**
     * Update robot position using only alliance goal detections (recommended for
     * DECODE)
     * 
     * @return True if position was successfully updated, false otherwise
     */
    public boolean updateRobotPositionFromAllianceGoals() {
        AprilTagDetection bestDetection = getBestAllianceGoalDetection();
        if (bestDetection == null) {
            if (telemetry != null) {
                telemetry.addData("AprilTag Localization", "No alliance goal detections");
            }
            return false;
        }

        TileCoordinate robotPos = calculateRobotPosition(bestDetection);
        if (robotPos == null) {
            if (telemetry != null) {
                telemetry.addData("AprilTag Localization", "Position calculation failed");
            }
            return false;
        }

        if (telemetry != null) {
            String alliance = getAllianceColor(bestDetection);
            telemetry.addData("AprilTag Localization", "Success - %s Alliance Goal", alliance);
            telemetry.addData("Detected Tag", "ID: %d, Range: %.1f", bestDetection.id, bestDetection.ftcPose.range);
            telemetry.addData("Robot Position", robotPos.getTilePosition());
            telemetry.addData("Robot Tab", robotPos.getTabPosition());
            telemetry.addData("Field Position", "X: %.1f, Y: %.1f", robotPos.getX(), robotPos.getY());
        }

        return true;
    }

	/**
	 * Update robot position using triangulation from both alliance goal tags
	 * when available. Falls back to best single alliance goal detection when
	 * only one is visible.
	 *
	 * Triangulation approach:
	 * - Compute independent robot poses from each valid detection (IDs 20, 24)
	 * - Fuse by simple averaging of positions and headings
	 * - If only one detection available, use that single pose
	 *
	 * @return True if position was successfully updated, false otherwise
	 */
	public boolean updateRobotPositionFromTriangulation() {
		List<AprilTagDetection> detections = aprilTag.getDetections();
		if (detections.isEmpty()) {
			if (telemetry != null) telemetry.addData("AprilTag Localization", "No detections");
			return false;
		}

		// Filter to valid alliance goal detections only (IDs 20 and 24) and within thresholds
		List<AprilTagDetection> goalDetections = detections.stream()
				.filter(tag -> tag.id == 20 || tag.id == 24)
				.filter(tag -> tag.ftcPose.range >= MIN_DETECTION_DISTANCE)
				.filter(tag -> tag.ftcPose.range <= MAX_DETECTION_DISTANCE)
				.filter(tag -> tag.decisionMargin >= MIN_DETECTION_CONFIDENCE)
				.collect(java.util.stream.Collectors.toList());

		if (goalDetections.isEmpty()) {
			if (telemetry != null) telemetry.addData("AprilTag Localization", "No alliance goal detections");
			return false;
		}

		TileCoordinate poseFrom20 = null;
		TileCoordinate poseFrom24 = null;
		double headingFrom20 = Double.NaN;
		double headingFrom24 = Double.NaN;

		for (AprilTagDetection det : goalDetections) {
			TileCoordinate pose = calculateRobotPosition(det);
			if (pose == null) continue;
			double heading = driveSubsystem != null ? driveSubsystem.getCurrentHeading() : Double.NaN;
			// calculateRobotPosition already updates driveSubsystem with heading; we derive heading directly from tag data too
			double[] tagFieldPos = getAprilTagFieldPosition(det.id);
			double tagHeading = Math.toRadians(tagFieldPos != null ? tagFieldPos[2] : 0);
			double relativeYaw = Math.toRadians(det.ftcPose.yaw);
			double computedHeading = tagHeading + relativeYaw;

			if (det.id == 20) {
				poseFrom20 = pose;
				headingFrom20 = computedHeading;
			} else if (det.id == 24) {
				poseFrom24 = pose;
				headingFrom24 = computedHeading;
			}
		}

		TileCoordinate fused;
		double fusedHeading;

		if (poseFrom20 != null && poseFrom24 != null) {
			// Simple average fusion
			double x = (poseFrom20.getX() + poseFrom24.getX()) / 2.0;
			double y = (poseFrom20.getY() + poseFrom24.getY()) / 2.0;
			fused = new TileCoordinate(x, y);
			// Average headings taking wrap-around into account
			double s = Math.sin(headingFrom20) + Math.sin(headingFrom24);
			double c = Math.cos(headingFrom20) + Math.cos(headingFrom24);
			fusedHeading = Math.atan2(s, c);
		} else if (poseFrom20 != null) {
			fused = poseFrom20;
			fusedHeading = headingFrom20;
		} else if (poseFrom24 != null) {
			fused = poseFrom24;
			fusedHeading = headingFrom24;
		} else {
			if (telemetry != null) telemetry.addData("AprilTag Localization", "No valid triangulation poses");
			return false;
		}

		if (driveSubsystem != null) {
			driveSubsystem.setPosition(fused);
			driveSubsystem.setHeading(fusedHeading);
		}

		if (telemetry != null) {
			telemetry.addData("AprilTag Triangulation", "Success");
			telemetry.addData("Field Position", "X: %.1f, Y: %.1f", fused.getX(), fused.getY());
			telemetry.addData("Heading", "%.1f°", Math.toDegrees(fusedHeading));
		}

		return true;
	}

    /**
     * Update telemetry with DECODE-specific localization information
     */
    public void updateDECODELocalizationTelemetry() {
        if (telemetry == null)
            return;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("AprilTag Detections", detections.size());

        if (detections.isEmpty()) {
            telemetry.addData("Localization Status", "No detections");
            return;
        }

        // Count different types of detections
        long blueGoals = detections.stream().filter(this::isBlueAllianceGoal).count();
        long redGoals = detections.stream().filter(this::isRedAllianceGoal).count();
        long obeliskTags = detections.stream().filter(this::isObeliskTag).count();

        telemetry.addData("Blue Alliance Goals", blueGoals);
        telemetry.addData("Red Alliance Goals", redGoals);
        telemetry.addData("Obelisk Tags", obeliskTags);

        AprilTagDetection best = getBestAllianceGoalDetection();
        if (best != null) {
            String alliance = getAllianceColor(best);
            telemetry.addData("Best Detection", "%s Alliance Goal - ID: %d, Range: %.1f, Confidence: %.2f",
                    alliance, best.id, best.ftcPose.range, best.decisionMargin);
        } else {
            telemetry.addData("Best Detection", "No alliance goal detected");
        }

        TileCoordinate currentPos = getCurrentTilePosition();
        if (currentPos != null) {
            telemetry.addData("Robot Tile", currentPos.getTilePosition());
            telemetry.addData("Robot Tab", currentPos.getTabPosition());
            telemetry.addData("Field Position", "X: %.1f, Y: %.1f", currentPos.getX(), currentPos.getY());
            telemetry.addData("Robot Heading", "%.1f°", Math.toDegrees(driveSubsystem.getCurrentHeading()));
        } else {
            telemetry.addData("Localization Status", "Failed to calculate position");
        }
    }
}
