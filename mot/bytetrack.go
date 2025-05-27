package mot

import (
	"fmt"

	"github.com/arthurkushman/go-hungarian"
	"github.com/google/uuid"
)

// MatchingAlgorithm is for algorithm type for matching detections to tracks
type MatchingAlgorithm uint16

const (
	// Use the Hungarian algorithm (Kuhn-Munkres) for optimal assignment
	MatchingAlgorithmHungarian MatchingAlgorithm = iota
	// Use a greedy algorithm for faster but potentially suboptimal assignment
	MatchingAlgorithmGreedy

	SCALE_FACTOR = 1_000_000.0
)

// ByteTracker is implementation of Multi-object tracker (MOT) called ByteTrack.
type ByteTracker struct {
	// Maximum number of frames an object can be missing before it is removed
	maxDisappeared int
	// Maximum distance between two objects to be considered the same
	minIoU float64
	// High detection confidence threshold
	highThresh float64
	// Low detection confidence threshold
	lowThresh float64
	// Algorithm to use for matching
	algorithm MatchingAlgorithm
	// Main storage
	Objects map[uuid.UUID]*SimpleBlob
}

// DefaultByteTracker creates a ByteTracker with default parameters.
func DefaultByteTracker() *ByteTracker {
	return &ByteTracker{
		maxDisappeared: 5,
		minIoU:         0.3,
		highThresh:     0.5,
		lowThresh:      0.3,
		algorithm:      MatchingAlgorithmHungarian,
		Objects:        make(map[uuid.UUID]*SimpleBlob),
	}
}

// NewByteTracker creates a new instance of ByteTracker with specified parameters.
func NewByteTracker(maxDisappeared int, minIoU, highThresh, lowThresh float64, algorithm MatchingAlgorithm) *ByteTracker {
	return &ByteTracker{
		maxDisappeared: maxDisappeared,
		minIoU:         minIoU,
		highThresh:     highThresh,
		lowThresh:      lowThresh,
		algorithm:      algorithm,
		Objects:        make(map[uuid.UUID]*SimpleBlob),
	}
}

// bboxPair is a helper struct to pair track ID with its bounding box.
type bboxPair struct {
	ID   uuid.UUID
	BBox Rectangle
}

// MatchObjects matches objects in the current frame with existing tracks.
// Detections are []*SimpleBlob and confidences are []float64.
// SimpleBlob has an ID and can be updated.
func (bt *ByteTracker) MatchObjects(detections []*SimpleBlob, confidences []float64) error {
	if len(detections) != len(confidences) {
		return fmt.Errorf("detections and confidences arrays must have the same length. Conf array size: %d. Detections array size: %d",
			len(confidences), len(detections))
	}

	// Predict next positions for all existing tracks via Kalman filter
	for _, track := range bt.Objects {
		track.PredictNextPosition()
	}

	// Get active tracks
	activeTrackIDs := make([]uuid.UUID, 0)
	activeTrackBBoxes := make([]bboxPair, 0)
	for id, track := range bt.Objects {
		if track.GetNoMatchTimes() < bt.maxDisappeared {
			activeTrackIDs = append(activeTrackIDs, id)
			activeTrackBBoxes = append(activeTrackBBoxes, bboxPair{
				ID:   id,
				BBox: track.GetBBox(),
			})
		}
	}

	// Set of matched tracks for stage 1
	matchedTracks := make(map[uuid.UUID]struct{})
	// Set of matched detection indices for stage 1
	matchedDetections := make(map[int]struct{}) // Using original detection index

	// 1. First stage: Match high confidence detections
	highDetectionIndices := make([]int, 0)
	for i, conf := range confidences {
		if conf >= bt.highThresh {
			highDetectionIndices = append(highDetectionIndices, i)
		}
	}

	// Associate high confidence detections with tracks
	// Calculate IoU matrix between tracks and high confidence detections
	if len(activeTrackBBoxes) > 0 && len(highDetectionIndices) > 0 {
		// Create IoU matrix: rows = tracks, columns = detections
		iouMatrix := bt.createIoUMatrix(activeTrackBBoxes, highDetectionIndices, detections)
		// Perform matching
		matches := bt.performMatching(iouMatrix, activeTrackBBoxes, highDetectionIndices)
		// Process matches
		err := bt.processMatches(matches, activeTrackBBoxes, highDetectionIndices, iouMatrix, detections, matchedTracks, matchedDetections)
		if err != nil {
			return fmt.Errorf("error processing matches in stage 1: %w", err)
		}
	}

	// 2. Second stage: Match low confidence detections with remaining tracks
	unmatchedTrackIDs := make([]uuid.UUID, 0)
	for _, id := range activeTrackIDs {
		if _, found := matchedTracks[id]; !found {
			unmatchedTrackIDs = append(unmatchedTrackIDs, id)
		}
	}
	unmatchedTrackBBoxes := make([]bboxPair, 0)
	for _, id := range unmatchedTrackIDs {
		if track, ok := bt.Objects[id]; ok { // Ensure track still exists
			unmatchedTrackBBoxes = append(unmatchedTrackBBoxes, bboxPair{
				ID:   id,
				BBox: track.GetBBox(),
			})
		}
	}
	lowDetectionIndices := make([]int, 0)
	for i, conf := range confidences {
		// Only consider detections not already matched
		if _, found := matchedDetections[i]; !found {
			if conf < bt.highThresh && conf >= bt.lowThresh {
				lowDetectionIndices = append(lowDetectionIndices, i)
			}
		}
	}

	// Associate remaining tracks with low confidence detections
	// Second association stage
	if len(unmatchedTrackBBoxes) > 0 && len(lowDetectionIndices) > 0 {
		// Create IoU matrix
		iouMatrix := bt.createIoUMatrix(unmatchedTrackBBoxes, lowDetectionIndices, detections)
		// Perform matching
		matches := bt.performMatching(iouMatrix, unmatchedTrackBBoxes, lowDetectionIndices)
		// Process matches
		err := bt.processMatches(matches, unmatchedTrackBBoxes, lowDetectionIndices, iouMatrix, detections, matchedTracks, matchedDetections)
		if err != nil {
			return fmt.Errorf("error processing matches in stage 2: %w", err)
		}
	}

	// 3. Add new tracks for unmatched high confidence detections
	for _, detIdx := range highDetectionIndices {
		if _, found := matchedDetections[detIdx]; !found {
			newBlob := detections[detIdx]
			newBlob.Activate()
			bt.Objects[newBlob.GetID()] = newBlob
		}
	}

	// 4. Increment no_match_times for unmatched tracks
	for id, track := range bt.Objects {
		if _, found := matchedTracks[id]; !found {
			track.IncNoMatch()
		}
	}

	// 5. Remove tracks that have disappeared for too long
	for id, track := range bt.Objects {
		if track.GetNoMatchTimes() >= bt.maxDisappeared {
			delete(bt.Objects, id)
		}
	}

	return nil
}

// GetActiveTracks returns a slice of active tracks.
func (bt *ByteTracker) GetActiveTracks() []*SimpleBlob {
	activeTracks := make([]*SimpleBlob, 0, len(bt.Objects))
	for _, track := range bt.Objects {
		if track.GetNoMatchTimes() < bt.maxDisappeared {
			activeTracks = append(activeTracks, track)
		}
	}
	return activeTracks
}

// createIoUMatrix is helper function to create IoU matrix
// trackBBoxes: a slice of structs containing track ID and its BBox
// detectionIndices: a slice of original indices into the `detections` array
// detections: the full slice of detected SimpleBlobs for the current frame
func (bt *ByteTracker) createIoUMatrix(
	trackBBoxes []bboxPair,
	detectionIndices []int,
	allDetections []*SimpleBlob,
) [][]float64 {
	iouMatrix := make([][]float64, len(trackBBoxes))
	for i, trkBox := range trackBBoxes {
		row := make([]float64, len(detectionIndices))
		for j, detIdx := range detectionIndices {
			detRect := allDetections[detIdx].GetBBox() // Assumes SimpleBlob has GetBBox()
			iouVal := IoU(trkBox.BBox, detRect)        // Assuming global IoU or from a utils package
			row[j] = iouVal
		}
		iouMatrix[i] = row
	}
	return iouMatrix
}

// performMatching is helper function to perform matching using Hungarian or Greedy algorithm
// trackBBoxes: the track bboxes for the current matching stage
// detectionIndices: the original detection indices for the current matching stage
// Returns: a slice of [2]int, where each element is {trackIndexInTrackBBoxes, detectionIndexInDetectionIndices}
func (bt *ByteTracker) performMatching(
	iouMatrix [][]float64,
	trackBBoxes []bboxPair,
	detectionIndices []int,
) [][2]int {
	switch bt.algorithm {
	case MatchingAlgorithmHungarian:
		if len(trackBBoxes) == 0 || len(detectionIndices) == 0 {
			return [][2]int{}
		}
		// Check if rows <= columns requirement is met for Hungarian
		if len(trackBBoxes) > len(detectionIndices) {
			// Fall back to greedy if we have more tracks than detections
			return bt.performGreedyMatching(iouMatrix, trackBBoxes, detectionIndices)
		}
		// Apply Hungarian algorithm
		assignmentsMap := hungarian.SolveMax(iouMatrix)
		// Convert map[int]map[int]float64 to [][2]int
		matches := make([][2]int, 0)
		for trackIndex, rowMap := range assignmentsMap {
			if len(rowMap) > 0 {
				// Assuming the inner map contains one entry: {detectionIndex: iou_value}
				var detectionIndex int
				for detIdx := range rowMap { // Get the first (and assumed only) key
					detectionIndex = detIdx
					break
				}
				// Ensure trackIndex and detectionIndex are within bounds of the current stage's slices
				if trackIndex < len(trackBBoxes) && detectionIndex < len(detectionIndices) {
					// @todo: maybe we should check the original IoU value here?
					// originalIoU := rowMap[detectionIndex]
					// if originalIoU >= bt.minIoU {
					// matches = append(matches, [2]int{trackIndex, detectionIndex})
					// }
					matches = append(matches, [2]int{trackIndex, detectionIndex})
				} else {
					fmt.Printf("Warning: Hungarian assignment out of bounds. TrackIdx: %d, DetIdx: %d\n", trackIndex, detectionIndex)
				}
			}
		}
		return matches
	case MatchingAlgorithmGreedy:
		return bt.performGreedyMatching(iouMatrix, trackBBoxes, detectionIndices)
	default:
		return bt.performGreedyMatching(iouMatrix, trackBBoxes, detectionIndices)
	}
}

// performGreedyMatching is helper function for greedy matching
func (bt *ByteTracker) performGreedyMatching(
	iouMatrix [][]float64,
	trackBBoxes []bboxPair,
	detectionIndices []int,
) [][2]int {
	matches := make([][2]int, 0)
	// Keep track of detection indices (relative to the current stage's detectionIndices slice) that are already matched
	matchedDetIndicesInStage := make(map[int]struct{})
	numTracksInStage := len(trackBBoxes)
	numDetectionsInStage := len(detectionIndices)
	if numTracksInStage == 0 || numDetectionsInStage == 0 {
		return matches
	}
	for i := 0; i < numTracksInStage; i++ { // Iterate through tracks of the current stage
		bestIoU := -1.0 // Initialize with a value lower than any possible IoU
		bestDetIdxInStage := -1
		for j := 0; j < numDetectionsInStage; j++ { // Iterate through detections of the current stage
			if _, found := matchedDetIndicesInStage[j]; found {
				continue // This detection (in current stage) is already matched
			}
			currentIoU := iouMatrix[i][j]
			// Also check against minIoU here
			if currentIoU > bestIoU && currentIoU >= bt.minIoU {
				bestIoU = currentIoU
				bestDetIdxInStage = j
			}
		}
		if bestDetIdxInStage != -1 {
			matches = append(matches, [2]int{i, bestDetIdxInStage})
			matchedDetIndicesInStage[bestDetIdxInStage] = struct{}{}
		}
	}
	return matches
}

// processMatches updates tracks and marks matched entities.
// matches: slice of (trackIndex, detectionIndex) pairs.
//
//	trackIndex is index into trackBBoxes.
//	detectionIndex is index into detectionIndices.
//
// trackBBoxes: the list of track ID/BBox structs used for this matching stage.
// detectionIndices: the list of original detection indices used for this stage.
// iouMatrix: the IoU matrix for this stage.
// allDetections: the full list of detections in the current frame.
// matchedTracks: set to add matched track IDs to.
// matchedDetections: set to add matched original detection indices to.
func (bt *ByteTracker) processMatches(
	matches [][2]int, // Assuming performMatching returns [][2]int{{trackIdx, detIdx}, ...}
	trackBBoxes []bboxPair,
	detectionIndices []int,
	iouMatrix [][]float64,
	allDetections []*SimpleBlob,
	matchedTracks map[uuid.UUID]struct{},
	matchedDetections map[int]struct{},
) error {
	for _, match := range matches {
		trackIdxInStage := match[0]
		detIdxInStage := match[1]
		iouVal := iouMatrix[trackIdxInStage][detIdxInStage]
		if iouVal >= bt.minIoU {
			trackID := trackBBoxes[trackIdxInStage].ID
			originalDetIdx := detectionIndices[detIdxInStage]
			if track, ok := bt.Objects[trackID]; ok {
				// Pass the detected blob
				err := track.Update(allDetections[originalDetIdx])
				if err != nil {
					return fmt.Errorf("failed to update track %s: %w", trackID, err)
				}
				track.ResetNoMatch()
				matchedTracks[trackID] = struct{}{}
				matchedDetections[originalDetIdx] = struct{}{}
			}
		}
	}
	return nil
}
