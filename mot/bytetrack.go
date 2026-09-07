package mot

import (
	"fmt"

	"github.com/arthurkushman/go-hungarian"
	"github.com/google/uuid"
)

// MatchingAlgorithm is for algorithm type for matching detections to tracks
type MatchingAlgorithm uint16

const (
	// MatchingAlgorithmHungarian uses the Hungarian algorithm (Kuhn-Munkres) for optimal assignment
	MatchingAlgorithmHungarian MatchingAlgorithm = iota
	// MatchingAlgorithmGreedy uses a greedy algorithm for faster but potentially suboptimal assignment
	MatchingAlgorithmGreedy

	SCALE_FACTOR = 1_000_000.0
)

// ByteTracker is implementation of Multi-object tracker (MOT) called ByteTrack.
// B is the blob type implementing Blob[B] interface.
type ByteTracker[B Blob[B]] struct {
	// Maximum number of frames an object can be missing before it is removed
	maxDisappeared int
	// When positive, tracks expire by unmatched time instead of by maxDisappeared frames. Default is 0
	maxLostSeconds float64
	// Maximum distance between two objects to be considered the same
	minIoU float64
	// High detection confidence threshold
	highThresh float64
	// Low detection confidence threshold
	lowThresh float64
	// Algorithm to use for matching
	algorithm MatchingAlgorithm
	// Main storage
	Objects map[uuid.UUID]B
}

// DefaultByteTracker creates a ByteTracker with default parameters.
func DefaultByteTracker[B Blob[B]]() *ByteTracker[B] {
	return &ByteTracker[B]{
		maxDisappeared: 5,
		minIoU:         0.3,
		highThresh:     0.5,
		lowThresh:      0.3,
		algorithm:      MatchingAlgorithmHungarian,
		Objects:        make(map[uuid.UUID]B),
	}
}

// NewByteTracker creates a new instance of ByteTracker with specified parameters.
func NewByteTracker[B Blob[B]](maxDisappeared int, minIoU, highThresh, lowThresh float64, algorithm MatchingAlgorithm) *ByteTracker[B] {
	return &ByteTracker[B]{
		maxDisappeared: maxDisappeared,
		minIoU:         minIoU,
		highThresh:     highThresh,
		lowThresh:      lowThresh,
		algorithm:      algorithm,
		Objects:        make(map[uuid.UUID]B),
	}
}

// SetMaxLostSeconds switches track expiry from a frame count to time: a track is
// removed once it has been unmatched for more than the given seconds. A frame
// count changes meaning whenever the effective frame rate does - frame skipping,
// a throttled detector, a stalled stream - while an occlusion lasts the same
// number of seconds regardless. Non-positive values are ignored
func (bt *ByteTracker[B]) SetMaxLostSeconds(seconds float64) {
	if seconds > 0 {
		bt.maxLostSeconds = seconds
	}
}

// SetMaxDisappeared switches track expiry back to a frame count (a track is removed once it reaches maxDisappeared missed frames)
func (bt *ByteTracker[B]) SetMaxDisappeared(maxDisappeared int) {
	bt.maxDisappeared = maxDisappeared
	bt.maxLostSeconds = 0
}

// GetMaxLostSeconds returns the time-based expiry limit, 0 when expiry is by frames
func (bt *ByteTracker[B]) GetMaxLostSeconds() float64 {
	return bt.maxLostSeconds
}

// GetMaxDisappeared returns the frame-based expiry limit; in effect only while GetMaxLostSeconds is 0
func (bt *ByteTracker[B]) GetMaxDisappeared() int {
	return bt.maxDisappeared
}

// SetDt rebuilds every track for a new cycle time. MatchObjects does the same
// from the first detection it receives, but a frame without detections carries
// none, so call this before it: otherwise on such frames the tracks are
// predicted and expired over whatever interval the previous frame had, not the
// real one
func (bt *ByteTracker[B]) SetDt(dt float64) {
	for _, object := range bt.Objects {
		object.SetDt(dt)
	}
}

// isAlive reports whether a track is still alive, i.e. not lost for longer than allowed
func (bt *ByteTracker[B]) isAlive(track B) bool {
	if bt.maxLostSeconds > 0 {
		return track.GetLostSeconds() <= bt.maxLostSeconds
	}
	return track.GetNoMatchTimes() < bt.maxDisappeared
}

// bboxPair is a helper struct to pair track ID with its bounding box.
type bboxPair struct {
	ID   uuid.UUID
	BBox Rectangle
}

// MatchObjects matches objects in the current frame with existing tracks.
// Detections are []B and confidences are []float64.
func (bt *ByteTracker[B]) MatchObjects(detections []B, confidences []float64) error {
	if len(detections) != len(confidences) {
		return fmt.Errorf("detections and confidences arrays must have the same length. Conf array size: %d. Detections array size: %d",
			len(confidences), len(detections))
	}

	// Adopt the real interval between calls, reported through the cycle time of
	// the incoming detections. See SimpleTracker.MatchObjects for why
	if len(detections) > 0 {
		dt := detections[0].GetDt()
		for _, track := range bt.Objects {
			track.SetDt(dt)
		}
	}

	// Predict next positions for all existing tracks via Kalman filter
	for _, track := range bt.Objects {
		track.PredictNextPosition()
	}

	// Get active tracks
	activeTrackIDs := make([]uuid.UUID, 0)
	activeTrackBBoxes := make([]bboxPair, 0)
	for id, track := range bt.Objects {
		if bt.isAlive(track) {
			activeTrackIDs = append(activeTrackIDs, id)
			activeTrackBBoxes = append(activeTrackBBoxes, bboxPair{
				ID:   id,
				BBox: track.GetPredictedBBox(),
			})
		}
	}

	// Set of matched tracks for stage 1
	matchedTracks := make(map[uuid.UUID]struct{})
	// Set of matched detection indices for stage 1
	matchedDetections := make(map[int]struct{})

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
		if track, ok := bt.Objects[id]; ok {
			unmatchedTrackBBoxes = append(unmatchedTrackBBoxes, bboxPair{
				ID:   id,
				BBox: track.GetPredictedBBox(),
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
		if !bt.isAlive(track) {
			delete(bt.Objects, id)
		}
	}

	return nil
}

// GetActiveTracks returns a slice of active tracks.
func (bt *ByteTracker[B]) GetActiveTracks() []B {
	activeTracks := make([]B, 0, len(bt.Objects))
	for _, track := range bt.Objects {
		if bt.isAlive(track) {
			activeTracks = append(activeTracks, track)
		}
	}
	return activeTracks
}

// createIoUMatrix is helper function to create IoU matrix.
// trackBBoxes: a slice of structs containing track ID and its BBox.
// detectionIndices: a slice of original indices into the detections array.
// detections: the full slice of detected blobs for the current frame.
func (bt *ByteTracker[B]) createIoUMatrix(
	trackBBoxes []bboxPair,
	detectionIndices []int,
	allDetections []B,
) [][]float64 {
	iouMatrix := make([][]float64, len(trackBBoxes))
	for i, trkBox := range trackBBoxes {
		row := make([]float64, len(detectionIndices))
		for j, detIdx := range detectionIndices {
			detRect := allDetections[detIdx].GetBBox()
			iouVal := IoU(trkBox.BBox, detRect)
			row[j] = iouVal
		}
		iouMatrix[i] = row
	}
	return iouMatrix
}

// performMatching is helper function to perform matching using Hungarian or Greedy algorithm.
// trackBBoxes: the track bboxes for the current matching stage.
// detectionIndices: the original detection indices for the current matching stage.
// Returns: a slice of [2]int, where each element is {trackIndexInTrackBBoxes, detectionIndexInDetectionIndices}.
func (bt *ByteTracker[B]) performMatching(
	iouMatrix [][]float64,
	trackBBoxes []bboxPair,
	detectionIndices []int,
) [][2]int {
	switch bt.algorithm {
	case MatchingAlgorithmHungarian:
		if len(trackBBoxes) == 0 || len(detectionIndices) == 0 {
			return [][2]int{}
		}
		numTracks := len(trackBBoxes)
		numDetections := len(detectionIndices)

		var paddedMatrix [][]float64
		var actualNumTracks, actualNumDetections int
		if numTracks == numDetections {
			// Square matrix - use as is
			paddedMatrix = iouMatrix
			actualNumTracks = numTracks
			actualNumDetections = numDetections
		} else {
			// Rectangular matrix - pad to make it square
			paddedSize := maxInt(numTracks, numDetections)
			paddedMatrix = make([][]float64, paddedSize)
			// Initialize with zeros (dummy IoU values)
			for i := 0; i < paddedSize; i++ {
				paddedMatrix[i] = make([]float64, paddedSize)
			}
			// Copy original IoU values
			for i := 0; i < numTracks; i++ {
				for j := 0; j < numDetections; j++ {
					paddedMatrix[i][j] = iouMatrix[i][j]
				}
			}
			// Padding is done with 0.0 values (lowest IoU)
			actualNumTracks = numTracks
			actualNumDetections = numDetections
		}
		// Apply Hungarian algorithm
		assignmentsMap := hungarian.SolveMax(paddedMatrix)
		// Convert map[int]map[int]float64 to [][2]int
		matches := make([][2]int, 0)
		for trackIndex, rowMap := range assignmentsMap {
			if len(rowMap) > 0 {
				// Assuming the inner map contains one entry: {detectionIndex: iou_value}
				var detectionIndex int
				// Get the first (and assumed only) key
				for detIdx := range rowMap {
					detectionIndex = detIdx
					break
				}
				// Ensure trackIndex and detectionIndex are within bounds of the
				// current stage's slices. Anything outside landed on a padding
				// column added to square the cost matrix, which is the ordinary
				// outcome whenever there are more detections than tracks - drop
				// it silently, the way the Rust implementation does
				if trackIndex < actualNumTracks && detectionIndex < actualNumDetections {
					matches = append(matches, [2]int{trackIndex, detectionIndex})
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

// performGreedyMatching is helper function for greedy matching.
func (bt *ByteTracker[B]) performGreedyMatching(
	iouMatrix [][]float64,
	trackBBoxes []bboxPair,
	detectionIndices []int,
) [][2]int {
	matches := make([][2]int, 0)
	// Keep track of detection indices that are already matched
	matchedDetIndicesInStage := make(map[int]struct{})
	numTracksInStage := len(trackBBoxes)
	numDetectionsInStage := len(detectionIndices)
	if numTracksInStage == 0 || numDetectionsInStage == 0 {
		return matches
	}
	// Iterate through tracks of the current stage
	for i := 0; i < numTracksInStage; i++ {
		// Initialize with a value lower than any possible IoU
		bestIoU := -1.0
		bestDetIdxInStage := -1
		// Iterate through detections of the current stage
		for j := 0; j < numDetectionsInStage; j++ {
			if _, found := matchedDetIndicesInStage[j]; found {
				// This detection (in current stage) is already matched
				continue
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
// trackBBoxes: the list of track ID/BBox structs used for this matching stage.
// detectionIndices: the list of original detection indices used for this stage.
// iouMatrix: the IoU matrix for this stage.
// allDetections: the full list of detections in the current frame.
// matchedTracks: set to add matched track IDs to.
// matchedDetections: set to add matched original detection indices to.
func (bt *ByteTracker[B]) processMatches(
	matches [][2]int,
	trackBBoxes []bboxPair,
	detectionIndices []int,
	iouMatrix [][]float64,
	allDetections []B,
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
				// Hand the track's identity back to the caller's detection, the
				// way SimpleTracker does. Callers read the id straight off the
				// detections they passed in; without this every detection keeps
				// the throwaway id it was constructed with, so a perfectly
				// tracked object looks like a brand new object on every frame
				allDetections[originalDetIdx].SetID(trackID)
				matchedTracks[trackID] = struct{}{}
				matchedDetections[originalDetIdx] = struct{}{}
			}
		}
	}
	return nil
}
