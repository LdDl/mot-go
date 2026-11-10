package mot

import (
	"fmt"

	"github.com/google/uuid"
)

// MatchingAlgorithm is for algorithm type for matching detections to tracks
type MatchingAlgorithm uint16

const (
	// MatchingAlgorithmHungarian uses the Hungarian algorithm (Kuhn-Munkres) for optimal assignment
	MatchingAlgorithmHungarian MatchingAlgorithm = iota
	// MatchingAlgorithmGreedy uses a greedy algorithm for faster but potentially suboptimal assignment
	MatchingAlgorithmGreedy

	// SCALE_FACTOR quantizes IoU costs for Hungarian assignment.
	SCALE_FACTOR = 1_000_000.0
)

// ByteTracker is implementation of Multi-object tracker (MOT) called ByteTrack.
// B is the blob type implementing Blob[B] interface.
type ByteTracker[B Blob[B]] struct {
	// Maximum number of frames an object can be missing before it is removed
	maxDisappeared int
	// When positive, tracks expire by unmatched time instead of by maxDisappeared frames. Default is 0
	maxLostSeconds float64
	// Minimum accepted IoU, including scores equal to the threshold
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

// SetMaxDisappeared switches expiry back to frames. A positive limit expires on that missed frame; zero expires on the first miss.
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
	return track.GetNoMatchTimes() < max(1, bt.maxDisappeared)
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

	// Missed but unexpired tracks remain eligible for matching.
	for _, track := range bt.Objects {
		track.Deactivate()
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

	for id, track := range bt.Objects {
		if _, found := matchedTracks[id]; !found {
			track.PredictNextPosition()
			track.IncNoMatch()
		}
		if !bt.isAlive(track) {
			delete(bt.Objects, id)
		}
	}

	// Registration follows missed-track handling so the creation frame is not a miss.
	for _, detIdx := range highDetectionIndices {
		if _, found := matchedDetections[detIdx]; !found {
			newBlob := detections[detIdx]
			newBlob.Activate()
			bt.Objects[newBlob.GetID()] = newBlob
		}
	}

	return nil
}

// GetActiveTracks returns all unexpired tracks, including missed ones.
// Use Exists on SimpleBlob or BlobBBox to select tracks detected in the latest frame.
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
		return hungarianMatching(iouMatrix, bt.minIoU)
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
	candidates := make([][]matchCandidate, len(detectionIndices))
	row := make([]matchCandidate, 0, len(trackBBoxes))
	for detection := range detectionIndices {
		row = row[:0]
		for track := range trackBBoxes {
			score := iouMatrix[track][detection]
			if score >= bt.minIoU {
				row = append(row, matchCandidate{track, -score})
			}
		}
		candidates[detection] = append([]matchCandidate(nil), row...)
	}
	matches := greedyMatching(candidates, len(trackBBoxes))
	for i := range matches {
		matches[i][0], matches[i][1] = matches[i][1], matches[i][0]
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
				// Advance the filter exactly once before applying the measurement.
				track.PredictNextPosition()
				err := track.Update(allDetections[originalDetIdx])
				if err != nil {
					return fmt.Errorf("failed to update track %s: %w", trackID, err)
				}
				track.Activate()
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
