package mot

import (
	"github.com/google/uuid"
)

// IoUTracker is a naive implementation of Multi-object tracker (MOT) with IoU matching.
// Uses hybrid IoU + distance matching for better recovery when IoU is zero.
type IoUTracker[B Blob[B]] struct {
	// Max no match (max number of frames when object could not be found again)
	maxNoMatch int
	// When positive, tracks expire by unmatched time instead of by maxNoMatch frames. Default is 0
	maxLostSeconds float64
	// Strict threshold for the hybrid IoU and distance score
	iouThreshold float64
	// Storage for tracked objects
	Objects map[uuid.UUID]B
}

// NewDefaultIoUTracker creates a default instance of IoUTracker.
// Default values: maxNoMatch=75, iouThreshold=0.3
func NewDefaultIoUTracker[B Blob[B]]() *IoUTracker[B] {
	return &IoUTracker[B]{
		maxNoMatch:   75,
		iouThreshold: 0.3,
		Objects:      make(map[uuid.UUID]B),
	}
}

// NewIoUTracker creates a new instance of IoUTracker with specified parameters.
func NewIoUTracker[B Blob[B]](maxNoMatch int, iouThreshold float64) *IoUTracker[B] {
	return &IoUTracker[B]{
		maxNoMatch:   maxNoMatch,
		iouThreshold: iouThreshold,
		Objects:      make(map[uuid.UUID]B),
	}
}

// SetMaxLostSeconds switches track expiry from a frame count to time: a track is
// removed once it has been unmatched for more than the given seconds. A frame
// count changes meaning whenever the effective frame rate does - frame skipping,
// a throttled detector, a stalled stream - while an occlusion lasts the same
// number of seconds regardless. Non-positive values are ignored
func (tracker *IoUTracker[B]) SetMaxLostSeconds(seconds float64) {
	if seconds > 0 {
		tracker.maxLostSeconds = seconds
	}
}

// SetMaxNoMatch switches track expiry back to a frame count (an object is removed after more than maxNoMatch missed frames)
func (tracker *IoUTracker[B]) SetMaxNoMatch(maxNoMatch int) {
	tracker.maxNoMatch = maxNoMatch
	tracker.maxLostSeconds = 0
}

// GetMaxLostSeconds returns the time-based expiry limit, 0 when expiry is by frames
func (tracker *IoUTracker[B]) GetMaxLostSeconds() float64 {
	return tracker.maxLostSeconds
}

// GetMaxNoMatch returns the frame-based expiry limit; in effect only while GetMaxLostSeconds is 0
func (tracker *IoUTracker[B]) GetMaxNoMatch() int {
	return tracker.maxNoMatch
}

// SetDt rebuilds every track for a new cycle time. MatchObjects does the same
// from the first detection it receives, but a frame without detections carries
// none, so call this before it: otherwise on such frames the tracks are
// predicted and expired over whatever interval the previous frame had, not the
// real one
func (tracker *IoUTracker[B]) SetDt(dt float64) {
	for _, object := range tracker.Objects {
		object.SetDt(dt)
	}
}

// isExpired reports whether an unmatched track has been lost for longer than allowed
func (tracker *IoUTracker[B]) isExpired(object B) bool {
	if tracker.maxLostSeconds > 0 {
		return object.GetLostSeconds() > tracker.maxLostSeconds
	}
	return object.GetNoMatchTimes() > tracker.maxNoMatch
}

type iouPrediction struct {
	id     uuid.UUID
	bbox   Rectangle
	center Point
}

// MatchObjects matches new detections to existing tracks using hybrid IoU and distance.
func (tracker *IoUTracker[B]) MatchObjects(newObjects []B) error {
	if len(newObjects) > 0 {
		tracker.SetDt(newObjects[0].GetDt())
	}
	tracks := make([]iouPrediction, 0, len(tracker.Objects))
	for id, object := range tracker.Objects {
		object.Deactivate()
		prediction := Rectangle{}
		if len(newObjects) > 0 {
			prediction = object.GetPredictedBBox()
		}
		tracks = append(tracks, iouPrediction{id: id, bbox: prediction, center: Point{X: prediction.X + prediction.Width/2, Y: prediction.Y + prediction.Height/2}})
	}
	candidates := make([][]matchCandidate, len(newObjects))
	row := make([]matchCandidate, 0, len(tracks))
	for detection, object := range newObjects {
		row = row[:0]
		bbox, center := object.GetBBox(), object.GetCenter()
		for track, predicted := range tracks {
			iou := IoU(bbox, predicted.bbox)
			distance := euclideanDistance(predicted.center, center)
			distanceScore := 1 / (1 + distance*0.01)
			score := distanceScore * 0.5
			if iou > 0.05 {
				score = iou*0.8 + distanceScore*0.2
			}
			if score > 0 && score > tracker.iouThreshold {
				row = append(row, matchCandidate{track, -score})
			}
		}
		candidates[detection] = append([]matchCandidate(nil), row...)
	}
	matchedTracks := make([]bool, len(tracks))
	matchedDetections := make([]bool, len(newObjects))
	for _, match := range greedyMatching(candidates, len(tracks)) {
		detection, track := match[0], match[1]
		id := tracks[track].id
		object := tracker.Objects[id]
		object.PredictNextPosition()
		if err := object.Update(newObjects[detection]); err != nil {
			return err
		}
		object.Activate()
		object.ResetNoMatch()
		newObjects[detection].SetID(id)
		matchedTracks[track] = true
		matchedDetections[detection] = true
	}
	for track, entry := range tracks {
		object := tracker.Objects[entry.id]
		if !matchedTracks[track] {
			object.PredictNextPosition()
			object.IncNoMatch()
		}
		if tracker.isExpired(object) {
			delete(tracker.Objects, entry.id)
		}
	}
	for detection, object := range newObjects {
		if !matchedDetections[detection] {
			object.Activate()
			tracker.Objects[object.GetID()] = object
		}
	}
	return nil
}
