package mot

import (
	"math"

	"github.com/google/uuid"
	"github.com/pkg/errors"
)

// SimpleTracker is naive implementation of Multi-object tracker (MOT).
// B is the blob type implementing Blob[B] interface.
type SimpleTracker[B Blob[B]] struct {
	// Main storage
	Objects map[uuid.UUID]B
	// Threshold distance (most of time in pixels). Default 30.0
	minDistThreshold float64
	// Max no match (max number of frames when object could not be found again). Default is 75
	maxNoMatch int
}

// NewSimpleTrackerDefault creates default instance of SimpleTracker
func NewSimpleTrackerDefault[B Blob[B]]() *SimpleTracker[B] {
	return &SimpleTracker[B]{
		Objects:          make(map[uuid.UUID]B),
		minDistThreshold: 30.0,
		maxNoMatch:       75,
	}
}

// NewSimpleTracker creates new instance of SimpleTracker
func NewNewSimpleTracker[B Blob[B]](minDistThreshold float64, maxNoMatch int) *SimpleTracker[B] {
	return &SimpleTracker[B]{
		Objects:          make(map[uuid.UUID]B),
		minDistThreshold: minDistThreshold,
		maxNoMatch:       maxNoMatch,
	}
}

func (tracker *SimpleTracker[B]) MatchObjects(newObjects []B) error {
	for objectID := range tracker.Objects {
		// Make sure that object is marked as deactivated
		tracker.Objects[objectID].Deactivate()
		tracker.Objects[objectID].PredictNextPosition()
	}
	blobsToRegister := make(map[uuid.UUID]B)
	priorityQueue := make(distanceHeap[B], 0)
	for i, newObject := range newObjects {
		minID := uuid.UUID{}
		minDistance := math.MaxFloat64
		for objectID, object := range tracker.Objects {
			dist := newObject.DistanceTo(object)
			distPredicted := newObject.DistanceTo(object)
			distVerifided := math.Min(dist, distPredicted)
			if distVerifided < minDistance {
				minDistance = distVerifided
				minID = objectID
			}
		}
		distBlob := distanceBlob[B]{
			underlying: newObjects[i],
			distance:   minDistance,
			id:         minID,
		}
		priorityQueue.Push(&distBlob)
	}

	// We need to prevent double update of objects
	reservedObjects := make(map[uuid.UUID]struct{})

	for priorityQueue.Len() > 0 {
		blobPoped := priorityQueue.Pop()
		minDistance := blobPoped.distance
		minID := blobPoped.id
		underlyingBlob := blobPoped.underlying
		// Check if object is already reserved
		// Since we are using priority queue with min-heap then we garantee that we will update existing objects with min distance only once.
		// For other objects with the same min_id we can create new objects
		if _, ok := reservedObjects[minID]; ok {
			// Register it immediately and continue
			blobsToRegister[underlyingBlob.GetID()] = underlyingBlob
			continue
		}
		// Additional check to filter objects
		if minDistance < underlyingBlob.GetDiagonal()*0.5 || minDistance < tracker.minDistThreshold {
			if _, ok := tracker.Objects[minID]; ok {
				err := tracker.Objects[minID].Update(underlyingBlob)
				if err != nil {
					return errors.Wrapf(err, "Can't update blob with id %s", minID.String())
				}
				// Last but not least:
				// We need to update ID of new object to match existing one
				underlyingBlob.SetID(minID)
				reservedObjects[minID] = struct{}{}
			} else {
				panic("should be impossible")
			}
		} else {
			// Otherwise register object as a new one
			blobsToRegister[underlyingBlob.GetID()] = underlyingBlob
		}
	}

	for blobID := range blobsToRegister {
		tracker.Objects[blobID] = blobsToRegister[blobID]
	}

	// Clean up existing data
	for objectID := range tracker.Objects {
		tracker.Objects[objectID].IncNoMatch()
		// Remove object if it was not found for a long time
		if tracker.Objects[objectID].GetNoMatchTimes() > tracker.maxNoMatch {
			delete(tracker.Objects, objectID)
		}
	}
	return nil
}
